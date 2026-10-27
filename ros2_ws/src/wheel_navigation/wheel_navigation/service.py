"""A/B navigation owner, SDK observations and durable command reconciliation."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json
import logging
import math
import threading
import time
import uuid

from wheel_navigation.journal import Journal
from wheel_sdk_bridge.http_client import SDKError
from wheel_sdk_bridge.lift_client import base_ready, base_stationary


class NavigationError(RuntimeError):
    def __init__(self, message, status=409):
        super().__init__(message)
        self.status = status


def object_value(value):
    if isinstance(value, str):
        try: value = json.loads(value)
        except ValueError: return {}
    return value if isinstance(value, dict) else {}


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class NavigationService:
    def __init__(self, atlas, lift, journal_path, *, enabled=False, mission_timeout=180.,
                 feedback_max_age=2., poll_interval=.5, stop_timeout=8., clock=time.time,
                 station_points=None):
        self.atlas, self.lift = atlas, lift
        self.journal = Journal(journal_path)
        self.enabled = enabled
        self.owner = uuid.uuid4().hex
        self.clock = clock
        self.mission_timeout = mission_timeout
        self.feedback_max_age = feedback_max_age
        self.poll_interval = poll_interval
        self.stop_timeout = stop_timeout
        self.shutdown = threading.Event()
        self.worker = None
        self.worker_error = ''
        self.station_points = station_points or {'source_pallet':'A','target_pallet':'B'}

    def start_worker(self):
        if self.enabled and self.worker is None:
            self.worker = threading.Thread(target=self._work, name='atlas-navigation-owner', daemon=True)
            self.worker.start()

    def close(self):
        self.shutdown.set()
        if self.worker:
            self.worker.join(timeout=8.)

    def _work(self):
        while not self.shutdown.wait(self.poll_interval):
            try:
                # No external calls when idle. Reads never start/prepare SDK modules.
                with self.journal.transaction() as data:
                    active = data.get('active')
                if active:
                    self.snapshot(advance=True)
            except Exception as exc:
                # Journal keeps the in-flight state; snapshot will expose corruption/errors.
                # Never restart a movement after an exception.
                self.worker_error = str(exc)
                logging.getLogger(__name__).exception('Navigation monitor failed; movement is never retried')

    def _read(self):
        def attempt(fn):
            try: return {'connected':True, 'data':fn(), 'error':''}
            except SDKError as exc: return {'connected':False, 'data':{}, 'error':str(exc)}
        with ThreadPoolExecutor(max_workers=2) as pool:
            a = pool.submit(attempt, self.atlas.snapshot)
            b = pool.submit(attempt, self.lift.snapshot)
            return a.result(), b.result()

    def _fresh(self, atlas):
        nav = object_value(atlas.get('status', {}).get('nav_status'))
        pipeline = object_value(atlas.get('status', {}).get('pipeline'))
        component = object_value(object_value(pipeline.get('components')).get('navigation'))
        age = component.get('age_ms')
        return (component.get('online') is True and finite(age) and 0 <= age <= self.feedback_max_age*1000
                and all(finite(nav.get(key)) and 0 <= nav[key] <= self.feedback_max_age
                        for key in ('map_pose_age_sec', 'odom_age_sec')))

    @staticmethod
    def _points(atlas):
        points = atlas.get('points', {})
        if points.get('ok') is not True or not isinstance(points.get('points'), list):
            return {}
        selected = [p for p in points['points'] if isinstance(p, dict)
                    and p.get('name') in ('A','B') and all(finite(p.get(k)) for k in ('x','y','yaw'))]
        if len({p['name'] for p in selected}) != len(selected): return {}
        return {p['name']:p for p in selected}

    def _reasons(self, a, b):
        reasons = []
        if not self.enabled: reasons.append('导航控制未开启：现场环境就绪后使用 --enable-navigation')
        if not a['connected']: reasons.append('陆图 SDK 未连接：'+a['error'])
        if not b['connected']: reasons.append('机器人 SDK 未连接：'+b['error'])
        if not a['connected'] or not b['connected']: return reasons
        atlas = a['data']
        points = atlas.get('points', {})
        robot = atlas.get('robot', {})
        if points.get('map_name') != self.atlas.map_name or not points.get('map_revision'):
            reasons.append('地图点位接口不可用或地图版本缺失')
        if set(self._points(atlas)) != {'A','B'}: reasons.append('unstacking 地图缺少有效 A/B 点位')
        if points.get('recording_ready') is not True: reasons.append('unstacking 地图定位尚未就绪或位姿过期')
        if not self._fresh(atlas): reasons.append('导航/定位反馈过期或不可用')
        if robot.get('robot_type') != 'lift_robot' or robot.get('connected') is not True:
            reasons.append('陆图尚未连接升降机器人载体')
        if robot.get('ready') is not True or robot.get('estop') is not False:
            reasons.append('陆图载体未就绪或急停状态未确认')
        if robot.get('holder_client_id'): reasons.append('机器人控制权正由其他客户端持有')
        if not base_ready(b['data']): reasons.append('机器人底盘反馈/导航接收能力未就绪')
        return reasons

    @staticmethod
    def _current(data):
        return data['commands'].get(data.get('active') or data.get('last_command_id'))

    def _finish(self, data, command, state, reason):
        command.update(state=state, reason=reason, updated_at=self.clock())
        if data.get('active') == command['command_id']: data['active'] = None
        previous_id = command.get('previous_command_id')
        seen = set()
        while previous_id and previous_id not in seen:
            seen.add(previous_id)
            previous = data['commands'].get(previous_id)
            if not previous: break
            previous.update(state='CANCELLED', reason='已通过停车确认取消', updated_at=self.clock())
            previous_id = previous.get('previous_command_id')

    def _reconcile(self, data, a, b):
        command = data['commands'].get(data.get('active'))
        if not command: return
        if command['owner'] != self.owner:
            command.update(state='UNKNOWN', reason='后台已重启或存在其他控制进程；请明确停车并确认后再导航')
            return
        if not a['connected'] or not b['connected']:
            command.update(state='UNKNOWN', reason='SDK 连接中断，运动结果未知')
            return
        atlas, lift = a['data'], b['data']
        nav = object_value(atlas.get('status', {}).get('nav_status'))
        if command['stage'] == 'stop':
            confirmation = atlas.get('status', {}).get('stop_confirmation') or {}
            if (confirmation.get('stop_id') == command.get('remote_stop_id') and command.get('remote_stop_id')
                    and confirmation.get('state') == 'stopped' and confirmation.get('stopped') is True
                    and atlas.get('map', {}).get('nav_drive_active') is False
                    and base_stationary(lift)):
                self._finish(data, command, command.get('completion', 'STOPPED'), 'SDK 停车回执与底盘静止反馈已确认')
            elif confirmation.get('stop_id') == command.get('remote_stop_id') and confirmation.get('state') == 'failed':
                command.update(state='UNKNOWN', reason='停车未确认：'+str(confirmation.get('reason', 'failed')))
            elif self.clock() - command['stop_requested_at'] > self.stop_timeout:
                command.update(state='UNKNOWN', reason='停车确认超时，不能视为已停车')
            return
        if self.clock() - command['started_at'] > self.mission_timeout:
            command.update(state='UNKNOWN', reason='导航超时，准备请求停车')
            return
        if atlas.get('points', {}).get('map_revision') != command['map_revision']:
            command.update(state='UNKNOWN', reason='地图版本发生变化，准备请求停车')
            return
        if not self._fresh(atlas):
            command.update(state='UNKNOWN', reason='导航/定位反馈过期，准备请求停车')
            return
        sequence = nav.get('goal_sequence')
        if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence <= command['baseline_sequence']:
            return  # Old reached flags cannot complete the newly submitted command.
        expected = command['target']
        match = all(finite(nav.get(key)) and abs(nav[key]-expected[field]) <= 1e-5
                    for key,field in (('goal_x','x'),('goal_y','y')))
        yaw = nav.get('goal_yaw_rad')
        match = match and finite(yaw) and abs(math.atan2(math.sin(yaw-expected['yaw']), math.cos(yaw-expected['yaw']))) <= 1e-5
        if not match:
            command.update(state='UNKNOWN', reason='检测到其他导航目标，等待明确停车')
            return
        if (nav.get('goal_reached') is True and nav.get('state') == 'goal_reached'
                and nav.get('mission_remaining') == 0 and base_stationary(lift)):
            command.update(state='ARRIVED', reason='目标已到达，等待解除导航并确认停车')

    def _issue_stop(self, data, command, completion):
        command.update(stage='stop', state='STOPPING', completion=completion,
                       stop_requested_at=self.clock(), reason='停车请求已提交，等待实测确认')
        self.journal.save(data)
        try:
            response = self.atlas.stop()
            command['remote_stop_id'] = (response.get('stop_confirmation') or {}).get('stop_id')
            if not command['remote_stop_id']:
                command.update(state='UNKNOWN', reason='SDK 未返回可关联的 stop_id，停车结果未知')
        except SDKError as exc:
            command.update(state='UNKNOWN', reason='停车调用失败：'+str(exc))

    def snapshot(self, advance=False):
        with self.journal.transaction() as data:
            a,b = self._read()
            self._reconcile(data, a, b)
            current = data['commands'].get(data.get('active'))
            if (advance and self.enabled and current and current['owner'] == self.owner
                    and current['stage'] == 'go' and current['state'] in ('ARRIVED','UNKNOWN')):
                self._issue_stop(data, current, 'SUCCEEDED' if current['state'] == 'ARRIVED' else 'FAULT')
            reasons = self._reasons(a,b)
            if self.worker_error: reasons.append('导航监测异常：'+self.worker_error)
            moving = a['data'].get('map', {}).get('nav_drive_active')
            if data.get('active'): reasons.append('已有导航/停车命令未结束，请等待或明确停车')
            elif moving is not False: reasons.append('陆图导航仍处于活动状态或状态未知，请先停车')
            current = self._current(data)
            nav = object_value(a['data'].get('status', {}).get('nav_status'))
            lift_data = b['data']
            return dict(enabled=self.enabled, map_name=self.atlas.map_name,
                map_revision=a['data'].get('points', {}).get('map_revision'),
                points=list(self._points(a['data']).values()), can_go=not reasons, reasons=reasons,
                command=deepcopy(current), nav_state=nav.get('state','unknown'),
                localization=a['data'].get('status', {}).get('localization_status','unknown'),
                devices={
                    'atlas':dict(connected=a['connected'], base_url=self.atlas.http.base_url,
                        reason=a['error'] or a['data'].get('status', {}).get('localization_status','已连接'),
                        navigation_fresh=self._fresh(a['data']),
                        robot_type=a['data'].get('robot', {}).get('robot_type')),
                    'lift':dict(connected=b['connected'], base_url=self.lift.http.base_url,
                        reason=b['error'] or (lift_data.get('base_status') or {}).get('reason','已连接'),
                        base_ready=base_ready(lift_data), stationary=base_stationary(lift_data),
                        arms_fresh=(lift_data.get('arms') or {}).get('fresh') is True,
                        lift_fresh=(lift_data.get('lift_status') or {}).get('fresh') is True),
                })

    def navigate(self, point, command_id):
        if point not in ('A','B'): raise NavigationError('只允许 A 或 B 点', 422)
        if not self.enabled: raise NavigationError('现场测试未开启导航控制', 423)
        with self.journal.transaction() as data:
            previous = data['commands'].get(command_id)
            if previous:
                if previous['action'] != 'go' or previous.get('point') != point:
                    raise NavigationError('command_id 已用于其他请求')
                return deepcopy(previous)
            a,b = self._read()
            self._reconcile(data,a,b)
            reasons = self._reasons(a,b)
            if self.worker_error: reasons.append('导航监测异常，请停车并检查后台日志：'+self.worker_error)
            if data.get('active'): reasons.append('已有命令未结束，请等待或停车')
            if a['data'].get('map', {}).get('nav_drive_active') is not False:
                reasons.append('陆图导航正在运行或状态未知，请先停车')
            if reasons: raise NavigationError('；'.join(reasons))
            nav = object_value(a['data']['status'].get('nav_status'))
            baseline = nav.get('goal_sequence')
            if not isinstance(baseline, int) or isinstance(baseline, bool):
                raise NavigationError('导航 SDK 缺少目标序号，无法关联执行结果')
            command = dict(command_id=command_id, owner=self.owner, action='go', stage='go',
                state='SUBMITTING', point=point, map_name=self.atlas.map_name,
                map_revision=a['data']['points']['map_revision'], target=self._points(a['data'])[point],
                baseline_sequence=baseline, started_at=self.clock(), reason='正在提交导航')
            data['commands'][command_id]=command
            data.update(active=command_id,last_command_id=command_id)
            self.journal.save(data)  # Persist before the only movement POST. Never retry it.
            try:
                response=self.atlas.navigate(point,command['map_revision'])
                if (response.get('ok') is not True or response.get('nav_drive_active') is not True
                        or response.get('goal_published') is not True or response.get('mission_size') != 1):
                    command.update(state='UNKNOWN', reason='SDK 返回的导航受理信息不完整，结果未知')
                else:
                    command.update(state='MOVING', reason='SDK 已受理，等待本次目标到达及停车确认')
            except SDKError as exc:
                command.update(state='UNKNOWN' if exc.uncertain else 'REJECTED', reason=str(exc))
                if not exc.uncertain: data['active']=None
            return deepcopy(command)

    def stop(self, command_id):
        if not self.enabled: raise NavigationError('现场测试未开启导航控制', 423)
        with self.journal.transaction() as data:
            previous=data['commands'].get(command_id)
            if previous:
                if previous['action'] != 'stop': raise NavigationError('command_id 已用于其他请求')
                return deepcopy(previous)
            command=dict(command_id=command_id,owner=self.owner,action='stop',stage='stop',
                         state='STOPPING',previous_command_id=data.get('active'),started_at=self.clock())
            data['commands'][command_id]=command
            data.update(active=command_id,last_command_id=command_id)
            self._issue_stop(data,command,'STOPPED')
            return deepcopy(command)
