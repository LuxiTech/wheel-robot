"""File transport between a loopback web server and Isaac's simulation thread."""
import json
import os
import time
import uuid
from pathlib import Path

ACTIONS={'tour','goal','stop','reset'}


def atomic_json(path, data):
    path=Path(path)
    tmp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    tmp.write_text(json.dumps(data,ensure_ascii=False))
    os.replace(tmp,path)


class NavigationConsole:
    def __init__(self,runtime,folder,reset_environment):
        self.rt=runtime;self.folder=Path(folder);self.reset_environment=reset_environment
        self.folder.mkdir(parents=True,exist_ok=True)
        self.commands=self.folder/'commands';self.commands.mkdir(exist_ok=True)
        self.session=uuid.uuid4().hex;self.last_command=None
        self.publish()

    def poll(self):
        for path in sorted(self.commands.glob('*.json'))[:16]:
            try:
                cmd=json.loads(path.read_text())
                if cmd.get('session')!=self.session or abs(time.time()-cmd['created_at'])>5:
                    raise ValueError('指令已过期，请重新操作')
                action=cmd['action']
                if action not in ACTIONS:raise ValueError('未知指令')
                if action in ('tour','goal') and self.rt.active:raise ValueError('正在执行，请先停止')
                if action=='goal':
                    if cmd.get('goal') not in [p['id'] for p in self.rt.nav_config['points']]:raise ValueError('未知取物点')
                    self.rt.select_goal(cmd['goal'])
                elif action=='tour':self.rt.select_goal(None)
                elif action=='stop':
                    if self.rt.active:self.rt.request_stop()
                elif action=='reset':self.reset_environment()
                self.last_command=dict(id=cmd['id'],ok=True,message='指令已执行')
            except Exception as exc:
                self.last_command=dict(id=path.stem,ok=False,message=str(exc))
            finally:path.unlink(missing_ok=True)
        self.publish()

    def publish(self):
        rt=self.rt
        try:pose=rt.navigation_pose().tolist();pose_error=None
        except Exception as exc:pose=None;pose_error=str(exc)
        atomic_json(self.folder/'status.json',dict(session=self.session,updated_at=time.time(),active=rt.active,
            finished=rt.finished,error=rt.error,states=rt.states,simulation_seconds=rt.time,
            localization=rt.nav_localization,pose=pose,pose_error=pose_error,
            config=rt.nav_config,obstacles=rt.static_bounds,
            other_robot=rt.base('B')[[0,1,3]].tolist(),
            selected_goals=rt.selected_goals,reached=rt.nav_reached,
            plan=rt.nav_paths[-1] if rt.nav_paths else None,
            events=rt.events[-12:],last_command=self.last_command))
