"""Validated local command queue; callbacks execute only on the simulation thread."""
import json
import time
from pathlib import Path


def validate_command(cmd, operator):
    action = cmd.get('action')
    if action not in {'gripper', 'lift', 'load', 'reset', 'start', 'stop', 'mode', 'goal', 'tour'}:
        raise ValueError('未知操作')
    mode = operator.get('mode')
    if action in {'gripper', 'lift', 'load'} and mode != 'manual':
        raise ValueError('请先切换到手动模式')
    if action in {'start', 'goal', 'tour'}:
        if mode == 'manual':
            raise ValueError('请先切换到自动模式')
        if operator.get('active'):
            raise ValueError('任务正在执行')
        if action == 'start' and operator.get('finished'):
            raise ValueError('任务已结束，请先复位')
    if action in {'goal', 'tour'} and mode != 'navigation':
        raise ValueError('当前不是导航模式')
    if action == 'goal' and cmd.get('goal') not in operator.get('goals', []):
        raise ValueError('未知导航工位')
    if action in {'gripper', 'lift'} and cmd.get('robot') not in {'Robot_A', 'Robot_B'}:
        raise ValueError('未知机器人')
    if action == 'gripper' and (cmd.get('side') not in {'left', 'right'} or type(cmd.get('closed')) is not bool):
        raise ValueError('夹爪参数无效')
    if action == 'lift' and cmd.get('height') not in (.15, .35):
        raise ValueError('仅支持 0.15 m 或 0.35 m 升降目标')
    if action == 'load' and (type(cmd.get('slot')) is not int or cmd['slot'] not in (0, 1)):
        raise ValueError('未知交接区')
    if action == 'mode':
        if mode == 'navigation' or operator.get('active'):
            raise ValueError('当前不能切换模式，请先停止任务')
        if cmd.get('mode') not in {'manual', 'relay'}:
            raise ValueError('未知模式')
        if type(cmd.get('count')) is not int or not 1 <= cmd['count'] <= operator.get('max_boxes', 18):
            raise ValueError('箱数无效')
    return action


class RobotConsole:
    def __init__(self, folder, session, snapshot, execute):
        self.queue = Path(folder) / 'commands'
        self.queue.mkdir(exist_ok=True)
        self.session = session
        self.snapshot = snapshot
        self.execute = execute
        self.results = []
        self.last_poll = 0.

    def poll(self):
        now = time.monotonic()
        if now - self.last_poll < .1:
            return
        self.last_poll = now
        for path in sorted(self.queue.glob('*.json'))[:16]:
            try:
                cmd = json.loads(path.read_text())
                if cmd.get('session') != self.session or abs(time.time()-cmd['created_at']) > 10:
                    raise ValueError('指令已过期，请重新操作')
                validate_command(cmd, self.snapshot())
                message = self.execute(cmd) or '指令已执行'
                result = dict(id=path.stem, ok=True, message=message)
            except Exception as exc:
                result = dict(id=path.stem, ok=False, message=str(exc))
            finally:
                path.unlink(missing_ok=True)
            self.results.append(result)
            self.results = self.results[-16:]

    def status(self):
        return dict(self.snapshot(), results=self.results.copy())
