import json
import sys
import tempfile
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from robot_console_ipc import RobotConsole, validate_command


class RobotConsoleTests(unittest.TestCase):
    def test_manual_parameters_and_mode_interlock(self):
        command=dict(action='gripper',robot='Robot_A',side='left',closed=True)
        self.assertEqual(validate_command(command,dict(mode='manual')), 'gripper')
        for bad in [dict(command,robot='other'),dict(command,closed='true'),dict(command,side='other')]:
            with self.assertRaises(ValueError):
                validate_command(bad,dict(mode='manual'))
        with self.assertRaises(ValueError):
            validate_command(command,dict(mode='relay'))
        with self.assertRaises(ValueError):
            validate_command(dict(action='lift',robot='Robot_A',height=3.5),dict(mode='manual'))

    def test_running_task_and_mode_changes(self):
        for action in ('start','mode'):
            with self.assertRaises(ValueError):
                validate_command(dict(action=action,mode='manual',count=1),dict(mode='relay',active=True))
        self.assertEqual(validate_command(dict(action='stop'),dict(mode='relay',active=True)), 'stop')
        with self.assertRaises(ValueError):
            validate_command(dict(action='start'),dict(mode='relay',finished=True))

    def test_queue_executes_once_rejects_old_sessions_and_rechecks_mode(self):
        with tempfile.TemporaryDirectory() as folder:
            calls=[]
            state=dict(mode='manual')
            console=RobotConsole(folder,'new',lambda:state,lambda cmd:calls.append(cmd))
            for name,session,action in [('1','old','reset'),('2','new','reset'),('3','new','gripper')]:
                (console.queue/(name+'.json')).write_text(json.dumps(dict(session=session,
                    action=action,created_at=time.time(),robot='Robot_A',side='left',closed=True)))
            state['mode']='relay'
            console.poll()
            self.assertEqual([c['action'] for c in calls],['reset'])
            self.assertEqual([r['ok'] for r in console.results],[False,True,False])
            console.last_poll=0
            console.poll()
            self.assertEqual(len(calls),1)


if __name__=='__main__':
    unittest.main()
