"""Create an explicitly synthetic IK/motion fixture for testing the Isaac adapter.

Run with the project venv. This does not exercise SDK or perception and cannot
be passed as a successful real capture to the production replay command.
"""
from dataclasses import asdict
import datetime
import json
from pathlib import Path
import sys
import uuid
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/wheel_grasp_core'))
from wheel_grasp.kinematics import RobotModel, PairIK
from wheel_grasp.planning import plan_pair


def main():
    cfg = yaml.safe_load((ROOT/'config/grasp/runtime.container.yaml').read_text())
    tcp = {s: np.eye(4) for s in ('left', 'right')}
    for side in tcp:
        tcp[side][:3,3] = cfg['tcp'][side]
    ik = PairIK(RobotModel(ROOT/cfg['urdf']), tcp)
    home = json.loads((ROOT/'config/robot_config.json').read_text())['home']
    joints = {n: home.get(n,0.) for n in ik.names}
    goal = dict(joints, lift_joint=.005, openarm_left_joint1=.03, openarm_right_joint1=-.03)
    targets = ik.fk(goal)
    plan = plan_pair(ik,targets,joints,steps=4,
                     speed_limits={n:.03 if n=='lift_joint' else .2 for n in ik.names})
    pose = np.eye(4);pose[:3,3] = [3,0,.5]
    run_id = 'synthetic-motion-' + datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S') + '-' + uuid.uuid4().hex[:6]
    directory = ROOT/'outputs/grasp/runs'/run_id
    directory.mkdir(parents=True)
    files = {'runtime.json':cfg, 'joints.json':joints,
             'manifest.json':dict(mode='synthetic_adapter_test', real_sdk=False, real_perception=False),
             'perception.json':dict(pair_id='synthetic-fixture',pose=pose.tolist(),pregrasp={s:t.tolist() for s,t in targets.items()}),
             'trajectory.json':dict(run_id=run_id,pair_id='synthetic-fixture',mode='simulation_only',base_fixed=True,
                                    phases=[dict(phase='pregrasp',**asdict(plan))])}
    for filename, value in files.items():
        (directory/filename).write_text(json.dumps(value,indent=2))
    print(directory.relative_to(ROOT))


if __name__ == '__main__':
    main()
