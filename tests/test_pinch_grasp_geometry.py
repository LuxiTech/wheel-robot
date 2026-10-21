"""The supplied pinch fingers must close across, rather than along, the rim."""
import json
import math
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from relay_runtime import RelayRuntime


class PinchGraspGeometryTests(unittest.TestCase):
    def test_source_approach_descends_without_crossing_the_front_rim(self):
        rt = RelayRuntime.__new__(RelayRuntime)
        rt.settings = {'source_vertical_approach': .09}
        rt.hand_targets = lambda *a, **k: {'left': np.eye(4)}
        for progress in (0., .5, 1.):
            target = rt.rim_approach_targets('A', np.eye(4), progress)['left']
            np.testing.assert_allclose(target[:2, 3], 0.)
            self.assertAlmostEqual(target[2, 3], .09 * (1 - progress))

    def test_both_wrist_camera_mounts_stay_above_the_pinch_point(self):
        urdf = ROOT / 'assets/robots/relay/relay_robot.urdf'
        if not urdf.exists():
            self.skipTest('External robot assets not installed')
        root = ET.parse(urdf)
        rt = RelayRuntime.__new__(RelayRuntime)
        rt.c = json.loads((ROOT / 'config/relay_config.json').read_text())
        rt.settings = json.loads((ROOT / 'config/relay_demo_config.json').read_text())
        rt.settings.update(gripper_frame_yaw_degrees={'left': 90., 'right': -90.},
                           gripper_pitch_degrees=80.)
        rt.grasp_transforms = {'A': None}
        rt.base = lambda _: np.array([0., 0., 0., math.pi / 2])
        tcp = np.array([-.02193, 0., -.16])
        for side, target in rt.hand_targets('A', np.eye(4)).items():
            mount = root.find(f"joint[@name='{side}_wrist_eb435i_mount_joint']/origin")
            offset = np.fromstring(mount.get('xyz'), sep=' ') - tcp
            self.assertGreater(float((target[:3, :3] @ offset)[2]), .08)

    def test_finger_closing_direction_matches_rim_normal(self):
        urdf = ROOT / 'assets/robots/relay/relay_robot.urdf'
        if not urdf.exists():
            self.skipTest('External robot assets not installed')
        root = ET.parse(urdf)
        rt = RelayRuntime.__new__(RelayRuntime)
        rt.c = json.loads((ROOT / 'config/relay_config.json').read_text())
        rt.settings = json.loads((ROOT / 'config/relay_demo_config.json').read_text())
        rt.settings['gripper_frame_yaw_degrees'] = -90.
        rt.grasp_transforms = {'A': None, 'B': None}
        for box_yaw in (0., math.pi / 2):
            for heading in (math.pi / 2, -math.pi / 2, math.pi):
                for who in ('A', 'B'):
                    with self.subTest(box_yaw=box_yaw, heading=heading, who=who):
                        rt.base = lambda _: np.array([0., 0., 0., heading])
                        box = np.eye(4)
                        box[:3, :3] = Rotation.from_euler('z', box_yaw).as_matrix()
                        for side, target in rt.hand_targets(who, box).items():
                            joint = root.find(f"joint[@name='openarm_{side}_finger_joint1']")
                            axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                            closing = target[:3, :3] @ np.cross(axis, [0., 0., -1.])
                            self.assertAlmostEqual(abs(float(closing @ box[:3, 0])), 1., places=7)


if __name__ == '__main__':
    unittest.main()
