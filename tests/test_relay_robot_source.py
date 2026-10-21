"""Verify the relay asset preserves the supplied robot and fixed camera calibration."""
import json
import itertools
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/wheel_grasp_core'))
try:
    from wheel_grasp.kinematics import RobotModel
except ModuleNotFoundError as exc:
    if exc.name not in ('wheel_grasp', 'wheel_grasp.kinematics'):
        raise
    raise unittest.SkipTest('Mount ros2_ws to validate relay camera kinematics') from exc


class RelayRobotSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = ROOT / 'assets/robots' / json.loads(
            (ROOT / 'config/profiles/robot_relay.json').read_text())['source_urdf']
        cls.generated = ROOT / 'assets/robots/relay/relay_robot.urdf'
        if not cls.source.exists() or not cls.generated.exists():
            raise unittest.SkipTest('Prepare external relay robot assets first')

    def test_mesh_references_exist(self):
        for path in (self.source, self.generated):
            for mesh in ET.parse(path).iter('mesh'):
                self.assertTrue((path.parent / mesh.get('filename')).is_file())

    def test_original_grippers_preserved(self):
        source, generated = ET.parse(self.source), ET.parse(self.generated)
        for side in ('left', 'right'):
            for tag, suffix in (('link', 'ee_base_link'), ('link', 'ee_link1'),
                                ('link', 'ee_link2'), ('joint', 'finger_joint1'),
                                ('joint', 'finger_joint2')):
                query = f"{tag}[@name='openarm_{side}_{suffix}']"
                original, actual = source.find(query), generated.find(query)
                # Mesh paths are rebased, but all physical and kinematic data stay intact.
                for el in original.iter('mesh'):
                    el.set('filename', str((self.source.parent / el.get('filename')).resolve()))
                for el in actual.iter('mesh'):
                    el.set('filename', str((self.generated.parent / el.get('filename')).resolve()))
                self.assertEqual(ET.tostring(original), ET.tostring(actual))

    def test_camera_and_arm_calibration_preserved(self):
        source, generated = RobotModel(self.source), RobotModel(self.generated)
        values = {name: 0.0 for name in source.joints}
        for height in (0.0, 0.2, 0.4):
            values['lift_joint'] = height
            for frame in ('head_d455_color_optical_frame',
                          'openarm_left_ee_base_link', 'openarm_right_ee_base_link'):
                np.testing.assert_allclose(generated.transform(frame, values),
                                           source.transform(frame, values), atol=1e-10)
        frame = 'head_d455_color_optical_frame'
        low = generated.transform(frame, dict(values, lift_joint=0.0))
        high = generated.transform(frame, dict(values, lift_joint=0.4))
        np.testing.assert_allclose(high[:3, 3], low[:3, 3], atol=1e-10)
        np.testing.assert_allclose(high[:3, :3], low[:3, :3], atol=1e-10)

    def test_b_receiving_box_fits_camera_at_wait_and_dock(self):
        from scipy.spatial.transform import Rotation
        scene=json.loads((ROOT/'config/relay_config.json').read_text())
        settings=json.loads((ROOT/'config/relay_demo_config.json').read_text())
        calibration=json.loads((ROOT/'config/grasp/runtime.sim.json').read_text())['simulation']['camera']
        model=RobotModel(self.generated)
        robot=scene['robots'][1]
        slot=scene['handoff']['slots'][1]
        sx,sy,sz=scene['crate']['size']
        points=np.array(list(itertools.product([-sx/2,sx/2],[-sy/2,sy/2],[0,sz])))
        points+=np.array([*slot,scene['handoff']['size'][2]])
        for distance in (settings['b_handoff_observation_distance'],settings['b_handoff_dock_distance']):
            base=np.eye(4)
            base[:3,:3]=Rotation.from_euler('z',robot['yaw']).as_matrix()
            base[:3,3]=[slot[0]-.08,slot[1]-distance,robot['position'][2]]
            camera=base@model.transform('head_d455_color_optical_frame',{'lift_joint':.4})
            inverse=np.linalg.inv(camera)
            xyz=points@inverse[:3,:3].T+inverse[:3,3]
            self.assertTrue(np.all(xyz[:,2]>.03))
            uv=xyz[:,:2]/xyz[:,2,None]*[calibration['fx'],calibration['fy']]+[calibration['cx'],calibration['cy']]
            self.assertTrue(np.all(uv>5),uv)
            self.assertTrue(np.all(uv<[calibration['width']-5,calibration['height']-5]),uv)


if __name__ == '__main__':
    unittest.main()
