"""Fixed-body model topology, camera assets and EE coordinate regressions."""
import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from common import ROOT
from arm_ik import ArmIK


class FixedRobotTests(unittest.TestCase):
    def setUp(self):
        self.path = ROOT / 'assets/robots/r680_openarm/r680_fixed_openarm_sim.urdf'
        self.root = ET.parse(self.path).getroot()

    def test_topology_and_assets(self):
        joints = self.root.findall('joint')
        self.assertFalse(any(j.get('type') == 'prismatic' for j in joints))
        self.assertEqual(sum(j.get('type') != 'fixed' and j.find('mimic') is None for j in joints), 19)
        names = {link.get('name') for link in self.root.findall('link')}
        for joint in joints:
            self.assertIn(joint.find('parent').get('link'), names)
            self.assertIn(joint.find('child').get('link'), names)
        for camera in ('head_d455', 'left_wrist_eb435i', 'right_wrist_eb435i'):
            self.assertIn(camera + '_color_optical_frame', names)
        for mesh in self.root.findall('.//mesh'):
            self.assertTrue((self.path.parent / mesh.get('filename')).is_file())
        mass = sum(float(m.get('value')) for m in self.root.findall('link/inertial/mass'))
        self.assertAlmostEqual(mass, 59.5620073, places=5)

    def test_fixed_shoulder_and_tcp_frame(self):
        cfg = json.loads((ROOT / 'config/robot_config.json').read_text())
        for side in ('left', 'right'):
            ik = ArmIK(self.path, side)
            q = [cfg['home'].get(n, 0) for n in ik.names]
            np.testing.assert_allclose(ik.fk(q, 0), ik.fk(q, .4))
            self.assertAlmostEqual(ik.tcp[0], -.00143)
            z = sum(t[2, 3] for n, kind, t, a in ik.chain[:4])
            self.assertAlmostEqual(z, 1.00312)

    def test_pregrasp_path_stays_on_continuous_branch(self):
        cfg = json.loads((ROOT / 'config/transport_config.json').read_text())['manipulation']
        for side, sign in (('left', 1), ('right', -1)):
            ik = ArmIK(self.path, side)
            from arm_ik import squeeze_orientation
            seed = np.asarray(cfg['unfold_seed'][side])
            start = np.array(cfg['unfold_center']) + [0, sign*cfg['unfold_half_width'], 0]
            seed = np.asarray(list(ik.solve(start, squeeze_orientation(side), seed).joints.values()))
            end = np.array([.42, sign*(.15+cfg['pregrasp_clearance']), .86])
            for u in np.linspace(0, 1, 30):
                result = ik.solve((1-u)*start+u*end, squeeze_orientation(side), seed, attempts=1)
                self.assertTrue(result.success, result)
                q = np.asarray(list(result.joints.values()))
                self.assertLess(np.max(abs(q-seed)), .35)
                seed = q

    def test_head_calibration_comes_from_urdf(self):
        from arm_ik import fixed_frame_transform
        transform = fixed_frame_transform(self.path, 'head_d455_depth_frame')
        np.testing.assert_allclose(transform[:3, 3], [.183562318, .047725407, .946356443])
        with self.assertRaises(ValueError):
            fixed_frame_transform(self.path, 'left_wrist_eb435i_depth_frame')
