"""Lift geometry and meter-based controller regression checks without Isaac."""
import json
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from common import ROOT
from robot_control import RobotControl, OmniKinematics


class Value:
    def Set(self, value):
        self.value = value


class Drive:
    def __init__(self):
        self.position = Value()

    def GetTargetPositionAttr(self):
        return self.position


class LiftTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads((ROOT / 'config/profiles/robot_lift.json').read_text())
        self.urdf = ET.parse(ROOT / 'assets/robots' / self.cfg['source_urdf']).getroot()

    def test_lowest_geometry_and_limits(self):
        def z(name):
            return float(self.urdf.find(f"joint[@name='{name}']/origin").get('xyz').split()[2])
        for side in ('left', 'right'):
            height = z('base_link_to_lift_column_joint') + z('lift_joint') + z(f'openarm_{side}_base_link_mount_joint') + z(f'openarm_{side}_joint1') - z('lift_bottom_reference_joint')
            self.assertAlmostEqual(height, 0.106)
            self.assertAlmostEqual(height + 0.4, 0.506)
        lim = self.urdf.find("joint[@name='lift_joint']/limit")
        self.assertEqual([float(lim.get(k)) for k in ('lower', 'upper', 'velocity')], [0, 0.4, 0.05])
        self.assertEqual(self.cfg['home']['lift_joint'], 0)

    def test_meshes_resolve(self):
        folder = (ROOT / 'assets/robots' / self.cfg['source_urdf']).parent
        for mesh in self.urdf.findall('.//mesh'):
            self.assertTrue((folder / mesh.get('filename')).is_file())

    def test_lift_clamps_and_uses_meters(self):
        ctl = RobotControl.__new__(RobotControl)
        ctl.cfg = self.cfg
        ctl.meta = {'wheels': []}
        ctl.ik = OmniKinematics([])
        ctl.specs = {'lift_joint': {'kind': 'lift', 'lower': 0, 'upper': 0.4, 'velocity': 0.05}}
        ctl.drives = {'lift_joint': Drive()}
        ctl.targets = {'lift_joint': 0}
        ctl.sent = ctl.targets.copy()
        ctl.written_targets = {}
        ctl.requested = [0, 0, 0]
        ctl.actual = [0, 0, 0]
        ctl.remaining = None
        ctl.set_joint('lift_joint', 1)
        self.assertEqual(ctl.targets['lift_joint'], 0.4)
        ctl.update(0.1)
        self.assertAlmostEqual(ctl.drives['lift_joint'].position.value, 0.005)
        ctl.set_joint('lift_joint', -1)
        self.assertEqual(ctl.targets['lift_joint'], 0)
        ctl.update(0.1)
        self.assertEqual(ctl.drives['lift_joint'].position.value, 0)
        ctl.set_joint('lift_joint', 0.3)
        ctl.home()
        self.assertEqual(ctl.targets['lift_joint'], 0)
        with self.assertRaises(ValueError):
            ctl.set_joint('lift_joint', float('nan'))
