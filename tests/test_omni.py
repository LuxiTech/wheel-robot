import json
import math
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from common import ROOT
from robot_control import OmniKinematics


class OmniTests(unittest.TestCase):
    def setUp(self):
        cfg = json.loads((ROOT / "config/robot_config.json").read_text())
        root = ET.parse(ROOT / "assets/robots" / cfg["source_urdf"]).getroot()
        wheels = []
        for j in root.findall("joint"):
            if j.get("type") != "continuous":
                continue
            link = root.find("link[@name='%s']" % j.find("child").get("link"))
            wheels.append(
                {
                    "axis": list(map(float, j.find("axis").get("xyz").split())),
                    "position": list(map(float, j.find("origin").get("xyz").split())),
                    "radius": float(
                        link.find("collision/geometry/cylinder").get("radius")
                    ),
                    "max_velocity": float(j.find("limit").get("velocity")),
                }
            )
        self.ik = OmniKinematics(wheels)

    def test_round_trip(self):
        for v in ([0.3, 0, 0], [0, 0.3, 0], [0, 0, 0.8], [0.4, -0.3, 0.5]):
            for expected, actual in zip(v, self.ik.forward(self.ik.inverse(v))):
                self.assertAlmostEqual(expected, actual, places=8)

    def test_front_wheel_does_not_drive_forward(self):
        self.assertLess(abs(self.ik.inverse([0.2, 0, 0])[0]), 1e-9)

    def test_wheel_signs(self):
        self.assertLess(self.ik.inverse([0, 0.2, 0])[0], 0)
        self.assertTrue(all(v < 0 for v in self.ik.inverse([0, 0, 0.5])))

    def test_speed_saturation(self):
        q = self.ik.inverse([10, 10, 20])
        v = self.ik.forward(q)
        self.assertLessEqual(math.hypot(*v[:2]), 1 + 1e-8)
        self.assertLessEqual(abs(v[2]), 1.5 + 1e-8)
        self.assertLessEqual(max(map(abs, q)), 20 + 1e-8)

    def test_reject_nan(self):
        with self.assertRaises(ValueError):
            self.ik.inverse([math.nan, 0, 0])
