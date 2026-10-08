import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from common import load_config
from setup_task import PlacementTask


class PlacementTests(unittest.TestCase):
    def setUp(self):
        self.task = PlacementTask(load_config())
        self.state = dict(
            position=[5, -3.5, 0.86],
            half_extents=[0.15, 0.12, 0.11],
            linear_velocity=[0, 0, 0],
            angular_velocity=[0, 0, 0],
            released=True,
            supported=True,
            dt=0.1,
        )

    def settle(self):
        for _ in range(12):
            self.task.update(**self.state)
        return self.task.success

    def test_stable_placement(self):
        self.assertTrue(self.settle())

    def test_rejects_no_contact(self):
        self.state["supported"] = False
        self.assertFalse(self.settle())

    def test_rejects_grasped_or_unknown(self):
        for value in [False, None]:
            self.state["released"] = value
            self.assertFalse(self.settle())

    def test_rejects_overhang(self):
        self.state["position"][0] = 5.7
        self.assertFalse(self.settle())

    def test_rejects_rotation_motion(self):
        self.state["angular_velocity"] = [0, 0, 1]
        self.assertFalse(self.settle())

    def test_requires_continuous_hold(self):
        for _ in range(6):
            self.task.update(**self.state)
        self.state["supported"] = False
        self.task.update(**self.state)
        self.state["supported"] = True
        for _ in range(6):
            self.task.update(**self.state)
        self.assertFalse(self.task.success)

    def test_reset(self):
        self.settle()
        self.task.reset()
        self.assertFalse(self.task.success)

    def test_rejects_nan(self):
        self.state["position"][2] = float("nan")
        self.assertFalse(self.settle())


if __name__ == "__main__":
    unittest.main()
