"""Reachability and collision margins for the customer demo layout."""
import json
import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from common import ROOT, load_config, table_by_id
from arm_ik import ArmIK, squeeze_orientation
from floor_demo import load_demo


class FloorDemoTests(unittest.TestCase):
    def test_observed_second_pickup_offset_does_not_jump(self):
        demo = load_demo()
        for side, sign in (('left', 1), ('right', -1)):
            ik = ArmIK(ROOT / 'assets/robots/r680_lift_openarm/r680_lift_openarm_sim.urdf', side)
            start = np.array([.4319806, sign*.185+.0081423, .30])
            result = ik.solve(start, squeeze_orientation(side), demo['ground_seed'][side], lift=0)
            self.assertTrue(result.success)
            seed = np.array(list(result.joints.values()))
            end = start.copy()
            end[2] = .14  # Recorded second-pickup pose that jumped with 10 Hz waypoints.
            for u in np.linspace(0, 1, 54)[1:]:
                result = ik.solve((1-u)*start+u*end, squeeze_orientation(side), seed, lift=0, attempts=2)
                self.assertTrue(result.success)
                q = np.array(list(result.joints.values()))
                self.assertLess(np.max(abs(q-seed)), .35)
                seed = q

    def test_ground_to_workbench_continuous_ik(self):
        demo = load_demo()
        cfg = json.loads((ROOT / 'config/profiles/robot_lift.json').read_text())
        for side, sign in (('left', 1), ('right', -1)):
            ik = ArmIK(ROOT / 'assets/robots' / cfg['source_urdf'], side)
            seed = np.array(demo['ground_seed'][side])
            stages = [(.42, .185, demo['grasp_height'], 0),
                      (.42, .15-demo['squeeze_depth'], demo['grasp_height'], 0),
                      (.42, .15-demo['squeeze_depth'], .46, 0),
                      (.42, .15-demo['squeeze_depth'], demo['carry_center'][2], .4),
                      (demo['place_forward'], .15-demo['squeeze_depth'], .75+demo['grasp_height']-.005, .4),
                      (.48, .185, .75+demo['grasp_height']-.005, .4),
                      (.42, .22, .94, .4)]
            for forward, width, height, lift in stages:
                start = ik.fk(seed, lift)[:3, 3]
                end = np.array([forward, sign*width, height])
                for u in np.linspace(0, 1, 30)[1:]:
                    ans = ik.solve((1-u)*start+u*end, squeeze_orientation(side), seed, lift=lift, attempts=2)
                    self.assertTrue(ans.success, (side, height, ans))
                    q = np.array(list(ans.joints.values()))
                    self.assertLess(np.max(abs(q-seed)), .35)
                    seed = q

    def test_three_slots_fit_and_floor_pickup_lanes_clear(self):
        demo = load_demo()
        scene = load_config()
        table = table_by_id(scene, scene['task']['target'])
        width = scene['box']['size'][0]
        self.assertEqual(len(demo['floor_positions']), 3)
        self.assertEqual(len(demo['slot_offsets']), 3)
        for offset in demo['slot_offsets']:
            self.assertLess(abs(offset)+width/2+.02, table['size'][0]/2)
            self.assertLessEqual(abs(offset)+demo['slot_width']/2, table['size'][0]/2)
        for a, b in zip(demo['floor_positions'], demo['floor_positions'][1:]):
            self.assertGreater(b[0]-a[0], .43+width/2+.1)
        self.assertGreater(min(np.diff(demo['slot_offsets'])), width+.04)
        self.assertGreater(min(np.diff(demo['slot_offsets'])), demo['slot_width'])


if __name__ == '__main__':
    unittest.main()
