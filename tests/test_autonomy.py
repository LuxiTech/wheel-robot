"""Numerical regression tests; run with Isaac's bundled NumPy/SciPy."""
import json
import math
import sys
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common import ROOT
from arm_ik import ArmIK, squeeze_orientation
from lidar_slam import LidarSLAM, rotation2


class IKTests(unittest.TestCase):
    def setUp(self):
        self.cfg=json.loads((ROOT/'config/robot_config.json').read_text())

    def test_two_arm_grasp_and_lift(self):
        for side,sign in [('left',1),('right',-1)]:
            ik=ArmIK(ROOT/'assets/robots'/self.cfg['source_urdf'],side)
            q=[self.cfg['home'].get(n,0) for n in ik.names]
            for goal in ([.44,sign*.15,.86],[.42,sign*.15,.95]):
                result=ik.solve(goal,squeeze_orientation(side),q,attempts=5)
                self.assertTrue(result.success,result)
                q=np.array(list(result.joints.values()))
                self.assertTrue(np.all(q>=ik.lower) and np.all(q<=ik.upper))
                self.assertLess(np.linalg.norm(ik.fk(q,.4)[:3,3]-goal),.008)

    def test_rejects_unreachable_and_nan(self):
        ik=ArmIK(ROOT/'assets/robots'/self.cfg['source_urdf'],'left')
        seed=[self.cfg['home'].get(n,0) for n in ik.names]
        self.assertFalse(ik.solve([5,0,2],np.eye(3),seed,attempts=1).success)
        with self.assertRaises(ValueError):
            ik.solve([np.nan,0,0],np.eye(3),seed)

    def test_prismatic_travel_translates_tcp(self):
        ik=ArmIK(ROOT/'assets/robots/r680_lift_openarm/r680_lift_openarm_sim.urdf','right')
        q=[self.cfg['home'].get(n,0) for n in ik.names]
        np.testing.assert_allclose(ik.fk(q,.4)[:3,3]-ik.fk(q,0)[:3,3],[0,0,.4],atol=1e-9)


class SLAMTests(unittest.TestCase):
    @staticmethod
    def scan(slam,pose):
        # Synthetic asymmetric rectangular room. SLAM never receives these walls.
        dirs=slam.directions @ rotation2(pose[2]).T
        ranges=np.full(len(dirs),slam.max_range)
        for axis,plane in [(0,-3),(0,4),(1,-2),(1,3)]:
            denom=dirs[:,axis]
            t=(plane-pose[axis])/np.where(abs(denom)<1e-9,1e-9,denom)
            ranges=np.minimum(ranges,np.where(t>0,t,slam.max_range))
        return ranges

    def test_scan_match_corrects_encoder_drift(self):
        slam=LidarSLAM([0,0,0],resolution=.04,beams=240,max_range=8,bounds=(-5,-4,6,5))
        for _ in range(4): self.assertTrue(slam.update(self.scan(slam,[0,0,0])))
        truth=np.zeros(3)
        for _ in range(20):
            truth[0]+=.04
            slam.predict([.044,0,.006],1.)
            self.assertTrue(slam.update(self.scan(slam,truth)))
        estimated=np.linalg.norm(slam.pose[:2]-truth[:2])
        odom=np.linalg.norm(slam.odom_pose[:2]-truth[:2])
        self.assertLess(estimated,.06)
        self.assertLess(estimated,odom)
        self.assertLess(abs(slam.pose[2]),.02)

    def test_planner_routes_around_inflated_obstacle(self):
        slam=LidarSLAM([-1,0,0],resolution=.05,bounds=(-2,-2,2,2))
        a,b=slam.cells([[-.1,-.5],[.1,.5]])
        slam.log_odds[a[1]:b[1]+1,a[0]:b[0]+1]=5
        path=slam.plan([1,0],radius=.25)
        self.assertTrue(path)
        blocked=slam.planning_grid(.25)
        cells=slam.cells(np.array(path))
        self.assertFalse(blocked[cells[:,1],cells[:,0]].any())
        self.assertGreater(max(abs(p[1]) for p in path),.7)
        self.assertFalse(slam.plan([0,0],radius=.25))

    def test_invalid_scan_does_not_change_map(self):
        slam=LidarSLAM([0,0,0])
        with self.assertRaises(ValueError): slam.update(np.full(240,np.nan))
        self.assertEqual(np.count_nonzero(slam.log_odds),0)
        with self.assertRaises(ValueError): slam.predict([0,0,0],float('nan'))
        with self.assertRaises(ValueError): slam.update(np.full(240,-1.))
