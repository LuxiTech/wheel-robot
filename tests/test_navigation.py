import json
import math
import sys
import unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from grid_navigation import GridPlanner,NoPath,collider_bounds,path_length


class NavigationTests(unittest.TestCase):
    def test_clear_space_uses_direct_line(self):
        p=GridPlanner([-3,-3,3,3],.05,[],.3)
        route=p.plan([-1,-1],[1,1])
        self.assertEqual(len(route),2)
        self.assertAlmostEqual(path_length(route),math.sqrt(8))

    def test_detour_respects_inflated_footprint(self):
        p=GridPlanner([-3,-3,3,3],.05,[(-.3,-1,.3,1)],.3)
        route=p.plan([-2,0],[2,0])
        self.assertGreater(path_length(route),4.)
        self.assertTrue(all(p.visible(a,b) for a,b in zip(route,route[1:])))
        self.assertTrue(any(abs(y)>1.3 for x,y in route))

    def test_blocked_goal_and_disconnected_map_stop(self):
        p=GridPlanner([-2,-2,2,2],.1,[(-.1,-2,.1,2)],.2)
        for goal in ([0,0],[1,0],[4,0]):
            with self.assertRaises(NoPath):p.plan([-1,0],goal)

    def test_shortcut_cannot_cut_obstacle_corner(self):
        p=GridPlanner([-3,-3,3,3],.05,[(0,0,1,1)],.3)
        self.assertFalse(p.visible([-.5,.5],[.5,-.5]))

    def test_all_pickup_stations_reachable(self):
        cfg=json.loads((ROOT/'config/navigation_demo_config.json').read_text())
        obstacles=collider_bounds(json.loads((ROOT/'assets/generated/relay/static_colliders.json').read_text()))
        obstacles.append((.67,-1.08,1.53,-.22))
        p=GridPlanner(cfg['bounds'],cfg['resolution'],obstacles,cfg['robot_radius']+.06)
        start=[-1.1,-.65]
        for station in cfg['points']:
            route=p.plan(start,station['pose'][:2])
            self.assertTrue(all(p.visible(a,b) for a,b in zip(route,route[1:])))
            start=station['pose'][:2]

    def test_livo_adapter_uses_estimate_and_rejects_stale_or_foreign_session(self):
        import tempfile
        import numpy as np
        from livo_sim_sensors import LivoSimSensors
        from relay_task import RelayFault
        bridge=LivoSimSensors.__new__(LivoSimSensors)
        bridge.run_id='test'
        bridge.anchor=np.array([1.,2.,math.pi/2])
        bridge.offset=np.array([0.,0.,1.25])
        data=dict(run_id='test',stamp=5.,samples=20,source='FAST-LIVO2',
                  position=[1.,0.,0.],quaternion_xyzw=[0.,0.,0.,1.])
        with tempfile.TemporaryDirectory() as folder:
            bridge.folder=Path(folder)
            target=bridge.folder/'odometry.json'
            target.write_text(json.dumps(data))
            np.testing.assert_allclose(bridge.estimated_pose(5.1),[1.,3.,math.pi/2],atol=1e-8)
            with self.assertRaises(RelayFault):bridge.estimated_pose(6.)
            data['run_id']='old';target.write_text(json.dumps(data))
            with self.assertRaises(RelayFault):bridge.estimated_pose(5.1)


if __name__=='__main__':unittest.main()
