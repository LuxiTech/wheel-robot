"""CUDA acceptance: execution, bounds, locked joints, infeasibility and caching."""
import json,sys,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from whole_body_control import WholeBodyIK

class CudaIKTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ik=WholeBodyIK(ROOT/'assets/robots/relay/relay_robot.urdf',backend='cuda')
        cfg=json.loads((ROOT/'config/relay_robot_config.json').read_text())
        cls.seed={n:cfg['home'].get(n,0.) for n in cls.ik.names}

    def targets(self,q):
        return {s:a.fk([q[n] for n in a.names],q['lift_joint']) for s,a in self.ik.arms.items()}

    def test_cuda_executes_and_matches_known_fk(self):
        q={**self.seed,'lift_joint':.31}
        r=self.ik.solve(self.targets(q),np.zeros(4),self.seed,lift=.31)
        self.assertTrue(r.success)
        self.assertLess(r.position_error,1e-5)
        self.assertTrue(self.ik.gpu.device.is_cuda)
        np.testing.assert_allclose(r.base,0,atol=1e-6)
        self.assertAlmostEqual(r.joints['lift_joint'],.31,places=6)

    def test_local_bounds_and_inactive_arm_remain_fixed(self):
        q={**self.seed,'openarm_left_joint4':.18}
        r=self.ik.solve({'left':self.targets(q)['left']},np.zeros(4),self.seed,
                        joint_window=.15,fixed_joints={'lift_joint':.4},attempts=2)
        self.assertTrue(r.success)
        for n in self.ik.arms['right'].names:
            self.assertAlmostEqual(r.joints[n],self.seed[n],places=6)
        for n in self.ik.names[1:]:
            self.assertLessEqual(abs(r.joints[n]-self.seed[n]),.150001)
        self.assertAlmostEqual(r.joints['lift_joint'],.4,places=6)

    def test_unreachable_target_rejected(self):
        t=np.eye(4);t[:3,3]=10
        self.assertFalse(self.ik.solve({'left':t},np.zeros(4),self.seed,attempts=1).success)

    def test_cuda_never_falls_back_to_cpu_optimization(self):
        from unittest.mock import patch
        q={**self.seed,'openarm_left_joint4':.12}
        with patch('whole_body_control.least_squares',side_effect=AssertionError('CPU optimization used')):
            self.assertTrue(self.ik.solve(self.targets(q),np.zeros(4),self.seed).success)

    def test_cache_key_tracks_actual_target_and_seed(self):
        target=self.targets(self.seed)
        self.ik.solve(target,np.zeros(4),self.seed)
        before=self.ik.gpu.calls
        self.ik.solve(target,np.zeros(4),self.seed)
        self.assertEqual(self.ik.gpu.calls,before)
        target['left']=target['left'].copy();target['left'][0,3]+=.001
        self.ik.solve(target,np.zeros(4),self.seed)
        self.assertGreater(self.ik.gpu.calls,before)

    def test_invalid_input_is_rejected(self):
        t=np.eye(4);t[0,3]=np.nan
        with self.assertRaises(ValueError):self.ik.solve({'left':t},np.zeros(4),self.seed)
        with self.assertRaises(ValueError):self.ik.solve(self.targets(self.seed),np.zeros(4),self.seed,lift=.5)

if __name__=='__main__':unittest.main()
