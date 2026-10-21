"""Reject locally reachable carry poses whose later deposit cannot be reached."""
import json, math, sys, unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from relay_runtime import RelayRuntime
from relay_task import RelayFault

class AdaptiveHandoffTests(unittest.TestCase):
    def test_carry_height_tracks_table_instead_of_old_absolute_height(self):
        for height in (.45,.6):
            rt,_,_=self.fixture()
            rt.c['handoff']['size'][2]=height
            rt.grasp_base_yaw={};rt.grasp_object_yaw={}
            def inspect(index,staging,final):
                self.assertAlmostEqual(staging[2,3],max(.62,height+.07))
                self.assertAlmostEqual(final[2,3],height+.12)
                raise RelayFault('captured planning targets')
            rt.plan_handoff_carry=inspect
            with self.assertRaisesRegex(RelayFault,'captured planning targets'):
                list(rt.prepare_handoff_carry('A',0))

    def fixture(self):
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.c=json.loads((ROOT/'config/relay_config.json').read_text())
        rt.settings=json.loads((ROOT/'config/relay_demo_config.json').read_text())
        rt.initial=[{'position':[0.,.3,0.],'layer':1,'row':0}]
        rt.base=lambda who:np.array([-1.,-.95,0.,math.pi/2])
        pose=np.eye(4);pose[:3,3]=[-1.,-.37,.62]
        rt.observe=lambda index:pose.copy()
        rt.hand_targets=lambda who,target,**kw:{'left':target.copy(),'right':target.copy()}
        rt.event=lambda *a,**kw:None
        rt.ik=SimpleNamespace(names=['lift_joint'],lower=np.array([0.]),upper=np.array([.4]))
        staging=pose.copy();staging[2,3]=.67;final=staging.copy();final[2,3]=.72
        return rt,staging,final

    def test_dock_fallback_only_retreats_and_rechecks_actual_arrival(self):
        rt,_,_=self.fixture();base=rt.base('A').copy();origin=base.copy();calls=[];moves=[]
        rt.base=lambda who:base.copy()
        rt.actual_hands=lambda who:{'left':np.eye(4),'right':np.eye(4)}
        rt.grasp_transforms={};rt.gripping=lambda *args:True
        rt.verify_handoff_clearance=lambda index:None
        def plan(index,position,seed):
            calls.append(position.copy())
            if position[1]>origin[1]-.019:raise RelayFault('near dock unreachable')
            return ['checked']
        def navigate(who,xy,yaw,**kwargs):
            self.assertTrue(kwargs['preserve_posture'])
            moves.append(np.array(xy));base[:2]=xy
            yield
        rt.preflight_handoff_place=plan;rt.navigate=navigate
        worker=rt.prepare_handoff_deposit(0);next(worker)
        with self.assertRaises(StopIteration) as done:next(worker)
        self.assertEqual(done.exception.value,['checked'])
        self.assertEqual(len(moves),1)
        self.assertAlmostEqual(moves[0][0],origin[0])
        self.assertAlmostEqual(moves[0][1],origin[1]-.02)
        self.assertEqual(len(calls),3)  # Initial, predicted, actual arrival.

    def test_unreachable_dock_candidates_do_not_move_base(self):
        rt,_,_=self.fixture()
        rt.actual_hands=lambda who:{'left':np.eye(4),'right':np.eye(4)}
        rt.grasp_transforms={};rt.gripping=lambda *args:True
        rt.verify_handoff_clearance=lambda index:None
        def reject(*args):raise RelayFault('unreachable')
        rt.preflight_handoff_place=reject
        with self.assertRaisesRegex(RelayFault,'safe dock adjustments'):
            list(rt.prepare_handoff_deposit(0))

    def test_layer_dispatch_and_mirrored_side(self):
        rt,_,_=self.fixture()
        rt.initial=[{'layer':0,'row':0},{'layer':1,'row':0},{'layer':2,'row':1}]
        bottom=rt.source_carry_strategy(0);middle=rt.source_carry_strategy(1);top=rt.source_carry_strategy(2)
        self.assertEqual(bottom[0],'middle')
        self.assertIs(bottom[1],middle[1])
        self.assertEqual(top[0],'top')
        self.assertEqual(top[2],-.04)
        self.assertNotEqual(top[1]['forward'],middle[1]['forward'])

    def test_feasible_primary_does_not_search_other_candidates(self):
        rt,staging,final=self.fixture();calls=[]
        def plan(who,target,**kw):
            calls.append(kw)
            return [{'lift_joint':.3}]
        rt.plan_hands=plan
        _,chosen=rt.plan_handoff_carry(0,staging,final)
        self.assertEqual(chosen['strategy'],'middle')
        self.assertFalse(chosen['fallback'])
        self.assertEqual(chosen['forward'],.55)
        self.assertEqual(len(calls),5)  # Stage, lift, approach, lower, withdrawal.

    def test_all_deposit_paths_rejected_prevents_carry_execution(self):
        rt,staging,final=self.fixture();checked=[]
        def plan(who,target,**kw):
            if kw.get('base') is not None:
                checked.append(kw['base'])
                raise RelayFault('deposit blocked')
            return [{'lift_joint':.3}]
        rt.plan_hands=plan
        with self.assertRaisesRegex(RelayFault,'carry and deposit'):
            rt.plan_handoff_carry(0,staging,final)
        self.assertTrue(checked)
        self.assertTrue(all(np.allclose(b,[-.26,1.33,0.,-math.pi/2]) for b in checked))

    def test_infeasible_fixed_reach_uses_candidate_and_checks_withdrawal(self):
        rt,staging,final=self.fixture();withdrawals=[]
        def plan(who,target,**kw):
            if kw.get('base') is None:
                reach=target['left'][1,3]+.95
                if reach<.57:raise RelayFault('fixed reach joint limit')
            else:
                withdrawals.append(kw['seconds'])
                self.assertIsNotNone(kw['seed'])
            return [{'lift_joint':.3}]
        rt.plan_hands=plan
        legs,chosen=rt.plan_handoff_carry(0,staging,final)
        self.assertGreaterEqual(chosen['forward'],.57)
        self.assertTrue(chosen['deposit_preflight'])
        self.assertIn(2.,withdrawals)
        self.assertEqual(len(legs),2)
        self.assertAlmostEqual(legs[-1][0][2,3],.72)

if __name__=='__main__':unittest.main()
