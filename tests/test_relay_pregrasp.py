import json,math,sys,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from relay_runtime import RelayRuntime
from relay_task import RelayFault

class PregraspTests(unittest.TestCase):
    def test_future_dock_grasp_does_not_depend_on_departure_heading(self):
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.c=json.loads((ROOT/'config/relay_config.json').read_text())
        rt.settings=json.loads((ROOT/'config/relay_demo_config.json').read_text())
        rt.grasp_transforms={'B':None};rt.pose_symmetry={};rt.grasp_forward={}
        pose=np.eye(4);pose[:3,3]=[*rt.c['handoff']['slots'][1],rt.c['handoff']['size'][2]]
        rt.observe=lambda index:pose.copy()
        rt.event=lambda *a,**k:None
        rt.joints=lambda who:{}
        def solve(targets,base,q,**kw):
            # At a north-facing dock the left hand must use the west rim.
            self.assertLess(targets['left'][0,3],pose[0,3])
            self.assertGreater(targets['right'][0,3],pose[0,3])
            self.assertAlmostEqual(base[3],math.pi/2)
            for side,target in targets.items():
                np.testing.assert_allclose(target,expected[side])
            return SimpleNamespace(success=True,joints={})
        rt.ik=SimpleNamespace(solve=solve)
        def move(*a,**kw):yield
        rt.move_joints=move
        rt.base=lambda who:np.array([.26,.17,0.,math.pi/2])
        expected=rt.rim_approach_targets('B',pose)
        for yaw in (-math.pi,-.001,0.,.001,math.pi/2,math.pi):
            with self.subTest(departure_yaw=yaw):
                rt.base=lambda who:np.array([1.55,-.95,0.,yaw])
                list(rt.prepare_handoff_pick('B',14))

    def test_handoff_approach_descends_vertically_without_sideways_insertion(self):
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.settings={'handoff_vertical_approach':.08}
        rt.hand_targets=lambda *a,**k:{s:np.eye(4) for s in ('left','right')}
        for progress in (0.,.5,1.):
            targets=rt.rim_approach_targets('B',np.eye(4),progress)
            for pose in targets.values():
                np.testing.assert_allclose(pose[:2,3],0)
                self.assertAlmostEqual(pose[2,3],.08*(1-progress))

    def test_closure_requires_actual_alignment_elbow_margin_and_rest(self):
        for error,elbow,speed,passed in [(0.,.8,0.,True),(.035,.8,0.,False),(0.,0.,0.,False),(0.,.8,.04,False)]:
            with self.subTest(error=error,elbow=elbow,speed=speed):
                rt=RelayRuntime.__new__(RelayRuntime);rt.time=0.
                rt.event=lambda *a,**k:None
                rt.base=lambda who:np.zeros(4)
                rt.observe=lambda i:np.eye(4)
                rt.hand_targets=lambda *a,**k:{s:np.eye(4) for s in ('left','right')}
                actual=np.eye(4);actual[0,3]=error
                rt.actual_hands=lambda who:{s:actual for s in ('left','right')}
                rt.joints=lambda who:{f'openarm_{s}_joint4':elbow for s in ('left','right')}
                rt.robots={'Robot_B':SimpleNamespace(get_linear_velocity=lambda:np.array([speed,0,0]),get_angular_velocity=lambda:np.zeros(3))}
                def run():
                    for _ in rt.confirm_pregrasp('B',12,('left','right')):rt.time+=.05
                if passed:run()
                else:
                    with self.assertRaises(RelayFault):run()

    def test_small_gravity_deflection_is_corrected_without_relaxing_tolerance(self):
        rt=RelayRuntime.__new__(RelayRuntime);rt.time=0.
        rt.event=lambda *a,**k:None
        rt.base=lambda who:np.zeros(4)
        rt.observe=lambda i:np.eye(4)
        rt.hand_targets=lambda *a,**k:{s:np.eye(4) for s in ('left','right')}
        sent={'left':0.,'right':0.,'lift_joint':.4}
        def actual(who,commanded=False):
            result={s:np.eye(4) for s in ('left','right')}
            for side,pose in result.items():pose[2,3]=sent[side]-(0. if commanded else .011)
            return result
        rt.actual_hands=actual
        rt.joints=lambda who:{f'openarm_{s}_joint4':1.2 for s in ('left','right')}
        rt.robots={'Robot_B':SimpleNamespace(get_linear_velocity=lambda:np.zeros(3),get_angular_velocity=lambda:np.zeros(3))}
        rt.controls={'Robot_B':SimpleNamespace(sent=sent,set_joint=sent.__setitem__)}
        rt.hand_goals={}
        def solve(target,*a,**k):
            return SimpleNamespace(success=True,joints={s:pose[2,3] for s,pose in target.items()})
        rt.ik=SimpleNamespace(names=('left','right'),solve=solve)
        for _ in rt.confirm_pregrasp('B',12,('left','right')):rt.time+=.05
        self.assertLess(rt.time,3.)
        self.assertTrue(all(abs(actual('B')[s][2,3])<.008 for s in ('left','right')))
        self.assertTrue(all(0<sent[s]<=.02 for s in ('left','right')))

    def test_one_retry_opens_and_retreats_before_redocking(self):
        rt=RelayRuntime.__new__(RelayRuntime);rt.settings={'handoff_vertical_approach':.08};rt.c={'handoff':{'slots':[[-.34,.75],[.34,.75]],'size':[1.3,.72,.75]}}
        calls=[];attempt=[0]
        rt.event=lambda *a,**k:None
        rt.actual_hands=lambda who:{s:np.eye(4) for s in ('left','right')}
        rt.gripping=lambda *a:True
        def noop(*a,**k):yield
        rt.approach_rim=noop;rt.hands=noop;rt.prepare_handoff_pick=noop
        def confirm(*a):
            attempt[0]+=1
            if attempt[0]==1:raise RelayFault('misaligned')
            yield
        def gripper(who,sides,closed,**kw):calls.append('close' if closed else 'open');yield
        def retreat(*a,**kw):calls.append('retreat');yield
        def dock(*a,**kw):
            calls.append('dock')
            yield from kw['prepare']
        rt.confirm_pregrasp=confirm;rt.gripper=gripper;rt.retreat=retreat;rt.dock=dock
        list(rt.close_on_rim('B',12,('left','right')))
        self.assertEqual(calls,['open','retreat','dock','close'])
        self.assertEqual(attempt[0],2)
if __name__=='__main__':unittest.main()
