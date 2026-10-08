"""Offset staging, bottom support and feedback-controlled base pushing."""
import json,sys,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from relay_runtime import RelayRuntime
from relay_task import RelayFault,target_order

class PlacementTests(unittest.TestCase):
    def runtime(self):
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.c=json.loads((ROOT/'config/relay_config.json').read_text())
        rt.settings=json.loads((ROOT/'config/relay_demo_config.json').read_text())
        return rt

    def test_bottom_and_upper_staging_retain_support(self):
        rt=self.runtime()
        for slot in target_order(rt.c):
            distance,margin,center,half,height,closed=rt.push_parameters(slot['position'])
            self.assertAlmostEqual(distance,.12 if slot['layer'] else .20)
            self.assertEqual(closed,bool(slot['layer']))
            sign=-1 if slot['row'] else 1
            staged=np.array(slot['position'][:2])+[sign*distance,0.]
            self.assertTrue(np.all(np.abs(staged-center)<=half-margin+1e-8))
            self.assertLess(height,.05 if slot['layer'] else .12)

    def test_release_and_withdraw_precede_push_and_final_goal_is_unchanged(self):
        for row,neighbor in ((0,None),(1,None),(0,('left',1)),(1,('right',-1))):
            rt=self.runtime();rt.grasp_transforms={'B':None};rt.releasing={'B':False};rt.held={'B':0}
            rt.perception='ground_truth';rt.settings['motion_time_scale']=1.
            goal=np.array([2.75 if row==0 else 2.45,.4,.13]);pose=np.eye(4);pose[:3,3]=goal
            rt.observe=lambda i:pose.copy();rt.gripping=lambda *a:True
            rt.placement_supported=lambda *a:True
            rt.actual_hands=lambda *a,**kw:{'left':np.eye(4),'right':np.eye(4)}
            rt.hand_targets=lambda who,t,**kw:{'left':t.copy(),'right':t.copy()}
            rt.base=lambda who:np.array([3.5 if row==0 else 1.7,.4,0.,np.pi if row==0 else 0.])
            rt.crates=[SimpleNamespace(get_linear_velocity=lambda:np.zeros(3),get_angular_velocity=lambda:np.zeros(3))]
            calls=[];rt.event=lambda *a,**kw:None
            def select_neighbor(who,index,t):
                np.testing.assert_allclose(t[:3,3],goal)
                return neighbor
            rt.placement_neighbor_side=select_neighbor
            def sequential(who,index,t,side,sign,*,final_target):
                self.assertEqual((side,sign),neighbor)
                np.testing.assert_allclose(final_target[:3,3],goal)
                calls.append(('sequential',t[:3,3].copy()))
                pose[:]=t
                calls.append(('release',False));calls.append(('hands',))
                rt.held[who]=None
                yield
            rt.place_sequential=sequential
            def lower(who,index,t):calls.append(('lower',t[:3,3].copy()));pose[:]=t;yield
            def hands(who,t,**kw):calls.append(('hands',));pose[:]=t['left'];yield
            def grip(who,sides,closed):calls.append(('release',closed));yield
            def navigate(*a,**kw):calls.append(('navigate',));yield
            def wait(*a):yield
            def push(who,index,position,forward,distance):
                self.assertIsNone(rt.held[who]);calls.append(('push',));np.testing.assert_allclose(position,goal);yield
            def verify(who,index,position):calls.append(('verify',));np.testing.assert_allclose(position,goal);yield
            rt.lower_held_box=lower;rt.hands=hands;rt.gripper=grip;rt.navigate=navigate
            rt.wait=wait;rt.push_with_base=push;rt.verify_placement=verify
            list(rt.place('B',0,goal,push=True))
            self.assertEqual(calls[0][0],'sequential' if neighbor else 'lower')
            np.testing.assert_allclose(calls[0][1],goal+([0.,0.,0.] if neighbor else [.20 if row==0 else -.20,0.,.06]))
            names=[c[0] for c in calls]
            if neighbor:
                self.assertNotIn('navigate',names)
                self.assertNotIn('push',names)
            else:
                self.assertLess(names.index('release'),names.index('navigate'))
                self.assertLess(names.index('navigate'),names.index('push'))
            self.assertEqual(names[names.index('release')+1],'hands')
            self.assertEqual(names[-1],'verify')

    def test_same_layer_contact_is_not_bottom_support(self):
        rt=self.runtime();pose=np.eye(4);pose[2,3]=.35
        neighbor=pose.copy();neighbor[0,3]=.4
        rt.initial=[{'path':'held'},{'path':'neighbor'}]
        rt.observe=lambda i:pose if i==0 else neighbor
        rt.box_contacts=lambda i:{'neighbor'}
        self.assertFalse(rt.placement_supported(0,[0,0,.35]))
        neighbor[2,3]=.13
        self.assertTrue(rt.placement_supported(0,[0,0,.35]))

    def test_push_uses_box_feedback_and_rejects_stall(self):
        rt=self.runtime();commands=[]
        rt.controls={'Robot_B':SimpleNamespace(set_twist=lambda *v:commands.append(v),stop=lambda:commands.append('stop'))}
        rt.time=0.;rt.perception='ground_truth';rt.station_poses={}
        rt.base=lambda who:np.array([0.,0.,0.,np.pi/2])
        pose=np.eye(4);rt.observe=lambda index:pose.copy();rt.event=lambda *a,**kw:None
        goal=np.array([0.,.1,0.]);forward=np.array([0.,1.,0.])
        worker=rt.push_with_base('B',12,goal,forward,.16);next(worker)
        pose[1,3]=.098;rt.time=.1;next(worker)
        self.assertEqual(commands[-1],'stop')
        rt.time=.7
        with self.assertRaises(StopIteration):next(worker)
        pose[0,3]=-.012;rt.time=0.
        worker=rt.push_with_base('B',12,goal,forward,.16);next(worker)
        self.assertLess(commands[-1][1],0.)
        worker.close();pose[0,3]=0.;pose[1,3]=0.;rt.time=0.
        worker=rt.push_with_base('B',12,goal,forward,.16);next(worker);rt.time=3.1
        with self.assertRaisesRegex(RelayFault,'stalled'):next(worker)

if __name__=='__main__':unittest.main()
