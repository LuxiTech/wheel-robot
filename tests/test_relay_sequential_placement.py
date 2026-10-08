"""Neighbor-aware sequential placement and single-hand support guards."""
import json,math,sys,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from relay_runtime import RelayRuntime
from relay_task import RelayFault

class SequentialPlacementTests(unittest.TestCase):
    def runtime(self):
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.c=json.loads((ROOT/'config/relay_config.json').read_text())
        rt.settings=json.loads((ROOT/'config/relay_demo_config.json').read_text())
        rt.grasp_transforms={'B':None};rt.anchored_hands={'B':{}}
        rt.base=lambda who:np.array([0.,0.,0.,math.pi/2])
        return rt

    def test_neighbors_map_to_actual_hand_for_both_approach_directions(self):
        for yaw in (0.,math.pi/2,math.pi,3*math.pi/2):
            for sign in (-1,1):
                for facing in (-1,1):
                    with self.subTest(yaw=yaw,sign=sign,facing=facing):
                        rt=self.runtime();target=np.eye(4)
                        target[:3,:3]=Rotation.from_euler('z',yaw).as_matrix()
                        rt.base=lambda who:np.array([0.,0.,0.,yaw+facing*math.pi/2])
                        other=target.copy();other[:3,3]+=target[:3,0]*sign*.4
                        rt.initial=[{'path':'current'},{'path':'neighbor'}]
                        rt.observe=lambda i:target if i==0 else other
                        side,actual_sign=rt.placement_neighbor_side('B',0,target)
                        self.assertEqual(actual_sign,sign)
                        self.assertEqual(side,'left' if sign*facing<0 else 'right')
                        other[2,3]+=.22  # Another layer is not a lateral neighbor.
                        self.assertIsNone(rt.placement_neighbor_side('B',0,target))

    def test_tilted_box_keeps_lower_edge_supported_and_wall_outside_neighbor(self):
        rt=self.runtime()
        for yaw in (0.,math.pi/2,math.pi):
            for sign in (-1,1):
                target=np.eye(4);target[:3,:3]=Rotation.from_euler('z',yaw).as_matrix();target[2,3]=.35
                pose=rt.edge_placement_pose(target,sign)
                corners=np.array([[x,y,z] for x in (-.2,.2) for y in (-.15,.15) for z in (0.,.22)])
                world=corners@pose[:3,:3].T+pose[:3,3]
                local=(world-target[:3,3])@target[:3,:3]
                feet=rt.placement_support_corners()@pose[:3,:3].T+pose[:3,3]
                self.assertAlmostEqual(feet[:,2].min(),.35)
                self.assertLessEqual((sign*local[:,0]).max(),.200001)
                self.assertLessEqual((sign*local[:,0]).max(),.2-rt.settings['upper_placement_side_clearance']+1e-12)
                self.assertGreater(world[:,2].max(),.35+.22)

    def test_upper_approach_aligns_in_lane_using_final_slot_neighbor(self):
        for sign in (-1,1):
            rt=self.runtime();rt.held={'B':0}
            rt.base=lambda who:np.array([3.65,.4,0.,math.pi])
            observed=np.eye(4);observed[:3,:3]=Rotation.from_euler('z',math.pi/2).as_matrix()
            observed[:3,3]=[3.07,.48,.73]
            rt.observe=lambda i:observed.copy();rt.event=lambda *a,**kw:None
            final=np.array([2.75,.4,.57]);staged=final.copy()
            def neighbor(who,index,target):
                np.testing.assert_allclose(target[:3,3],final)
                return ('right' if sign>0 else 'left',sign)
            rt.placement_neighbor_side=neighbor
            calls=[]
            def lower(who,index,target):calls.append(target.copy());yield
            rt.lower_held_box=lower
            list(rt.prepare_target_approach('B',staged,final,np.array([3.32,.4])))
            self.assertEqual(len(calls),1)
            p=calls[0]
            self.assertAlmostEqual(p[0,3],3.08)
            self.assertLess(sign*(p[1,3]-.4),0.)
            self.assertGreaterEqual(p[2,3],final[2]+.06)
            self.assertLessEqual(p[2,3],.73)
            np.testing.assert_allclose(p[:3,2],[0.,0.,1.],atol=1e-12)

    def test_both_hands_descend_with_neighbor_side_leading(self):
        rt=self.runtime()
        for sign in (-1,1):
            target=np.eye(4);target[2,3]=.13
            low=rt.edge_placement_pose(target,sign)
            high=low.copy();high[2,3]+=.30
            before=rt.hand_targets('B',high)
            after=rt.hand_targets('B',low)
            leader='left' if sign<0 else 'right'
            follower='right' if leader=='left' else 'left'
            for side in ('left','right'):
                self.assertAlmostEqual(before[side][2,3]-after[side][2,3],.30)
            self.assertLess(after[leader][2,3],after[follower][2,3])

    def test_carry_uses_measured_upright_pose_and_rejects_uncorrected_sag(self):
        for follows in (True,False):
            rt=self.runtime();rt.upright_loads={};pose=np.eye(4)
            pose[:3,:3]=Rotation.from_euler('xyz',[.08,0.,.3]).as_matrix()
            pose[2,3]=.73
            rt.observe=lambda i:pose.copy();rt.event=lambda *a,**kw:None
            verified=[];commands=[]
            rt.verify_handoff_clearance=lambda i:verified.append(i)
            def lower(who,index,target):
                commands.append(target.copy())
                if follows:pose[:]=target
                yield
            rt.lower_held_box=lower
            if follows:
                list(rt.prepare_handoff_carry('B',0))
                self.assertEqual(verified,[0])
                np.testing.assert_allclose(pose[:3,2],[0,0,1],atol=1e-12)
                self.assertAlmostEqual(Rotation.from_matrix(pose[:3,:3]).as_euler('xyz')[2],.3)
            else:
                with self.assertRaisesRegex(RelayFault,'upright'):
                    list(rt.prepare_handoff_carry('B',0))
                self.assertEqual(len(commands),3)
                self.assertFalse(verified)

    def test_a_upright_guard_starts_after_leveling_and_keeps_zero_tilt_targets(self):
        rt=self.runtime();pose=np.eye(4)
        rt.held={'A':6};rt.upright_loads={};rt.contact_loss={'A':0.}
        rt.releasing={'A':False};rt.gripping=lambda *args:True
        rt.observe=lambda index:pose.copy()
        pose[:3,:3]=Rotation.from_euler('x',4.,degrees=True).as_matrix()
        rt.monitor_loads(.05)  # Single-end pickup has not entered carry mode.
        rt.upright_loads={'A':6}
        rt.monitor_loads(.05)  # Small carry tilt is now acceptable for A.
        rt.event=lambda *args,**kwargs:None
        list(rt.establish_upright_carry('A',6))  # No precision correction required.
        pose[:3,:3]=Rotation.from_euler('x',11.,degrees=True).as_matrix()
        with self.assertRaisesRegex(RelayFault,'upright'):
            rt.monitor_loads(.05)
        pose[:3,:3]=np.eye(3)
        rt.monitor_loads(.05)
        for key in ('source_retreat_roll_degrees','handoff_carry_tilt_degrees','handoff_place_tilt_degrees'):
            self.assertEqual(rt.settings[key],0.)

    def test_b_transport_preserves_grasp_posture_and_rejects_tilt(self):
        rt=self.runtime();rt.held={'B':0};rt.time=0.;rt.station_poses={}
        commands=[];pose=np.eye(4)
        rt.controls={'Robot_B':SimpleNamespace(set_twist=lambda *v:commands.append(v))}
        rt.observe=lambda i:pose.copy()
        # No IK object or arm targets: loaded navigation must only drive base.
        worker=rt.navigate('B',[0.,1.]);next(worker);worker.close()
        self.assertTrue(commands)
        pose[:3,:3]=Rotation.from_euler('x',.08).as_matrix()
        with self.assertRaisesRegex(RelayFault,'not upright'):
            next(rt.navigate('B',[0.,1.]))

    def test_withdrawal_corrects_supported_box_drift_with_remaining_hand(self):
        rt=self.runtime();reference=np.eye(4);reference[2,3]=.57
        measured=reference.copy();measured[0,3]+=.005
        rt.observe=lambda i:measured.copy()
        rt.actual_hands=lambda *a,**kw:{s:np.eye(4) for s in ('left','right')}
        targets=[]
        def hands(who,t,**kw):
            targets.append(t);measured[:]=reference;yield
        rt.hands=hands
        list(rt.withdraw_placement_hand('B','right',hold='left',box=0,box_pose=reference))
        self.assertAlmostEqual(targets[0]['left'][0,3],-.005)
        self.assertAlmostEqual(targets[-1]['right'][2,3],rt.settings['placement_hand_clearance'])
        self.assertAlmostEqual(targets[-1]['right'][1,3],-.03)

    def test_upper_release_requires_both_depth_rims_under_leading_foot(self):
        for follows in (True,False):
            rt=self.runtime();target=np.eye(4);target[2,3]=.35
            tilted=rt.edge_placement_pose(target,1)
            measured=tilted.copy();measured[1,3]+=.003
            below=np.eye(4);below[2,3]=.13
            rt.initial=[{'path':'held'},{'path':'lower'}]
            rt.observe=lambda i:measured.copy() if i==0 else below.copy()
            rt.box_contacts=lambda i:{'lower'}
            rt.placement_supported=lambda *a:True;rt.event=lambda *a,**kw:None
            commands=[]
            def lower(who,index,pose):
                commands.append(pose.copy())
                if follows:measured[:]=pose
                yield
            rt.lower_held_box=lower
            if follows:
                list(rt.align_upper_support('B',0,target,tilted,1))
                self.assertEqual(len(commands),1)
                self.assertAlmostEqual(measured[1,3],0.)
            else:
                with self.assertRaisesRegex(RelayFault,'both lower crate rims'):
                    list(rt.align_upper_support('B',0,target,tilted,1))

    def test_same_layer_contact_never_counts_as_bottom_support(self):
        rt=self.runtime();pose=np.eye(4);pose[2,3]=.35
        neighbor=pose.copy();neighbor[0,3]=.4
        rt.initial=[{'path':'held'},{'path':'neighbor'}]
        rt.observe=lambda i:pose if i==0 else neighbor
        rt.box_contacts=lambda i:{'neighbor'}
        self.assertFalse(rt.placement_supported(0,[0,0,.35]))
        neighbor[2,3]=.13
        self.assertTrue(rt.placement_supported(0,[0,0,.35]))
        pose[2,3]+=.05
        self.assertFalse(rt.placement_supported(0,[0,0,.35]))

    def test_single_hand_guard_requires_both_support_and_remaining_grasp(self):
        rt=self.runtime();rt.held={'B':0};rt.releasing={'B':False};rt.contact_loss={'B':0.}
        rt.placement_single_hand={'B':dict(side='right',position=[0,0,.13])}
        rt.placement_supported=lambda *a:True
        rt.gripping=lambda who,i,side:side=='right'
        rt.monitor_loads(1.)
        rt.gripping=lambda *a:False
        with self.assertRaisesRegex(RelayFault,'remaining-hand'):rt.monitor_loads(.1)
        rt.gripping=lambda *a:True;rt.placement_supported=lambda *a:False
        with self.assertRaisesRegex(RelayFault,'bottom support'):rt.monitor_loads(.1)

    def test_neighbor_hand_retracts_before_other_lowers_and_base_never_pushes(self):
        for side,sign in (('left',-1),('right',1)):
            with self.subTest(side=side):
                rt=self.runtime();target=np.eye(4);target[2,3]=.13
                final=target.copy()
                pose=target.copy();pose[2,3]=.75
                rt.placement_single_hand={};rt.held={'B':0};rt.releasing={'B':False}
                rt.event=lambda *a,**kw:None
                rt.observe=lambda i:pose.copy()
                rt.actual_hands=lambda *a,**kw:{s:np.eye(4) for s in ('left','right')}
                rt.gripping=lambda *a:True;rt.placement_supported=lambda *a:True
                calls=[]
                def lower(who,index,t,**kw):
                    calls.append(('lower',kw.get('sides',('left','right'))))
                    if len(kw.get('sides',('left','right')))==1:
                        self.assertAlmostEqual(t[0,1],0.)
                        self.assertAlmostEqual(t[2,1],0.)
                        self.assertAlmostEqual(t[1,1],1.)
                        self.assertAlmostEqual(t[1,3],final[1,3])
                        feet=rt.placement_support_corners()@t[:3,:3].T+t[:3,3]
                        self.assertAlmostEqual(feet[:,2].min(),final[2,3])
                    pose[:]=t;yield
                def grip(who,sides,closed,**kw):calls.append(('release',sides));yield
                def withdraw(who,s,**kw):calls.append(('withdraw',s));yield
                def wait(*a):yield
                rt.lower_held_box=lower;rt.gripper=grip;rt.withdraw_placement_hand=withdraw;rt.wait=wait
                rt.navigate=lambda *a,**kw:self.fail('Placement must not move the base')
                list(rt.place_sequential('B',0,target,side,sign,final_target=final))
                other='right' if side=='left' else 'left'
                self.assertEqual(calls[:4],[('lower',('left','right')),('lower',('left','right')),('release',(side,)),('withdraw',side)])
                self.assertTrue(all(c==('lower',(other,)) for c in calls[4:-2]))
                self.assertEqual(calls[-2:],[('release',(other,)),('withdraw',other)])
                np.testing.assert_allclose(pose[:2,3],final[:2,3],atol=1e-12)
                self.assertAlmostEqual(pose[2,3],.13)
                self.assertFalse(rt.placement_single_hand)
                self.assertIsNone(rt.held['B'])

    def test_no_gripper_opens_until_first_edge_has_bottom_support(self):
        rt=self.runtime();target=np.eye(4);target[2,3]=.13
        pose=target.copy();pose[2,3]=.75
        rt.observe=lambda i:pose.copy();rt.event=lambda *a,**kw:None
        rt.placement_supported=lambda *a:False
        heights=[]
        def lower(who,index,t,**kw):
            heights.append(t[2,3]);pose[:]=t;pose[2,3]+=.01;yield
        def wait(*a):yield
        rt.lower_held_box=lower;rt.wait=wait
        rt.gripper=lambda *a,**kw:self.fail('Unsupported box must retain both hands')
        with self.assertRaisesRegex(RelayFault,'no bottom support'):
            list(rt.place_sequential('B',0,target,'left',-1))
        self.assertTrue(all(h>=heights[1] for h in heights[2:]),
                        'Support search must not command a deeper deck penetration')

    def test_descent_stops_before_next_segment_once_bottom_is_supported(self):
        rt=self.runtime();pose=np.eye(4);pose[2,3]=.3
        target=pose.copy();target[2,3]=.13
        support=[False];calls=[]
        rt.observe=lambda i:pose.copy()
        rt.placement_supported=lambda *a:support[0]
        rt.gripping=lambda *a:True
        rt.actual_hands=lambda *a,**kw:{s:np.eye(4) for s in ('left','right')}
        rt.hand_targets=lambda who,t,**kw:t
        def hands(who,t,**kw):
            calls.append(t.copy());pose[:]=t;support[0]=True;yield
        rt.hands=hands
        list(rt.lower_held_box('B',0,target,stop_on_support=[0,0,.13]))
        self.assertEqual(len(calls),1)

if __name__=='__main__':unittest.main()
