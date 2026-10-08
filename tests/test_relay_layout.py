import copy
import json
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from relay_layout import slot_position, pallet_yaw, world_half_extent
from relay_task import target_order, source_order


class LayoutTests(unittest.TestCase):
    def setUp(self):
        self.scene = json.loads((ROOT/'config/relay_config.json').read_text())
        self.scene['pallet']['yaw_degrees'] = 90

    def test_parallel_base_and_arm_preparation_waits_for_both(self):
        from types import SimpleNamespace
        from relay_runtime import RelayRuntime
        for travel_steps,arm_steps in ((2,5),(5,2)):
            rt=RelayRuntime.__new__(RelayRuntime)
            calls=[]
            rt.controls={'Robot_A':SimpleNamespace(stop=lambda:calls.append('stop'))}
            def action(name,steps):
                for _ in range(steps):
                    calls.append(name)
                    yield
            ticks=len(list(rt.move_and_prepare('A',action('base',travel_steps),action('arm',arm_steps))))
            self.assertEqual(ticks,max(travel_steps,arm_steps))
            self.assertEqual(calls[:2],['base','arm'])
            self.assertEqual(calls[-1],'stop')

    def test_parallel_failure_closes_other_action_and_stops_base(self):
        from types import SimpleNamespace
        from relay_runtime import RelayRuntime
        from relay_task import RelayFault
        rt=RelayRuntime.__new__(RelayRuntime)
        calls=[]
        rt.controls={'Robot_A':SimpleNamespace(stop=lambda:calls.append('stop'))}
        def travel():
            try:
                while True:
                    yield
            finally:
                calls.append('closed')
        def prepare():
            yield
            raise RelayFault('arm tracking failed')
        with self.assertRaises(RelayFault):
            list(rt.move_and_prepare('A',travel(),prepare()))
        self.assertEqual(calls,['closed','stop'])

    def test_fixed_dock_finishes_arm_preparation_before_entering_stack(self):
        from types import SimpleNamespace
        from relay_runtime import RelayRuntime
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.c=self.scene
        rt.settings=json.loads((ROOT/'config/relay_demo_config.json').read_text())
        rt.last_dock={}
        rt.controls={'Robot_A':SimpleNamespace(stop=lambda:None)}
        rt.base=lambda who:[rt.settings['fixed_pallet_lanes']['A'][0],.8,0.,math.pi]
        route=[]
        prepared=[]
        def navigate(who,xy,yaw=None):
            route.append(list(xy))
            if len(route)==2:
                self.assertTrue(prepared)
            yield
        def prepare():
            for _ in range(5):
                yield
            prepared.append(True)
        rt.navigate=navigate
        list(rt.dock('A',slot_position(self.scene,'source',0,0,2),prepare=prepare()))
        self.assertEqual(len(route),2)
        self.assertTrue(all(p[1]!=rt.settings['transit_y'] for p in route))

    def test_quarter_turn_preserves_box_and_swaps_footprint(self):
        scene = self.scene
        points = [slot_position(scene,'source',c,r,0) for r in range(2) for c in range(3)]
        self.assertEqual(len({round(p[0],6) for p in points}),2)
        self.assertEqual(len({round(p[1],6) for p in points}),3)
        half = world_half_extent(scene['crate']['size'],pallet_yaw(scene))
        self.assertAlmostEqual(max(p[0] for p in points)-min(p[0] for p in points)+2*half[0],.6)
        self.assertAlmostEqual(max(p[1] for p in points)-min(p[1] for p in points)+2*half[1],1.2)
        self.assertEqual(scene['crate']['size'],[.4,.3,.22])
        pallet = world_half_extent(scene['pallet']['size'],pallet_yaw(scene))
        for p in points:
            for axis in (0,1):
                self.assertLessEqual(abs(p[axis]-scene['pallet']['source'][axis])+half[axis],pallet[axis])

    def test_rotated_target_grid_keeps_middle_first_and_all_slots_unique(self):
        slots = target_order(self.scene)
        self.assertEqual(len(slots),18)
        self.assertEqual(len({tuple(s['position']) for s in slots}),18)
        self.assertEqual([s['column'] for s in slots[:6]],[1,1,0,0,2,2])
        self.assertTrue(all(s['yaw']==math.pi/2 for s in slots))
        initial=[dict(layer=k,row=r,column=c) for k in range(3) for r in range(2) for c in range(3)]
        self.assertEqual(source_order(initial)[:6],[12,14,17,15,16,13])

    def test_zero_rotation_keeps_legacy_coordinates(self):
        old=copy.deepcopy(self.scene)
        old['pallet']['yaw_degrees']=0
        old['pallet']['source']=[-2.1,.8]  # Original unrotated fixture.
        p=slot_position(old,'source',0,0,0)
        for value,expected in zip(p,[-2.5,.65,.13]):
            self.assertAlmostEqual(value,expected)

    def test_source_front_rear_is_independent_of_left_deposit(self):
        from relay_task import source_handoff_far, source_handoff_slot
        initial=json.loads((ROOT/'assets/generated/relay/initial_state.json').read_text())['crates']
        ordered=[initial[i] for i in source_order(initial)[:6]]
        self.assertEqual([source_handoff_slot(self.scene,s) for s in ordered],[0]*6)
        self.assertEqual([source_handoff_far(self.scene,s) for s in ordered],
                         [False,True,True,False,True,True])

    def test_rear_handoff_dock_and_retreat_stay_in_rear_aisle(self):
        from relay_runtime import RelayRuntime
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.c=self.scene
        rt.settings=json.loads((ROOT/'config/relay_demo_config.json').read_text())
        rt.last_dock={}
        rt.held={'A':None}
        route=[]
        def navigate(who,xy,yaw=None):
            route.append((list(xy),yaw))
            yield
        rt.navigate=navigate
        rt.stow=lambda who: iter(())
        xy=self.scene['handoff']['slots'][0]
        list(rt.dock('A',[*xy,.6],handoff=True,far=True))
        self.assertEqual(route[0][0][1],rt.settings['source_rear_aisle_y'])
        self.assertAlmostEqual(route[-1][0][1],xy[1]+rt.settings['handoff_dock_distance'])
        rt.base=lambda who: [*route[-1][0],0.,-math.pi/2]
        list(rt.retreat('A',handoff=True,far=True))
        self.assertEqual(route[-1][0],[route[-2][0][0],rt.settings['source_rear_aisle_y']])
        self.assertNotEqual(route[-1][0][0],-1.)
        self.assertTrue(all(p[1]>xy[1] and yaw==-math.pi/2 for p,yaw in route))

    def test_handoff_docking_offset_follows_slot_not_robot_identity(self):
        from relay_runtime import RelayRuntime
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.c=self.scene
        rt.settings=json.loads((ROOT/'config/relay_demo_config.json').read_text())
        rt.last_dock={}
        route=[]
        def navigate(who,xy,yaw=None):
            route.append(list(xy))
            yield
        rt.navigate=navigate
        for who,slot in (('A',0),('A',1),('B',1)):
            route.clear()
            xy=self.scene['handoff']['slots'][slot]
            list(rt.dock(who,[*xy,.75],handoff=True))
            self.assertAlmostEqual(route[-1][0],xy[0]+(.08 if slot==0 else -.08))
            self.assertAlmostEqual(route[-1][1],xy[1]-rt.settings['b_handoff_dock_distance' if who=='B' else 'handoff_dock_distance'])
            self.assertEqual(route[-2],[route[-1][0],rt.settings['transit_y']])

    def test_rotated_support_and_side_docking(self):
        import numpy as np
        from relay_runtime import RelayRuntime
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.c=self.scene
        rt.settings=json.loads((ROOT/'config/relay_demo_config.json').read_text())
        rt.last_dock={}
        rt.base=lambda who:[rt.settings['fixed_pallet_lanes']['A'][0],.8,0.,math.pi]
        route=[]
        def navigate(who,xy,yaw=None):
            route.append((list(xy),yaw))
            yield
        rt.navigate=navigate
        position=slot_position(self.scene,'source',0,0,2)
        list(rt.dock('A',position))
        self.assertAlmostEqual(route[-1][0][0],position[0]+rt.settings['dock_distance'])
        self.assertAlmostEqual(abs(route[-1][1]),math.pi)
        self.assertTrue(all(xy[0]>self.scene['pallet']['source'][0]+.3 for xy,yaw in route))
        position=slot_position(self.scene,'target',0,1,2)
        np.testing.assert_allclose(world_half_extent(self.scene['crate']['size'],pallet_yaw(self.scene)),[.15,.2])

    def test_a_leaves_handoff_directly_via_same_end_aisle(self):
        from relay_runtime import RelayRuntime
        for y in (-.95,2.1):
            for row in (0,1):
                rt=RelayRuntime.__new__(RelayRuntime)
                rt.c=self.scene
                rt.settings=json.loads((ROOT/'config/relay_demo_config.json').read_text())
                rt.last_dock={}
                rt.base=lambda who:[-.26,y,0.,math.pi/2]
                route=[]
                def navigate(who,xy,yaw=None):
                    route.append(list(xy))
                    yield
                rt.navigate=navigate
                list(rt.dock('A',slot_position(self.scene,'source',2,row,2),far=bool(row)))
                lane=rt.settings['fixed_pallet_lanes']['A'][row]
                self.assertEqual(route[:2],[[-.26,y],[lane,y]])
                self.assertTrue(all(p!=[-1.,-.95] for p in route))
                if y>0:
                    self.assertTrue(all(p[1]>0 for p in route))

    def test_new_initial_layout_is_source_a_handoff_b_target(self):
        a,b=self.scene['robots']
        source=self.scene['pallet']['source']
        target=self.scene['pallet']['target']
        table=self.scene['handoff']['center']
        self.assertTrue(source[0]<a['position'][0]<table[0]<b['position'][0]<target[0])
        self.assertEqual(a['position'][1],source[1])
        self.assertEqual(b['position'][1],target[1])
        self.assertEqual(a['yaw'],math.pi)
        self.assertEqual(b['yaw'],0.)

    def test_hand_targets_rotate_with_box_and_base(self):
        import numpy as np
        from scipy.spatial.transform import Rotation
        from relay_runtime import RelayRuntime
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.c=self.scene
        rt.settings=json.loads((ROOT/'config/relay_demo_config.json').read_text())
        rt.grasp_transforms={'A':None}
        rt.grasp_forward={'A':{'left':-.1,'right':-.1}}
        rt.base=lambda who:np.array([0,0,0,math.pi/2])
        before=rt.hand_targets('A',np.eye(4))
        turn=np.eye(4);turn[:3,:3]=Rotation.from_euler('z',math.pi/2).as_matrix()
        rt.base=lambda who:np.array([0,0,0,math.pi])
        after=rt.hand_targets('A',turn)
        for side in ('left','right'):
            np.testing.assert_allclose(after[side],turn @ before[side],atol=1e-10)

    def test_grasp_points_are_rim_centres_even_on_a_tilted_box(self):
        import numpy as np
        from scipy.spatial.transform import Rotation
        from relay_runtime import RelayRuntime
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.c=self.scene
        rt.settings=json.loads((ROOT/'config/relay_demo_config.json').read_text())
        rt.grasp_transforms={'A':None,'B':None}
        pose=np.eye(4)
        pose[:3,:3]=Rotation.from_euler('xyz',[.1,-.05,math.pi/2]).as_matrix()
        pose[:3,3]=[-1.95,.4,.57]
        for who in ('A','B'):
            for yaw in (0.,math.pi):
                rt.base=lambda who:np.array([0,0,0,yaw])
                for t in rt.hand_targets(who,pose).values():
                    local=np.linalg.inv(pose) @ t
                    self.assertAlmostEqual(local[1,3],0.)
                    self.assertAlmostEqual(abs(local[0,3]),.2-rt.settings['grasp_inset'])
                    self.assertAlmostEqual(local[2,3],.22-rt.settings['grasp_below_top'])

    def test_second_hand_joins_without_releasing_first_hand(self):
        import numpy as np
        from relay_runtime import RelayRuntime
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.settings=json.loads((ROOT/'config/relay_demo_config.json').read_text())
        rt.perception='ground_truth'
        pose=np.eye(4)
        rt.observe=lambda index:pose.copy()
        rt.base=lambda who:np.array([0,0,0,math.pi])
        rt.actual_hands=lambda who,**kw:{s:np.eye(4) for s in ('left','right')}
        rt.anchored_hands={'A':{}}
        rt.event=lambda *a,**kw:None
        rt.gripping=lambda *a:True
        calls=[]
        def hands(who,targets,**kw):
            calls.append(('hands',tuple(targets)))
            if len(calls)==1:
                pose[:3,3]=targets['left'][:3,3]
                self.assertLess(pose[1,3],0.)
                self.assertGreater(pose[2,3],0.)
            else:
                self.assertIn('left',rt.anchored_hands[who])
            yield
        def approach(who,index,sides):
            self.assertEqual(sides,('right',))
            self.assertIn('left',rt.anchored_hands[who])
            yield
        def gripper(who,sides,closed,**kw):
            calls.append(('gripper',sides,closed))
            self.assertTrue(closed)
            self.assertEqual(sides,('right',))
            self.assertIn('left',rt.anchored_hands[who])
            yield
        rt.hands=hands
        rt.approach_rim=approach
        rt.gripper=gripper
        rt.hand_targets=lambda who,pose,**kw:{'right':pose.copy()}
        list(rt.join_second_gripper('A',12,'left'))
        self.assertEqual(calls,[('hands',('left',)),('hands',('right',)),('gripper',('right',),True)])
        self.assertEqual(rt.anchored_hands['A'],{})

    def test_fixed_source_retreat_clears_both_sides_before_longitudinal_travel(self):
        import numpy as np
        from scipy.spatial.transform import Rotation
        from relay_runtime import RelayRuntime
        from relay_task import RelayFault
        for far in (False,True):
            for follows in (False,True):
                with self.subTest(far=far,follows=follows):
                    rt=RelayRuntime.__new__(RelayRuntime)
                    rt.c=self.scene
                    rt.settings=json.loads((ROOT/'config/relay_demo_config.json').read_text())
                    sign=-1 if far else 1
                    source_x=self.scene['pallet']['source'][0]
                    base=np.array([source_x+sign*.895,.4,0.,0. if far else math.pi])
                    pose=np.eye(4)
                    pose[:3,:3]=Rotation.from_euler('z',math.pi/2).as_matrix()
                    pose[:3,3]=[source_x+sign*.292,.4,.665]
                    rt.base=lambda who:base.copy()
                    rt.observe=lambda index:pose.copy()
                    rt.held={'A':12}
                    rt.event=lambda *a,**kw:None
                    rt.hand_targets=lambda who,target:target
                    rt.plan_hands=lambda *args,**kwargs:[{}]
                    route=[]
                    def hands(who,target,**kw):
                        self.assertEqual(len(route),1, 'Retreat chassis before changing arm posture')
                        if follows:
                            pose[:]=target
                        yield
                    def navigate(who,xy,yaw=None,**kwargs):
                        route.append(list(xy))
                        if len(route)==1:
                            self.assertTrue(kwargs.get('preserve_posture'))
                        if follows:
                            pose[:2,3]+=np.asarray(xy)-base[:2]
                        base[:2]=xy
                        yield
                    rt.hands=hands
                    rt.navigate=navigate
                    if follows:
                        list(rt.retreat('A',far=far))
                        self.assertEqual(route[-1],[rt.settings['fixed_pallet_lanes']['A'][int(far)],rt.settings['transit_y']])
                        self.assertGreater(sign*(pose[0,3]-source_x)-.15,.35)
                    else:
                        with self.assertRaises(RelayFault):
                            list(rt.retreat('A',far=far))
                        self.assertEqual(len(route),1)
                        self.assertEqual(route[0][1],.4)

    def test_retreat_fallback_clears_stack_before_replanning_both_sides(self):
        import numpy as np
        from scipy.spatial.transform import Rotation
        from relay_runtime import RelayRuntime
        from relay_task import RelayFault
        for far in (False,True):
            rt=RelayRuntime.__new__(RelayRuntime)
            rt.c=self.scene
            rt.settings=json.loads((ROOT/'config/relay_demo_config.json').read_text())
            sign=-1 if far else 1
            center=self.scene['pallet']['source'][0]
            base=np.array([center+sign*.895,.4,0.,0. if far else math.pi])
            pose=np.eye(4);pose[:3,:3]=Rotation.from_euler('z',math.pi/2).as_matrix()
            pose[:3,3]=[center+sign*.292,.4,.42]
            rt.base=lambda who:base.copy()
            rt.observe=lambda index:pose.copy()
            rt.held={'A':6};rt.event=lambda *a,**kw:None
            rt.hand_targets=lambda who,target:target
            route=[];plans=[];executed=[]
            def navigate(who,xy,yaw=None,**kwargs):
                route.append(list(xy));pose[:2,3]+=np.asarray(xy)-base[:2];base[:2]=xy
                yield
            def plan(who,target,**kwargs):
                plans.append(target.copy())
                if len(plans)==1:raise RelayFault('fixture joint limit')
                self.assertEqual(len(route),2)
                self.assertGreaterEqual(sign*(pose[0,3]-center)-.15,.399)
                self.assertAlmostEqual(target[2,3],pose[2,3])
                return [{'lift_joint':.35}]
            def hands(who,target,**kwargs):
                self.assertEqual(kwargs['planned'],[{'lift_joint':.35}])
                executed.append(target.copy());pose[:]=target
                yield
            rt.navigate=navigate;rt.plan_hands=plan;rt.hands=hands
            list(rt.retreat('A',far=far))
            self.assertEqual(len(executed),1)
            self.assertEqual(len(route),3)
            self.assertEqual(route[0][1],route[1][1])
            self.assertEqual(route[-1][0],route[1][0])

    def test_split_carry_checks_final_lift_before_executing_first_leg(self):
        import numpy as np
        from relay_runtime import RelayRuntime
        from relay_task import RelayFault
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.c=self.scene
        rt.settings=json.loads((ROOT/'config/relay_demo_config.json').read_text())
        rt.grasp_base_yaw={};rt.grasp_object_yaw={}
        rt.initial=[{'position':[0.,.3,0.],'layer':1,'row':0}]*18
        rt.base=lambda who:np.array([-1.,-.95,0.,math.pi/2])
        pose=np.eye(4);pose[:3,3]=[-1.,-.4,.42]
        rt.observe=lambda index:pose.copy()
        rt.hand_targets=lambda who,target:target
        rt.event=lambda *a,**kw:None
        calls=[];executed=[]
        def plan(who,target,**kwargs):
            calls.append(kwargs)
            if len(calls)==1 or kwargs['seconds']==4.:
                raise RelayFault('fixture infeasible final height')
            return [{'lift_joint':.3}]
        def hands(*args,**kwargs):
            executed.append(args)
            yield
        rt.plan_hands=plan;rt.hands=hands
        with self.assertRaisesRegex(RelayFault,'No continuous handoff carry and deposit alternative'):
            list(rt.prepare_handoff_carry('A',6))
        self.assertEqual(executed,[])
        self.assertGreater(sum(c['seconds']==4. for c in calls),0)


if __name__=='__main__':
    unittest.main()
