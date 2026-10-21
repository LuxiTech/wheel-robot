"""Relay state transitions, physical confirmation and shared-lift IK regression."""
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from relay_task import (HandoffCoordinator, HandoffObservation, RelayFault,
                        RelayInventory, source_order, target_order, source_handoff_slot)


def observation(left=(),right=(),stable=(),clear=True,home=True,valid=True):
    return HandoffObservation(tuple(left),tuple(right),frozenset(stable),clear,home,valid)


class RelayTests(unittest.TestCase):
    def setUp(self):
        self.h = HandoffCoordinator()

    def test_unknown_or_intrusion_blocks_entry(self):
        for o in (observation(valid=False),observation(clear=False),observation(home=False)):
            self.h.observe(o)
            self.assertIsNone(self.h.acquire('A',12))

    def test_left_occupied_waits_and_right_empty_waits(self):
        self.h.observe(observation(left=[12],stable=[12]))
        self.assertIsNone(self.h.acquire('A',14))
        self.assertIsNone(self.h.acquire('B',12))

    def test_full_right_blocks_pusher_but_allows_left_buffer(self):
        self.h.observe(observation(right=[12],stable=[12]))
        self.assertIsNotNone(self.h.acquire('A',14))
        self.h.observe(observation(left=[14],right=[12],stable=[12,14]))
        self.h.commit(self.h.action)
        self.assertIsNone(self.h.acquire('PUSHER',14))
        self.assertIsNotNone(self.h.acquire('B',12))

    def test_right_deposit_waits_for_right_and_commits_only_on_that_side(self):
        self.h.observe(observation(right=[14],stable=[14]))
        self.assertIsNone(self.h.acquire('A',12,slot=1))
        self.h.observe(observation(left=[15],stable=[15]))
        action=self.h.acquire('A',12,slot=1)
        self.assertIsNotNone(action)
        self.assertEqual(self.h.status()['deposit_slot'],1)
        self.assertIsNone(self.h.acquire('PUSHER',15))
        self.assertIsNone(self.h.acquire('B',12))
        for o in (observation(left=[12],stable=[12]),
                  observation(left=[15],right=[12],stable=[15]),
                  observation(right=[12],stable=[12],clear=False)):
            self.h.observe(o)
            with self.assertRaises(RelayFault):
                self.h.commit(action)
        self.h.observe(observation(left=[15],right=[12],stable=[15,12]))
        self.assertTrue(self.h.commit(action))
        self.assertIsNone(self.h.deposit_slot)
        self.assertIsNone(self.h.acquire('PUSHER',15))
        b=self.h.acquire('B',12)
        self.h.observe(observation(left=[15],stable=[15]))
        self.h.commit(b)
        self.assertIsNotNone(self.h.acquire('PUSHER',15))

    def test_all_layers_follow_diagram_and_left_deposit(self):
        initial=json.loads((ROOT/'assets/generated/relay/initial_state.json').read_text())['crates']
        scene=json.loads((ROOT/'config/relay_config.json').read_text())
        queue=source_order(initial)
        self.assertEqual(queue,[12,14,17,15,16,13,6,8,11,9,10,7,0,2,5,3,4,1])
        self.assertEqual([source_handoff_slot(scene,initial[i]) for i in queue],
                         [0]*18)
        inv=RelayInventory(initial,18)
        for box in queue:
            inv.transfer(box,'SOURCE','A')
            side=source_handoff_slot(scene,initial[box])
            inv.transfer(box,'A','R' if side else 'L')
            if not side:
                inv.transfer(box,'L','PUSHER')
                inv.transfer(box,'PUSHER','R')
            self.assertEqual(inv.reserve_target(box),len(inv.completed))
            inv.transfer(box,'R','B')
            inv.transfer(box,'B','TARGET')
        self.assertEqual(inv.completed,queue)

    def test_runtime_a_uses_selected_slot_for_lock_pose_and_ownership(self):
        from relay_runtime import RelayRuntime
        initial=json.loads((ROOT/'assets/generated/relay/initial_state.json').read_text())['crates']
        for index,slot in ((12,0),(16,0)):
            with self.subTest(index=index):
                rt=RelayRuntime.__new__(RelayRuntime)
                rt.c=json.loads((ROOT/'config/relay_config.json').read_text())
                rt.settings={'transit_y':-.7,'source_rear_aisle_y':2.1}
                rt.initial=initial
                rt.skill_test=None
                rt.inventory=RelayInventory(initial,18)
                rt.inventory.queue=[index]
                rt.handoff=HandoffCoordinator()
                rt.handoff.observe(observation())
                rt.time=0
                rt.event=lambda *a,**kw: None
                rt.base=lambda who: [-1.,-.7,0,0]
                def noop(*args,**kwargs):
                    yield
                for name in ('stow','navigate','dock','pick','retreat','prepare_handoff_carry','establish_upright_carry'):
                    setattr(rt,name,noop)
                placed=[]
                def place(who,box,position):
                    self.assertEqual(rt.handoff.deposit_slot,slot)
                    placed.append(position)
                    yield
                rt.place=place
                rt.refresh_handoff=lambda dt: rt.handoff.observe(observation(
                    left=[index] if slot==0 else [],right=[index] if slot==1 else [],stable=[index]))
                list(rt.run_a())
                self.assertEqual(placed,[[*rt.c['handoff']['slots'][slot],rt.c['handoff']['size'][2]]])
                self.assertEqual(rt.inventory.owners[index],'L' if slot==0 else 'R')
                self.assertIsNone(rt.handoff.owner)

    def test_complete_transaction_and_idempotence(self):
        self.h.observe(observation())
        a=self.h.acquire('A',12)
        self.assertIsNone(self.h.acquire('B',12))
        with self.assertRaises(RelayFault):
            self.h.commit(a)
        self.h.observe(observation(left=[12],stable=[12]))
        self.assertTrue(self.h.commit(a))
        self.assertFalse(self.h.commit(a))
        p=self.h.acquire('PUSHER',12)
        self.h.observe(observation(home=False))  # In transit: both sensors can be empty.
        self.assertIsNone(self.h.acquire('A',14))
        with self.assertRaises(RelayFault):
            self.h.commit(p)
        self.h.observe(observation(right=[12],stable=[12]))
        self.h.commit(p)
        b=self.h.acquire('B',12)
        self.h.observe(observation())
        self.h.commit(b)
        self.assertEqual(self.h.transfers,1)

    def test_stale_commit_does_not_release_new_owner(self):
        self.h.observe(observation())
        action=self.h.acquire('A',1)
        with self.assertRaises(RelayFault):
            self.h.commit('A:1:999')
        self.assertEqual(self.h.action,action)

    def test_fault_retains_lock_and_reset_clears_everything(self):
        self.h.observe(observation())
        action=self.h.acquire('A',12)
        self.h.fail('lost localization')
        with self.assertRaises(RelayFault):
            self.h.commit(action)
        self.assertEqual(self.h.owner,'A')
        self.h.reset()
        self.assertIsNone(self.h.owner)
        self.assertIsNone(self.h.fault)
        self.assertFalse(self.h.observation.valid)

    def test_multiple_occupants_fail_closed(self):
        self.h.observe(observation(left=[1,2],stable=[1,2]))
        self.assertIsNotNone(self.h.fault)
        self.assertIsNone(self.h.acquire('PUSHER',1))

    def test_order_and_ownership_conservation(self):
        initial=json.loads((ROOT/'assets/generated/relay/initial_state.json').read_text())['crates']
        scene=json.loads((ROOT/'config/relay_config.json').read_text())
        self.assertEqual(source_order(initial)[:6],[12,14,17,15,16,13])
        slots=target_order(scene)
        self.assertEqual([(s['column'],s['row']) for s in slots[:6]],[(1,1),(1,0),(0,1),(0,0),(2,1),(2,0)])
        inv=RelayInventory(initial,18)
        box=inv.queue[0]
        for old,new in zip(['SOURCE','A','L','PUSHER','R','B'],['A','L','PUSHER','R','B','TARGET']):
            if old=='R':
                self.assertEqual(inv.reserve_target(box),0)
                self.assertEqual(inv.reserve_target(box),0)
            inv.transfer(box,old,new)
        self.assertEqual(inv.completed,[box])
        self.assertEqual(len(inv.owners),18)
        with self.assertRaises(RelayFault):
            inv.transfer(box,'B','TARGET')


class WholeBodyTests(unittest.TestCase):
    def test_shared_lift_and_fixed_base_recover_known_pose(self):
        import numpy as np
        from whole_body_control import WholeBodyIK
        ik=WholeBodyIK(ROOT/'assets/robots/relay/relay_robot.urdf')
        cfg=json.loads((ROOT/'config/relay_robot_config.json').read_text())
        seed={n:cfg['home'].get(n,0.) for n in ik.names}
        targets={s:arm.fk([seed[n] for n in arm.names],.31) for s,arm in ik.arms.items()}
        result=ik.solve(targets,np.zeros(4),seed,lift=.31)
        self.assertTrue(result.success)
        self.assertAlmostEqual(result.joints['lift_joint'],.31,places=7)
        np.testing.assert_allclose(result.base,0,atol=1e-8)

    def test_unreachable_target_is_rejected(self):
        import numpy as np
        from whole_body_control import WholeBodyIK
        ik=WholeBodyIK(ROOT/'assets/robots/relay/relay_robot.urdf')
        seed={n:0.1 for n in ik.names}
        t=np.eye(4)
        t[:3,3]=[8,0,5]
        self.assertFalse(ik.solve({'left':t},np.zeros(4),seed,attempts=1).success)

    def test_synchronized_quintic_respects_slowest_axis(self):
        from whole_body_control import minimum_duration,smoothstep
        self.assertAlmostEqual(minimum_duration({'lift':0.,'arm':0.},{'lift':.4,'arm':1.}, {'lift':.05,'arm':.6}),15.)
        self.assertEqual(smoothstep(0),0.)
        self.assertEqual(smoothstep(1),1.)


class PoseTests(unittest.TestCase):
    def test_six_dof_fit_recovers_rigid_transform(self):
        import numpy as np
        from scipy.spatial.transform import Rotation
        from relay_perception import CratePoseEstimator
        # Sample three noncoplanar visible surfaces of an asymmetric fixture.
        colliders=[dict(position=[0,0,.11],size=[.4,.3,.22])]
        rng=np.random.default_rng(7)
        local=rng.uniform([-.2,-.15,0],[.2,.15,.22],size=(300,3))
        local[:100,0]=-.2
        local[100:200,1]=-.15
        local[200:,2]=.22
        rotation=Rotation.from_euler('xyz',[.04,-.03,.06]).as_matrix()
        position=np.array([.8,.5,.57])
        points=local @ rotation.T+position
        prior=np.eye(4)
        prior[:3,3]=position+[.009,-.005,.007]
        result=CratePoseEstimator(colliders).fit(points,prior)
        np.testing.assert_allclose(result.pose[:3,3],position,atol=.002)
        self.assertLess(np.linalg.norm(Rotation.from_matrix(result.pose[:3,:3] @ rotation.T).as_rotvec()),.015)

    def test_empty_or_single_plane_depth_rejected(self):
        import numpy as np
        from relay_perception import CratePoseEstimator
        fit=CratePoseEstimator([dict(position=[0,0,.11],size=[.4,.3,.22])])
        with self.assertRaises(RelayFault):
            fit.fit([],np.eye(4))
        points=np.array([[x,y,.22] for x in np.linspace(-.1,.1,10) for y in np.linspace(-.1,.1,10)])
        with self.assertRaises(RelayFault):
            fit.fit(points,np.eye(4))


class PusherInterlockTests(unittest.TestCase):
    def make_pusher(self):
        import numpy as np
        from relay_pusher import HandoffPusher
        class Attribute:
            value=0.
            def Set(self,value):
                self.value=value
        class Slider:
            position=np.zeros(3)
            def get_world_pose(self):
                return self.position.copy(),np.array([1.,0,0,0])
        p=HandoffPusher.__new__(HandoffPusher)
        p.target_attr=Attribute()
        p.slider=Slider()
        p.home=np.zeros(3)
        p.stroke=.68
        p.settings=dict(settle_seconds=.5,speed=.12,return_speed=.2)
        p.sensor_period=.05
        p.start_permitted=lambda i:False
        p.motion_permitted=lambda:True
        p.occupants=lambda slot: [12] if slot==0 else []
        p.ready=lambda i:True
        p.reset()
        return p

    def test_robot_lock_prevents_start_then_release_permits(self):
        p=self.make_pusher()
        for _ in range(20):
            p.update(.05)
        self.assertEqual(p.state,'WAITING')
        self.assertEqual(p.target,0.)
        p.start_permitted=lambda i:True
        for _ in range(20):
            p.update(.05)
        self.assertEqual(p.state,'PUSHING')
        self.assertGreater(p.target,0.)

    def test_intrusion_holds_push_and_return(self):
        p=self.make_pusher()
        p.state='PUSHING'
        p.selected=12
        p.target=.2
        p.slider.position[0]=.15
        p.motion_permitted=lambda:False
        p.update(.05)
        self.assertEqual(p.state,'BLOCKED')
        self.assertEqual(p.target,.15)
        p.state='RETURNING'
        p.update(.05)
        self.assertEqual(p.target,.15)


class ContactTrackerTests(unittest.TestCase):
    def test_actor_paths_decode_once_and_reset_invalidates(self):
        from relay_contacts import ActorPathCache
        calls=[]
        def decode(actor):
            calls.append(actor)
            return '/World/Crates/Crate_'+str(actor)
        cache=ActorPathCache(decode)
        self.assertEqual(cache(12),cache(12))
        self.assertEqual(calls,[12])
        cache.clear()
        self.assertEqual(cache(12),'/World/Crates/Crate_12')
        self.assertEqual(calls,[12,12])

    def test_robot_table_contact_is_tracked_without_a_crate(self):
        from relay_contacts import ContactTracker
        tracker=ContactTracker()
        link='/World/Robots/Robot_A/Geometry/openarm_left_link1'
        table='/World/Collisions/HandoffTop'
        tracker.update(link,table,1,2)
        self.assertIn(table,tracker.actors[link])
        tracker.update(link,table,1,2,lost=True)
        self.assertEqual(tracker.actors[link],set())

    def test_one_shape_lost_does_not_remove_other_support_contacts(self):
        from relay_contacts import ContactTracker
        tracker=ContactTracker()
        box='/World/Crates/Crate_12'
        support='/World/Crates/Crate_06'
        tracker.update(box,support,1,2)
        tracker.update(box,support,3,4)
        tracker.update(box,support,1,2,lost=True)
        self.assertIn(support,tracker.actors[box])
        tracker.update(support,box,4,3,lost=True)
        self.assertNotIn(support,tracker.actors[box])
        self.assertNotIn(box,tracker.actors[support])

    def test_persistent_events_are_idempotent_and_reset_clears_shapes(self):
        from relay_contacts import ContactTracker
        tracker=ContactTracker()
        box='/World/Crates/Crate_12'
        for _ in range(3):
            tracker.update(box,'finger',1,2)
        tracker.update(box,'finger',1,2,lost=True)
        self.assertEqual(tracker.actors[box],set())
        tracker.update(box,'finger',1,2)
        actors=tracker.actors
        tracker.clear()
        self.assertEqual(actors,{})
        self.assertEqual(tracker.pairs,{})


class RuntimeGuardTests(unittest.TestCase):
    def test_source_retreat_checks_load_extent_before_sideways_motion(self):
        import numpy as np
        from relay_runtime import RelayRuntime
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.c={'crate':{'size':[.4,.3,.22]}}
        rt.initial=[{'position':[-2.1,.95,.57]}]
        pose=np.eye(4)
        pose[:3,3]=[-1.7,1.243,.664]
        rt.observe=lambda index:pose
        with self.assertRaisesRegex(RelayFault,'source rear edge'):
            rt.verify_source_rear_clearance(17)
        pose[1,3]=1.49
        rt.verify_source_rear_clearance(17)

    def test_placement_servo_stops_on_slip_or_missing_contact(self):
        import numpy as np
        from relay_runtime import RelayRuntime
        rt=RelayRuntime.__new__(RelayRuntime)
        observed=np.eye(4)
        observed[2,3]=.8
        rt.observe=lambda index:observed.copy()
        rt.gripping=lambda *args:True
        rt.grasp_transforms={}
        rt.actual_hands=lambda *args,**kwargs:{'left':np.eye(4),'right':np.eye(4)}
        rt.hand_targets=lambda who,pose:pose
        def slip(*args,**kwargs):
            observed[0,3]+=.08
            yield
        rt.hands=slip
        target=observed.copy()
        target[2,3]=.2
        worker=rt.lower_held_box('B',12,target)
        next(worker)
        with self.assertRaisesRegex(RelayFault,'tracking error'):
            next(worker)
        rt.gripping=lambda *args:False
        with self.assertRaisesRegex(RelayFault,'bilateral contact'):
            next(rt.lower_held_box('B',12,target))

    def test_loaded_navigation_does_not_accumulate_feedback_sag(self):
        import numpy as np
        from types import SimpleNamespace
        from relay_runtime import RelayRuntime
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.time=0.
        rt.station_poses={}
        rt.held={'A':15}
        rt.settings={'base_speed_loaded':.15}
        sent={'lift_joint':.3,'arm':.5}
        rt.controls={'Robot_A':SimpleNamespace(sent=sent,set_twist=lambda *v:None)}
        rt.base=lambda who:np.zeros(4)
        rt.joints=lambda who:{'lift_joint':.29,'arm':.45}
        def fk(q,lift):
            pose=np.eye(4)
            pose[2,3]=lift+q[0]
            return pose
        rt.ik=SimpleNamespace(names=list(sent),
            arms={'left':SimpleNamespace(names=['arm'],fk=fk)},
            solve=lambda *a,**kw:SimpleNamespace(success=True))
        goals=[]
        def hands(who,targets,**kwargs):
            goals.append(targets['left'][2,3])
            yield
        rt.hands=hands
        for _ in range(4):
            worker=rt.navigate('A',[1.,0.])
            next(worker)
            worker.close()
        np.testing.assert_allclose(goals,[.8]*4)

    def test_handoff_clearance_uses_lowest_corner_of_tilted_box(self):
        import numpy as np
        from scipy.spatial.transform import Rotation
        from relay_runtime import RelayRuntime
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.c={'crate':{'size':[.4,.3,.22]},'handoff':{'size':[1.3,.72,.75]}}
        pose=np.eye(4)
        pose[:3,:3]=Rotation.from_euler('x',20,degrees=True).as_matrix()
        pose[2,3]=.84  # Origin clears the guide; the lower corner does not.
        rt.observe=lambda index:pose
        with self.assertRaisesRegex(RelayFault,'Insufficient load clearance'):
            rt.verify_handoff_clearance(12)
        pose[2,3]=.90
        rt.verify_handoff_clearance(12)

    def test_arm_contact_with_tabletop_stops_before_further_motion(self):
        from relay_runtime import RelayRuntime
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.contacts={'/World/Robots/Robot_A/Geometry/openarm_left_link1':{'/World/Collisions/HandoffTop'}}
        with self.assertRaisesRegex(RelayFault,'contacted the handoff tabletop'):
            rt.validate_state()

    def make_loaded_runtime(self):
        from relay_runtime import RelayRuntime
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.held={'A':12,'B':None}
        rt.releasing={'A':False,'B':False}
        rt.contact_loss={'A':0.,'B':0.}
        rt.gripping=lambda *args:False
        rt.supported=lambda index:True
        return rt

    def test_contact_loss_while_carrying_faults_after_debounce(self):
        rt=self.make_loaded_runtime()
        rt.monitor_loads(.3)
        with self.assertRaises(RelayFault):
            rt.monitor_loads(.3)

    def test_intentional_release_requires_continuous_support(self):
        rt=self.make_loaded_runtime()
        rt.releasing['A']=True
        rt.monitor_loads(2.)
        self.assertEqual(rt.contact_loss['A'],0.)
        rt.supported=lambda index:False
        with self.assertRaises(RelayFault):
            rt.monitor_loads(.1)

    def test_knuckle_or_single_finger_contact_is_not_a_grasp(self):
        from relay_runtime import RelayRuntime
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.initial=[{'path':'crate'}]
        prefix='/World/Robots/Robot_A/Geometry/openarm_left_'
        rt.contacts={'crate':{prefix+'ee_base_link'}}
        self.assertFalse(rt.gripping('A',0,'left'))
        rt.contacts['crate']={prefix+'ee_link1'}
        self.assertFalse(rt.gripping('A',0,'left'))
        rt.contacts['crate'].add(prefix+'ee_link2')
        self.assertTrue(rt.gripping('A',0,'left'))
        self.assertFalse(rt.gripping('A',0,'right'))

    def test_invalid_joint_and_zero_quaternion_fail_closed(self):
        import numpy as np
        from types import SimpleNamespace
        from relay_runtime import RelayRuntime
        rt=RelayRuntime.__new__(RelayRuntime)
        rt.contacts={}
        rt.physics_valid=True
        rt.controls={w:SimpleNamespace(specs={'joint':dict(kind='arm',lower=-1.,upper=1.)}) for w in ('Robot_A','Robot_B')}
        rt.joints=lambda who:{'joint':10.}
        with self.assertRaises(RelayFault):
            rt.validate_state()
        self.assertFalse(rt.physics_valid)
        rt.physics_valid=True
        rt.joints=lambda who:{'joint':0.}
        rt.robots={w:SimpleNamespace(get_world_pose=lambda:(np.zeros(3),np.zeros(4))) for w in ('Robot_A','Robot_B')}
        with self.assertRaises(RelayFault):
            rt.validate_state()
        self.assertFalse(rt.physics_valid)


if __name__=='__main__':
    unittest.main()
