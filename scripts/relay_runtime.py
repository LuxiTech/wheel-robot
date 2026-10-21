"""Coroutine-based physical relay. Only initialization may move rigid-body poses.

The commissioning sensor is explicitly ground truth. Every manipulation action
uses articulation drives and verified rigid-body contacts, never an attachment.
"""
import json
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from common import ROOT, OUTPUTS
from whole_body_control import WholeBodyIK, transform, minimum_duration, smoothstep
from relay_layout import pallet_yaw, rotate_xy, world_half_extent
from relay_task import (RelayFault, RelayInventory, HandoffCoordinator,
                        HandoffObservation, target_order, source_handoff_slot, source_handoff_far)


def wrap(angle):
    return (angle+math.pi) % (2*math.pi)-math.pi


class RelayRuntime:
    def __init__(self, scene, initial, robots, controls, crates, pusher, contacts,
                 *, count=18, perception='ground_truth', skill_test=None, handoff_target=0, source_box=None,
                 settings_overrides=None):
        self.c, self.initial = scene, initial['crates']
        self.settings = json.loads((ROOT/'config/relay_demo_config.json').read_text())
        self.settings.update(settings_overrides or {})
        self.robots, self.controls, self.crates = robots, controls, crates
        self.pusher, self.contacts = pusher, contacts
        self.count, self.perception, self.skill_test = count, perception, skill_test
        self.handoff_target=handoff_target
        self.source_box=source_box
        self.ik = WholeBodyIK(ROOT/'assets/robots/relay/relay_robot.urdf', self.settings['tcp'],
                             backend=self.settings.get('ik_backend','cuda'),device=self.settings.get('ik_device','cuda:0'))
        # Simulation gain tuning only. Keep original URDF force/torque and
        # velocity limits; improve load tracking without inflating actuator power.
        for ctl in controls.values():
            for name,spec in ctl.specs.items():
                kind=spec['kind']
                if kind in ('arm','gripper'):
                    ctl.drives[name].GetStiffnessAttr().Set(self.settings[kind+'_stiffness']*math.pi/180)
                    ctl.drives[name].GetDampingAttr().Set(self.settings[kind+'_damping']*math.pi/180)
                    if kind=='gripper':
                        ctl.drives[name].GetMaxForceAttr().Set(min(spec['effort'],self.settings['gripper_torque_limit']))
        from pxr import UsdShade,UsdPhysics,PhysxSchema
        stage=next(iter(controls.values())).drives['lift_joint'].GetPrim().GetStage()
        # The importer instances collision meshes. Ordinary Traverse skips
        # their mesh proxies, so per-finger friction/shape edits otherwise
        # silently miss every finger collider.
        for prim in list(stage.Traverse()):
            if (prim.IsInstance() and 'collision' in prim.GetName() and
                    any(part.endswith(('_ee_link1','_ee_link2')) for part in str(prim.GetPath()).split('/'))):
                prim.SetInstanceable(False)
        material=UsdShade.Material.Define(stage,'/World/Materials/RelayFingerRubber')
        physics_material=UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_material.CreateStaticFrictionAttr(self.settings['finger_static_friction'])
        physics_material.CreateDynamicFrictionAttr(self.settings['finger_dynamic_friction'])
        physics_material.CreateRestitutionAttr(0.)
        self.finger_collision_overrides=[]
        for prim in stage.Traverse():
            path=str(prim.GetPath())
            if '/Robots/' in path and prim.HasAPI(UsdPhysics.RigidBodyAPI) and ('openarm_' in prim.GetName() or prim.GetName()=='lift_carriage_link'):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
            if any(part.endswith(('_ee_link1', '_ee_link2')) for part in path.split('/')):
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material,materialPurpose='physics')
                    collision=PhysxSchema.PhysxCollisionAPI.Apply(prim)
                    collision.CreateContactOffsetAttr(.001)
                    collision.CreateRestOffsetAttr(0.)
                    if self.settings.get('pinch_collision_approximation')=='convexDecomposition' and prim.GetTypeName()=='Mesh':
                        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexDecomposition')
                        decomposition=PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
                        decomposition.CreateMaxConvexHullsAttr(64)
                        decomposition.CreateHullVertexLimitAttr(64)
                        decomposition.CreateVoxelResolutionAttr(100000)
                        decomposition.CreateErrorPercentageAttr(.5)
                        decomposition.CreateMinThicknessAttr(.0005)
                        decomposition.CreateShrinkWrapAttr(True)
                        self.finger_collision_overrides.append(path)
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateMaxDepenetrationVelocityAttr(1.)
        if self.settings.get('pinch_collision_approximation')=='convexDecomposition':
            for robot_name in controls:
                for side in ('left','right'):
                    for finger in (1,2):
                        if not any(f'/{robot_name}/' in path and f'/openarm_{side}_ee_link{finger}/' in path
                                   for path in self.finger_collision_overrides):
                            raise RelayFault(f'Missing native finger collision configuration: {robot_name} {side} {finger}')
        self.targets = target_order(scene)
        self.pusher.start_permitted = self.allow_push
        self.pusher.sensor_period = .05
        self.pusher.motion_permitted = lambda: self.handoff.owner == 'PUSHER' and self.arms_clear()
        self.reset()

    def reset(self):
        self.inventory = RelayInventory(self.initial, self.count)
        if self.source_box is not None:
            self.inventory.queue=[self.source_box]
            self.inventory.owners={self.source_box:'SOURCE'}
        if self.skill_test=='handoff':
            self.inventory.owners[self.inventory.queue[0]]='R'
            self.inventory.target_reservations[self.inventory.queue[0]]=self.handoff_target
        self.handoff = HandoffCoordinator()
        self.states = {'A': 'IDLE', 'B': 'IDLE'}
        self.workers = {}
        self.active = False
        self.finished = False
        self.error = None
        self.time = 0.
        self.wall_started = time.perf_counter()
        self.events = []
        self.telemetry = []
        self.held = {'A': None, 'B': None}
        self.upright_loads = {}
        self.releasing = {'A': False, 'B': False}
        self.placement_single_hand = {}
        self.stable = {}
        self.placement_stable = {}
        self.push_action = None
        self.push_started = False
        self.last_telemetry = -1.
        self.pending_start = False
        self.stop_requested = False
        self.last_handoff_update = -1.
        self.last_progress = -10.
        self.last_worker_update = -1.
        self.contact_loss = {'A':0.,'B':0.}
        self.synchronized_samples = 0
        self.station_poses = {}
        self.last_dock = {}
        self.grasp_object_yaw = {}
        self.hand_goals = {}
        self.grasp_transforms = {'A':None,'B':None}
        self.anchored_hands = {'A':{},'B':{}}
        self.pose_symmetry = {}
        self.grasp_base_yaw = {}
        self.grasp_forward = {w:{s:self.settings['grasp_forward_offset'] for s in ('left','right')} for w in ('A','B')}
        self.working_boxes={'A':None,'B':None}
        self.physics_valid=True
        if self.perception in ('depth','rgbd'):
            if self.perception == 'rgbd':
                from relay_rgbd_perception import ModelRGBDPerception
                if isinstance(getattr(self,'depth_sensor',None),ModelRGBDPerception):
                    self.depth_sensor.reset()
                else:
                    self.depth_sensor = ModelRGBDPerception(self)
            else:
                from relay_perception import SimDepthPerception
                self.depth_sensor = SimDepthPerception(self)
            if self.skill_test=='handoff':
                prior=np.eye(4)
                prior[:3,3]=[*self.c['handoff']['slots'][1],self.c['handoff']['size'][2]]
                self.depth_sensor.set_prior(self.inventory.queue[0],prior)

    def event(self, who, state, **data):
        self.states[who] = state
        row = dict(t=round(self.time, 3), robot=who, state=state, **data)
        self.events.append(row)
        print('RELAY_EVENT', json.dumps(row), flush=True)

    def start(self):
        if self.active or self.finished:
            return
        self.active = True
        self.workers = {'A': self.run_a(), 'B': self.run_b()}
        self.event('A', 'START', perception=self.perception)
        self.event('B', 'WAIT_RIGHT')

    def request_stop(self):
        self.stop_requested = True

    def base(self, who):
        p, q = self.robots['Robot_'+who].get_world_pose()
        yaw = Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_euler('xyz')[2]
        return np.r_[p, yaw]

    def joints(self, who):
        robot = self.robots['Robot_'+who]
        return dict(zip(robot.dof_names, robot.get_joint_positions()))

    def actual_hands(self, who, *, commanded=False):
        b = self.base(who)
        root = np.eye(4)
        root[:3, 3] = b[:3]
        root[:3, :3] = Rotation.from_euler('z', b[3]).as_matrix()
        joints = self.controls['Robot_'+who].sent if commanded else self.joints(who)
        return {s: root @ a.fk([joints[n] for n in a.names], joints['lift_joint'])
                for s,a in self.ik.arms.items()}

    def observe(self, index):
        if self.perception != 'ground_truth':
            for who,held in self.held.items():
                if held==index and self.grasp_transforms[who] is not None:
                    # Propagate the search ROI using encoder FK and the known
                    # grasp; do not read the carried object's simulator pose.
                    prior=self.actual_hands(who)['left'] @ np.linalg.inv(self.grasp_transforms[who]['left'])
                    if self.pose_symmetry.get(index,False):
                        prior[:3,:3]=prior[:3,:3] @ Rotation.from_euler('z',math.pi).as_matrix()
                    self.depth_sensor.set_prior(index,prior)
            pose=self.depth_sensor.observe(index)
        else:
            p,q = self.crates[index].get_world_pose()
            pose=transform(p,q)
        if self.pose_symmetry.get(index,False):
            pose=pose.copy()
            pose[:3,:3]=pose[:3,:3] @ Rotation.from_euler('z',math.pi).as_matrix()
        return pose

    def box_contacts(self, index):
        return self.contacts.get(self.initial[index]['path'], set())

    def gripping(self, who, index, side=None):
        prefix = '/World/Robots/Robot_'+who
        paths=[p for p in self.box_contacts(index) if prefix in p and any(p.endswith(f'_ee_link{i}') for i in (1,2))]
        if side is None:
            return bool(paths)
        return all(any(p.endswith(f'openarm_{side}_ee_link{i}') for p in paths)
                   for i in (1,2))

    def supported(self, index):
        return any('/Collisions/' in p or '/Crates/' in p for p in self.box_contacts(index))

    def arms_clear(self):
        # Include all arm-link centres and the held crate bounds, not just TCPs.
        tx,ty = self.c['handoff']['center']
        sx,sy,h = self.c['handoff']['size']
        for who in ('A','B'):
            b = self.base(who)
            r = Rotation.from_euler('z',b[3]).as_matrix()
            joints = self.joints(who)
            for arm in self.ik.arms.values():
                end, points = arm.fk([joints[n] for n in arm.names],joints['lift_joint'],links=True)
                for p in np.vstack([points, end[:3,3]]) @ r.T + b[:3]:
                    if abs(p[0]-tx) < sx/2+.12 and abs(p[1]-ty) < sy/2+.12 and p[2] > h-.15:
                        return False
            index = self.held[who]
            if index is not None:
                p,_ = self.crates[index].get_world_pose()
                if abs(p[0]-tx) < sx/2+.22 and abs(p[1]-ty) < sy/2+.17 and p[2] > h-.25:
                    return False
        return True

    def refresh_handoff(self, dt):
        stable = set()
        for i,crate in enumerate(self.crates):
            still = np.linalg.norm(crate.get_linear_velocity()) < .02 and np.linalg.norm(crate.get_angular_velocity()) < .05
            self.stable[i] = self.stable.get(i,0.)+dt if still else 0.
            if self.stable[i] >= .5 and self.supported(i):
                stable.add(i)
        actual = float(self.pusher.slider.get_world_pose()[0][0]-self.pusher.home[0])
        self.handoff.observe(HandoffObservation(tuple(self.pusher.occupants(0)), tuple(self.pusher.occupants(1)),
                             frozenset(stable), self.arms_clear(),
                             self.pusher.state == 'WAITING' and abs(actual) < .006))

    def validate_state(self):
        for actor,others in self.contacts.items():
            if '/Robots/' in actor and any('/Collisions/HandoffTop' in other for other in others):
                raise RelayFault(f'Robot link contacted the handoff tabletop: {actor}')
        for who in ('A','B'):
            q=self.joints(who)
            for name,spec in self.controls['Robot_'+who].specs.items():
                value=float(q[name])
                if not math.isfinite(value) or (spec['kind']!='wheel' and
                        (value<spec['lower']-.12 or value>spec['upper']+.12)):
                    self.physics_valid=False
                    raise RelayFault(f'{who} invalid actuator state: {name}={value}')
            p,quat=self.robots['Robot_'+who].get_world_pose()
            if not np.isfinite(p).all() or not np.isfinite(quat).all() or np.linalg.norm(quat)<.5 or abs(p[2])>.2:
                self.physics_valid=False
                raise RelayFault(f'{who} invalid mobile-base state')
        for index,spec in enumerate(self.initial):
            if index in self.working_boxes.values() or self.inventory.owners.get(index,'SOURCE')!='SOURCE':
                continue
            if self.skill_test=='direct' and index>12:
                continue
            p,_=self.crates[index].get_world_pose()
            tolerance=self.settings['position_tolerance'] if spec.get('fixture_target') else .025
            if np.linalg.norm(p-np.asarray(spec['position']))>tolerance:
                location='Preplaced target' if spec.get('fixture_target') else 'Unselected source'
                raise RelayFault(f'{location} crate {index} was disturbed')

    def allow_push(self, index):
        if not self.active or self.error:
            return False
        if self.handoff.owner == 'PUSHER':
            return self.handoff.box == index
        if self.inventory.owners.get(index) != 'L':
            return False
        action = self.handoff.acquire('PUSHER',index)
        if action:
            self.inventory.transfer(index,'L','PUSHER')
            self.push_action = action
            self.push_started = False
            self.events.append(dict(t=self.time, robot='PUSHER', state='TRANSFER', box=index))
            return True
        return False

    def monitor_loads(self, dt):
        for who,index in self.held.items():
            if index is None:
                self.contact_loss[who]=0.
                continue
            if (who=='A' and getattr(self,'upright_loads',{}).get(who)==index and
                    self.observe(index)[2,2]<math.cos(math.radians(self.settings.get('a_carry_tilt_tolerance_degrees',10.)))):
                raise RelayFault('A load exceeded upright transport tolerance')
            single=getattr(self,'placement_single_hand',{}).get(who)
            if single is not None:
                if not self.placement_supported(index,single['position']):
                    raise RelayFault(f'{who} lost bottom support during sequential placement')
                if not self.releasing[who] and not self.gripping(who,index,single['side']):
                    raise RelayFault(f'{who} lost remaining-hand contact during sequential placement')
                self.contact_loss[who]=0.
                continue
            if self.releasing[who]:
                if not self.supported(index):
                    raise RelayFault(f'{who} lost support during release of box {index}')
                self.contact_loss[who]=0.
                continue
            both=all(self.gripping(who,index,s) for s in ('left','right'))
            self.contact_loss[who]=0. if both else self.contact_loss[who]+dt
            if self.contact_loss[who]>.5:
                raise RelayFault(f'{who} lost grasp contact while carrying box {index}')

    def tick(self, dt):
        if not self.active:
            return
        self.time += dt
        try:
            if self.stop_requested:
                raise RelayFault('Operator requested stop; reset required before restart')
            if self.time-self.last_handoff_update >= .1:
                self.validate_state()
                sensor_dt=self.time-max(0.,self.last_handoff_update)
                self.refresh_handoff(sensor_dt)
                self.last_handoff_update = self.time
                self.monitor_loads(sensor_dt)
            if self.handoff.fault:
                raise RelayFault(self.handoff.fault)
            if self.pusher.state == 'FAULT':
                raise RelayFault('Handoff pusher fault')
            if self.time > self.settings['max_sim_seconds']:
                raise RelayFault('Demo time limit exceeded')
            if self.push_action:
                if self.pusher.state != 'WAITING':
                    self.push_started = True
                elif self.push_started:
                    box = self.handoff.box
                    # Pusher reaches home between the slower sensor updates.
                    # Refresh completion evidence before committing the lock.
                    self.refresh_handoff(0.)
                    self.handoff.commit(self.push_action)
                    self.inventory.transfer(box,'PUSHER','R')
                    if self.perception in ('depth','rgbd'):
                        prior=np.eye(4)
                        prior[:3,3]=[*self.c['handoff']['slots'][1],self.c['handoff']['size'][2]]
                        self.depth_sensor.set_prior(box,prior)
                    self.push_action = None
            # B goes first so a full R is serviced before A reserves L again.
            if self.time-self.last_worker_update >= .05:
                self.last_worker_update=self.time
                for who,goal in self.station_poses.items():
                    b=self.base(who)
                    delta=goal[:2]-b[:2]
                    heading=wrap(goal[2]-b[3])
                    supporting=who in self.placement_single_hand
                    gain=6. if supporting else 2.5
                    deadband=.001 if supporting else .003
                    min_speed=.015 if supporting else .025
                    speed=np.clip(gain*delta,-.08,.08)
                    for axis in (0,1):
                        if abs(delta[axis])>deadband and abs(speed[axis])<min_speed:
                            speed[axis]=math.copysign(min_speed,delta[axis])
                        elif abs(delta[axis])<=deadband:
                            speed[axis]=0.
                    local=Rotation.from_euler('z',-b[3]).as_matrix()[:2,:2] @ speed
                    yaw_deadband=.0025 if supporting else .005
                    min_turn=.03 if supporting else .06
                    turn=float(np.clip(gain*heading,-.15,.15)) if abs(heading)>yaw_deadband else 0.
                    if abs(heading)>yaw_deadband and abs(turn)<min_turn:
                        turn=math.copysign(min_turn,heading)
                    self.controls['Robot_'+who].set_twist(*local,turn)
                for who in ('B','A'):
                    worker = self.workers.get(who)
                    if worker is not None:
                        try:
                            next(worker)
                        except StopIteration:
                            self.workers[who] = None
            if self.time-self.last_telemetry >= .25:
                self.last_telemetry = self.time
                moving={}
                for who in ('A','B'):
                    robot=self.robots['Robot_'+who]
                    dq=dict(zip(robot.dof_names,robot.get_joint_velocities()))
                    moving[who]=dict(base=float(np.linalg.norm(robot.get_linear_velocity()[:2])),
                                     lift=abs(float(dq['lift_joint'])),
                                     arms=max(abs(float(dq[n])) for n in self.ik.names[1:]))
                    if moving[who]['base']>.02 and moving[who]['lift']>.002 and moving[who]['arms']>.01:
                        self.synchronized_samples+=1
                self.telemetry.append(dict(t=self.time, states=self.states.copy(),
                    bases={w:self.base(w).tolist() for w in ('A','B')},
                    lifts={w:float(self.joints(w)['lift_joint']) for w in ('A','B')},
                    held=self.held.copy(), handoff=self.handoff.status(), velocities=moving,
                    held_tilt_degrees={w:math.degrees(math.acos(float(np.clip(self.observe(i)[2,2],-1.,1.))))
                                       for w,i in self.held.items() if i is not None}))
            if self.time-self.last_progress >= 10.:
                self.last_progress=self.time
                print('RELAY_PROGRESS',json.dumps(dict(t=self.time,states=self.states,
                      bases={w:self.base(w).tolist() for w in ('A','B')})),flush=True)
            if not any(self.workers.values()):
                self.active = False
                self.finished = True
                for ctl in self.controls.values():
                    ctl.stop()
                self.write_report()
        except Exception as exc:
            self.error = f'{type(exc).__name__}: {exc}'
            self.handoff.fail(self.error)
            self.active = False
            self.finished = True
            for ctl in self.controls.values():
                ctl.stop()
                # Freeze at feedback, not at a future waypoint after a failure.
            for who in ('A','B'):
                ctl = self.controls['Robot_'+who]
                for name,value in self.joints(who).items():
                    if name in ctl.targets and ctl.specs[name]['kind'] != 'gripper':
                        spec=ctl.specs[name]
                        if math.isfinite(value) and spec['lower']-.12<=value<=spec['upper']+.12:
                            ctl.targets[name] = ctl.sent[name] = float(np.clip(value,spec['lower'],spec['upper']))
            self.pusher.target = float(np.clip(self.pusher.slider.get_world_pose()[0][0]-self.pusher.home[0],0,self.pusher.stroke))
            self.pusher.target_attr.Set(self.pusher.target)
            self.pusher.state = 'FAULT'
            self.write_report()
            print('RELAY_FAULT', self.error, flush=True)

    def wait(self, seconds):
        end = self.time+seconds
        while self.time < end:
            yield

    def acquire(self, who, index, *, slot=0):
        start = self.time
        while True:
            action = self.handoff.acquire(who,index,slot=slot)
            if action:
                return action
            if self.time-start > self.settings['max_wait_seconds']:
                raise RelayFault(f'{who} handoff wait exceeded limit; retaining load')
            yield

    def move_joints(self, who, target, *, minimum=1., timeout=25.):
        ctl = self.controls['Robot_'+who]
        current = {n:float(ctl.sent[n]) for n in target}
        limits = {n:min(ctl.specs[n]['velocity'],ctl.cfg[ctl.specs[n]['kind']]['command_velocity_limit']) for n in target}
        duration = minimum_duration(current,target,limits,minimum*self.settings.get('motion_time_scale',1.))
        start = self.time
        while self.time-start < duration:
            u = smoothstep((self.time-start)/duration)
            for n,v in target.items():
                ctl.set_joint(n,current[n]+u*(v-current[n]))
            yield
        for n,v in target.items():
            ctl.set_joint(n,v)
        stable = 0.
        while self.time-start < duration+timeout:
            q = self.joints(who)
            error = max(abs(q[n]-v) for n,v in target.items())
            stable = stable+.05 if error < self.settings['joint_tolerance'] else 0.
            if stable > .2:
                return
            yield
        raise RelayFault(f'{who} joint tracking timeout, error={error:.3f}')

    def gripper(self, who, sides, closed, index=None, *, support=None):
        target = {f'openarm_{s}_finger_joint1': (1 if s=='left' else -1)*self.settings['gripper_closed' if closed else 'gripper_open'] for s in sides}
        ctl = self.controls['Robot_'+who]
        torque=self.settings['single_gripper_torque_limit' if closed and len(sides)==1 else 'gripper_torque_limit']
        for n in target:
            ctl.drives[n].GetStiffnessAttr().Set(self.settings['gripper_stiffness']*math.pi/180)
            ctl.drives[n].GetMaxForceAttr().Set(min(ctl.specs[n]['effort'],torque))
        for n,v in target.items():
            ctl.set_joint(n,v)
        # Contact closure intentionally does not demand reaching the empty-jaw
        # angle. Re-observe while closing so a nudged rim is not missed.
        if closed and index is not None:
            start=self.time
            first=self.observe(index)
            closure_base=self.base(who).copy()
            while self.time-start<2.2:
                if who=='B':
                    current_base=self.base(who)
                    if (np.linalg.norm(current_base[:2]-closure_base[:2])>.015 or
                            abs(wrap(current_base[3]-closure_base[3]))>.03):
                        raise RelayFault('B base moved during closure; reopen and realign')
                pose=self.observe(index)
                if np.linalg.norm(pose[:3,3]-first[:3,3])>.06:
                    raise RelayFault('Crate moved excessively during closure')
                if self.settings.get('freeze_contact_during_closure',False):
                    q=self.joints(who)
                    seated=all(self.gripping(who,index,s) and
                               abs(q[f'openarm_{s}_finger_joint1'])<.9*self.settings['gripper_open']
                               for s in sides)
                    if seated:
                        # Keep the seated wrists still while the fingers build
                        # preload; chasing a compliant rim can tip the crate.
                        yield from self.wait(.15)
                        continue
                hands={**self.anchored_hands[who],**self.hand_targets(who,pose,sides=sides)}
                solution=self.ik.solve(hands,self.base(who),{n:ctl.sent[n] for n in self.ik.names},
                                       attempts=1,joint_window=.15,lift=ctl.sent['lift_joint'])
                if not solution.success:
                    raise RelayFault(f'Rim tracking during closure is infeasible: {solution.position_error:.4f}m / {solution.angle_error:.3f}rad')
                for n,v in solution.joints.items():
                    ctl.set_joint(n,v)
                self.hand_goals[who]=hands
                yield from self.wait(.15)
            yield from self.wait(.4)
        elif support is not None:
            if closed or len(sides)!=1:
                raise ValueError('Supported release requires one opening hand')
            box,other,reference=support
            start=self.time
            free=self.actual_hands(who,commanded=True)[sides[0]].copy()
            while self.time-start<2.2:
                observed=self.observe(box)
                holding=reference @ np.linalg.inv(observed) @ self.actual_hands(who,commanded=True)[other]
                yield from self.hands(who,{sides[0]:free,other:holding},seconds=.15,settle=.05)
        else:
            yield from self.wait(2.2)

    def navigate(self, who, xy, yaw=math.pi/2, *, lift=None, preserve_posture=False):
        ctl = self.controls['Robot_'+who]
        if (self.held[who] is not None and lift is None and
                (who=='B' or getattr(self,'upright_loads',{}).get(who)==self.held[who])):
            # Do not redistribute lift/arm motion on every route segment:
            # compliant friction grasps do not preserve box pose under it.
            preserve_posture=True
        b=self.base(who)
        if (self.held[who] is None and lift is None and
                np.linalg.norm(np.asarray(xy)-b[:2])<.008 and abs(wrap(yaw-b[3]))<.015):
            # Initial/reset poses already coincide with the taught waiting
            # stations. Do not run a redundant arrival/settling sequence.
            ctl.stop()
            self.station_poses[who]=np.array([*xy,yaw])
            return
        self.station_poses.pop(who,None)
        start = self.time
        speed = self.settings['base_speed_loaded' if self.held[who] is not None else 'base_speed_empty']
        posture=None
        if self.held[who] is not None and not preserve_posture:
            # Solve a body-relative hand path while moving the base. The lift
            # and arms share the same waypoint time; hands preserve their
            # relative transform throughout the small posture adjustment.
            # Reusing measured FK here integrates gravity deflection at every
            # route segment. Preserve the command preload in the body frame.
            current_lift=float(ctl.sent['lift_joint'])
            target_lift=min(.4,current_lift+.015) if current_lift<.375 else current_lift-.015
            if lift is not None:
                target_lift=lift
            targets=self.actual_hands(who,commanded=True)
            feasible=self.ik.solve(targets,self.base(who),
                {n:ctl.sent[n] for n in self.ik.names},lift=target_lift,
                attempts=1,joint_window=.3)
            if feasible.success:
                posture=self.hands(who,targets,seconds=4.,lift=target_lift)
            elif lift is not None:
                raise RelayFault(f'{who} requested navigation lift is infeasible')
        elif lift is not None:
            ctl.set_joint('lift_joint',lift)
        stable = 0.
        best_distance=float('inf')
        last_progress=self.time
        while self.time-start < 90:
            if who=='B' and self.held[who] is not None:
                carried=self.observe(self.held[who])
                if carried[2,2]<math.cos(self.settings['angle_tolerance']):
                    raise RelayFault('B load is not upright during transport')
            if posture is not None:
                try:
                    next(posture)
                except StopIteration:
                    posture=None
            b = self.base(who)
            delta = np.asarray(xy)-b[:2]
            heading = wrap(yaw-b[3])
            remaining=float(np.linalg.norm(delta))+.2*abs(heading)
            if remaining<best_distance-.002:
                best_distance=remaining
                last_progress=self.time
            elif self.time-last_progress>6.:
                raise RelayFault(f'{who} navigation stalled; possible obstacle at {b[:2].tolist()}')
            vel = delta*1.2
            vel *= min(1.,speed/max(np.linalg.norm(vel),1e-9))
            distance=float(np.linalg.norm(delta))
            if distance>.007 and np.linalg.norm(vel)<.028:
                vel=delta/distance*.028
            if distance<=.007:
                vel[:]=0.
            body = Rotation.from_euler('z',-b[3]).as_matrix()[:2,:2] @ vel
            turn_limit=self.settings.get('base_turn_speed',.35)
            turn=float(np.clip(1.5*heading,-turn_limit,turn_limit))
            if abs(heading)>.01 and abs(turn)<.08:
                turn=math.copysign(.08,heading)
            ctl.set_twist(*body,turn if abs(heading)>.01 else 0.)
            stable = stable+.05 if np.linalg.norm(delta)<.008 and abs(heading)<.015 else 0.
            if stable > .3 and posture is None:
                ctl.stop()
                self.station_poses[who]=np.array([*xy,yaw])
                yield from self.wait(.3)
                return
            yield
        raise RelayFault(f'{who} navigation timed out at {b.tolist()}, goal={list(xy)}')

    def hand_targets(self, who, box_pose, *, spread=0., sides=('left','right'), base_yaw=None):
        b = self.base(who)
        heading = b[3] if base_yaw is None else base_yaw
        object_yaw=Rotation.from_matrix(box_pose[:3,:3]).as_euler('xyz')[2]
        facing=1 if math.sin(heading-object_yaw)>=0 else -1
        nominal_rot=Rotation.from_euler('z',facing*math.pi/2).as_matrix()
        half = self.c['crate']['size'][0]/2-self.settings['grasp_inset']
        ans = {}
        for side in sides:
            sign = 1 if side == 'left' else -1
            frame_yaw = self.settings.get('gripper_frame_yaw_degrees',0.)
            if isinstance(frame_yaw,dict):
                frame_yaw = frame_yaw[side]
            if self.grasp_transforms[who] is not None:
                t=box_pose @ self.grasp_transforms[who][side]
            else:
                local=np.eye(4)
                local[:3,3]=[-sign*facing*half,0.,
                             self.c['crate']['size'][2]-self.settings['grasp_below_top']]
                # Pinch across the rim, approaching diagonally from above and
                # in front. Tilting about the closing axis preserves rim-normal
                # closure while keeping the wrist within its height envelope.
                pitch=Rotation.from_euler('x',self.settings['handoff_gripper_pitch_degrees' if who=='B' else 'gripper_pitch_degrees'],degrees=True).as_matrix()
                local[:3,:3]=(nominal_rot @
                    Rotation.from_euler('z',-math.pi/2).as_matrix() @
                    pitch @
                    Rotation.from_euler('z',(0 if side=='left' else math.pi) +
                        math.radians(frame_yaw)).as_matrix())
                t=box_pose @ local
            t[:3,3]+=t[:3,2]*spread
            ans[side] = t
        if getattr(self,'perception',None)=='rgbd':
            self.depth_sensor.record_targets(who,ans,kind='rule_targets',box_pose=box_pose,spread=spread)
        return ans

    def rim_approach_targets(self,who,pose,progress=0.,sides=('left','right'), *, base_yaw=None):
        heading = {} if base_yaw is None else {'base_yaw':base_yaw}
        vertical = (self.settings['handoff_vertical_approach'] if who=='B' else
                    self.settings.get('source_vertical_approach',0.))
        if vertical:
            targets=self.hand_targets(who,pose,sides=sides,**heading)
            for t in targets.values():
                t[2,3]+=vertical*(1-progress)
            return targets
        return self.hand_targets(who,pose,spread=self.settings['approach_clearance']*(1-progress),sides=sides,**heading)

    def approach_rim(self,who,index,sides,seconds=2.5):
        """Track a settling crate throughout the final open-jaw approach."""
        ctl=self.controls['Robot_'+who]
        start=self.time
        while self.time-start<seconds:
            progress=min(1.,(self.time-start)/seconds)
            pose=self.observe(index)
            target=self.rim_approach_targets(who,pose,progress,sides)
            target={**self.anchored_hands[who],**target}
            solution=self.ik.solve(target,self.base(who),{n:ctl.sent[n] for n in self.ik.names},
                                   lift=ctl.sent['lift_joint'],attempts=self.settings.get('path_ik_attempts',1),joint_window=.2)
            if not solution.success:
                raise RelayFault(f'Observed rim approach infeasible: {solution.position_error:.4f}m / {solution.angle_error:.3f}rad')
            for n,v in solution.joints.items():
                ctl.set_joint(n,v)
            self.hand_goals[who]=target
            yield from self.wait(.15)
        pose=self.observe(index)
        yield from self.hands(who,self.hand_targets(who,pose,sides=sides),seconds=.5,
                              lift=ctl.sent['lift_joint'])

    def plan_hands(self, who, targets, *, seconds=2., lift=None, attempts=1, seed=None, base=None):
        """Preflight a complete continuous path without issuing drive commands."""
        targets={**self.anchored_hands[who],**targets}
        ctl = self.controls['Robot_'+who]
        base = self.base(who) if base is None else np.asarray(base)
        joints = {n:(ctl.sent if seed is None else seed)[n] for n in self.ik.names}
        starts = self.actual_hands(who)
        root = np.eye(4)
        root[:3,:3] = Rotation.from_euler('z',base[3]).as_matrix()
        root[:3,3] = base[:3]
        # Use command FK to preserve an established contact preload.
        for s,arm in self.ik.arms.items():
            starts[s] = root @ arm.fk([joints[n] for n in arm.names],joints['lift_joint'])
        paths=[]
        # Keep Cartesian waypoint spacing; shorten timing only within joint limits.
        for u in np.linspace(0,1,max(3,int(seconds*8)))[1:]:
            step_targets={}
            for s,t in targets.items():
                p=np.eye(4)
                p[:3,3]=(1-u)*starts[s][:3,3]+u*t[:3,3]
                p[:3,:3]=Slerp([0,1],Rotation.from_matrix([starts[s][:3,:3],t[:3,:3]]))([u]).as_matrix()[0]
                step_targets[s]=p
            solution=self.ik.solve(step_targets,base,joints,lift=lift,attempts=attempts,joint_window=.3)
            if not solution.success:
                raise RelayFault(f'{who} Cartesian path infeasible at {u:.2f}: {solution.position_error:.4f}m / {solution.angle_error:.3f}rad')
            if max(abs(solution.joints[n]-joints[n]) for n in self.ik.names[1:])>.45:
                raise RelayFault(f'{who} IK branch discontinuity')
            joints=solution.joints
            paths.append(joints)
        return paths

    def hands(self, who, targets, *, seconds=2., lift=None, cartesian=True, settle=.3, planned=None):
        targets={**self.anchored_hands[who],**targets}
        if getattr(self,'perception',None)=='rgbd':
            self.depth_sensor.record_targets(who,targets,kind='motion_targets')
        self.hand_goals[who]={s:t.copy() for s,t in targets.items()}
        ctl=self.controls['Robot_'+who]
        if not cartesian:
            joints={n:ctl.sent[n] for n in self.ik.names}
            fixed={n:joints[n] for s in self.anchored_hands[who] for n in self.ik.arms[s].names}
            solution=self.ik.solve(targets,self.base(who),joints,lift=lift,attempts=4,fixed_joints=fixed)
            if not solution.success:
                raise RelayFault(f'{who} whole-body IK infeasible: {solution.position_error:.4f}m / {solution.angle_error:.3f}rad')
            yield from self.move_joints(who,solution.joints,minimum=seconds)
            return
        paths=planned if planned is not None else self.plan_hands(who,targets,seconds=seconds,lift=lift,
            attempts=self.settings.get('path_ik_attempts',1))
        motion_seconds=seconds*self.settings.get('motion_time_scale',1.)
        for target in paths:
            limits={n:min(ctl.specs[n]['velocity'],ctl.cfg[ctl.specs[n]['kind']]['command_velocity_limit']) for n in target}
            duration=max(motion_seconds/len(paths),max(abs(v-ctl.sent[n])/limits[n] for n,v in target.items()))
            for n,v in target.items():
                ctl.set_joint(n,v)
            yield from self.wait(duration)
        yield from self.wait(settle)

    def move_and_prepare(self, who, travel, prepare):
        """Advance independent base and joint commands on the same control tick."""
        pending=[travel,prepare]
        try:
            while pending:
                for action in pending[:]:
                    try:
                        next(action)
                    except StopIteration:
                        pending.remove(action)
                if pending:
                    yield
        finally:
            for action in pending:
                action.close()
            self.controls['Robot_'+who].stop()

    def dock(self, who, position, *, handoff=False, far=False, prepare=None, placement_target=None, dock_distance=None):
        if handoff:
            far = who == 'A'
        self.last_dock[who] = dict(far=far, handoff=handoff)
        angle = pallet_yaw(self.c)
        if angle and not handoff:
            # For the 90-degree layout, reach the two short pallet sides from
            # the front aisle; do not cross the pallet or the handoff table.
            center = np.asarray(self.c['pallet']['source' if who=='A' else 'target'])
            sign = -1 if far else 1
            direction = np.asarray(rotate_xy([0.,sign],angle))
            yaw = math.atan2(direction[1],direction[0])
            distance=self.settings['dock_distance'] if dock_distance is None else dock_distance
            dock = np.asarray(position[:2])-direction*distance
            lane = center-direction*.95
            lane[0] = self.settings['fixed_pallet_lanes'][who][int(far)]
            def travel():
                if who=='A':
                    b=self.base(who)
                    # Reuse the current pallet-side lane, or cross via the
                    # nearest end aisle. Never cut across the source pallet.
                    if abs(b[0]-lane[0])>.12:
                        aisle=min((self.settings['transit_y'],self.settings['source_rear_aisle_y']),
                                  key=lambda y:abs(y-b[1]))
                        yield from self.navigate(who,[b[0],aisle])
                        yield from self.navigate(who,[lane[0],aisle],yaw)
                else:
                    yield from self.navigate(who,[lane[0],self.settings['transit_y']],yaw)
                yield from self.navigate(who,[lane[0],dock[1]],yaw)
            if prepare is not None:
                yield from self.move_and_prepare(who,travel(),prepare)
            else:
                yield from travel()
            # Finish arm preparation in the side lane before entering the stack.
            if placement_target is not None:
                yield from self.prepare_target_approach(who,position,placement_target,dock)
            yield from self.navigate(who,dock,yaw)
            return
        yaw = -math.pi/2 if far else math.pi/2
        sign = -1 if far else 1
        distance = self.settings['handoff_dock_distance' if handoff else 'dock_distance']
        if handoff and who=='B':
            distance=self.settings['b_handoff_dock_distance']
        x = float(position[0])
        if handoff:
            # Keep the base between the front table legs, which sit at x=±.57.
            table_center = sum(s[0] for s in self.c['handoff']['slots']) / 2
            x += .08 if x < table_center else -.08
        y = float(position[1]-sign*distance)
        # Travel outside pallet footprints. Far-row approaches use the outer aisle.
        if handoff:
            # Approach front or rear in its clear aisle, never across the table.
            aisle=self.settings['source_rear_aisle_y'] if far else self.settings['transit_y']
            if who=='B':
                # Observe/unfold at the dedicated viewing station before
                # approaching. Do not move the camera away during inference.
                if prepare is not None:
                    yield from prepare
                yield from self.navigate(who,[x,y],yaw)
                return
            travel=self.navigate(who,[x,aisle],yaw)
            if prepare is not None:
                yield from self.move_and_prepare(who,travel,prepare)
            else:
                yield from travel
        elif far:
            if prepare is not None:
                yield from prepare
            outer = -3.2 if who == 'A' else 3.2
            rear = self.settings['source_rear_aisle_y'] if who=='A' else 1.85
            yield from self.navigate(who,[outer,self.settings['transit_y']])
            # Split the half-turn through the outward heading. Small yaw
            # errors must not choose an inward load sweep beside the pallet.
            yield from self.navigate(who,[outer,self.settings['transit_y']],math.pi if who=='A' else 0.)
            yield from self.navigate(who,[outer,rear],yaw)
            yield from self.navigate(who,[x,rear],yaw)
        elif prepare is not None:
            yield from prepare
        yield from self.navigate(who,[x,y],yaw)

    def retreat(self, who, *, far=False, handoff=False, handoff_far=False):
        b=self.base(who)
        if handoff:
            far = who == 'A'
            yaw=-math.pi/2 if far else math.pi/2
            aisle=self.settings['source_rear_aisle_y'] if far else self.settings['transit_y']
            yield from self.navigate(who,[b[0],aisle],yaw)
            if self.held[who] is None:
                yield from self.stow(who)
            if who!='A':
                yield from self.navigate(who,[1.,aisle],yaw)
            return
        if pallet_yaw(self.c) and not handoff:
            center=np.asarray(self.c['pallet']['source' if who=='A' else 'target'])
            direction=np.asarray(rotate_xy([0.,-1 if far else 1],pallet_yaw(self.c)))
            lane=center-direction*.95
            lane[0]=self.settings['fixed_pallet_lanes'][who][int(far)]
            # Move the chassis first without changing the established pinch.
            if who=='A' and self.held[who] is not None:
                self.event(who,'BASE_RETREAT_WITH_LOAD',box=self.held[who])
            yield from self.navigate(who,[lane[0],b[1]],b[3],preserve_posture=True)
            if who=='A' and self.held[who] is not None:
                index=self.held[who]
                required=self.c['source_stack']['rows']*self.c['source_stack']['pitch'][1]/2+.10
                clearance=self.source_side_clearance(index,direction)
                if clearance<required:
                    # Obtain clearance with the chassis, before any arm motion.
                    # The extra margin absorbs base arrival and vision errors.
                    extra=required-clearance+.015
                    base=self.base(who)
                    point=base[:2]-direction*extra
                    self.check_source_retreat_corridor(who,base[:2],point)
                    self.event(who,'CLEAR_SOURCE_WITH_FIXED_GRASP',box=index,distance=float(extra),
                               clearance_before=clearance,required=required)
                    yield from self.navigate(who,point,base[3],preserve_posture=True)
                    lane[0]=point[0]
                clearance=self.source_side_clearance(index,direction)
                if clearance<required:
                    raise RelayFault('Source clearance not established; refusing arm retraction')
                self.event(who,'SOURCE_CLEARANCE_VERIFIED',box=index,clearance=clearance,required=required)
                pose=self.upright_pose(self.observe(index))
                pose[:2,3]-=direction*self.settings['source_retreat_retract']
                # Keep the measured box height: no extra lift near the upper stop.
                self.event(who,'PREPARE_RETREAT_CARRY',box=index)
                targets=self.hand_targets(who,pose)
                planned=self.plan_hands(who,targets,seconds=3.,attempts=self.settings.get('path_ik_attempts',4))
                yield from self.hands(who,targets,seconds=3.,planned=planned)
            if self.held[who] is not None:
                pose=self.observe(self.held[who])
                sx,sy,sz=self.c['crate']['size']
                corners=np.array([[x,y,z] for x in (-sx/2,sx/2) for y in (-sy/2,sy/2) for z in (0.,sz)])
                world=corners @ pose[:3,:3].T+pose[:3,3]
                outward=-direction
                clearance=np.min((world[:,:2]-center) @ outward)
                if clearance < self.c['source_stack']['rows']*self.c['source_stack']['pitch'][1]/2+.05:
                    raise RelayFault('载箱尚未离开旋转垛边缘，禁止沿侧通道移动')
            else:
                yield from self.stow(who)
            yield from self.navigate(who,[lane[0],self.settings['source_rear_aisle_y'] if handoff_far else self.settings['transit_y']],b[3])
            return
        if far:
            outer=-3.2 if who=='A' else 3.2
            rear=self.settings['source_rear_aisle_y'] if who=='A' else 1.85
            yield from self.navigate(who,[b[0],rear],-math.pi/2)
            if who=='A' and self.held[who] is not None:
                self.verify_source_rear_clearance(self.held[who])
            if who=='B' and self.held[who] is None:
                # Fold in the clear rear aisle before turning past filled
                # outer target columns with the empty grippers extended.
                yield from self.stow(who)
            yield from self.navigate(who,[outer,rear],-math.pi/2)
            yield from self.navigate(who,[outer,rear],math.pi if who=='A' else 0.)
            yield from self.navigate(who,[outer,self.settings['transit_y']])
        else:
            yield from self.navigate(who,[b[0],self.settings['transit_y']])

    def check_source_retreat_corridor(self,who,start,end):
        """Conservative chassis swept bounds against scene obstacles and robot B."""
        start,end=np.asarray(start),np.asarray(end)
        if np.linalg.norm(end-start)>.35:
            raise RelayFault('Source clearance requires excessive extra retreat; check perception')
        radius=.43  # Same conservative chassis envelope as navigation_demo_config.
        low=np.minimum(start,end)-radius-.05
        high=np.maximum(start,end)+radius+.05
        floor=np.asarray(self.c['room']['floor_size'][:2])/2
        if np.any(low < -floor) or np.any(high > floor) or high[1]>self.c['room']['back_wall_y']-.1:
            raise RelayFault('Source retreat corridor reaches the room boundary')
        obstacles=[('handoff',np.asarray(self.c['handoff']['center']),np.asarray(self.c['handoff']['size'][:2])/2)]
        half=np.asarray(world_half_extent(self.c['pallet']['size'][:2],pallet_yaw(self.c)))
        for name in ('source','target'):
            obstacles.append((name+' pallet',np.asarray(self.c['pallet'][name]),half))
        if getattr(self,'robots',None):
            obstacles.append(('other robot',self.base('B' if who=='A' else 'A')[:2],np.array([radius,radius])))
        for name,center,extent in obstacles:
            if np.all(high>center-extent) and np.all(low<center+extent):
                raise RelayFault('Source retreat corridor blocked by '+name)

    def source_side_clearance(self,index,direction):
        pose=self.observe(index)
        sx,sy,sz=self.c['crate']['size']
        corners=np.array([[x,y,z] for x in (-sx/2,sx/2) for y in (-sy/2,sy/2) for z in (0.,sz)])
        world=corners @ pose[:3,:3].T+pose[:3,3]
        return float(np.min((world[:,:2]-np.asarray(self.c['pallet']['source'])) @ -direction))

    def verify_source_rear_clearance(self,index):
        pose=self.observe(index)
        sx,sy,sz=self.c['crate']['size']
        corners=np.array([[x,y,z] for x in (-sx/2,sx/2) for y in (-sy/2,sy/2) for z in (0.,sz)])
        front=float(np.min((corners @ pose[:3,:3].T+pose[:3,3])[:,1]))
        source_back=max(s['position'][1] for s in self.initial)+sy/2
        if front<source_back+.05:
            raise RelayFault('Load has not cleared the source rear edge before lateral travel')

    @staticmethod
    def upright_pose(pose):
        pose=pose.copy()
        yaw=Rotation.from_matrix(pose[:3,:3]).as_euler('xyz')[2]
        pose[:3,:3]=Rotation.from_euler('z',yaw).as_matrix()
        return pose

    def establish_upright_carry(self,who,index):
        tolerance=self.settings.get('a_carry_tilt_tolerance_degrees',10.) if who=='A' else 1.
        for _ in range(3):
            pose=self.observe(index)
            if pose[2,2]>=math.cos(math.radians(tolerance)):
                break
            yield from self.lower_held_box(who,index,self.upright_pose(pose))
        tilt=math.degrees(math.acos(float(np.clip(self.observe(index)[2,2],-1.,1.))))
        if tilt>tolerance:
            raise RelayFault(f'Cannot establish upright {who} carrying posture')
        self.upright_loads[who]=index
        self.event(who,'UPRIGHT_CARRY_VERIFIED',box=index,tilt_degrees=tilt)

    def prepare_handoff_carry(self,who,index):
        self.event(who,'RAISE_FOR_HANDOFF',box=index)
        if who=='B':
            # Correct measured compliant-drive sag before any base motion.
            # A zero-tilt command alone does not make the carried box upright.
            yield from self.establish_upright_carry(who,index)
            self.verify_handoff_clearance(index)
            return
        b=self.base(who)
        pose=self.observe(index)
        tilt=self.settings['handoff_pick_tilt_degrees' if who=='B' else 'handoff_carry_tilt_degrees']
        clearance=self.settings['handoff_pick_clearance' if who=='B' else 'handoff_carry_clearance']
        turn=wrap(b[3]-self.grasp_base_yaw.get(who,b[3]))
        carried_yaw=wrap(self.grasp_object_yaw.get(who,0.)+turn)
        # Far-row pickup faces the opposite direction. Preserve that half-turn
        # in the object frame after driving around the pallet; the carry tilt
        # is about the world x axis at this forward-facing handoff approach.
        pose[:3,:3]=(Rotation.from_euler('x',tilt*(1 if math.sin(b[3])>=0 else -1),degrees=True).as_matrix() @
                       Rotation.from_euler('z',carried_yaw).as_matrix())
        pose[:2,3]=b[:2]+Rotation.from_euler('z',b[3]).as_matrix()[:2,:2] @ [self.settings['handoff_carry_forward'],0.]
        pose[2,3]=self.c['handoff']['size'][2]+clearance
        # Retract first, then raise, to stay inside the high-position workspace.
        retract=pose.copy()
        retract[2,3]=max(float(self.observe(index)[2,3]),self.c['handoff']['size'][2]+self.settings['handoff_carry_staging_clearance'])
        legs,chosen=self.plan_handoff_carry(index,retract,pose)
        self.event(who,'HANDOFF_CARRY_PLANNED',box=index,**chosen)
        for box_pose,goal,path,seconds in legs:
            yield from self.hands(who,goal,seconds=seconds,planned=path)
            if not all(self.gripping(who,index,side) for side in ('left','right')):
                raise RelayFault('Adaptive handoff carry lost bilateral grip')
            if np.linalg.norm(self.observe(index)[:3,3]-box_pose[:3,3])>.04:
                raise RelayFault('Adaptive handoff carry tracking error')
        self.verify_handoff_clearance(index)

    def handoff_place_goals(self,index,base):
        """The same upright poses used for A's actual deposit and withdrawal."""
        slot=source_handoff_slot(self.c,self.initial[index])
        target=np.eye(4)
        target[:3,3]=[*self.c['handoff']['slots'][slot],self.c['handoff']['size'][2]]
        yaw=Rotation.from_matrix(self.observe(index)[:3,:3]).as_euler('xyz')[2]
        target[:3,:3]=Rotation.from_euler('z',math.pi if math.cos(yaw)<0 else 0.).as_matrix()
        above=target.copy();above[2,3]+=.06
        above[:3,:3]=Rotation.from_euler('x',self.settings['handoff_place_tilt_degrees']*
                                       (1 if math.sin(base[3])>=0 else -1),degrees=True).as_matrix()@target[:3,:3]
        target[2,3]-=.002
        return [(self.hand_targets('A',above),3.),(self.hand_targets('A',target),2.5),
                (self.hand_targets('A',target,spread=self.settings['approach_clearance']),2.)]

    def preflight_handoff_place(self,index,base,seed):
        plans=[]
        for goal,seconds in self.handoff_place_goals(index,base):
            path=self.plan_hands('A',goal,seconds=seconds,attempts=4,seed=seed,base=base)
            plans.append(path);seed=path[-1]
        return plans

    def prepare_handoff_deposit(self,index):
        """Recheck the real grasp, backing away from the table only if needed."""
        def measured_plan(base):
            observed=self.observe(index)
            self.grasp_transforms['A']={s:np.linalg.inv(observed)@t for s,t in self.actual_hands('A').items()}
            return self.preflight_handoff_place(index,base,None)
        base=self.base('A')
        try:
            return measured_plan(base)
        except RelayFault as exc:
            self.event('A','HANDOFF_DEPOSIT_REPLAN',box=index,reason=str(exc))
        self.verify_handoff_clearance(index)
        if not all(self.gripping('A',index,s) for s in ('left','right')):
            raise RelayFault('Cannot adjust handoff dock without bilateral grip')
        # Only retreat along the already traversed docking lane. Never move
        # closer to the table legs or sideways into their footprint.
        axis=Rotation.from_euler('z',base[3]).as_matrix()[:2,0]
        failures=[]
        for distance in self.settings.get('handoff_dock_retreat_candidates',[.02,.04,.06]):
            candidate=base.copy();candidate[:2]-=axis*distance
            try:
                self.preflight_handoff_place(index,candidate,None)
            except RelayFault as exc:
                failures.append(str(exc));continue
            self.event('A','ADJUST_HANDOFF_DOCK',box=index,retreat=distance,position=candidate[:2].tolist())
            yield from self.navigate('A',candidate[:2],candidate[3],preserve_posture=True)
            self.verify_handoff_clearance(index)
            try:
                return measured_plan(self.base('A'))
            except RelayFault as exc:
                failures.append(str(exc))
        raise RelayFault('No reachable deposit from safe dock adjustments: '+'; '.join(failures))

    def source_carry_strategy(self,index):
        """Dispatch by source layer; the bottom deliberately reuses middle."""
        spec=self.initial[index]
        layer=int(spec['layer'])
        name='top' if layer==self.c['source_stack']['layers']-1 else 'middle'
        profile=self.settings['source_carry_strategies'][name]
        lateral=float(profile['lateral'])*(-1 if spec['row'] else 1)
        return name,profile,lateral

    def plan_handoff_carry(self,index,staging,final):
        """Search continuous carry AND deposit paths before issuing any commands.

        Base docking remains on the existing validated lane. Transport poses are
        varied in the clear aisle, without changing the grasp or IK tolerances.
        """
        base=self.base('A');rotation=Rotation.from_euler('z',base[3]).as_matrix()
        observed=self.observe(index);start=self.upright_pose(observed)
        relative=rotation.T@(observed[:3,3]-base[:3])
        slot=source_handoff_slot(self.c,self.initial[index])
        far=source_handoff_far(self.c,self.initial[index]);sign=-1 if far else 1
        xy=self.c['handoff']['slots'][slot]
        center=sum(p[0] for p in self.c['handoff']['slots'])/2
        dock=np.array([xy[0]+(.08 if xy[0]<center else -.08),
                       xy[1]-sign*self.settings['handoff_dock_distance'],base[2],sign*math.pi/2])
        # The tested layer-specific path is primary. Only search alternatives
        # if its complete carry/deposit preflight fails.
        strategy,profile,primary_lateral=self.source_carry_strategy(index)
        primary_forward=float(profile['forward'])
        staging=staging.copy()
        staging[2,3]=max(float(observed[2,3]),self.c['handoff']['size'][2]+float(profile['staging_clearance']))
        forwards=list(dict.fromkeys([primary_forward,round(float(np.clip(relative[0],.50,.63)),3),
                                     *profile['fallback_forward']]))
        failures=[];valid=[]
        for forward in forwards:
            for lateral in dict.fromkeys([primary_lateral,0.,-primary_lateral]):
                target=staging.copy();end=final.copy()
                target[:2,3]=end[:2,3]=base[:2]+(rotation@np.array([forward,lateral,0.]))[:2]
                for order in (('direct',),('rise','retract','rotate'),('retract','rise','rotate')):
                    current=start.copy();seed=None;legs=[];paths=[]
                    try:
                        for phase in order:
                            current=current.copy()
                            if phase=='direct':current=target.copy()
                            elif phase=='rise':current[2,3]=target[2,3]
                            elif phase=='retract':current[:2,3]=target[:2,3]
                            else:current[:3,:3]=target[:3,:3]
                            goal=self.hand_targets('A',current)
                            path=self.plan_hands('A',goal,seconds=3.,attempts=4,seed=seed)
                            legs.append((current,goal,path,3.));paths.extend(path);seed=path[-1]
                        goal=self.hand_targets('A',end)
                        path=self.plan_hands('A',goal,seconds=4.,attempts=4,seed=seed)
                        legs.append((end,goal,path,4.));paths.extend(path)
                        deposit=self.preflight_handoff_place(index,dock,path[-1])
                        for path in deposit:paths.extend(path)
                    except RelayFault as exc:
                        failures.append(dict(forward=forward,lateral=lateral,order=list(order),reason=str(exc)))
                        continue
                    margin=min(min((p[n]-self.ik.lower[i])/(self.ik.upper[i]-self.ik.lower[i]),
                                   (self.ik.upper[i]-p[n])/(self.ik.upper[i]-self.ik.lower[i]))
                               for p in paths for i,n in enumerate(self.ik.names))
                    movement=float(np.linalg.norm(target[:3,3]-observed[:3,3]))
                    # Prefer a useful joint margin, then less movement/extra legs.
                    score=min(margin,.03)-.02*movement-.001*(len(legs)-2)
                    chosen=dict(forward=forward,lateral=lateral,order=list(order),strategy=strategy,
                                source_layer=int(self.initial[index]['layer']),
                                joint_margin_fraction=float(margin),dock=dock.tolist(),deposit_preflight=True)
                    if forward==primary_forward and lateral==primary_lateral and order==('direct',):
                        chosen.update(fallback=False,feasible_candidates=1)
                        return legs,chosen
                    chosen['fallback']=True
                    valid.append((score,legs,chosen))
                    break
        if not valid:
            self.event('A','HANDOFF_CARRY_CANDIDATES_REJECTED',box=index,candidates=failures)
            raise RelayFault('No continuous handoff carry and deposit alternative: '+
                             '; '.join(f['reason'] for f in failures[-3:]))
        _,legs,chosen=max(valid,key=lambda item:item[0])
        chosen['feasible_candidates']=len(valid)
        return legs,chosen

    def verify_handoff_clearance(self,index):
        carried=self.observe(index)
        sx,sy,sz=self.c['crate']['size']
        corners=np.array([[x,y,z] for x in (-sx/2,sx/2) for y in (-sy/2,sy/2) for z in (0.,sz)])
        lowest=float(np.min((corners @ carried[:3,:3].T+carried[:3,3])[:,2]))
        if lowest<self.c['handoff']['size'][2]+.052:
            raise RelayFault('Insufficient load clearance above handoff guides')

    def prepare_handoff_pick(self,who,index):
        """Unfold in the clear aisle into the posture needed at the future dock."""
        self.event(who,'PREPARE_HANDOFF_GRASP',box=index)
        self.grasp_transforms[who]=None
        self.pose_symmetry[index]=False
        pose=self.observe(index)
        if abs(wrap(Rotation.from_matrix(pose[:3,:3]).as_euler('xyz')[2]-(pallet_yaw(self.c) if who=='A' else 0.)))>math.pi/2:
            self.pose_symmetry[index]=True
            pose=self.observe(index)
        self.grasp_forward[who]={s:self.settings['handoff_grasp_forward_offset'] for s in ('left','right')}
        slot=self.c['handoff']['slots'][1]
        future=np.array([slot[0]-.08,slot[1]-self.settings['b_handoff_dock_distance'],self.base(who)[2],math.pi/2])
        # Assign left/right rims in the future dock frame, before the chassis
        # has turned. Using its current heading can swap the arms on return.
        pre=self.rim_approach_targets(who,pose,base_yaw=future[3])
        q=self.joints(who)
        solution=self.ik.solve(pre,future,q,attempts=4)
        if not solution.success:
            raise RelayFault('Handoff pregrasp posture is outside the future dock workspace')
        # Capture at the viewing station, then unfold outside the tabletop's
        # swept volume. The subsequent dock keeps this posture on approach.
        self.event(who,'RETREAT_TO_UNFOLD',box=index)
        base=self.base(who)
        yield from self.navigate(who,[base[0],self.settings['transit_y']],base[3],preserve_posture=True)
        yield from self.move_joints(who,solution.joints,minimum=4.)

    def confirm_pregrasp(self,who,index,sides):
        """Do not close while chassis or fingertips are still misaligned."""
        self.event(who,'VERIFY_PREGRASP',box=index)
        began=self.time
        baseline=self.base(who).copy()
        stable=0.
        last_correction=-1.
        while self.time-began<3.:
            desired=self.hand_targets(who,self.observe(index),sides=sides)
            actual=self.actual_hands(who)
            b=self.base(who)
            aligned=all(np.linalg.norm(actual[s][:3,3]-desired[s][:3,3])<.008 and
                        Rotation.from_matrix(desired[s][:3,:3] @ actual[s][:3,:3].T).magnitude()<.08 for s in sides)
            stationary=(np.linalg.norm(b[:2]-baseline[:2])<.005 and abs(wrap(b[3]-baseline[3]))<.01
                        and np.linalg.norm(self.robots['Robot_'+who].get_linear_velocity()[:2])<.01
                        and np.linalg.norm(self.robots['Robot_'+who].get_angular_velocity())<.03)
            elbows=all(self.joints(who)[f'openarm_{s}_joint4']>.20 for s in sides)
            stable=stable+.05 if aligned and stationary and elbows else 0.
            if stable>=.25:
                return
            if not stationary:
                baseline=b.copy()
            errors={s:desired[s][:3,3]-actual[s][:3,3] for s in sides}
            # Correct small steady tracking deflection before closure. Keep the
            # original acceptance tolerances, torque limits and a 20 mm budget.
            if (stationary and elbows and not aligned and self.time-last_correction>=.15
                    and all(np.linalg.norm(e)<.02 for e in errors.values())):
                commanded=self.actual_hands(who,commanded=True)
                corrected={s:commanded[s].copy() for s in sides}
                for side in sides:
                    delta=errors[side]*.5
                    delta*=min(1.,.002/max(np.linalg.norm(delta),1e-9))
                    offset=corrected[side][:3,3]+delta-desired[side][:3,3]
                    offset*=min(1.,.02/max(np.linalg.norm(offset),1e-9))
                    corrected[side][:3,3]=desired[side][:3,3]+offset
                ctl=self.controls['Robot_'+who]
                result=self.ik.solve(corrected,b,{n:ctl.sent[n] for n in self.ik.names},
                                     lift=ctl.sent['lift_joint'],attempts=1,joint_window=.1)
                if result.success:
                    for n,value in result.joints.items():ctl.set_joint(n,value)
                    self.hand_goals[who]=corrected
                last_correction=self.time
            yield
        self.event(who,'PREGRASP_REJECTED',box=index,
                   position_errors_m={s:float(np.linalg.norm(actual[s][:3,3]-desired[s][:3,3])) for s in sides},
                   actual={s:actual[s][:3,3].tolist() for s in sides},
                   desired={s:desired[s][:3,3].tolist() for s in sides},
                   elbows={s:float(self.joints(who)[f'openarm_{s}_joint4']) for s in sides},
                   base=self.base(who).tolist())
        raise RelayFault('B pregrasp not aligned/stable or elbow clearance insufficient')

    def close_on_rim(self,who,index,sides):
        for attempt in range(2 if who=='B' else 1):
            try:
                yield from self.approach_rim(who,index,sides)
                if who=='B':
                    yield from self.confirm_pregrasp(who,index,sides)
                yield from self.gripper(who,sides,True,index=index)
                if not all(self.gripping(who,index,s) for s in sides):
                    raise RelayFault(f'{who} no bilateral finger contact on each requested rim')
                return
            except RelayFault:
                if who!='B' or attempt:
                    raise
                self.event(who,'RETRY_HANDOFF_GRASP',box=index)
                yield from self.gripper(who,('left','right'),False)
                withdrawal=self.actual_hands(who)
                for t in withdrawal.values():
                    t[2,3]+=self.settings['handoff_vertical_approach']
                yield from self.hands(who,withdrawal,seconds=2.)
                yield from self.retreat(who,handoff=True)
                position=[*self.c['handoff']['slots'][1],self.c['handoff']['size'][2]]
                yield from self.dock(who,position,handoff=True,prepare=self.prepare_handoff_pick(who,index))
                # Retain the B handoff lock throughout this one bounded retry.

    def join_second_gripper(self,who,index,exposed):
        """Keep the first pinch closed while the opposite hand joins at mid-rim."""
        other='right' if exposed=='left' else 'left'
        pose=self.observe(index)
        yaw=self.base(who)[3]
        lateral=np.array([-math.sin(yaw),math.cos(yaw),0.])
        if exposed=='right':
            lateral=-lateral
        shift=lateral*self.settings['assisted_lateral_pull']
        shift[2]=self.settings['assisted_lift']
        self.event(who,'SINGLE_END_LIFT',box=index,side=exposed)
        raised=self.actual_hands(who,commanded=True)[exposed].copy()
        raised[:3,3]+=shift
        yield from self.hands(who,{exposed:raised},seconds=3.)
        if self.perception in ('depth','rgbd'):
            prior=pose.copy()
            prior[:3,3]+=shift
            self.depth_sensor.set_prior(index,prior)
        observed=self.observe(index)
        tilt=math.acos(float(np.clip(observed[2,2],-1.,1.)))
        moved=float(np.dot(observed[:3,3]-pose[:3,3],lateral))
        if (moved<self.settings['assisted_min_lateral_clearance'] or
                tilt>math.radians(self.settings['max_assisted_tilt_deg']) or
                not self.gripping(who,index,exposed)):
            raise RelayFault('Single-end lift did not create a stable second-side clearance')
        self.anchored_hands[who]={exposed:self.actual_hands(who,commanded=True)[exposed].copy()}
        self.event(who,'SECOND_HAND_JOIN',box=index,side=other)
        # Recompute the second rim centre from the tilted box's observed pose.
        pre=self.rim_approach_targets(who,observed,sides=(other,))
        yield from self.hands(who,pre,seconds=3.,
                              cartesian=not self.settings.get('second_hand_joint_approach',False))
        if not self.gripping(who,index,exposed):
            raise RelayFault('First gripper lost contact while the second hand approached')
        yield from self.approach_rim(who,index,(other,))
        yield from self.gripper(who,(other,),True,index=index)
        if not all(self.gripping(who,index,s) for s in (exposed,other)):
            raise RelayFault('Second hand join failed to establish bilateral contact')
        self.event(who,'SECOND_HAND_JOIN_VERIFIED',box=index,first=exposed,second=other)
        self.anchored_hands[who]={}

    def pick(self, who, index, *, assisted=False, exposed='left', staged=False):
        self.working_boxes[who]=index
        self.event(who,'ASSISTED_GRASP' if assisted else 'DIRECT_GRASP',box=index)
        self.pose_symmetry[index]=False
        pose=self.observe(index)
        if abs(wrap(Rotation.from_matrix(pose[:3,:3]).as_euler('xyz')[2]-(pallet_yaw(self.c) if who=='A' else 0.)))>math.pi/2:
            self.pose_symmetry[index]=True
            pose=self.observe(index)
        self.grasp_base_yaw[who]=self.base(who)[3]
        self.grasp_object_yaw[who]=Rotation.from_matrix(pose[:3,:3]).as_euler('xyz')[2]
        self.grasp_transforms[who]=None
        self.anchored_hands[who]={}
        # Both pinches target the centre of their respective rim, not a corner.
        start_z=float(pose[2,3])
        yield from self.gripper(who,('left','right'),False)
        pre=self.rim_approach_targets(who,pose)
        # With one blocked side, keep its arm out of the stack until clearance exists.
        sides=(exposed,) if assisted else ('left','right')
        unfold={s:pre[s].copy() for s in sides}
        forward=Rotation.from_euler('z',self.base(who)[3]).as_matrix()[:,0]
        for t in unfold.values():
            distance=float(np.dot(t[:3,3]-self.base(who)[:3],forward))
            t[:3,3]-=forward*min(self.settings.get('source_unfold_retract',.20),max(0.,distance-.28))
        if who!='B':
            yield from self.hands(who,unfold,cartesian=False)
        yield from self.hands(who,{s:pre[s] for s in sides},seconds=3.)
        pose=self.observe(index)
        target=self.hand_targets(who,pose,sides=sides)
        yield from self.close_on_rim(who,index,sides)
        self.events.append(dict(t=self.time,robot=who,state='CLAMP_EVIDENCE',box=index,
            contacts=sorted(self.box_contacts(index)),
            gripper_positions={s:float(self.joints(who)[f'openarm_{s}_finger_joint1']) for s in sides}))
        if not all(self.gripping(who,index,s) for s in sides):
            raise RelayFault(f'{who} no verified gripper contact on box {index}; contacts={sorted(self.box_contacts(index))}')
        if assisted:
            yield from self.join_second_gripper(who,index,exposed)
        if not all(self.gripping(who,index,s) for s in ('left','right')):
            raise RelayFault(f'{who} bilateral grasp not established')
        # Gentle closure avoids tipping the supported crate. Once both pinches
        # exist, increase holding effort before carrying its weight and moment.
        ctl=self.controls['Robot_'+who]
        for side in ('left','right'):
            name=f'openarm_{side}_finger_joint1'
            ctl.drives[name].GetStiffnessAttr().Set(
                self.settings.get('hold_gripper_stiffness',self.settings['gripper_stiffness'])*math.pi/180)
            ctl.drives[name].GetMaxForceAttr().Set(min(ctl.specs[name]['effort'],self.settings['hold_gripper_torque_limit']))
        yield from self.wait(.5)
        if not all(self.gripping(who,index,s) for s in ('left','right')):
            raise RelayFault('Bilateral grasp lost while establishing holding effort')
        self.anchored_hands[who]={}
        pose=self.observe(index)
        self.grasp_transforms[who]={s:np.linalg.inv(pose) @ t for s,t in self.actual_hands(who).items()}
        clearance=self.settings['handoff_pick_clearance'] if who=='B' else self.settings['lift_clearance']
        pose[2,3]+=clearance
        if who=='A':
            heading=self.base(who)[3]
            pose[:2,3]-=self.settings.get('source_lift_retreat',0.)*np.array([math.cos(heading),math.sin(heading)])
        if who=='B':
            pose[:3,:3]=Rotation.from_euler('x',self.settings['handoff_pick_tilt_degrees'],degrees=True).as_matrix()
        lift_goal=pose.copy()
        self.event(who,'TEST_LIFT',box=index)
        yield from self.hands(who,self.hand_targets(who,pose),seconds=3.)
        if self.perception in ('depth','rgbd'):
            self.depth_sensor.set_prior(index,pose)
        required_rise=.015 if who=='B' else .045
        if self.crates[index].get_world_pose()[0][2] < start_z+required_rise:
            raise RelayFault(f'{who} box did not lift')
        if not all(self.gripping(who,index,s) for s in ('left','right')):
            raise RelayFault(f'{who} lost bilateral contact during lift')
        self.held[who]=index
        # Level only after clearing the support; keep both measured grasps rigid.
        pose=self.observe(index)
        if who=='A' and self.settings.get('source_lift_retreat',0.):
            away=-np.array([math.cos(self.base(who)[3]),math.sin(self.base(who)[3])])
            # Frictional settling can consume the requested stack clearance.
            # Preserve that clearance when leveling instead of moving back
            # toward the neighbouring crate with the observed, sagged pose.
            pose[:2,3]+=away*max(0.,float(np.dot(lift_goal[:2,3]-pose[:2,3],away)))
        pose[:3,:3]=(Rotation.from_euler('x',self.settings['handoff_pick_tilt_degrees'],degrees=True).as_matrix()
                       if who=='B' else Rotation.from_euler('z',self.grasp_object_yaw[who]).as_matrix())
        pose[2,3]=max(float(pose[2,3]),start_z+clearance)
        yield from self.hands(who,self.hand_targets(who,pose),seconds=3.)
        yield from self.wait(self.settings.get('grasp_hold_seconds',1.))
        final=self.observe(index)
        sx,sy,_=self.c['crate']['size']
        corners=np.array([[x,y,0.] for x in (-sx/2,sx/2) for y in (-sy/2,sy/2)])
        lowest=float(np.min((corners @ final[:3,:3].T+final[:3,3])[:,2]))
        if lowest < start_z+(.015 if who=='B' else .035) or self.supported(index):
            raise RelayFault(f'Box {index} has not cleared its support after leveling (bottom={lowest:.4f}, support={start_z:.4f})')
        if not all(self.gripping(who,index,s) for s in ('left','right')):
            raise RelayFault('Both finger pads on both hands must retain contact after leveling')
        self.event(who,'GRASP_VERIFIED',box=index,assisted=assisted,
                   clearance_m=lowest-start_z,position=final[:3,3].tolist(),
                   contacts=sorted(self.box_contacts(index)))

    def lower_held_box(self,who,index,target, *, sides=('left','right'), stop_on_support=None):
        """Re-observe the friction grasp between short placement segments."""
        start=self.observe(index)
        angle=Rotation.from_matrix(target[:3,:3] @ start[:3,:3].T).magnitude()
        count=max(1,int(math.ceil(max(np.linalg.norm(target[:3,3]-start[:3,3])/.025,
                                      angle/math.radians(5.)))))
        rotations=Slerp([0,1],Rotation.from_matrix([start[:3,:3],target[:3,:3]]))
        for u in np.linspace(0.,1.,count+1)[1:]:
            if stop_on_support is not None and self.placement_supported(index,stop_on_support):
                return
            observed=self.observe(index)
            if not all(self.gripping(who,index,s) for s in sides):
                raise RelayFault('Placement servo lost bilateral contact' if len(sides)==2 else 'Placement servo lost remaining-hand contact')
            # Keep the drive preload while updating the hand/object relation.
            # A long descent cannot assume that a friction pinch is rigid.
            self.grasp_transforms[who]={s:np.linalg.inv(observed) @ t
                for s,t in self.actual_hands(who,commanded=True).items()}
            pose=np.eye(4)
            pose[:3,3]=(1-u)*start[:3,3]+u*target[:3,3]
            pose[:3,:3]=rotations([u]).as_matrix()[0]
            if hasattr(self,'events'):
                actual=self.actual_hands(who)
                commanded=self.actual_hands(who,commanded=True)
                row=dict(t=self.time,robot=who,state='PLACEMENT_SEGMENT',box=index,
                         observed=observed.tolist(),desired=pose.tolist(),
                         hand_preload_m={s:float(np.linalg.norm(commanded[s][:3,3]-actual[s][:3,3])) for s in sides},
                         contacts=sorted(self.box_contacts(index)))
                self.events.append(row)
                print('PLACEMENT_TRACE',json.dumps(row),flush=True)
            yield from self.hands(who,self.hand_targets(who,pose,**({'sides':sides} if len(sides)==1 else {})),seconds=.45)
            error=np.linalg.norm(self.observe(index)[:3,3]-pose[:3,3])
            if error>.04:
                raise RelayFault(f'Placement servo tracking error: {error:.4f}m')

    def placement_neighbor_side(self, who, index, target):
        """Detect occupied same-layer rims, then map the rim to robot hands."""
        width, depth, height = self.c['crate']['size']
        neighbors=[]
        for other in range(len(self.initial)):
            if other==index:
                continue
            local=target[:3,:3].T @ (self.observe(other)[:3,3]-target[:3,3])
            if (abs(local[2])<.04 and abs(abs(local[0])-width)<.06
                    and abs(local[1])<depth*.5):
                neighbors.append(1 if local[0]>0 else -1)
        if not neighbors:
            return None
        if len(set(neighbors))>1:
            raise RelayFault('Both placement rims are obstructed; no single-hand exit corridor')
        sign=neighbors[0]
        targets=self.hand_targets(who,target)
        side=max(targets,key=lambda side:sign*(target[:3,:3].T @
                 (targets[side][:3,3]-target[:3,3]))[0])
        return side,sign

    def placement_side_clearance(self,target):
        if target[2,3]>self.c['pallet']['size'][2]+self.c['crate']['size'][2]/2:
            return self.settings.get('upper_placement_side_clearance',.065)
        return self.settings.get('placement_side_clearance',.035)

    def placement_support_corners(self):
        """Use the actual bottom feet, not the wider upper rim envelope."""
        if not hasattr(self,'_placement_support_corners'):
            parts=json.loads((ROOT/'assets/generated/relay/crate_colliders.json').read_text())
            corners=[]
            for part in parts:
                if part['name'].startswith('StackFoot'):
                    p=np.asarray(part['position']);h=np.asarray(part['size'])/2
                    corners.extend(p+np.array([x*h[0],y*h[1],-h[2]])
                                   for x in (-1,1) for y in (-1,1))
            if not corners:
                raise RelayFault('Crate bottom support geometry is missing')
            self._placement_support_corners=np.asarray(corners)
        return self._placement_support_corners

    def prepare_target_approach(self,who,position,final_position,dock):
        """Align a held upper-layer box in the clear lane, before approaching."""
        if final_position[2]<=self.c['pallet']['size'][2]+self.c['crate']['size'][2]/2:
            return
        index=self.held[who]
        observed=self.observe(index)
        target=np.eye(4)
        yaw=pallet_yaw(self.c)
        if math.cos(Rotation.from_matrix(observed[:3,:3]).as_euler('xyz')[2]-yaw)<0:
            yaw+=math.pi
        target[:3,:3]=Rotation.from_euler('z',yaw).as_matrix()
        target[:3,3]=final_position
        neighbor=self.placement_neighbor_side(who,index,target)
        target[:2,3]=np.asarray(position[:2])+self.base(who)[:2]-dock
        if neighbor is not None:
            target=self.edge_placement_pose(target,neighbor[1])
            # Reserve the side corridor while remaining upright in transit.
            # The neighbor-side tilt starts only after the base is docked.
            target[:3,:3]=Rotation.from_euler('z',yaw).as_matrix()
        # Lower in the clear side lane before combining forward reach with
        # transverse clearance. Keep the upright bottom above the support.
        target[2,3]=(max(final_position[2]+.06,min(observed[2,3],final_position[2]+.11))
                     if neighbor is not None else max(observed[2,3],target[2,3]+.06))
        self.event(who,'ALIGN_BEFORE_TARGET_APPROACH',box=index)
        yield from self.lower_held_box(who,index,target)

    def edge_placement_pose(self, target, sign):
        """Lower the occupied edge first while keeping the tilted wall clear."""
        width,_,height=self.c['crate']['size']
        angle=math.radians(self.settings['placement_edge_tilt_degrees'])
        pose=target.copy()
        pose[:3,:3]=target[:3,:3] @ Rotation.from_euler('y',sign*angle).as_matrix()
        edge=np.array([sign*max(abs(self.placement_support_corners()[:,0])),0.,0.])
        pose[:3,3]+=target[:3,:3] @ edge-pose[:3,:3] @ edge
        # Keep the whole tilted box behind its final side plane. This is a
        # continuous held-box leveling path, not a released staging/push step.
        pose[:3,3]-=target[:3,0]*sign*(height*math.sin(angle)+self.placement_side_clearance(target))
        return pose

    def placement_supported(self,index,position):
        """Require bottom contact, never count a same-layer neighbor as support."""
        pose=self.observe(index)
        width,depth,height=self.c['crate']['size']
        corners=self.placement_support_corners()
        lowest=float((corners @ pose[:3,:3].T+pose[:3,3])[:,2].min())
        if abs(lowest-position[2])>.015:
            return False
        contacts=self.box_contacts(index)
        if abs(position[2]-self.c['pallet']['size'][2])<.015:
            return any('/Collisions/TargetPallet' in p for p in contacts)
        for other,spec in enumerate(self.initial):
            if other!=index and spec['path'] in contacts:
                support=self.observe(other)
                if abs(support[2,3]+height-position[2])<.015:
                    return True
        return False

    def align_upper_support(self,who,index,target,tilted,sign):
        """Align both ends of the leading foot onto the hollow lower rim."""
        height=self.c['crate']['size'][2]
        if target[2,3]<=self.c['pallet']['size'][2]+height/2:
            return
        supports=[i for i,spec in enumerate(self.initial) if i!=index and
                  spec['path'] in self.box_contacts(index) and
                  abs(self.observe(i)[2,3]+height-target[2,3])<.015]
        if not supports:
            raise RelayFault('No lower crate available for depth alignment')
        support=min(supports,key=lambda i:np.linalg.norm(self.observe(i)[:2,3]-target[:2,3]))
        feet=self.placement_support_corners()
        edge=feet[np.isclose(sign*feet[:,0],max(sign*feet[:,0]))]
        inner=self.c['crate']['size'][1]/2-self.c['crate']['rim_width']
        for _ in range(7):
            below=self.observe(support);observed=self.observe(index)
            world=edge@observed[:3,:3].T+observed[:3,3]
            local=(world-below[:3,3])@below[:3,:3]
            plane=np.dot(observed[:3,1],target[:3,1])
            if (local[:,1].min()<=-inner-.0004 and local[:,1].max()>=inner+.0004 and
                    plane>math.cos(.005) and self.placement_supported(index,target[:3,3])):
                self.event(who,'SUPPORTED_DEPTH_ALIGNED',box=index,support=support,
                           foot_depth_span=[float(local[:,1].min()),float(local[:,1].max())])
                return
            goal=tilted.copy()
            axis=target[:3,1]
            goal[:3,3]+=axis*np.dot(below[:3,3]-target[:3,3],axis)
            yield from self.lower_held_box(who,index,goal)
        raise RelayFault('Leading foot does not span both lower crate rims; refusing release')

    def withdraw_placement_hand(self,who,side, *, hold=None, box=None, box_pose=None):
        if hold is not None and box is not None:
            # Keep the supported object pose, rather than a stale holding TCP,
            # while the load transfers from two friction grasps to one.
            start=self.actual_hands(who,commanded=True)[side].copy()
            raised=start.copy();raised[2,3]+=self.settings['placement_hand_clearance']
            retracted=raised.copy();yaw=self.base(who)[3]
            retracted[:2,3]-=.03*np.array([math.cos(yaw),math.sin(yaw)])
            for end in (raised,retracted):
                for u in np.linspace(0.,1.,5)[1:]:
                    free=start.copy();free[:3,3]=(1-u)*start[:3,3]+u*end[:3,3]
                    observed=self.observe(box)
                    if np.linalg.norm(observed[:3,3]-box_pose[:3,3])>.03:
                        raise RelayFault('Box slipped while withdrawing neighbor hand')
                    holding=box_pose @ np.linalg.inv(observed) @ self.actual_hands(who,commanded=True)[hold]
                    yield from self.hands(who,{side:free,hold:holding},seconds=.4)
                start=end
            return
        if hold is not None:
            self.anchored_hands[who]={hold:self.actual_hands(who,commanded=True)[hold].copy()}
        try:
            raised=self.actual_hands(who,commanded=True)[side].copy()
            raised[2,3]+=self.settings['placement_hand_clearance']
            yield from self.hands(who,{side:raised},seconds=2.)
            retracted=raised.copy()
            yaw=self.base(who)[3]
            retracted[:2,3]-=(.06 if hold is not None else .10)*np.array([math.cos(yaw),math.sin(yaw)])
            yield from self.hands(who,{side:retracted},seconds=2.)
        finally:
            self.anchored_hands[who]={}

    def place_sequential(self,who,index,target,side,sign, *, final_target=None):
        final_target=target.copy() if final_target is None else final_target.copy()
        other='right' if side=='left' else 'left'
        tilted=self.edge_placement_pose(target,sign)
        tilted[2,3]-=.0005
        # Align inside the transverse clearance corridor before descending.
        # Upper-layer transport was already aligned in the clear side lane.
        high=tilted.copy()
        high[2,3]+=(.03 if target[2,3]>self.c['pallet']['size'][2]+self.c['crate']['size'][2]/2 else self.c['crate']['size'][2]+.06)
        yield from self.lower_held_box(who,index,high)
        # Both hands descend together with the box tilted: the neighbor-side
        # edge reaches support first. Neither arm is anchored during descent.
        self.event(who,'LOWER_BOTH_HANDS',box=index,leading_side=side)
        yield from self.lower_held_box(who,index,tilted,sides=('left','right'),stop_on_support=target[:3,3])
        yield from self.wait(.3)
        # Resolve small compliant-drive residuals with both hands still closed.
        # Bound the correction; side contact must never authorize releasing.
        for _ in range(3):
            if self.placement_supported(index,target[:3,3]):
                break
            observed=self.observe(index)
            correction=tilted[:3,3]-observed[:3,3]
            if np.linalg.norm(correction)>.03:
                break
            corrected=tilted.copy()
            corrected[:3,3]+=np.clip(correction,-.012,.012)
            # Never turn tracking compensation into penetration of the deck.
            # The nominal tilted pose already includes its contact bias.
            corrected[2,3]=max(corrected[2,3],tilted[2,3])
            yield from self.lower_held_box(who,index,corrected,sides=('left','right'),stop_on_support=target[:3,3])
            yield from self.wait(.3)
        if not self.placement_supported(index,target[:3,3]):
            raise RelayFault('Neighbor-side edge has no bottom support; refusing single-hand release')
        yield from self.align_upper_support(who,index,target,tilted,sign)
        actual=self.observe(index)
        if sign*actual[2,0]>=-.005:
            raise RelayFault('Neighbor-side edge is not lower; refusing single-hand release')
        if not self.gripping(who,index,other):
            raise RelayFault('Remaining placement hand has no verified grasp')
        self.placement_single_hand[who]=dict(side=other,position=target[:3,3].copy())
        supported_pose=self.observe(index).copy()
        self.event(who,'RETRACT_NEIGHBOR_HAND',box=index,side=side,pose=supported_pose.tolist())
        yield from self.gripper(who,(side,),False,support=(index,other,supported_pose))
        yield from self.withdraw_placement_hand(who,side,hold=other,box=index,box_pose=supported_pose)
        self.anchored_hands[who]={side:self.actual_hands(who,commanded=True)[side].copy()}
        self.event(who,'LOWER_REMAINING_HAND',box=index,side=other)
        self.event(who,'ARM_PUSH_AND_LEVEL',box=index,side=other,
                   final_position=final_target[:3,3].tolist())
        fixed_base=self.base(who).copy()
        if who in getattr(self,'station_poses',{}):
            dock=self.station_poses[who]
            fixed_base[:2]=dock[:2]
            fixed_base[3]=dock[2]
        try:
            # Preserve support as the remaining gripper levels the box into
            # its exact final slot. The base stays at its dock throughout.
            start=self.observe(index)
            angle=math.atan2(-sign*start[2,0],start[2,2])
            for fraction in np.linspace(0.,1.,25)[1:]:
                theta=max(0.,angle)*(1-min(1.,fraction/.8))
                pose=target.copy()
                pose[:3,3]=(1-fraction)*target[:3,3]+fraction*final_target[:3,3]
                pose[:3,:3]=target[:3,:3] @ Rotation.from_euler('y',sign*theta).as_matrix()
                edge=np.array([sign*max(abs(self.placement_support_corners()[:,0])),0.,0.])
                pose[:3,3]+=target[:3,:3] @ edge-pose[:3,:3] @ edge
                # Keep the side gap until the box is flat. Closing it while
                # tilted lets compliant tracking error sweep the upper rim
                # into the neighbor even though the ideal path is tangent.
                clearance=self.placement_side_clearance(target)*min(1.,(1-fraction)/.2)
                pose[:3,3]-=target[:3,0]*sign*(self.c['crate']['size'][2]*math.sin(theta)+clearance)
                # Once supported, follow the foot pivot exactly. Reapplying
                # a downward contact bias at every segment accumulates force.
                yield from self.lower_held_box(who,index,pose,sides=(other,))
                actual=self.observe(index)
                # Rotation about the box's local Y axis is permitted; its
                # direction in world space must remain fixed (no yaw/twist).
                if np.dot(actual[:3,1],final_target[:3,1])<math.cos(self.settings['angle_tolerance']):
                    raise RelayFault('Box twisted out of its vertical placement plane')
                if abs(np.dot(actual[:3,3]-final_target[:3,3],final_target[:3,1]))>self.settings['position_tolerance']:
                    raise RelayFault('Longitudinal drift during transverse arm placement')
                b=self.base(who)
                if np.linalg.norm(b[:2]-fixed_base[:2])>.008 or abs(wrap(b[3]-fixed_base[3]))>.015:
                    raise RelayFault('Base moved during supported arm placement')
        finally:
            self.anchored_hands[who]={}
        if not self.placement_supported(index,target[:3,3]):
            raise RelayFault('Final placement has no bottom support')
        actual=self.observe(index)
        if (np.linalg.norm(actual[:3,3]-final_target[:3,3])>self.settings['position_tolerance'] or
                Rotation.from_matrix(actual[:3,:3] @ final_target[:3,:3].T).magnitude()>self.settings['angle_tolerance']):
            raise RelayFault('Arm placement has not reached its final pose; retaining remaining grip')
        self.releasing[who]=True
        yield from self.gripper(who,(other,),False)
        self.held[who]=None
        self.releasing[who]=False
        self.placement_single_hand.pop(who,None)
        yield from self.withdraw_placement_hand(who,other)

    def push_parameters(self,position):
        upper=position[2]>self.c['pallet']['size'][2]+self.c['crate']['size'][2]/2
        if upper:
            return (self.settings['upper_push_distance'],self.settings['upper_push_support_margin'],
                    np.asarray(position[:2]),np.asarray(world_half_extent(self.c['crate']['size'][:2],pallet_yaw(self.c))),
                    self.settings['upper_push_height'],True)
        return (self.settings['push_distance'],self.settings['push_support_margin'],
                np.asarray(self.c['pallet']['target']),np.asarray(world_half_extent(self.c['pallet']['size'][:2],pallet_yaw(self.c))),
                .11,False)

    def place(self, who, index, position, *, push=False):
        if not all(self.gripping(who,index,s) for s in ('left','right')):
            raise RelayFault('Cannot place without verified bilateral contact')
        observed=self.observe(index)
        self.grasp_transforms[who]={s:np.linalg.inv(observed) @ t for s,t in self.actual_hands(who).items()}
        a_place_paths=(yield from self.prepare_handoff_deposit(index)) if who=='A' else None
        target=np.eye(4)
        target[:3,3]=position
        desired_yaw=pallet_yaw(self.c) if who=='B' else 0.
        observed_yaw=Rotation.from_matrix(observed[:3,:3]).as_euler('xyz')[2]
        if math.cos(observed_yaw-desired_yaw)<0:
            desired_yaw+=math.pi
        target[:3,:3]=Rotation.from_euler('z',desired_yaw).as_matrix()
        # Select the neighbor at the FINAL slot before adding the staging
        # offset, which can otherwise hide it from the adjacency check.
        neighbor=self.placement_neighbor_side(who,index,target) if who=='B' else None
        self.event(who,'PLACE_APPROACH',box=index,push=bool(push and neighbor is None),
                   arm_push=neighbor is not None)
        final_target=target.copy()
        if push and neighbor is None:
            push_distance,support_margin,support_center,support_half,push_height,closed_push=self.push_parameters(position)
            # Stage toward the approached edge in the pallet frame, keeping
            # the hands clear of neighboring crates until they are withdrawn.
            sign=1. if math.sin(self.base(who)[3]-pallet_yaw(self.c))>=0 else -1.
            forward=np.array([*rotate_xy([0.,sign],pallet_yaw(self.c)),0.])
            target[:3,3]-=forward*push_distance
            offset=np.abs(target[:2,3]-support_center)
            transverse=int(abs(forward[0])>abs(forward[1]))
            half_box=world_half_extent(self.c['crate']['size'][:2],pallet_yaw(self.c))
            if (offset[transverse]+half_box[transverse]>support_half[transverse]+1e-8 or
                    np.any(offset>support_half-support_margin)):
                raise RelayFault('Push staging position would exceed its support footprint')
        if neighbor is not None:
            self.event(who,'SEQUENTIAL_STAGING',box=index,leading_side=neighbor[0],
                       staging_position=target[:3,3].tolist(),final_position=list(position))
            yield from self.place_sequential(who,index,target,*neighbor,final_target=final_target)
        else:
            above=target.copy()
            above[2,3]+=.06
            if who=='A':
                above[:3,:3]=(Rotation.from_euler('x',self.settings['handoff_place_tilt_degrees']*(1 if math.sin(self.base(who)[3])>=0 else -1),degrees=True).as_matrix() @ target[:3,:3])
                yield from self.hands(who,self.hand_targets(who,above),seconds=3.,planned=a_place_paths[0])
            else:
                yield from self.lower_held_box(who,index,above)
            target[2,3]-=.002
            yield from self.hands(who,self.hand_targets(who,target),seconds=2.5,
                                  planned=a_place_paths[1] if a_place_paths else None)
            yield from self.wait(.4)
            supported=self.placement_supported(index,position) if who=='B' else self.supported(index)
            if not supported:
                raise RelayFault('No bottom support contact; refusing to release')
            self.releasing[who]=True
            yield from self.gripper(who,('left','right'),False)
            self.held[who]=None
            self.releasing[who]=False
            withdrawal=self.hand_targets(who,target,spread=self.settings['approach_clearance'])
            if who=='B':
                for hand in withdrawal.values():
                    hand[2,3]+=self.settings['release_vertical_clearance']
            yield from self.hands(who,withdrawal,seconds=2.,planned=a_place_paths[2] if a_place_paths else None)
        if push and neighbor is None:
            self.event(who,'PUSH_IN',box=index)
            b=self.base(who)
            yield from self.navigate(who,b[:2]-forward[:2]*self.settings['push_base_retreat'],
                                     math.atan2(forward[1],forward[0]))
            yield from self.wait(.5)
            if self.perception in ('depth','rgbd'):
                self.depth_sensor.set_prior(index,target)
            pose=self.observe(index)
            offset=np.abs(pose[:2,3]-support_center)
            if (not self.placement_supported(index,position) or
                    np.any(offset>support_half-support_margin) or
                    np.linalg.norm(self.crates[index].get_linear_velocity())>.03 or
                    np.linalg.norm(self.crates[index].get_angular_velocity())>.05):
                raise RelayFault('Staged crate is not stably supported for pushing')
            if closed_push:
                yield from self.gripper(who,('left','right'),True)
            rot=Rotation.from_euler('z',self.base(who)[3]).as_matrix()
            t={}
            for side,sign in (('left',1),('right',-1)):
                hand=np.eye(4)
                hand[:3,3]=pose[:3,3]+rot @ [-self.c['crate']['size'][1]/2-.035,sign*.10,push_height]
                hand[:3,:3]=(rot @ Rotation.from_euler('z',-math.pi/2).as_matrix() @
                               Rotation.from_euler('x',math.pi/2).as_matrix() @
                               Rotation.from_euler('z',0 if side=='left' else math.pi).as_matrix())
                t[side]=hand
            yield from self.hands(who,t,seconds=3.)
            yield from self.push_with_base(who,index,position,forward,push_distance)
            b=self.base(who)
            yield from self.navigate(who,b[:2]-forward[:2]*.10,b[3])
        yield from self.verify_placement(who,index,position)

    def push_with_base(self,who,index,position,forward,distance):
        """Keep hand posture and push with slow base feedback from the box."""
        ctl=self.controls['Robot_'+who]
        start=self.base(who)
        self.station_poses.pop(who,None)
        began=last_progress=self.time
        best=float('inf')
        yaw=math.atan2(forward[1],forward[0])
        while self.time-began<30.:
            pose=self.observe(index)
            if self.perception in ('depth','rgbd'):
                self.depth_sensor.set_prior(index,pose)
            if math.acos(float(np.clip(pose[2,2],-1.,1.)))>math.radians(5.):
                raise RelayFault('Crate tilted excessively during the base push')
            error=np.asarray(position)-pose[:3,3]
            remaining=float(np.dot(error,forward))
            cross_error=error[:2]-remaining*forward[:2]
            if remaining<=.004 and np.linalg.norm(cross_error)<=.004:
                ctl.stop()
                b=self.base(who)
                self.station_poses[who]=np.array([*b[:2],b[3]])
                yield from self.wait(.5)
                self.event(who,'PUSH_POSITION_REACHED',box=index,remaining_m=remaining,
                           lateral_error_m=float(np.linalg.norm(cross_error)))
                return
            progress=float(np.linalg.norm(error[:2]))
            if progress<best-.002:
                best=progress
                last_progress=self.time
            b=self.base(who)
            travel=float(np.dot(b[:2]-start[:2],forward[:2]))
            lateral=b[:2]-start[:2]-travel*forward[:2]
            if travel>distance+.10 or np.linalg.norm(lateral)>.05 or self.time-last_progress>3.:
                raise RelayFault('Base push stalled or exhausted its allowed travel')
            speed=min(.035,max(.018,.3*remaining)) if remaining>.004 else 0.
            velocity=forward[:2]*speed+np.clip(1.5*cross_error,-.02,.02)
            body=Rotation.from_euler('z',-b[3]).as_matrix()[:2,:2] @ velocity
            heading=wrap(yaw-b[3])
            turn=float(np.clip(2*heading,-.1,.1)) if abs(heading)>.005 else 0.
            if turn and abs(turn)<.06:
                turn=math.copysign(.06,turn)
            ctl.set_twist(*body,turn)
            yield
        raise RelayFault('Base push timed out')

    def verify_placement(self, who, index, position):
        self.event(who,'VERIFY_PLACEMENT',box=index)
        start=self.time
        stable=0.
        while self.time-start<8.:
            crate=self.crates[index]
            p,q=crate.get_world_pose()
            rotation=Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix()
            angle=min(np.linalg.norm(Rotation.from_matrix(rotation @ Rotation.from_euler('z',a).as_matrix()).as_rotvec()) for a in (-(pallet_yaw(self.c) if who=='B' else 0.),math.pi-(pallet_yaw(self.c) if who=='B' else 0.)))
            valid=(np.linalg.norm(p-np.array(position))<self.settings['position_tolerance'] and
                   angle<self.settings['angle_tolerance'] and self.supported(index) and
                   not self.gripping(who,index) and
                   np.linalg.norm(crate.get_linear_velocity())<self.settings['linear_stable'] and
                   np.linalg.norm(crate.get_angular_velocity())<self.settings['angular_stable'])
            stable=stable+.05 if valid else 0.
            if stable>=self.settings['stable_seconds']:
                return
            yield
        raise RelayFault(f'Box {index} did not settle at target; '
                         f'position_error={np.linalg.norm(p-position):.4f}m, '
                         f'angle_error={angle:.4f}rad, supported={self.supported(index)}, '
                         f'gripper_contact={self.gripping(who,index)}, '
                         f'linear_speed={np.linalg.norm(crate.get_linear_velocity()):.4f}m/s, '
                         f'angular_speed={np.linalg.norm(crate.get_angular_velocity()):.4f}rad/s')

    def run_a(self):
        if self.skill_test=='handoff':
            self.event('A','FIXTURE_IDLE')
            return
        yield from self.prepare_startup('A')
        for index in self.inventory.queue:
            spec=self.initial[index]
            deposit_slot=source_handoff_slot(self.c,spec)
            side='LEFT'
            handoff_far=source_handoff_far(self.c,spec)
            self.event('A','NAVIGATE_SOURCE',box=index)
            yield from self.dock('A',spec['position'],far=bool(spec['row']),prepare=self.stow('A'))
            assisted=spec['column'] != 1 and self.skill_test != 'direct'
            # Facing the far row swaps the robot's left/right world sides.
            exposed='left' if (spec['column']==0) != bool(spec['row']) else 'right'
            yield from self.pick('A',index,assisted=assisted,exposed=exposed)
            self.inventory.transfer(index,'SOURCE','A')
            if self.skill_test and self.skill_test!='retreat':
                self.event('A','SKILL_COMPLETE',box=index)
                return
            yield from self.establish_upright_carry('A',index)
            self.event('A','CARRY_TO_'+side,box=index)
            yield from self.retreat('A',far=bool(spec['row']),handoff_far=handoff_far)
            aisle=self.settings['source_rear_aisle_y'] if handoff_far else self.settings['transit_y']
            yield from self.navigate('A',[-1.,aisle],-math.pi/2 if handoff_far else math.pi/2)
            yield from self.prepare_handoff_carry('A',index)
            self.event('A','HOLD_WAIT',box=index)
            action=yield from self.acquire('A',index,slot=deposit_slot)
            self.event('A','DOCK_HANDOFF_'+side,box=index)
            position=[*self.c['handoff']['slots'][deposit_slot],self.c['handoff']['size'][2]]
            yield from self.dock('A',position,handoff=True,far=handoff_far)
            yield from self.place('A',index,position)
            yield from self.retreat('A',handoff=True,far=handoff_far)
            self.refresh_handoff(0.)
            self.handoff.commit(action)
            self.inventory.transfer(index,'A','L')
            if self.skill_test=='retreat':
                self.event('A','SKILL_COMPLETE',box=index)
                return
        self.event('A','COMPLETE')

    def run_b(self):
        if self.skill_test in ('grasp','direct','retreat'):
            return
        yield from self.prepare_startup('B')
        for _ in range(self.count):
            # Return via the front aisle, then face the entire receiving box.
            slot_xy=self.c['handoff']['slots'][1]
            view_xy=[slot_xy[0]-.08,slot_xy[1]-self.settings['b_handoff_observation_distance']]
            base=self.base('B')
            if np.linalg.norm(base[:2]-view_xy)>.008:
                yield from self.navigate('B',[base[0],self.settings['transit_y']],math.pi/2)
                yield from self.navigate('B',[view_xy[0],self.settings['transit_y']],math.pi/2)
            yield from self.navigate('B',view_xy,math.pi/2)
            self.event('B','WAIT_RIGHT')
            start=self.time
            while True:
                ready=[i for i,o in self.inventory.owners.items() if o=='R']
                if ready:
                    index=ready[0]
                    break
                if self.time-start>self.settings['max_wait_seconds']:
                    raise RelayFault('B waiting for R timed out')
                yield
            target_index=self.inventory.reserve_target(index)
            slot=self.targets[target_index]
            action=yield from self.acquire('B',index)
            self.event('B','DOCK_HANDOFF_RIGHT',box=index)
            yield from self.dock('B',[*self.c['handoff']['slots'][1],self.c['handoff']['size'][2]],
                                 handoff=True,prepare=self.prepare_handoff_pick('B',index))
            yield from self.pick('B',index)
            yield from self.prepare_handoff_carry('B',index)
            yield from self.retreat('B',handoff=True)
            self.refresh_handoff(0.)
            self.handoff.commit(action)
            self.inventory.transfer(index,'R','B')
            self.event('B','CARRY_TO_TARGET',box=index,target=target_index)
            dock_position=list(slot['position'])
            neighbor=None
            if slot['column']!=1:
                probe=np.eye(4)
                probe[:3,:3]=Rotation.from_euler('z',pallet_yaw(self.c)).as_matrix()
                probe[:3,3]=slot['position']
                neighbor=self.placement_neighbor_side('B',index,probe)
                distance=self.push_parameters(slot['position'])[0]
                # Neighbor placements align depth before descending; only
                # transverse clearance is closed by the remaining arm.
                offset=rotate_xy([0.,(1 if slot['row'] else -1)*distance*(0. if neighbor else 1.)],pallet_yaw(self.c))
                dock_position[:2]=np.asarray(dock_position[:2])+offset
            yield from self.dock('B',dock_position,far=bool(slot['row']),placement_target=slot['position'],
                                 dock_distance=self.settings['neighbor_dock_distance'] if neighbor else None)
            yield from self.place('B',index,slot['position'],push=slot['column']!=1)
            yield from self.retreat('B',far=bool(slot['row']))
            self.inventory.transfer(index,'B','TARGET')
            self.event('B','BOX_COMPLETE',box=index)
            yield from self.stow('B')
        for index in self.inventory.completed:
            slot=self.targets[self.inventory.target_reservations[index]]
            yield from self.verify_placement('B',index,slot['position'])
        self.event('B','COMPLETE')

    def prepare_startup(self, who):
        """Back out with stationary arms before folding beside the handoff."""
        base=self.base(who)
        cx,cy=self.c['handoff']['center']
        sx,sy,_=self.c['handoff']['size']
        if abs(base[0]-cx)<sx/2+.5 and abs(base[1]-cy)<sy/2+.8:
            self.event(who,'STARTUP_CLEARANCE')
            aisle=self.settings['source_rear_aisle_y'] if base[1]>cy else self.settings['transit_y']
            # Preserve the heading and arm commands throughout the retreat;
            # never run the fold concurrently with this chassis movement.
            yield from self.navigate(who,[base[0],aisle],base[3],preserve_posture=True)
        self.event(who,'STARTUP_STOW')
        yield from self.stow(who)

    def stow(self, who):
        """Keep the unused hand out of a crate's outward staging corridor."""
        if self.held[who] is not None:
            raise RelayFault('Cannot stow while holding a crate')
        ctl=self.controls['Robot_'+who]
        working_pose=ctl.cfg.get('stow',ctl.cfg['home'])
        target={n:float(working_pose.get(n,0.)) for arm in self.ik.arms.values() for n in arm.names}
        target['openarm_left_joint1']=-1.4
        target['openarm_right_joint1']=1.4
        yield from self.move_joints(who,target,minimum=3.)

    def write_report(self):
        from pxr import UsdGeom
        stage=next(iter(self.controls['Robot_A'].drives.values())).GetPrim().GetStage()
        frame_positions={}
        hand_positions={}
        physical=getattr(self,'physics_poses',None)
        snapshot=physical.read() if physical is not None else None
        for who in ('A','B'):
            ctl=self.controls['Robot_'+who]
            frame_positions[who]={}
            for side in ('left','right'):
                frame=f'openarm_{side}_ee_base_link'
                path='/World/Robots/Robot_'+who+ctl.meta['frames'][frame]
                frame_positions[who][side]=(physical.frame_position(path,snapshot) if physical is not None else
                    list(UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0).ExtractTranslation()))
            try:
                hand_positions[who]={s:t.tolist() for s,t in self.actual_hands(who).items()}
            except ValueError:
                hand_positions[who]=None
        report=dict(passed=not self.skill_test and not self.error and len(self.inventory.completed)==self.count,
                    skill_passed=bool(self.skill_test and not self.error and
                        (self.states['A']=='SKILL_COMPLETE' or (self.skill_test=='handoff' and self.states['B']=='COMPLETE'))),
                    error=self.error, states=self.states, completed_boxes=len(self.inventory.completed),
                    requested_boxes=self.count, simulation_seconds=self.time,
                    wall_seconds=time.perf_counter()-self.wall_started,
                    ik=dict(self.ik.stats),
                    loop_performance=getattr(self,'loop_performance',{}),
                    perception=self.perception, hardware_validated=False,
                    perception_details=('ideal ray-hit instance labels, synthetic pan/tilt depth, known-model local 6D registration'
                                        if self.perception=='depth' else ('URDF RGB-D + NanoOWL + NanoSAM + FoundationPose; task-layout association' if self.perception=='rgbd' else 'simulator object poses; commissioning only')),
                    handoff_sensing='ideal pose-based region sensors',
                    force_control='compliant position drives; no wrist force sensor loop',
                    self_collision_enabled=False,
                    finger_collision_overrides=list(self.finger_collision_overrides),
                    physics_state_valid=self.physics_valid,
                    task_settings=self.settings,
                    skill_test=self.skill_test,
                    source_box=self.source_box,
                    fixture_initialization=self.skill_test in ('direct','handoff','retreat'),
                    fixture_target_slot=self.handoff_target if self.skill_test=='handoff' else None,
                    usd_ee_base_frame_positions=frame_positions,
                    synchronized_motion_samples=self.synchronized_samples,
                    localization='simulator_base_pose',
                    object_pose_writes_after_initialization=0, attachments_used=False,
                    active_contacts={path:sorted(others) for path,others in self.contacts.items()},
                    owners=self.inventory.owners, handoff=self.handoff.status(), events=self.events,
                    hands=hand_positions,
                    hand_goals={w:{s:t.tolist() for s,t in hands.items()} for w,hands in self.hand_goals.items()},
                    joints={w:{n:float(v) for n,v in self.joints(w).items() if n in self.controls['Robot_'+w].targets} for w in ('A','B')},
                    joint_targets={w:self.controls['Robot_'+w].sent.copy() for w in ('A','B')},
                    boxes=[dict(id=i,position=c.get_world_pose()[0].tolist(),
                                quaternion=c.get_world_pose()[1].tolist()) for i,c in enumerate(self.crates)])
        (OUTPUTS/'relay_demo_report.json').write_text(json.dumps(report,indent=2))
        if self.perception=='rgbd':
            from runlog import write_json
            write_json(self.depth_sensor.root/'relay_report.json',report)
        with (OUTPUTS/'relay_demo_runs.jsonl').open('a') as history:
            history.write(json.dumps(dict(timestamp=time.time(),passed=report['passed'],
                skill_passed=report['skill_passed'],error=self.error,simulation_seconds=self.time,
                settings=self.settings,events=self.events))+'\n')
        (OUTPUTS/'relay_demo_telemetry.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in self.telemetry))
        print('RELAY_DEMO_REPORT',json.dumps({k:v for k,v in report.items() if k not in ('events','boxes')}),flush=True)
        return report
