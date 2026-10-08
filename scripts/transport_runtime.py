"""Isaac bindings: simulated sensors, motor interface, contacts and evaluation.

Only the sensor and evaluator adapters read simulator poses. SLAM/navigation
receive laser ranges and encoders; manipulation receives depth-derived box
coordinates and joint encoders. No runtime object/robot pose writes or welds.
"""
import json
import hashlib
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema, PhysicsSchemaTools
import omni.physx
from omni.physx.bindings._physx import ContactEventType
from common import ROOT, OUTPUTS, table_by_id
from robot_control import RobotControl
from lidar_slam import LidarSLAM, rotation2, wrap
from arm_ik import ArmIK, fixed_frame_transform
from setup_task import PlacementTask


class ResetRequested(Exception):
    """Unwind a running task before restoring physics state on the main thread."""


class SimSensors:
    def __init__(self, robot, stage, meta, cfg):
        self.robot, self.stage, self.meta, self.cfg = robot, stage, meta, cfg
        self.query = omni.physx.get_physx_scene_query_interface()
        self.rng = np.random.default_rng(17)
        self.box_path = '/World/Boxes/Box_00'
        self.angles = np.linspace(-math.pi,math.pi,cfg['beams'],endpoint=False)
        self.head = fixed_frame_transform(ROOT/'assets/robots'/meta['source_urdf'], 'head_d455_depth_frame')

    def truth_transform(self):
        # This is the simulator's sensor-generation boundary, never a localization output.
        p,q = self.robot.get_world_pose()
        t = np.eye(4)
        t[:3,3] = p
        t[:3,:3] = Rotation.from_quat([q[1],q[2],q[3],q[0]]).as_matrix()
        return t

    def ray(self, origin, direction, max_range):
        closest = [float(max_range), None]
        def hit(h):
            path = str(h.collision)
            if not path.startswith('/World/Robot') and h.distance < closest[0]:
                closest[:] = [float(h.distance),path]
            return True
        self.query.raycast_all(tuple(map(float,origin)),tuple(map(float,direction)),float(max_range),hit)
        return closest

    def laser(self):
        t = self.truth_transform()
        origin = t[:3,3] + t[:3,:3] @ np.array([0,0,self.cfg['height']])
        directions = np.c_[np.cos(self.angles),np.sin(self.angles),np.zeros(len(self.angles))] @ t[:3,:3].T
        ranges = np.array([self.ray(origin,d,self.cfg['max_range'])[0] for d in directions])
        hits = ranges < self.cfg['max_range']-.01
        ranges[hits] += self.rng.normal(0,self.cfg['range_noise'],np.sum(hits))
        return np.clip(ranges,.01,self.cfg['max_range'])

    def box_depth(self, box_size, floor=False):
        """Segmented ray depth with calibrated head extrinsics and known box size.

        Instance labels model ideal segmentation; no box transform is queried.
        Returns the box center in base_footprint coordinates, or None if hidden.
        """
        world_camera = self.truth_transform() @ self.head
        points = []
        table_points = []
        columns = []
        rows = []
        for row,pitch in enumerate(np.linspace(-math.radians(29),math.radians(29),35)):
            for column,yaw in enumerate(np.linspace(-math.radians(43.5),math.radians(43.5),61)):
                d = np.array([math.cos(pitch)*math.cos(yaw),math.cos(pitch)*math.sin(yaw),math.sin(pitch)])
                distance,path = self.ray(world_camera[:3,3],world_camera[:3,:3] @ d,2.5)
                if path and path.startswith(self.box_path + '/'):
                    # Back-project measured depth through nominal camera calibration.
                    distance += self.rng.normal(0,.0005)
                    points.append(self.head[:3,3]+self.head[:3,:3] @ (d*distance))
                    columns.append(column)
                    rows.append(row)
                elif path and (path.endswith('/Floor') if floor else path.endswith('_Top')):
                    table_points.append(self.head[:3,3]+self.head[:3,:3] @ (d*distance))
        from box_perception import fit_box
        result = fit_box(points,table_points,rows,columns,box_size)
        if result is None:
            return None
        center,self.last_box_yaw = result
        return center,len(points)



class TransportRuntime:
    def __init__(self, app, world, robot, box, stage, scene_cfg, task_cfg, initial_pose, render_every=12):
        self.app,self.world,self.robot,self.box,self.stage = app,world,robot,box,stage
        self.c,self.cfg = scene_cfg,task_cfg
        self.dt = scene_cfg['physics']['dt']
        self.ctl = RobotControl(stage)
        self.dofs = robot.dof_names
        self.wheel_indices = [self.dofs.index(w['joint']) for w in self.ctl.meta['wheels']]
        self.iks = {side:ArmIK(ROOT/'assets/robots'/self.ctl.cfg['source_urdf'],side) for side in ('left','right')}
        laser = task_cfg['slam']
        self.slam = LidarSLAM(initial_pose,resolution=laser['resolution'],max_range=laser['max_range'],beams=laser['beams'])
        self.sensors = SimSensors(robot,stage,self.ctl.meta,laser)
        self.time,self.steps = 0.,0
        self.wall_started = None
        self.before_render = None
        self.performance = dict(control=0., physics=0., rendering=0., sensing=0., evaluation=0.)
        self.render_every = render_every
        self.contacts = {}
        self.box_path = '/World/Boxes/Box_00'
        self.box_index = 0
        self.completed_boxes = []
        self.all_contacts = {}
        self.events = []
        self.phase = 'INITIALIZE'
        self.abort_requested = False
        self.reset_requested = False
        self.test_reset_callback = None
        self.retreat_distance = 0.0
        self.status_model = None
        self.viewer_transform = None
        self.follow_view = True
        self.initial_box = self.box.get_world_pose()[0].copy()
        self.minimum_carry_height = float('inf')
        self.minimum_carry_bottom_height = float('inf')
        self.maximum_carry_tilt_deg = 0.0
        self.scan_failed = 0
        self.ranges = None
        self.last_scan = -100.
        self.released = False
        self.placement = PlacementTask(scene_cfg)
        source_config = dict(scene_cfg, task=dict(scene_cfg['task'], target=scene_cfg['task']['source']))
        self.source_placement = PlacementTask(source_config)
        self.max_box_height = 0.
        self.localization_errors = []
        self.held_samples = 0
        self.lost_samples = 0
        self.last_path = []
        self.tracking_squared_error = 0.0
        self.tracking_samples = 0
        self.telemetry = []
        PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath('/World/Boxes/Box_00')).CreateThresholdAttr(0.)
        self.subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(self.on_contacts)

    def configure_boxes(self, boxes):
        import copy
        self.boxes = boxes
        self.initial_box_positions = [box.get_world_pose()[0].copy() for box in boxes]
        self.placements = []
        self.box_released = [False] * len(boxes)
        target = table_by_id(self.c, self.c['task']['target'])
        for offset in self.cfg['demo']['slot_offsets']:
            config = copy.deepcopy(self.c)
            slot = table_by_id(config, config['task']['target'])
            slot['xy'][0] = target['xy'][0] + offset
            slot['size'][0] = self.cfg['demo']['slot_width']
            self.placements.append(PlacementTask(config))
        self.select_box(0)

    def select_box(self, index):
        self.box_index = index
        self.box = self.boxes[index]
        self.initial_box = self.initial_box_positions[index]
        self.box_path = f'/World/Boxes/Box_{index:02d}'
        self.sensors.box_path = self.box_path
        self.contacts = self.all_contacts.setdefault(self.box_path, {})
        self.placement = self.placements[index]
        self.released = self.box_released[index]
        self.lost_samples = 0

    def on_contacts(self, headers, data):
        for h in headers:
            a,b = [str(PhysicsSchemaTools.intToSdfPath(p)) for p in (h.actor0,h.actor1)]
            for box_path, other in ((a, b), (b, a)):
                if not box_path.startswith('/World/Boxes/Box_'):
                    continue
                contacts = self.all_contacts.setdefault(box_path, {})
                if h.type == ContactEventType.CONTACT_LOST:
                    contacts.pop(other, None)
                elif h.num_contact_data:
                    contacts[other] = self.time
            self.contacts = self.all_contacts.setdefault(self.box_path, {})

    def contacting(self, text):
        # Contact remains valid while a body sleeps; CONTACT_LOST clears it.
        return any(text in p for p in self.contacts)

    def display_status(self):
        if 'demo' not in self.cfg:
            return self.phase
        labels = {'BUILD_MAP': 'Preparing', 'NAVIGATE_FLOOR_BOX': 'Approaching box',
                  'OBSERVE_FLOOR_BOX': 'Locating box', 'FLOOR_OBSERVED': 'Aligning',
                  'PREGRASP': 'Reaching down', 'SQUEEZE': 'Gripping',
                  'LIFT_BOX': 'Lifting from floor', 'CARRY': 'Moving to workbench',
                  'PLACE_APPROACH': 'Aligning at workbench', 'LOWER_BOX': 'Placing',
                  'RELEASE': 'Releasing', 'VERIFY_PLACEMENT': 'Checking placement',
                  'CLEAR_WORKBENCH': 'Clearing workbench', 'RETREAT': 'Backing away', 'RETREAT_COMPLETE': 'Backing away',
                  'BOX_COMPLETE': 'Box delivered', 'COMPLETE': 'Batch complete'}
        return f'{len(self.completed_boxes)}/{len(self.boxes)} | Box {self.box_index+1} | {labels.get(self.phase, self.phase)} | {self.time:.1f}s'

    def event(self, phase, **info):
        self.phase = phase
        item = dict(time=round(self.time,3),phase=phase,**info)
        if 'demo' in self.cfg:
            item.update(box_index=self.box_index, completed=len(self.completed_boxes), total=len(self.boxes))
            item.update(wall_seconds=round(time.perf_counter()-self.wall_started, 3) if self.wall_started else 0.,
                        performance={k: round(v, 3) for k, v in self.performance.items()})
        self.events.append(item)
        if self.status_model is not None:
            self.status_model.set_value(self.display_status())
        print('TRANSPORT_EVENT',json.dumps(item),flush=True)

    def joint_values(self, side):
        positions = self.robot.get_joint_positions()
        return np.array([positions[self.dofs.index(n)] for n in self.iks[side].names])

    def step(self, duration=None):
        if self.wall_started is None:
            self.wall_started = time.perf_counter()
        count = 1 if duration is None else max(1,round(duration/self.dt))
        for _ in range(count):
            if self.test_reset_callback is not None:
                self.test_reset_callback()
            if self.reset_requested:
                self.ctl.stop()
                raise ResetRequested()
            if self.abort_requested:
                self.ctl.stop()
                raise RuntimeError('Transport aborted by user')
            if not self.app.is_running():
                raise RuntimeError('Window closed')
            if self.time > self.cfg['timeout']:
                raise RuntimeError('Task time limit exceeded')
            tick = time.perf_counter()
            self.ctl.update(self.dt)
            self.performance['control'] += time.perf_counter()-tick
            if self.viewer_transform is not None and self.follow_view and self.steps % self.render_every == 0:
                x,y = map(float,self.slam.pose[:2])
                self.viewer_transform.Set(Gf.Matrix4d().SetLookAt(
                    Gf.Vec3d(x+1.8,y+1.8,1.7),Gf.Vec3d(x,y-.2,.7),Gf.Vec3d(0,0,1)).GetInverse())
            render = self.steps % self.render_every == 0
            if render and self.before_render is not None:
                self.before_render(self.steps // self.render_every)
            tick = time.perf_counter()
            self.world.step(render=render)
            self.performance['rendering' if render else 'physics'] += time.perf_counter()-tick
            self.steps += 1
            self.time += self.dt
            if 'demo' in self.cfg and self.status_model is not None and self.steps % 60 == 0:
                self.status_model.set_value(self.display_status())
            tick = time.perf_counter()
            rates = self.robot.get_joint_velocities()[self.wheel_indices]
            self.slam.predict(self.ctl.ik.forward(rates.tolist()),self.dt)
            if self.time-self.last_scan >= 1/self.cfg['slam']['scan_hz']:
                self.last_scan = self.time
                self.ranges = self.sensors.laser()
                accepted = self.slam.update(self.ranges)
                self.scan_failed = 0 if accepted else self.scan_failed+1
                if self.scan_failed > 10:
                    self.ctl.stop()
                    raise RuntimeError('SLAM lost: scan match rejected for 2 seconds')
            self.performance['sensing'] += time.perf_counter()-tick
            tick = time.perf_counter()
            # Ground truth below is evaluation/safety only, never motion targeting.
            bp,bq = self.box.get_world_pose()
            rp,rq = self.robot.get_world_pose()
            if not np.all(np.isfinite(np.r_[bp,bq,rp,rq])):
                raise RuntimeError('Nonfinite physics state')
            if abs(rq[1])+abs(rq[2]) > .25:
                raise RuntimeError('Robot tilt safety stop')
            self.max_box_height = max(self.max_box_height,float(bp[2]))
            if self.steps % 120 == 0:
                self.localization_errors.append(float(np.linalg.norm(rp[:2]-self.slam.pose[:2])))
            if self.steps % max(1, round(.1 / self.dt)) == 0:
                names = [n for n, spec in self.ctl.specs.items() if spec['kind'] in ('arm', 'gripper')]
                measured = self.robot.get_joint_positions()
                feedback = [float(measured[self.dofs.index(n)]) for n in names]
                commanded = [self.ctl.sent[n] for n in names]
                self.tracking_squared_error += float(np.sum((np.array(feedback)-commanded)**2))
                self.tracking_samples += len(names)
                self.telemetry.append(dict(time_s=self.time, phase=self.phase, box_index=self.box_index, joint_names=names,
                    command_rad=commanded, measured_rad=feedback, wheel_velocity_rad_s=rates.tolist(),
                    estimated_base_pose=self.slam.pose.tolist(), evaluation_box_position_m=bp.tolist(), evaluation_box_quaternion_wxyz=bq.tolist(),
                    evaluation_base_position_m=rp.tolist()))
            rot = Rotation.from_quat([bq[1],bq[2],bq[3],bq[0]]).as_matrix()
            extents = abs(rot) @ (np.array(self.c['box']['size'])/2)
            self.placement.update(position=bp,half_extents=extents,
                linear_velocity=self.box.get_linear_velocity(),angular_velocity=self.box.get_angular_velocity(),
                released=self.released,supported=self.contacting('/'+self.c['task']['target']+'_Top'),dt=self.dt)
            self.source_placement.update(position=bp, half_extents=extents,
                linear_velocity=self.box.get_linear_velocity(), angular_velocity=self.box.get_angular_velocity(),
                released=not self.contacting('/World/Robot'),
                supported=self.contacting('/'+self.c['task']['source']+'_Top'), dt=self.dt)
            if hasattr(self, 'boxes'):
                for i, other_box in enumerate(self.boxes):
                    if i == self.box_index:
                        continue
                    op, oq = other_box.get_world_pose()
                    matrix = Rotation.from_quat([oq[1], oq[2], oq[3], oq[0]]).as_matrix()
                    contacts = self.all_contacts.get(f'/World/Boxes/Box_{i:02d}', {})
                    self.placements[i].update(position=op, half_extents=abs(matrix) @ (np.array(self.c['box']['size'])/2),
                        linear_velocity=other_box.get_linear_velocity(), angular_velocity=other_box.get_angular_velocity(),
                        released=self.box_released[i], supported=any('/'+self.c['task']['target']+'_Top' in p for p in contacts), dt=self.dt)
            if self.phase in ('LIFT_BOX','CARRY','UNDOCK','PLACE_APPROACH'):
                if self.phase != "LIFT_BOX":
                    self.minimum_carry_height = min(self.minimum_carry_height,float(bp[2]))
                    self.minimum_carry_bottom_height = min(self.minimum_carry_bottom_height,float(bp[2]-extents[2]))
                    self.maximum_carry_tilt_deg = max(self.maximum_carry_tilt_deg,math.degrees(math.acos(float(np.clip(rot[2,2],-1,1)))))
                both = self.contacting('openarm_left') and self.contacting('openarm_right')
                self.held_samples += int(both)
                self.lost_samples = 0 if both else self.lost_samples+1
                if self.lost_samples*self.dt > .65:
                    self.ctl.stop()
                    raise RuntimeError('Lost bilateral grasp contact')
            self.performance['evaluation'] += time.perf_counter()-tick

    def source_placement_success(self):
        return self.source_placement.success

    def verify_initial_state(self, initial):
        # Evaluation-only initialization check: reject late physics rebuilds/reset.
        p, q = self.robot.get_world_pose()
        yaw = Rotation.from_quat([q[1],q[2],q[3],q[0]]).as_euler('xyz')[2]
        box_p, _ = self.box.get_world_pose()
        if (np.linalg.norm(p[:2]-np.asarray(initial[:2])) > .05 or
                abs(wrap(yaw-initial[2])) > .05 or
                np.linalg.norm(box_p[:2]-self.initial_box[:2]) > .02):
            raise RuntimeError('Simulation initialization changed before task execution')

    def move_joints(self, targets, timeout=16, tolerance=.035):
        for n,v in targets.items():
            self.ctl.set_joint(n,v)
        start = self.time
        stable = 0
        while self.time-start < timeout:
            self.step()
            q = self.robot.get_joint_positions()
            error = max(abs(q[self.dofs.index(n)]-v) for n,v in targets.items())
            sent_error = max(abs(self.ctl.sent[n]-v) for n,v in targets.items())
            stable = stable+1 if error < tolerance and sent_error < .002 else 0
            if stable*self.dt >= .25:
                return
        raise RuntimeError(f'Joint tracking timeout: error={error:.4f}')

    def solve_pair(self, center, half_width, seed=None):
        targets = {}
        for side,sign in (('left',1),('right',-1)):
            target = np.array(center)+np.array([0,sign*half_width,0])
            q = self.joint_values(side) if seed is None else seed[side]
            from arm_ik import squeeze_orientation
            result = self.iks[side].solve(target,squeeze_orientation(side),q,lift=self.ctl.sent.get('lift_joint', 0.0))
            if not result.success:
                raise RuntimeError(f'{side} IK infeasible: position={result.position_error:.4f}m angle={result.angle_error:.3f}rad target={target.tolist()}')
            targets.update(result.joints)
        return targets

    def pair_move(self, center, half_width, seconds=3.):
        # Cartesian waypoints are solved before motion; failure leaves the robot still.
        # A loaded friction grasp has elastic tracking error. Restarting each
        # segment from measured TCPs releases the squeeze preload and drops
        # the box. Continue the demo from sent targets, including release/withdrawal,
        # to preserve both preload and the selected continuous IK branch.
        loaded = 'demo' in self.cfg
        seeds = {side:np.array([self.ctl.sent[n] for n in self.iks[side].names]) if loaded else self.joint_values(side)
                 for side in self.iks}
        current = {side:self.iks[side].fk(seeds[side],self.ctl.sent.get('lift_joint', 0.0))[:3,3]
                   for side in self.iks}
        waypoints = []
        from arm_ik import squeeze_orientation
        samples_hz = 30 if 'demo' in self.cfg else 10
        for u in np.linspace(0,1,max(2,round(seconds*samples_hz)))[1:]:
            targets = {}
            for side,sign in (('left',1),('right',-1)):
                dest = np.array(center)+[0,sign*half_width,0]
                point = (1-u)*current[side]+u*dest
                ans = self.iks[side].solve(point,squeeze_orientation(side),seeds[side],
                                          lift=self.ctl.sent.get('lift_joint', 0.0),attempts=2)
                if not ans.success:
                    raise RuntimeError(f'Cartesian IK path infeasible: {side}, error {ans.position_error:.4f}')
                solution = np.array(list(ans.joints.values()))
                if np.max(np.abs(solution-seeds[side])) > .35:
                    raise RuntimeError('Cartesian IK branch discontinuity: '+side)
                seeds[side] = solution
                targets.update(ans.joints)
            waypoints.append(targets)
        for target in waypoints:
            for n,v in target.items(): self.ctl.set_joint(n,v)
            travel_time = max(abs(v-self.ctl.sent[n])/min(self.ctl.specs[n]["velocity"],self.ctl.cfg["arm"]["command_velocity_limit"]) for n,v in target.items())
            self.step(max(seconds/len(waypoints),travel_time))
        self.step(.5)

    def navigate(self, goal, yaw=-math.pi/2, carry=False, docking=False):
        goal = np.asarray(goal,dtype=float)
        start = self.time
        last_plan = -100.
        last_status = -100.
        closest = float('inf')
        last_progress = start
        path = []
        speed = self.cfg['navigation']['carry_speed' if carry else 'speed']
        if docking: speed = self.cfg['demo'].get('docking_speed', .09) if 'demo' in self.cfg else .055
        while self.time-start < self.cfg['navigation']['timeout']:
            pos = self.slam.pose
            delta = goal-pos[:2]
            angle = wrap(yaw-pos[2])
            if np.linalg.norm(delta) < closest-.015:
                closest = float(np.linalg.norm(delta))
                last_progress = self.time
            elif docking and 'demo' in self.cfg and self.time-last_progress > 12:
                self.ctl.stop()
                raise RuntimeError('Docking stalled: no progress for 12 seconds')
            # Map cells are 5 cm: final precision comes from depth/range servo,
            # not an unrealistically tight scan-map stopping threshold.
            tolerance = .015 if docking and 'demo' in self.cfg else self.cfg['navigation']['position_tolerance']
            if self.time-last_status > 5:
                print('NAV_PROGRESS',json.dumps(dict(time=round(self.time,2),pose=pos.tolist(),goal=goal.tolist(),error=float(np.linalg.norm(delta)),angle=angle)),flush=True)
                last_status = self.time
            if np.linalg.norm(delta)<tolerance and abs(angle)<self.cfg['navigation']['yaw_tolerance']:
                self.ctl.stop()
                self.step(self.cfg.get('demo', {}).get('navigation_settle', .5))
                return
            if not docking and self.time-last_plan > 1.0:
                radius = self.cfg['slam']['footprint_radius'] + (.09 if carry else 0)
                path = self.slam.plan(goal,radius)
                self.last_path = [p.tolist() for p in path]
                last_plan = self.time
                if not path:
                    raise RuntimeError('No collision-free path in the current SLAM map')
            target = goal
            if path and not docking and np.linalg.norm(delta) > .15:
                nearest = int(np.argmin([np.linalg.norm(p-pos[:2]) for p in path]))
                lookahead = max(5, math.ceil(speed/(1.2*self.slam.resolution))) if 'demo' in self.cfg else 5
                target = path[min(len(path)-1,nearest+lookahead)]
            world_v = 1.2*(target-pos[:2])
            world_v *= min(1,speed/max(np.linalg.norm(world_v),1e-9))
            if abs(angle) > .2 and not carry: world_v *= .2
            body_v = rotation2(-pos[2]) @ world_v
            # Range-based emergency stop, excluding the known table docking distance.
            if self.ranges is not None and np.linalg.norm(body_v) > .005:
                direction = math.atan2(body_v[1],body_v[0])
                mask = np.abs([wrap(a-direction) for a in self.slam.angles]) < .3
                clearance = .22 if docking else .36
                if np.min(self.ranges[mask]) < clearance:
                    self.ctl.stop()
                    raise RuntimeError('Laser obstacle safety stop')
            self.ctl.set_twist(float(body_v[0]),float(body_v[1]),float(np.clip(1.5*angle,-.35,.35)))
            self.step(.05)
        self.ctl.stop()
        raise RuntimeError('Navigation timeout')

    def retreat(self):
        """Simple timed reverse after release; no distance feedback or calibration."""
        distance = self.cfg['manipulation'].get('retreat_distance', .5)
        self.event('RETREAT', nominal_distance=distance)
        if 'demo' in self.cfg:
            goal = self.slam.pose[:2] + rotation2(self.slam.pose[2]) @ np.array([-distance, 0.])
            self.navigate(goal, docking=True)
            self.retreat_distance = distance
            self.event('RETREAT_COMPLETE', nominal_distance=distance)
            return
        self.ctl.set_twist(-.1, 0., 0.)
        try:
            self.step(distance / .1)
        finally:
            self.ctl.stop()
        self.step(.5)
        self.retreat_distance = distance  # commanded nominal travel, not a measurement
        self.event('RETREAT_COMPLETE', nominal_distance=distance)

    def snapshot(self, name):
        if not self.cfg.get('capture_stages', True):
            return
        import omni.replicator.core as rep
        from PIL import Image
        # Evaluation-only camera follows the actual robot for readable evidence.
        p,_ = self.robot.get_world_pose()
        p = list(map(float,p))
        camera = rep.create.camera(position=(p[0]+1.7,p[1]+1.5,1.7),look_at=(p[0],p[1]-.2,.65),focal_length=25)
        product = rep.create.render_product(camera,(960,720))
        rgb = rep.AnnotatorRegistry.get_annotator('rgb')
        rgb.attach(product)
        for _ in range(6): rep.orchestrator.step(rt_subframes=2,delta_time=0.)
        prefix = f'demo_box{self.box_index+1}' if 'demo' in self.cfg else 'transport'
        Image.fromarray(rgb.get_data()).save(str(OUTPUTS/f'{prefix}_{name}.png'))
        rgb.detach(product)
        product.destroy()
        import omni.timeline
        omni.timeline.get_timeline_interface().play()

    def report(self, error=None, mode='transport'):
        p,q = self.box.get_world_pose()
        test_success = (mode == 'manipulation-test' and self.phase == 'MANIPULATION_TEST_COMPLETE') or (mode == 'navigation-test' and self.phase == 'NAVIGATION_TEST_COMPLETE')
        report = dict(model=self.ctl.cfg["model"], source_urdf=self.ctl.cfg["source_urdf"],
                      robot_mass_kg=self.ctl.meta["total_mass"], hardware_validated=False,
                      solver_iterations=self.cfg.get('demo', {}).get('solver_iterations', self.ctl.meta.get("solver_iterations")),
                      source_sha256=hashlib.sha256((ROOT/"assets/robots"/self.ctl.cfg["source_urdf"]).read_bytes()).hexdigest(),
                      robot_config=self.ctl.cfg, task_config=self.cfg,
                      box_size_m=self.c["box"]["size"], box_mass_kg=self.c["box"]["mass"],
                      mode=mode,passed=bool((test_success or (mode == 'transport' and self.phase == 'COMPLETE'
                      and self.placement.success and not self.contacting('/World/Robot'))) and not error),
                      placement_success=bool(self.placement.success),error=error,phase=self.phase,
                      retreat_nominal_distance=self.retreat_distance,retreat_method='SLAM position and heading feedback' if 'demo' in self.cfg else 'timed reverse; no distance feedback',robot_box_contact=self.contacting('/World/Robot'),
                      duration_s=self.time,
                      wall_duration_s=time.perf_counter()-self.wall_started if self.wall_started else 0.,
                      performance_seconds=self.performance,
                      joint_tracking_rmse_rad=math.sqrt(self.tracking_squared_error/self.tracking_samples) if self.tracking_samples else None,
                      placement_error_m=float(np.linalg.norm(p[:2]-np.asarray(table_by_id(self.c,self.c['task']['target'])['xy']))) if self.placement.success else None,
                      simulation_seconds=self.time,events=self.events,box_position=p.tolist(),
                      max_box_height=self.max_box_height,box_displacement=(p-self.initial_box).tolist(),
                      minimum_carry_height=None if not np.isfinite(self.minimum_carry_height) else self.minimum_carry_height,
                      minimum_carry_bottom_height=None if not np.isfinite(self.minimum_carry_bottom_height) else self.minimum_carry_bottom_height,
                      maximum_carry_tilt_deg=self.maximum_carry_tilt_deg,
                      bilateral_contact_steps=self.held_samples,
                      slam_scans=self.slam.scans,slam_match_error=self.slam.match_error,
                      localization_max_error=max(self.localization_errors,default=0),
                      localization_final_error=self.localization_errors[-1] if self.localization_errors else None,
                      map_pose=self.slam.pose.tolist(),encoder_odometry=self.slam.odom_pose.tolist(),
                      simulator_pose_feedback_for_navigation=False,
                      simulator_pose_used_for_evaluation_and_safety=True,
                      object_pose_writes_after_initialization=0,
                      grasp_method='two pinch grippers with configured finger opening, friction contacts; no attachment joint',
                      perception='simulated planar lidar; ideal instance-segmented ray depth; known box dimensions')
        if 'demo' in self.cfg:
            report['boxes'] = [dict(index=i, released=self.box_released[i], placement_success=bool(p.success), position=self.boxes[i].get_world_pose()[0].tolist(), quaternion_wxyz=self.boxes[i].get_world_pose()[1].tolist(), initial_position=self.initial_box_positions[i].tolist(),
                linear_velocity=self.boxes[i].get_linear_velocity().tolist(), angular_velocity=self.boxes[i].get_angular_velocity().tolist(),
                contacts=list(self.all_contacts.get(f'/World/Boxes/Box_{i:02d}', {})), stable_seconds=p.stable_seconds) for i, p in enumerate(self.placements)]
            report['completed_boxes'] = len(self.completed_boxes)
            report['passed'] = bool(not error and self.phase == 'COMPLETE' and len(self.completed_boxes) == len(self.boxes) and all(p.success for p in self.placements) and all(not any('/World/Robot' in path for path in self.all_contacts.get(f'/World/Boxes/Box_{i:02d}', {})) for i in range(len(self.boxes))))
        self.slam.save(OUTPUTS)
        report_name = 'transport_report.json' if mode == 'transport' else 'transport_'+mode.replace('-test','')+'_report.json'
        if 'demo' in self.cfg:
            report_name = 'demo_report.json'
        telemetry_path = OUTPUTS / (report_name.replace('_report.json', '_telemetry.jsonl'))
        telemetry_path.write_text(''.join(json.dumps(row)+'\n' for row in self.telemetry))
        report['telemetry_file'] = telemetry_path.name
        (OUTPUTS/report_name).write_text(json.dumps(report,indent=2))
        print('TRANSPORT_REPORT',json.dumps(report),flush=True)
        return report
