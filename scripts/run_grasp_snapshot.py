"""Execute measured snapshot trajectories in an isolated Isaac scene, never hardware."""
import json
import math
import os
from pathlib import Path
import sys

sys.path.insert(0, '/workspace/tools/grasp')
sys.path.insert(0, '/workspace/grasp_core')
from runlog import RunLog, write_json

directory = Path(sys.argv[1])
os.chdir('/workspace')
log = RunLog(directory, 'simulate')
app = None
gui = os.environ.get('GRASP_GUI') == '1'
with log.stage('simulate'):
    with log.stage('load_sim_inputs'):
        cfg = json.loads((directory/'runtime.json').read_text())
        trajectory = json.loads((directory/'trajectory.json').read_text())
        perception = json.loads((directory/'perception.json').read_text())
        measured = json.loads((directory/'joints.json').read_text())
        if trajectory['run_id'] != directory.name or trajectory['pair_id'] != perception['pair_id']:
            raise ValueError('Run/frame mismatch')
        if trajectory['mode'] != 'simulation_only' or not trajectory['base_fixed']:
            raise ValueError('Unsupported execution contract')
        meta = json.loads(Path('assets/robots/generated/robot_metadata.json').read_text())
        if Path(cfg['urdf']).resolve() != (Path('assets/robots')/meta['source_urdf']).resolve():
            raise ValueError('Simulation asset and perception URDF mismatch')
    try:
        with log.stage('isaac_startup'):
            from isaacsim import SimulationApp
            app = SimulationApp({'headless': not gui, 'multi_gpu': False, 'disable_viewport_updates': not gui})
            import numpy as np
            import omni.usd
            import omni.physx
            from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf, PhysicsSchemaTools
            from isaacsim.core.api import World
            from isaacsim.core.prims import SingleArticulation
            from isaacsim.core.utils.types import ArticulationAction
            from scipy.spatial.transform import Rotation
            from wheel_grasp.planning import Trajectory, sample_trajectory
            from wheel_grasp.kinematics import RobotModel, PairIK
            simulation_source = json.loads((directory/'manifest.json').read_text()).get('source') == 'sim' if (directory/'manifest.json').exists() else False
            if simulation_source:
                from grasp_sim_scene import build_scene
                context = json.loads((directory/'capture_context.json').read_text())
                if 'base_state' not in context:
                    raise ValueError('Simulation replay lacks settled base state; capture again')
                world,stage,robot,meta,_,truth = build_scene(cfg,measured,context['base_state'])
                write_json(directory/'sim_scene_reuse.json',dict(scene='same configured capture scene',
                           estimated_pose_used_to_place_target=False))
            else:
                world = World(physics_dt=1/120, rendering_dt=1/120, stage_units_in_meters=1.)
                stage = omni.usd.get_context().get_stage()
                floor = UsdGeom.Cube.Define(stage, '/World/Ground')
                floor.CreateSizeAttr(1.)
                floor.AddTranslateOp().Set(Gf.Vec3d(0.,0.,-.05))
                floor.AddScaleOp().Set(Gf.Vec3d(20.,20.,.1))
                UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
                root = stage.DefinePrim('/World/Robot', 'Xform')
                root.GetReferences().AddReference(str(Path('assets/robots/generated/robot.usdc').resolve()))
                robot = world.scene.add(SingleArticulation('/World/Robot'+meta['root_path'], name='sdk_robot'))
                robot.set_default_state(position=np.array([0.,0.,.015]), orientation=np.array([1.,0.,0.,0.]))
                box_pose = np.asarray(perception['pose'], float)
                box_pose = box_pose.copy()
                box_pose[2,3] += .015
                target = UsdGeom.Xform.Define(stage, '/World/Target')
                matrix = Gf.Matrix4d(*box_pose.T.flatten().tolist())
                target.AddTransformOp().Set(matrix)
                # A static target supports approach validation, not carrying physics.
                for i, collider in enumerate(json.loads(Path(cfg['model']['colliders']).read_text())):
                    cube = UsdGeom.Cube.Define(stage, '/World/Target/part_'+str(i))
                    cube.CreateSizeAttr(1.)
                    cube.AddTranslateOp().Set(Gf.Vec3d(*collider['position']))
                    cube.AddScaleOp().Set(Gf.Vec3d(*collider['size']))
                    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            for prim in stage.Traverse():
                if str(prim.GetPath()).startswith('/World/Robot') and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
            if not simulation_source:
                world.reset()
            names = list(robot.dof_names)
            missing = set(measured)-set(names)
            if missing:
                raise ValueError('Measured joints absent in simulation: '+str(missing))
            q = np.asarray(robot.get_joint_positions()).copy()
            for name, value in measured.items():
                q[names.index(name)] = value
            # Gripper SDK radians are not calibrated to this URDF. Open in simulation explicitly.
            for name in names:
                if 'finger_joint' in name:
                    q[names.index(name)] = math.pi/4 if 'left' in name else -math.pi/4
            robot.set_joint_positions(q)
            robot.set_joint_velocities(np.zeros(len(names)))
            robot.set_linear_velocity(np.zeros(3))
            robot.set_angular_velocity(np.zeros(3))
            write_json(directory/'sim_initialization.json', dict(measured=measured,
                       simulated_grippers='open; not measured', passive_joints='USD defaults',
                       target='original simulation scene' if simulation_source else 'static perceived collider model',
                       other_obstacles='support and floor' if simulation_source else 'not reconstructed',
                       self_collision='inherited USD setting; not validated by this adapter'))
            active_names = list(measured)
            indices = np.array([names.index(n) for n in active_names])
            base_start = robot.get_world_pose()[0].copy()
            base_orientation = robot.get_world_pose()[1].copy()
            initial_base_transform = np.eye(4)
            initial_base_transform[:3,3] = base_start
            initial_base_transform[:3,:3] = Rotation.from_quat(base_orientation[[1,2,3,0]]).as_matrix()
            bad_contacts = []
            contact_details = []
            contact_mode = ['approach']
            finger_contacts = set()
            def on_contacts(headers, data):
                for h in headers:
                    if not h.num_contact_data:
                        continue
                    a,b = [str(PhysicsSchemaTools.intToSdfPath(x)) for x in (h.actor0,h.actor1)]
                    colliders = [str(PhysicsSchemaTools.intToSdfPath(x)) for x in (h.collider0,h.collider1)]
                    previous_count = len(bad_contacts)
                    if '/World/Robot' in a and '/World/Robot' in b:
                        bad_contacts.append([a,b])
                    if ('/World/Target' in a and '/World/Robot' in b) or ('/World/Target' in b and '/World/Robot' in a):
                        fingers=[x for x in colliders if 'openarm_' in x and ('_ee_link1' in x or '_ee_link2' in x)]
                        if contact_mode[0]=='pinch' and fingers:
                            finger_contacts.update(fingers)
                        else:
                            bad_contacts.append([a,b])
                    if ('/World/Support' in a and '/World/Robot' in b) or ('/World/Support' in b and '/World/Robot' in a):
                        bad_contacts.append([a,b])
                    if len(bad_contacts) > previous_count:
                        contact_details.append(dict(actors=[a,b], colliders=[str(PhysicsSchemaTools.intToSdfPath(x)) for x in (h.collider0,h.collider1)]))
            subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(on_contacts)
            tcp = {s: np.eye(4) for s in ('left','right')}
            for s in tcp:
                tcp[s][:3,3] = cfg['tcp'][s]
            ik = PairIK(RobotModel(cfg['urdf']), tcp, base=cfg['base_frame'])
            def status(message):
                log.event('viewer_status','updated',message=message)
                if gui:label.text=message
            def render_tick(tick):
                return gui and tick%4==0
            if gui:
                import omni.ui as ui
                from isaacsim.core.utils.viewports import set_camera_view
                set_camera_view(eye=np.array([1.7,-1.8,1.45]),target=np.array([.15,0,.55]))
                panel=ui.Window('RGB-D Grasp | simulation only',width=460,height=100)
                with panel.frame:label=ui.Label('Model perception complete; executing trajectory',word_wrap=True)
                for _ in range(10):world.render()
        dt = 1/120
        step = 0
        phase_errors = {}
        with (directory/'sim_steps.jsonl').open('w', buffering=1) as samples:
            for phase in trajectory['phases']:
                with log.stage('execute_'+phase['phase'], pair_id=trajectory['pair_id']):
                    status(phase['phase'].upper()+': moving to model-generated targets')
                    plan = Trajectory(phase['points'], phase['collision_checked'], False, phase['reason'])
                    duration = plan.points[-1]['time']
                    if not math.isfinite(duration) or not 0 <= duration <= 120:
                        raise ValueError('Trajectory duration outside simulation budget')
                    for tick in range(math.ceil((duration+2)/dt)):
                        desired = sample_trajectory(plan, tick*dt)
                        control = q.copy()
                        for n,v in desired.items():
                            control[names.index(n)] = v
                        robot.apply_action(ArticulationAction(joint_positions=control))
                        world.step(render=render_tick(step))
                        actual = np.asarray(robot.get_joint_positions())[indices]
                        errors = {n: float(abs(actual[i]-desired[n])) for i,n in enumerate(active_names)}
                        drift = float(np.linalg.norm(robot.get_world_pose()[0][:2]-base_start[:2]))
                        orientation = robot.get_world_pose()[1]
                        rotation_drift = float(2*np.arccos(np.clip(abs(np.dot(orientation,base_orientation)),0,1)))
                        samples.write(json.dumps(dict(step=step, phase=phase['phase'], time=tick*dt,
                                desired=desired, actual=dict(zip(active_names,actual.tolist())),
                                errors=errors, base_drift_m=drift, base_rotation_drift_rad=rotation_drift,
                                contacts=bad_contacts))+'\n')
                        step += 1
                        if not np.isfinite(actual).all() or drift > .02 or rotation_drift > .05:
                            raise ValueError('Simulation invalid state/base drift; inspect sim_steps.jsonl')
                        if bad_contacts:
                            write_json(directory/'contact_failure.json',dict(phase=phase['phase'],step=step,time=tick*dt,contacts=contact_details))
                            raise ValueError('Contact during approach: '+str(bad_contacts[:4]))
                        if tick*dt > .5 and any(v > (.02 if n=='lift_joint' else .4) for n,v in errors.items()):
                            raise ValueError('Simulation tracking divergence: '+str(errors))
                    if any(v > (.004 if n=='lift_joint' else .04) for n,v in errors.items()):
                        raise ValueError('Simulation failed to settle: '+str(errors))
                    # Validate actual simulated link poses, including base drift.
                    # Re-solving FK from float32 joints would reject tiny PhysX
                    # limit overshoots and would omit the floating base motion.
                    cache = UsdGeom.XformCache()
                    actual_tcp = {}
                    for side in tcp:
                        link = stage.GetPrimAtPath('/World/Robot'+meta['frames'][f'openarm_{side}_ee_base_link'])
                        world_link = np.asarray(cache.GetLocalToWorldTransform(link)).T
                        actual_tcp[side] = np.linalg.inv(initial_base_transform)@world_link@tcp[side]
                    expected = perception[phase['phase']]
                    phase_errors[phase['phase']] = {}
                    for side in actual_tcp:
                        error = np.linalg.norm(actual_tcp[side][:3,3]-np.asarray(expected[side])[:3,3])
                        angle = Rotation.from_matrix(np.asarray(expected[side])[:3,:3]@actual_tcp[side][:3,:3].T).magnitude()
                        phase_errors[phase['phase']][side] = dict(position_m=float(error),rotation_rad=float(angle))
                        write_json(directory/'tcp_errors.json',phase_errors)
                        if error > .01 or angle > .06:
                            raise ValueError('Final TCP error %s: %.4fm %.4frad' % (side,error,angle))
        carry_result=dict(grasp_and_carry_validated=False)
        if cfg.get('simulation',{}).get('carry',{}).get('enabled'):
            if not simulation_source:raise ValueError('Carry is only supported with simulation input')
            from grasp_carry import execute_carry
            carry_result=execute_carry(cfg=cfg,directory=directory,world=world,stage=stage,robot=robot,
                meta=meta,control=control.copy(),names=names,contacts=finger_contacts,bad_contacts=bad_contacts,
                contact_mode=contact_mode,log=log,render_tick=render_tick,status=status)
        write_json(directory/'simulate.result.json', dict(run_id=directory.name, status='completed',
                   steps=step, result='carry_completed' if carry_result['grasp_and_carry_validated'] else 'approach_targets_reached', **carry_result))
    finally:
        if app is not None:
            if gui:
                error=sys.exc_info()[1]
                if error is not None and 'status' in locals():status('STOPPED: '+str(error))
                if 'world' in locals():world.pause()
                while app.is_running():app.update()
            app.close(wait_for_replicator=False, exit_code=1 if sys.exc_info()[0] else 0)
