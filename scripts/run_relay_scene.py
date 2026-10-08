"""Live relay environment, physics acceptance tests and relay commissioning."""
import argparse
import json
import math
import sys
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROOT, OUTPUTS

parser = argparse.ArgumentParser()
parser.add_argument('--gui', action='store_true')
parser.add_argument('--physics-only', action='store_true', help='Publish state to the independent viewer; no renderer or cameras')
parser.add_argument('--render-profile', choices=['legacy','balanced'], default='legacy')
parser.add_argument('--render-benchmark', action='store_true', help='Start a bounded GUI task for render measurement')
parser.add_argument('--test', action='store_true')
parser.add_argument('--capture', action='store_true')
parser.add_argument('--steps', type=int, default=0)
parser.add_argument('--auto', action='store_true', help='Run autonomous relay commissioning')
parser.add_argument('--navigation-demo', action='store_true', help='A* navigation between fixed pickup stations; no manipulation')
parser.add_argument('--localization', choices=['ground_truth','fast_livo2'], default='ground_truth')
parser.add_argument('--navigation-goal', help='Visit one fixed station instead of the complete navigation tour')
parser.add_argument('--web-console', action='store_true', help='Use the separate local navigation page')
parser.add_argument('--camera-console', action='store_true', help='Publish all robots front cameras for the local dashboard')
parser.add_argument('--boxes', type=int, default=18)
parser.add_argument('--skill-test', choices=['grasp','direct','handoff','retreat'], help='Isolated physical skill; retreat preloads earlier source boxes at their target slots')
parser.add_argument('--handoff-target', type=int, help='Target sequence index for handoff fixture; initialize earlier target slots as occupied')
parser.add_argument('--source-box', type=int, help='Select a source crate for a single-box relay or dense grasp test without moving its neighbours')
parser.add_argument('--perception', choices=['ground_truth','depth'], default='ground_truth')
args = parser.parse_args()
if args.physics_only and (args.gui or args.capture or args.navigation_demo or args.perception != "ground_truth"):
    parser.error("--physics-only currently supports headless ground-truth relay without capture")
if args.render_benchmark and (not args.gui or not args.auto or args.steps<=0):
    parser.error("--render-benchmark requires --gui --auto --steps N")
if args.web_console and (not args.navigation_demo or not args.gui):
    parser.error('--web-console requires --navigation-demo --gui')
if args.navigation_demo:
    if args.test or args.skill_test or args.source_box is not None:
        parser.error('Navigation demo is separate from manipulation tests')
    args.auto=True
if args.localization!='ground_truth' and not args.navigation_demo:
    parser.error('FAST-LIVO2 commissioning currently requires --navigation-demo')
if args.navigation_goal and not args.navigation_demo:
    parser.error('--navigation-goal requires --navigation-demo')
if args.navigation_goal and args.navigation_goal not in [p['id'] for p in json.loads((ROOT/'config/navigation_demo_config.json').read_text())['points']]:
    parser.error('Unknown navigation station')
if args.test and (args.auto or args.skill_test):
    parser.error('Environment test and automatic relay are separate modes')
if args.skill_test and not args.auto:
    parser.error('--skill-test requires --auto (or simctl relay-demo)')
if args.skill_test=='handoff' and args.boxes!=1:
    parser.error('--skill-test handoff requires --boxes 1')
if args.handoff_target is not None and args.skill_test!='handoff':
    parser.error('--handoff-target is only available with --skill-test handoff')
if args.source_box is not None and (args.skill_test not in (None,'grasp','retreat') or args.boxes!=1 or not args.auto):
    parser.error('--source-box requires --auto --boxes 1, optionally with --skill-test grasp')
if args.skill_test=='retreat' and (args.source_box is None or args.boxes!=1):
    parser.error('--skill-test retreat requires --source-box and --boxes 1')
assets = ROOT/'assets/generated/relay'
c = json.loads((ROOT/'config/relay_config.json').read_text())
cfg = json.loads((ROOT/'config/relay_robot_config.json').read_text())
meta = json.loads((ROOT/'assets/robots/generated/relay/robot_metadata.json').read_text())
initial = json.loads((assets/'initial_state.json').read_text())
if not 1 <= args.boxes <= len(initial['crates']):
    parser.error(f'--boxes must be between 1 and {len(initial["crates"])}')
if args.source_box is not None and not 0 <= args.source_box < len(initial['crates']):
    parser.error('--source-box is outside the source inventory')
if args.skill_test=='retreat':
    from relay_task import source_order,target_order
    order=source_order(initial['crates'])
    targets=target_order(c)
    for slot,index in enumerate(order[:order.index(args.source_box)]):
        initial['crates'][index]['position']=targets[slot]['position']
        initial['crates'][index]['yaw']=targets[slot]['yaw']
        initial['crates'][index]['fixture_target']=True
if args.skill_test=='handoff':
    from relay_task import source_order, target_order
    args.handoff_target=0 if args.handoff_target is None else args.handoff_target
    targets=target_order(c)
    if not 0 <= args.handoff_target < len(targets):
        parser.error(f'--handoff-target must be between 0 and {len(targets)-1}')
    fixture_order=source_order(initial['crates'])
    initial['crates'][fixture_order[0]]['position']=[*c['handoff']['slots'][1],c['handoff']['size'][2]]
    initial['crates'][fixture_order[0]]['yaw']=0.
    for slot,index in enumerate(fixture_order[1:args.handoff_target+1]):
        initial['crates'][index]['position']=targets[slot]['position']
        initial['crates'][index]['yaw']=targets[slot]['yaw']
        initial['crates'][index]['fixture_target']=True
if json.loads((assets/'scene_config.json').read_text()) != c:
    raise SystemExit('Relay scene config changed: run simctl build-relay')
from isaacsim import SimulationApp
app = SimulationApp({'headless':not args.gui, 'multi_gpu':False, 'width':1280, 'height':720,
    'renderer':'RaytracedLighting', 'anti_aliasing':2 if args.render_profile=='balanced' else 1,
    'limit_cpu_threads':8 if args.physics_only else 16,
    'disable_viewport_updates':args.physics_only})
try:
    import numpy as np
    import omni.usd
    import omni.kit.app
    import omni.physx
    from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema, PhysicsSchemaTools
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate('isaacsim.core.api', True)
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
    from robot_control import RobotControl
    omni.usd.get_context().open_stage(str(assets/'main.usda'))
    for _ in range(12):
        app.update()
    app.reset_render_settings()
    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    from relay_render_metrics import RenderMetrics
    render_metrics=RenderMetrics(OUTPUTS,'physics-only' if args.physics_only else args.render_profile,c['physics']['dt'])
    if args.gui:
        from omni.kit.viewport.utility import get_active_viewport
        viewport=get_active_viewport()
        if viewport is not None:
            if args.render_profile=='balanced':
                viewport.fill_frame=False
                viewport.set_texture_resolution((1280,720))
            render_metrics.viewport_resolution=list(viewport.get_texture_resolution())
            print('RELAY_VIEWPORT',render_metrics.viewport_resolution,args.render_profile,flush=True)
    world = World(physics_dt=c['physics']['dt'], rendering_dt=c['physics']['dt'], stage_units_in_meters=1., backend='numpy', device='cpu')
    robots, controls = {}, {}
    for spec in c['robots']:
        name = spec['id']
        root = '/World/Robots/'+name
        robot = world.scene.add(SingleArticulation(root+meta['root_path'], name=name))
        robot.set_default_state(position=np.array(spec['position']), orientation=np.array([math.cos(spec['yaw']/2),0,0,math.sin(spec['yaw']/2)]))
        robots[name] = robot
        controls[name] = RobotControl(stage, root, cfg=cfg, meta=meta)
    crates = []
    for i, spec in enumerate(initial['crates']):
        crate = world.scene.add(SingleRigidPrim(spec['path'], name=f'crate_{i}'))
        fixture_position=spec['position']
        if args.skill_test == 'direct' and i>12:
            fixture_position=[-3.5+(i-13)*.55,-2.5,0.]
        crate_yaw=spec.get('yaw',0.)
        crate.set_default_state(position=np.array(fixture_position), orientation=np.array([math.cos(crate_yaw/2),0,0,math.sin(crate_yaw/2)]))
        crates.append(crate)
    from relay_pusher import HandoffPusher, geometry as pusher_geometry, ROOT as PUSHER_ROOT
    slider = world.scene.add(SingleRigidPrim(PUSHER_ROOT+'/Slider', name='handoff_slider'))
    slider.set_default_state(position=pusher_geometry(c)[0], orientation=np.array([1.,0,0,0]))
    pusher = HandoffPusher(stage, c, slider, crates)
    pusher_label = None
    print('RELAY_RESET_INITIAL', flush=True)
    world.reset()
    print('RELAY_INITIALIZED', {name:robot.num_dof for name,robot in robots.items()}, flush=True)
    for robot in robots.values():
        positions = [cfg['home'].get(n,0.) for n in robot.dof_names]
        robot.set_joints_default_state(positions=np.array(positions), velocities=np.zeros(robot.num_dof), efforts=np.zeros(robot.num_dof))
    world.reset()
    for robot in robots.values():
        robot.set_joint_velocities(np.zeros(robot.num_dof))
        robot.set_linear_velocity(np.zeros(3))
        robot.set_angular_velocity(np.zeros(3))
    print('RELAY_DEFAULTS_READY', flush=True)
    steps = 0
    checks, readings = {}, {}
    contacts = set()
    from relay_contacts import ContactTracker, ActorPathCache
    actor_path = ActorPathCache(PhysicsSchemaTools.intToSdfPath)
    contact_tracker = ContactTracker()
    active_contacts = contact_tracker.actors
    runtime = None
    runtime_label = None
    console = None
    state_publisher = None
    camera_streams = None
    robot_console = None

    def on_contacts(headers, data):
        from omni.physx.bindings._physx import ContactEventType
        for header in headers:
            a = actor_path(header.actor0)
            b = actor_path(header.actor1)
            lost = header.type == ContactEventType.CONTACT_LOST
            if lost or header.num_contact_data:
                contact_tracker.update(a, b, int(header.collider0), int(header.collider1), lost=lost)
            if header.num_contact_data:
                if '/Crates/' in a or '/Crates/' in b:
                    contacts.add(tuple(sorted((a,b))))

    subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(on_contacts)

    def step(count=1):
        global steps
        for _ in range(count):
            frame_started=phase_started=time.perf_counter()
            if robot_console is not None:
                robot_console.poll()
            if console is not None and steps % 12 == 0:
                console.poll()
            if runtime is not None:
                runtime.tick(c['physics']['dt'])
            for control in controls.values():
                control.update(c['physics']['dt'])
            pusher.update(c['physics']['dt'])
            if pusher_label is not None and steps % 12 == 0:
                pusher_label.text = f'Pusher: {pusher.state} | L:{int(pusher.left_present)} R:{int(pusher.right_present)} | Done:{pusher.completed}'
            if runtime_label is not None and runtime is not None and steps % 12 == 0:
                if args.navigation_demo:
                    localization_label='FAST-LIVO2 experimental' if runtime.nav_localization=='fast_livo2' else 'Simulation ground truth'
                    runtime_label.text = f'A: {runtime.states["A"]}\nStations {len(runtime.nav_reached)}/{len(runtime.selected_goals)} | A*\n{runtime.error or localization_label}'
                else:
                    runtime_label.text = f'A: {runtime.states["A"]} | B: {runtime.states["B"]}\nDone {len(runtime.inventory.completed)}/{runtime.count} | lock {runtime.handoff.owner}\n{runtime.error or runtime.perception}'
            render_period=12 if args.localization=='fast_livo2' else (60 if runtime is not None else 12)
            sensor_active=(runtime is not None and args.navigation_demo and runtime.sensor_bridge is not None and runtime.active)
            sensor_due=sensor_active and (runtime.sensor_bridge.tick_count+1)%12==0
            # The RGB render must coincide with the raw point-cloud/IMU
            # packet time, not with an unrelated global render counter.
            # Physics stays at 120 Hz. GUI rendering need not run at every
            # physics tick; keep sensor-triggered rendering aligned separately.
            camera_due=camera_streams.prepare_render() if camera_streams is not None else False
            if args.gui:
                render_period=2 if args.render_profile=='balanced' else 4
            render_due=sensor_due or (not sensor_active and steps % render_period == 0)
            if args.physics_only:
                render_due=False
            if runtime is not None:
                if not hasattr(runtime,'loop_performance'):
                    runtime.loop_performance=dict(control_seconds=0.,step_with_render_seconds=0.,
                                                  step_without_render_seconds=0.,rendered_steps=0,unrendered_steps=0,
                                                  camera_publish_seconds=0.,
                                                  state_validation_seconds=0.)
                runtime.loop_performance['control_seconds']+=time.perf_counter()-phase_started
            phase_started=time.perf_counter()
            world.step(render=render_due)
            measured_step_seconds=time.perf_counter()-phase_started
            if runtime is not None:
                runtime.loop_performance['step_with_render_seconds' if render_due else 'step_without_render_seconds']+=time.perf_counter()-phase_started
                runtime.loop_performance['rendered_steps' if render_due else 'unrendered_steps']+=1
            phase_started=time.perf_counter()
            if camera_streams is not None and (camera_due or time.time()-camera_streams.last_publish >= .2):
                camera_streams.publish(steps*c['physics']['dt'], runtime,
                                       robot_console.status() if robot_console else None, read_images=camera_due)
            if runtime is not None:
                runtime.loop_performance['camera_publish_seconds']+=time.perf_counter()-phase_started
                if camera_streams is not None:
                    runtime.loop_performance['camera_details']=dict(camera_streams.performance)
            if sensor_active:
                try:
                    runtime.sensor_bridge.tick(c['physics']['dt'])
                except Exception as exc:
                    for control in controls.values():
                        control.stop()
                    runtime.error=f'Sensor failure: {type(exc).__name__}: {exc}'
                    runtime.active=False;runtime.finished=True
                    runtime.states['A']='SENSOR_FAULT'
                    runtime.write_report()
            steps += 1
            phase_started=time.perf_counter()
            for name,robot in robots.items():
                if not np.isfinite(robot.get_joint_positions()).all():
                    error=f'Nonfinite articulation: {name} at step {steps}'
                    if runtime is not None:
                        runtime.error=error
                        runtime.physics_valid=False
                        runtime.active=False
                        runtime.finished=True
                        # Do not run FK on invalid joint state. Preserve the
                        # last finite diagnostics instead of leaving an old report.
                        report=dict(passed=False,skill_passed=False,error=error,
                            physics_state_valid=False,states=dict(runtime.states),
                            requested_boxes=runtime.count,
                            completed_boxes=len(runtime.inventory.completed),
                            simulation_seconds=runtime.time,events=runtime.events,
                            ik=dict(runtime.ik.stats),skill_test=args.skill_test,
                            fixture_target_slot=args.handoff_target,
                            last_finite_sample=runtime.telemetry[-1] if runtime.telemetry else None)
                        (OUTPUTS/'relay_demo_report.json').write_text(json.dumps(report,indent=2))
                        (OUTPUTS/'relay_demo_telemetry.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in runtime.telemetry))
                    raise RuntimeError(error)
            if runtime is not None:
                runtime.loop_performance['state_validation_seconds']+=time.perf_counter()-phase_started
            if state_publisher is not None and time.monotonic()-state_publisher.last>=.05:
                operator=robot_console.status()
                state_publisher.publish(steps*c['physics']['dt'],operator)
                now=time.time()
                if now-physics_status_clock[0]>=.25:
                    from navigation_console_ipc import atomic_json
                    atomic_json(OUTPUTS/'robot_cameras'/'operator_status.json',dict(
                        session=physics_session,updated_at=now,operator=operator,decoupled=True,
                        simulation_seconds=steps*c['physics']['dt'],
                        export_frames=state_publisher.frames,export_seconds=state_publisher.seconds,
                        export_skipped=state_publisher.channel.skipped))
                    physics_status_clock[0]=now
            render_metrics.record(render_due,measured_step_seconds,time.perf_counter()-frame_started)

    def set_gripper(robot_name, side, closed):
        if runtime is not None:
            return
        controls[robot_name].set_joint(f'openarm_{side}_gripper_left_outer_knuckle_joint', .65 if closed else 0.)

    def reset():
        if runtime is not None:
            runtime.reset()
        contact_tracker.clear()
        actor_path.clear()
        contacts.clear()
        pusher.reset()
        world.reset()
        if state_publisher is not None:
            state_publisher.physics_poses.reset()
        if args.navigation_demo and runtime is not None and runtime.sensor_bridge is not None:
            runtime.sensor_bridge.reset_session()
        slider.set_linear_velocity(np.zeros(3))
        slider.set_angular_velocity(np.zeros(3))
        # Clear cached PhysX velocities after reset, as in the existing demo.
        # Otherwise reinitializing two floating articulations can retain stale
        # root/roller velocities and destabilize the next physics steps.
        for robot in robots.values():
            robot.set_joint_velocities(np.zeros(robot.num_dof))
            robot.set_linear_velocity(np.zeros(3))
            robot.set_angular_velocity(np.zeros(3))
        for crate in crates:
            crate.set_linear_velocity(np.zeros(3))
            crate.set_angular_velocity(np.zeros(3))
        for control in controls.values():
            control.home()
            control.sent = control.targets.copy()
            control.written_targets.clear()

    def place_demo_crate(slot, index=None):
        # Explicit demo/test loading only; automatic pushing never sets crate poses.
        if runtime is not None or pusher.state != 'WAITING' or pusher.occupants(slot):
            return False
        if index is None:
            index = next((i for i in reversed(range(len(crates)))
                          if crates[i].get_world_pose()[0][0] < -1.), None)
        if index is None:
            return False
        crates[index].set_world_pose(position=np.array([*c['handoff']['slots'][slot],c['handoff']['size'][2]+.002]), orientation=np.array([1.,0,0,0]))
        crates[index].set_linear_velocity(np.zeros(3))
        crates[index].set_angular_velocity(np.zeros(3))
        return True

    def create_runtime():
        global runtime
        from relay_runtime import RelayRuntime
        runtime_class=RelayRuntime
        runtime_options={}
        if args.navigation_demo:
            from navigation_runtime import NavigationRuntime
            runtime_class=NavigationRuntime
            runtime_options['localization']=args.localization
            runtime_options['goal']=args.navigation_goal
        runtime = runtime_class(c, initial, robots, controls, crates, pusher, active_contacts,
                               count=args.boxes, perception=args.perception, skill_test=args.skill_test,
                               handoff_target=args.handoff_target or 0,source_box=args.source_box,**runtime_options)
        if state_publisher is not None:
            runtime.physics_poses=state_publisher.physics_poses
        # Runtime contact materials are authored before the task begins. Rebuild
        # PhysX views and clear velocities after those scene changes.
        reset()
        if args.navigation_demo and args.localization=='fast_livo2':
            from livo_sim_sensors import LivoSimSensors
            runtime.sensor_bridge=LivoSimSensors(runtime,stage,'/World/Robots/Robot_A'+meta['root_path'])

    if args.auto or args.skill_test:
        create_runtime()

    def operator_status():
        return dict(mode='navigation' if args.navigation_demo else ('relay' if runtime else 'manual'),
            states=dict(runtime.states) if runtime else {},
            active=bool(runtime and runtime.active), finished=bool(runtime and runtime.finished),
            error=runtime.error if runtime else None, count=runtime.count if runtime else args.boxes,
            ik=dict(runtime.ik.stats) if runtime is not None and hasattr(runtime,'ik') else None,
            max_boxes=len(crates), completed=len(runtime.inventory.completed) if runtime else 0,
            pusher=dict(state=pusher.state, left=bool(pusher.left_present), right=bool(pusher.right_present),
                        completed=pusher.completed),
            goals=[p['id'] for p in runtime.nav_config['points']] if args.navigation_demo else [],
            robots={name:dict(lift=round(float(robot.get_joint_positions()[list(robot.dof_names).index('lift_joint')]),3),
                targets={side:round(float(controls[name].targets[f'openarm_{side}_gripper_left_outer_knuckle_joint']),3)
                         for side in ('left','right')}) for name,robot in robots.items()})

    def execute_web_command(cmd):
        global runtime
        action=cmd['action']
        if action=='gripper':
            set_gripper(cmd['robot'],cmd['side'],cmd['closed'])
            return '夹爪目标已设置'
        if action=='lift':
            robot=robots[cmd['robot']]
            current=float(robot.get_joint_positions()[list(robot.dof_names).index('lift_joint')])
            if cmd['height']<current:
                bounds=UsdGeom.BBoxCache(0,['default','render'])
                lowest=min(float(bounds.ComputeWorldBound(stage.GetPrimAtPath(
                    '/World/Robots/'+cmd['robot']+meta['frames'][f'openarm_{side}_link1']
                )).ComputeAlignedRange().GetMin()[2]) for side in ('left','right'))
                if not math.isfinite(lowest) or lowest+cmd['height']-current<.025:
                    raise ValueError('当前手臂姿态下降后会碰到地面，请先抬起机械臂')
            controls[cmd['robot']].set_joint('lift_joint',cmd['height'])
            return '升降目标已设置'
        if action=='load':
            if not place_demo_crate(cmd['slot']):
                raise ValueError('放箱失败：交接区被占用、推箱机构忙碌，或源垛没有可用箱子')
            return '演示箱已放入交接区'
        if action=='reset':
            reset()
            return '环境已复位'
        if action=='start':
            runtime.start()
            return '自动任务已启动'
        if action=='stop':
            if runtime:
                runtime.request_stop()
            else:
                for name,control in controls.items():
                    control.stop()
                    positions=dict(zip(robots[name].dof_names,robots[name].get_joint_positions()))
                    for joint in control.targets:
                        if control.specs[joint]['kind']!='wheel':
                            control.set_joint(joint,float(positions[joint]))
            return '已请求停止并保持机器人'
        if action in ('goal','tour'):
            runtime.select_goal(cmd.get('goal') if action=='goal' else None)
            return '导航任务已启动'
        if action=='mode':
            reset()
            args.boxes=cmd['count']
            if cmd['mode']=='manual':
                runtime=None
                pusher.start_permitted=lambda index:True
                pusher.motion_permitted=lambda:True
                pusher.sensor_period=0.
            else:
                create_runtime()
            if args.physics_only:
                import carb
                carb.settings.get_settings().set('/physics/updateToUsd', runtime is None)
            return '模式已切换，环境已复位'


    if args.web_console:
        from navigation_console_ipc import NavigationConsole
        console=NavigationConsole(runtime,OUTPUTS/'navigation_console',reset)
    if args.gui:
        from omni.kit.viewport.utility import get_active_viewport
        get_active_viewport().camera_path = '/World/Overview'
    if args.gui and not args.web_console and not args.camera_console:
        import omni.ui as ui
        panel = ui.Window('Pickup navigation | A*' if args.navigation_demo else 'Dual robot relay | Environment controls', width=440, height=720 if args.navigation_demo else 490)
        with panel.frame:
            with ui.VStack(spacing=8):
                ui.Label(f'AG95 substitute / {len(crates)} hollow totes / {len(robots)} robots', height=24)
                ui.Label('Autonomous commissioning / physical contacts' if runtime else 'Environment controls / automatic pusher', height=24, word_wrap=True)
                pusher_label = ui.Label('Pusher: WAITING', height=24)
                if runtime:
                    runtime_label = ui.Label('Ready',height=65,word_wrap=True)
                    ui.Button('START navigation tour' if args.navigation_demo else 'START relay',clicked_fn=runtime.start)
                    ui.Button('STOP / hold',clicked_fn=runtime.request_stop)
                    if args.navigation_demo:
                        for point in runtime.nav_config['points']:
                            ui.Button(point['id'],clicked_fn=lambda name=point['id']:runtime.select_goal(name))
                        ui.Button('Restart full navigation tour',clicked_fn=lambda:runtime.select_goal(None))
                if runtime is None:
                    with ui.HStack(height=28):
                        ui.Button('Demo: load left', clicked_fn=lambda:place_demo_crate(0))
                        ui.Button('Demo: load right', clicked_fn=lambda:place_demo_crate(1))
                    for name in robots:
                        ui.Label(name, height=24)
                        for side in ('left', 'right'):
                            with ui.HStack(height=28):
                                ui.Label(side, width=50)
                                ui.Button('Open', clicked_fn=lambda n=name,s=side:set_gripper(n,s,False))
                                ui.Button('Close', clicked_fn=lambda n=name,s=side:set_gripper(n,s,True))
                        with ui.HStack(height=28):
                            ui.Button('Lift 0.15 m', clicked_fn=lambda n=name:controls[n].set_joint('lift_joint',.15))
                            ui.Button('Lift 0.35 m', clicked_fn=lambda n=name:controls[n].set_joint('lift_joint',.35))
                ui.Button('Reset environment', height=30, clicked_fn=reset)
    if args.physics_only:
        import uuid
        from relay_state_ipc import PhysicsPublisher
        from robot_console_ipc import RobotConsole
        physics_session=uuid.uuid4().hex
        physics_status_clock=[0.]
        robot_console=RobotConsole(OUTPUTS/'robot_cameras',physics_session,operator_status,execute_web_command)
        state_publisher=PhysicsPublisher(stage,robots,physics_session)
        if runtime is not None:
            runtime.physics_poses=state_publisher.physics_poses
        # Control uses live physics tensors/FK; the separate viewer receives
        # physical body poses, so per-tick USD pose writeback is unnecessary.
        import carb
        # Manual-mode bounding-box safety checks still use USD geometry.
        carb.settings.get_settings().set('/physics/updateToUsd', runtime is None or args.navigation_demo)
    elif args.camera_console:
        from robot_camera_streams import RobotCameraStreams
        camera_streams = RobotCameraStreams(stage, robots, meta, [*crates, slider])
        from robot_console_ipc import RobotConsole
        robot_console = RobotConsole(camera_streams.folder, camera_streams.session,
                                     operator_status, execute_web_command)
    step(360)
    print(f'RELAY_ENVIRONMENT_READY: {len(robots)} robots, four AG95 grippers, {len(crates)} dynamic totes', flush=True)
    if runtime is not None and (not args.gui or args.render_benchmark):
        if args.render_benchmark:
            resolution=render_metrics.viewport_resolution
            render_metrics=RenderMetrics(OUTPUTS,'physics-only' if args.physics_only else args.render_profile,c['physics']['dt'])
            render_metrics.viewport_resolution=resolution
        runtime.start()

    if args.test:
        checks['two_articulations'] = len(robots)==2 and sum(p.HasAPI(UsdPhysics.ArticulationRootAPI) for p in stage.Traverse())==2
        for name, robot in robots.items():
            actual = dict(zip(robot.dof_names, robot.get_joint_positions()))
            readings[name+'_startup_joints'] = {joint:float(actual[joint]) for joint in cfg['home']}
            checks[name+'_startup_lift'] = abs(float(actual['lift_joint'])-cfg['home']['lift_joint']) < .008
            checks[name+'_startup_arms'] = all(abs(float(actual[joint])-target)<.04
                for joint,target in cfg['home'].items() if joint.startswith('openarm_'))
        stack = c['source_stack']
        per_layer = stack['columns'] * stack['rows']
        checks['configured_dynamic_crates'] = len(crates)==per_layer*stack['layers'] and all(UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(s['path'])).GetRigidBodyEnabledAttr().Get() for s in initial['crates'])
        checks['independent_robot_dof_sets'] = all(robot.num_dof == 102 for robot in robots.values())
        checks['real_stack_contacts'] = any('/SourcePalletDeck' in ' '.join(p) for p in contacts) and any(all('/Crates/' in x for x in p) for p in contacts)
        settled = [crate.get_world_pose()[0].copy() for crate in crates]
        checks['source_stack_settled'] = all(np.linalg.norm(p-np.array(s['position'])) < .015 and np.linalg.norm(crate.get_linear_velocity()) < .02 for p,s,crate in zip(settled,initial['crates'],crates))
        # A vertical ray through each top crate must reach its floor, not a sealed lid.
        query = omni.physx.get_physx_scene_query_interface()
        ray_depths = []
        for i,spec in enumerate(initial['crates']):
            if spec['layer'] != c['source_stack']['layers']-1:
                continue
            p = crates[i].get_world_pose()[0]
            origin = (float(p[0]),float(p[1]),float(p[2]+c['crate']['size'][2]+.05))
            hit = query.raycast_closest(origin, (0.,0.,-1.), .5)
            ray_depths.append(float(hit['distance']) if hit['hit'] else -1.)
        readings['crate_center_ray_distances_m'] = ray_depths
        checks['open_crate_mouths'] = len(ray_depths)==per_layer and all(.24 < d < .27 for d in ray_depths)
        gripper_names = [s['name'] for s in meta['actuated_joints'] if s['kind']=='gripper']
        # Command only A-left first: distinguish independent drive wiring from copies.
        set_gripper('Robot_A','left',True)
        step(220)
        for name,robot in robots.items():
            q = robot.get_joint_positions()
            values = {n:float(q[robot.dof_names.index(n)]) for n in gripper_names}
            readings[name+'_independent_gripper_test'] = values
            checks[name+'_independent_gripper_test'] = all(abs(v-(.65 if name=='Robot_A' and 'openarm_left_' in n else 0.))<.045 for n,v in values.items())
        for name in robots:
            for side in ('left','right'):
                set_gripper(name,side,True)
        step(220)
        for name,robot in robots.items():
            q = robot.get_joint_positions()
            checks[name+'_all_grippers_close'] = all(abs(q[robot.dof_names.index(n)]-.65)<.045 for n in gripper_names)
            checks[name+'_mimic_coupling'] = all(abs(q[robot.dof_names.index(m['name'])]-q[robot.dof_names.index(m['reference'])])<.045 for m in meta['mimic_joints'])
            readings[name+'_closed'] = {n:float(q[robot.dof_names.index(n)]) for n in gripper_names}
        for name in robots:
            for side in ('left','right'):
                set_gripper(name,side,False)
        step(220)
        for name,robot in robots.items():
            q = robot.get_joint_positions()
            checks[name+'_grippers_reopen'] = all(abs(q[robot.dof_names.index(n)])<.045 for n in gripper_names)
            p,r = robot.get_world_pose()
            expected = next(s['position'] for s in c['robots'] if s['id']==name)
            checks[name+'_base_stable'] = bool(np.linalg.norm(p[:2]-np.array(expected[:2]))<.03 and abs(r[1])+abs(r[2])<.03)
        checks['stack_undisturbed'] = all(np.linalg.norm(crate.get_world_pose()[0]-p)<.008 for crate,p in zip(crates,settled))
        print('RELAY_PRE_RESET_CHECKS', json.dumps({k:bool(v) for k,v in checks.items()}), flush=True)
        reset()
        step(240)
        checks['reset_restores_stack'] = all(np.linalg.norm(crate.get_world_pose()[0]-np.array(s['position']))<.015 for crate,s in zip(crates,initial['crates']))
        readings['robot_dof_counts'] = {name:robot.num_dof for name,robot in robots.items()}
        # Place existing top-layer totes as test fixtures; all transfer motion is physical.
        checks['pusher_empty_idle'] = pusher.completed==0 and pusher.state=='WAITING' and abs(pusher.target)<1e-6
        moving, blocking = len(crates)-1, len(crates)-2
        place_demo_crate(1, blocking)
        place_demo_crate(0, moving)
        step(240)
        checks['pusher_right_occupied_interlock'] = pusher.state=='WAITING' and pusher.target==0. and pusher.completed==0
        crates[blocking].set_world_pose(position=np.array(initial['crates'][blocking]['position']),orientation=np.array([1.,0,0,0]))
        crates[blocking].set_linear_velocity(np.zeros(3))
        crates[blocking].set_angular_velocity(np.zeros(3))
        contacts.clear()
        trace=[]
        for _ in range(1400):
            step()
            trace.append(crates[moving].get_world_pose()[0].copy())
            if pusher.completed==1 and pusher.state=='WAITING':
                break
        step(120)
        target = np.array([*c['handoff']['slots'][1],c['handoff']['size'][2]])
        final, orientation = crates[moving].get_world_pose()
        readings['pusher_final_position_m'] = final.tolist()
        readings['pusher_state'] = pusher.state
        readings['pusher_cycles'] = pusher.completed
        checks['pusher_transfer_to_right'] = pusher.completed==1 and np.linalg.norm(final-target)<.025
        checks['pusher_real_contact'] = any(PUSHER_ROOT+'/Slider' in ' '.join(pair) and initial['crates'][moving]['path'] in pair for pair in contacts)
        checks['pusher_continuous_supported_motion'] = bool(np.max(np.linalg.norm(np.diff(np.array(trace),axis=0),axis=1))<.01 and all(abs(p[2]-target[2])<.02 for p in trace) and abs(orientation[0])>.998)
        checks['pusher_returns_home'] = pusher.state=='WAITING' and abs(slider.get_world_pose()[0][0]-pusher.home[0])<.006
        step(180)
        checks['pusher_no_repeat_on_right_box'] = pusher.completed==1 and pusher.state=='WAITING'
        # Leave successful handoff in place for its dedicated screenshot below.

    if args.capture:
        import omni.replicator.core as rep
        from PIL import Image
        capture_products = []
        def capture(filename, position, target, focal=28, resolution=(1440,900)):
            # Explicit, uniquely named USD cameras avoid Replicator graph/node
            # reuse when sequential render products are created and destroyed.
            camera_path = '/World/ReviewCameras/'+Path(filename).stem
            camera = UsdGeom.Camera.Define(stage,camera_path)
            camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*position),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse())
            camera.CreateFocalLengthAttr(focal)
            camera.CreateHorizontalApertureAttr(36.)
            camera.CreateClippingRangeAttr(Gf.Vec2f(.01,100.))
            product = rep.create.render_product(camera_path,resolution)
            capture_products.append(product)
            annotator = rep.AnnotatorRegistry.get_annotator('rgb')
            annotator.attach(product)
            for _ in range(12):
                rep.orchestrator.step(rt_subframes=4,delta_time=0.0)
            pixels = annotator.get_data()
            if not pixels.size:
                raise RuntimeError('Empty relay screenshot')
            Image.fromarray(pixels).save(str(OUTPUTS/filename))
            annotator.detach(product)
        if runtime is None:
            capture('relay_handoff.png',(1.8,-1.7,2.0),(0.,.75,.80),45)
            if args.test:
                reset()
                step(240)
                checks['pusher_reset_restores_environment'] = pusher.completed==0 and pusher.state=='WAITING' and all(np.linalg.norm(crate.get_world_pose()[0]-np.array(spec['position']))<.015 for crate,spec in zip(crates,initial['crates']))
            capture('relay_overview.png', c['camera']['position'],c['camera']['target'],38)
            stack_center = [*c['pallet']['source'], c['pallet']['size'][2]+stack_height/2] if (stack_height := c['source_stack']['layers']*c['crate']['size'][2]) else [*c['pallet']['source'], .4]
            capture('relay_totes.png',(-3.7,-1.25,1.85),stack_center,45)
            # Locate the actual wrist from the live USD transforms for a useful close-up.
            frame = stage.GetPrimAtPath('/World/Robots/Robot_A'+meta['frames']['openarm_left_gripper_ag95_base_link'])
            p = np.array(UsdGeom.Xformable(frame).ComputeLocalToWorldTransform(0).ExtractTranslation())
            print('RELAY_GRIPPER_CAMERA_TARGET', p.tolist(), flush=True)
            capture('relay_gripper.png',(p+np.array([.5,-.5,.3])).tolist(),p.tolist(),55,(1000,800))
        for product in capture_products:
            product.destroy()
        capture_products.clear()
        # Replicator pauses the timeline after capture. Resume before issuing
        # robot commands; otherwise step counters advance with frozen physics.
        import omni.timeline
        omni.timeline.get_timeline_interface().play()
    if args.test:
        report = dict(passed=all(checks.values()), checks={k:bool(v) for k,v in checks.items()}, readings=readings,
            scene='assets/generated/relay/main.usda',gripper='DH AG95 substitute',
            physics_steps=steps, autonomous_relay_implemented=True,
            autonomous_relay_validated=False,
            handoff_pusher_implemented=True, note='Environment and physical pusher acceptance only; this test does not validate autonomous grasp or full relay.')
        (OUTPUTS/'relay_environment_report.json').write_text(json.dumps(report,indent=2))
        print('RELAY_ENVIRONMENT_REPORT',json.dumps(report),flush=True)
        if not report['passed']:
            raise RuntimeError('Relay environment checks failed')
    if not args.test:
        count=0
        while app.is_running() and (args.steps==0 or count<args.steps):
            step()
            count+=1
            if runtime is not None and runtime.finished and not args.gui and not args.physics_only:
                if args.capture and runtime.physics_valid:
                    capture('relay_demo_final.png',c['camera']['position'],c['camera']['target'],38)
                    active=next((e['robot'] for e in reversed(runtime.events)
                                 if e.get('robot') in ('A','B')),'A')
                    b=runtime.base(active)
                    forward=np.array([np.cos(b[3]),np.sin(b[3])])
                    focus=b[:2]+forward*.55
                    box_index=next((e['box'] for e in reversed(runtime.events)
                                    if e.get('robot')==active and 'box' in e),None)
                    height=.9
                    if box_index is not None:
                        box_position=runtime.crates[box_index].get_world_pose()[0]
                        focus=box_position[:2]
                        height=float(box_position[2]+c['crate']['size'][2]/2)
                    capture('relay_demo_grasp.png',(float(focus[0]-1.1),float(focus[1]-.7),height+.75),
                            (float(focus[0]),float(focus[1]),height),45)
                    for product in capture_products:
                        product.destroy()
                if runtime.error:
                    raise RuntimeError(runtime.error)
                break
        print('RELAY_RENDER_PERFORMANCE',json.dumps(render_metrics.write()),flush=True)
        if runtime is not None and runtime.active and not args.render_benchmark:
            runtime.error = 'Execution interrupted before completion'
            runtime.write_report()
            if not args.gui:
                raise RuntimeError(runtime.error)
except Exception:
    import traceback
    traceback.print_exc()
    app.close(exit_code=1)
    raise
else:
    app.close()
