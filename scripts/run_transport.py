"""SLAM + dual-arm IK box transport in the existing industrial scene."""
import argparse
import json
import math
import sys
import traceback
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from common import ROOT, ASSETS, OUTPUTS, load_config, table_by_id

parser = argparse.ArgumentParser()
parser.add_argument('--demo',action='store_true',help='Three floor boxes to one workbench with the lift robot')
parser.add_argument('--gui',action='store_true')
parser.add_argument('--console-test',action='store_true',help='Exercise the console callbacks and a complete transport cycle')
parser.add_argument('--exit-after-test',action='store_true')
test_mode = parser.add_mutually_exclusive_group()
test_mode.add_argument('--manipulation-test',action='store_true',help='Initialize at the source dock to isolate physical grasp testing')
test_mode.add_argument('--navigation-test',action='store_true',help='Navigate both stations without manipulating the box')
args = parser.parse_args()
if args.demo and (args.manipulation_test or args.navigation_test):
    parser.error('--demo cannot be combined with legacy local tests')
if args.console_test and (args.manipulation_test or args.navigation_test):
    parser.error('--console-test exercises a complete delivery; omit local test modes')
if args.console_test and not args.gui:
    parser.error('--console-test requires --gui')
if args.exit_after_test and not args.console_test:
    parser.error('--exit-after-test requires --console-test')
mode = 'manipulation-test' if args.manipulation_test else 'navigation-test' if args.navigation_test else 'transport'
from isaacsim import SimulationApp
app = SimulationApp({'headless':not args.gui,'multi_gpu':False,'width':1280,'height':800,
    'renderer':'RaytracedLighting','anti_aliasing':1})
runtime = None
failed = None
try:
    import numpy as np
    import omni.usd
    import omni.kit.app
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate('isaacsim.core.api',True)
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
    from robot_control import load_robot_data
    from transport_runtime import TransportRuntime

    c = load_config()
    if json.loads((ASSETS/"scene_config.json").read_text()) != c:
        raise RuntimeError("Scene config changed: run ./scripts/simctl build-scene first")
    cfg = json.loads((ROOT/'config/transport_config.json').read_text())
    if args.demo:
        from floor_demo import load_demo
        cfg['demo'] = load_demo()
        cfg['capture_stages'] = not args.gui
        cfg['navigation'].update(cfg['demo']['navigation'])
        cfg['timeout'] = cfg['demo']['timeout']
    robot_cfg,meta = load_robot_data()
    if not args.demo and 'lift' in robot_cfg['source_urdf']:
        cfg = json.loads((ROOT/'config/profiles/transport_lift.json').read_text())
    omni.usd.get_context().open_stage(str(ASSETS/'main.usda'))
    for _ in range(12): app.update()
    app.reset_render_settings()
    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    if args.demo:
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
                iterations = cfg['demo']['solver_iterations']
                api.CreateSolverPositionIterationCountAttr(iterations['position'])
                api.CreateSolverVelocityIterationCountAttr(iterations['velocity'])
    source = table_by_id(c,c['task']['source'])
    target = table_by_id(c,c['task']['target'])
    def dock(table,clearance):
        return np.array([table['xy'][0],table['xy'][1]+table['size'][1]/2+clearance])
    if source['xy'][1] >= 0 or target['xy'][1] >= 0:
        raise RuntimeError('This transport setup supports the two south-side stations; replan approach geometry for north-side tables')
    initial = [*dock(source,.65),-math.pi/2] if args.manipulation_test else [*c['robot']['position'][:2],0.]
    if args.demo:
        initial = cfg['demo']['initial_pose']
    # One-time task setup: same box, moved to the reachable front edge of the source.
    box_initial = [source['xy'][0],source['xy'][1]+source['size'][1]/2-cfg['manipulation']['edge_inset'],
                   source['size'][2]+c['box']['size'][2]/2+.005]
    world = World(physics_dt=c['physics']['dt'],rendering_dt=c['physics']['dt'],
                  stage_units_in_meters=1.,backend='numpy',device='cpu')
    robot = world.scene.add(SingleArticulation('/World/Robot'+meta['root_path'],name='r680'))
    robot.set_default_state(position=np.array([*initial[:2],.015]),
                            orientation=np.array([math.cos(initial[2]/2),0,0,math.sin(initial[2]/2)]))
    box_initials = [[*xy, c['box']['size'][2]/2+.005] for xy in cfg['demo']['floor_positions']] if args.demo else [box_initial]
    boxes = []
    for i, position in enumerate(box_initials):
        path = f'/World/Boxes/Box_{i:02d}'
        if not stage.GetPrimAtPath(path).IsValid():
            stage.DefinePrim(path, 'Xform').GetReferences().AddReference(str(ASSETS/'box.usda'))
        item = world.scene.add(SingleRigidPrim(path, name=f'transport_box_{i}'))
        item.set_default_state(position=np.array(position), orientation=np.array([1.,0,0,0]))
        PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path)).CreateThresholdAttr(0.)
        if args.demo:
            # Keep all three boxes actively evaluated after release; sleeping
            # rigid bodies can retain stale tensor velocities/contact state.
            PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(path)).CreateSleepThresholdAttr(0.)
        boxes.append(item)
    if args.demo:
        # Matching colors identify each physical box and its workbench slot.
        colors = [(0.85, .46, .09), (.12, .48, .82), (.22, .66, .34)]
        for i, color in enumerate(colors):
            shader = UsdShade.Shader.Get(stage, f'/World/Boxes/Box_{i:02d}/Material/Shader')
            shader.GetInput('diffuseColor').Set(Gf.Vec3f(*color))
            x = target['xy'][0] + cfg['demo']['slot_offsets'][i]
            y = target['xy'][1] + target['size'][1]/2 - .2
            for j, (dx, dy, sx, sy) in enumerate([(-.235,0,.01,.34),(.235,0,.01,.34),(0,-.17,.48,.01),(0,.17,.48,.01)]):
                marker = UsdGeom.Cube.Define(stage, f'/World/DemoSlots/Slot_{i}_{j}')
                marker.CreateSizeAttr(1.)
                marker.AddTranslateOp().Set(Gf.Vec3d(x+dx,y+dy,target['size'][2]+.002))
                marker.AddScaleOp().Set(Gf.Vec3f(sx,sy,.002))
                marker.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    box = boxes[0]
    box_initial = box_initials[0]
    # Estimated rubber contact friction; preserve finger geometry and force limits.
    mat = UsdShade.Material.Define(stage,'/World/TransportFingerRubber')
    material = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    material.CreateStaticFrictionAttr(1.2)
    material.CreateDynamicFrictionAttr(1.0)
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.CollisionAPI) and 'openarm_' in str(p.GetPath()) and 'ee_link' in str(p.GetPath()):
            UsdShade.MaterialBindingAPI.Apply(p).Bind(mat,materialPurpose='physics')
    world.reset()
    robot.set_joints_default_state(positions=np.array([robot_cfg['home'].get(n,0) for n in robot.dof_names]),
                                   velocities=np.zeros(robot.num_dof),efforts=np.zeros(robot.num_dof))
    world.reset()
    def clear_reset_velocities():
        robot.set_joint_velocities(np.zeros(robot.num_dof))
        robot.set_linear_velocity(np.zeros(3))
        robot.set_angular_velocity(np.zeros(3))
        for item in boxes:
            item.set_linear_velocity(np.zeros(3))
            item.set_angular_velocity(np.zeros(3))
    def sync_ready_rendering():
        if args.gui:
            # Publish initialized articulation/rigid poses to rendering, then
            # restore exact defaults before creating the idle task runtime.
            world.step(render=True)
            world.reset(soft=True)
            clear_reset_velocities()
            if not all(np.allclose(item.get_world_pose()[0], pos, atol=1e-5) for item, pos in zip(boxes, box_initials)):
                raise RuntimeError('Box reset failed after render synchronization')
            if not np.allclose(robot.get_joint_positions(), [robot_cfg['home'].get(n,0) for n in robot.dof_names], atol=1e-5):
                raise RuntimeError('Robot reset failed after render synchronization')
            print('DISPLAY_INITIAL_STATE_READY', flush=True)
    clear_reset_velocities()
    sync_ready_rendering()
    app.reset_render_settings()
    from transport_task import execute_transport
    if args.demo:
        from floor_demo import execute_floor_demo as execute_transport
    from transport_runtime import ResetRequested
    import omni.timeline
    timeline = omni.timeline.get_timeline_interface()
    console = None
    if args.gui:
        from operator_console import OperatorConsole
        console = OperatorConsole(stage, meta, capture_events=args.console_test, demo=args.demo)

    def new_runtime():
        result = TransportRuntime(app,world,robot,box,stage,c,cfg,initial,
                                  render_every=cfg.get('demo', {}).get('render_every', 6) if args.gui else 30)
        if args.demo:
            result.configure_boxes(boxes)
        if console:
            console.runtime = result
            result.status_model = console.status_model
            result.before_render = console.update_views
        return result

    runtime = new_runtime()
    if not console:
        execute_transport(runtime,c,cfg,source,target,initial,args,mode)
        runtime.ctl.stop()
    else:
        timeline.pause()
        console.set_state('READY')
        idle_frames = 0
        test_stage = 0
        test_results = {}
        while app.is_running():
            if console.reset_requested:
                runtime.ctl.home()
                runtime.ctl.sent = runtime.ctl.targets.copy()
                runtime.ctl.update(runtime.dt)
                runtime.subscription = None
                world.reset()
                clear_reset_velocities()
                sync_ready_rendering()
                app.reset_render_settings()
                runtime = new_runtime()
                timeline.pause()
                console.reset_requested = False
                console.start_requested = False
                console.set_state('READY')
                idle_frames = 0
                if args.console_test:
                    q = robot.get_joint_positions()
                    test_results['reset_'+str(test_stage)] = bool(
                        np.allclose(q,[robot_cfg['home'].get(n,0) for n in robot.dof_names],atol=.001)
                        and all(np.allclose(item.get_world_pose()[0], pos, atol=.001) for item, pos in zip(boxes, box_initials))
                        and np.allclose(robot.get_world_pose()[0],[*initial[:2],.015],atol=.001)
                        and runtime.steps == 0 and runtime.slam.scans == 0
                        and np.max(np.abs(robot.get_joint_velocities())) < .001
                        and np.linalg.norm(box.get_linear_velocity()) < .001)
                    print("CONSOLE_RESET_CHECK",test_stage,test_results,flush=True)
                    if test_stage == 1: test_stage = 2
                    elif test_stage == 3:
                        test_stage = 4
                        test_results['passed'] = all(test_results.values())
                        (OUTPUTS/('demo_console_test_report.json' if args.demo else 'console_test_report.json')).write_text(json.dumps(test_results,indent=2))
                        print('CONSOLE_TEST',json.dumps(test_results),flush=True)
            if console.start_requested:
                console.start_requested = False
                console.set_state('RUNNING')
                timeline.play()
                try:
                    execute_transport(runtime,c,cfg,source,target,initial,args,mode)
                    runtime.ctl.stop()
                    console.set_state('COMPLETE' if mode == 'transport' else mode.upper()+' COMPLETE - RESET to restart')
                    console.capture('console_complete')
                    if args.console_test and test_stage == 2:
                        test_results['transport_complete'] = (all(p.success for p in runtime.placements) and len(runtime.completed_boxes) == len(boxes) if args.demo else runtime.placement.success) and runtime.retreat_distance == cfg['manipulation']['retreat_distance'] and not runtime.contacting('/World/Robot')
                        test_stage = 3
                        idle_frames = 0
                except ResetRequested:
                    pass
                except Exception as exc:
                    traceback.print_exc()
                    runtime.ctl.stop()
                    runtime.report(str(exc),mode=mode)
                    console.set_state('FAILED: '+str(exc))
                    if args.console_test:
                        test_results.update(passed=False,error=str(exc))
                        (OUTPUTS/('demo_console_test_report.json' if args.demo else 'console_test_report.json')).write_text(json.dumps(test_results,indent=2))
                        if args.exit_after_test:
                            failed = str(exc)
                            break
                finally:
                    runtime.ctl.stop()
                    timeline.pause()
            else:
                # Paused physics: rendering alone never starts autonomous motion.
                timeline.pause()
                app.update()
                idle_frames += 1
                if idle_frames == 60:
                    console.activate()
                if idle_frames == 120:
                    console.capture('console_ready' if test_stage == 0 else 'console_reset')
                if args.console_test and idle_frames == 150:
                    if test_stage == 0:
                        test_results['idle_no_motion'] = runtime.steps == 0 and all(np.allclose(item.get_world_pose()[0], pos, atol=.001) for item, pos in zip(boxes, box_initials))
                        test_stage = 1
                        if args.demo:
                            runtime.test_reset_callback = lambda: console.request_reset() if runtime.box_index == 1 and runtime.phase == 'LIFT_BOX' else None
                        else:
                            runtime.test_reset_callback = lambda: console.request_reset() if runtime.time >= 4 else None
                        console.request_start()
                    elif test_stage == 2:
                        console.request_start()
                    elif test_stage == 3:
                        console.request_reset()
                    elif test_stage == 4:
                        console.capture('console_reset')
                if args.console_test and test_stage == 4 and idle_frames == 160 and args.exit_after_test:
                    if not test_results["passed"]: failed = "Console validation failed"
                    break
except Exception as exc:
    failed = str(exc)
    traceback.print_exc()
    if runtime is not None:
        runtime.ctl.stop()
        try:
            runtime.snapshot('failed')
            runtime.report(failed,mode=mode)
        except Exception:
            traceback.print_exc()
finally:
    app.close(exit_code=1 if failed else 0)
