"""Independent Isaac viewport. No World, physics stepping, or task control."""
import argparse
import asyncio
import json
import math
import sys
import time
from types import SimpleNamespace
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from common import ROOT,OUTPUTS
from relay_state_ipc import StateChannel
from navigation_console_ipc import atomic_json

parser=argparse.ArgumentParser()
parser.add_argument('--fps',type=float,default=30.)
parser.add_argument('--frames',type=int,default=0)
parser.add_argument('--no-camera-images',action='store_true')
parser.add_argument('--camera-worker',action='store_true',help='Independent headless first-person image worker')
args=parser.parse_args()
if not 1<=args.fps<=60:parser.error('FPS must be 1–60')
import faulthandler
faulthandler.dump_traceback_later(60,repeat=True)
from isaacsim import SimulationApp
app=SimulationApp(dict(headless=args.camera_worker,multi_gpu=False,width=1280,height=720,
    window_width=1440,window_height=900,renderer='RaytracedLighting',anti_aliasing=2,
    limit_cpu_threads=2 if args.camera_worker else 6,extra_args=['--/app/window/title=双机拆码垛 · 独立显示']))
print("RELAY_VIEWER_APP_READY",flush=True)
faulthandler.cancel_dump_traceback_later()
try:
    import numpy as np
    import omni.usd
    import omni.timeline
    import omni.ui as ui
    from pxr import Gf,UsdGeom,UsdPhysics
    from omni.kit.viewport.utility import get_active_viewport
    context=omni.usd.get_context()
    if not context.open_stage(str(ROOT/'assets/generated/relay/main.usda')):
        raise RuntimeError('Cannot open relay display scene')
    print('RELAY_VIEWER_STAGE_OPEN',flush=True)
    stage=context.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
        if prim.IsA(UsdPhysics.Joint):
            UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
        if prim.IsA(UsdPhysics.Scene):prim.SetActive(False)
    timeline=omni.timeline.get_timeline_interface()
    timeline.stop()
    for _ in range(8):app.update()
    c=json.loads((ROOT/'config/relay_config.json').read_text())
    metadata=json.loads((ROOT/'assets/robots/generated/relay/robot_metadata.json').read_text())
    if not args.camera_worker:
        viewport=get_active_viewport()
        if viewport is None:raise RuntimeError('Main viewport unavailable')
        viewport.fill_frame=False
        viewport.set_texture_resolution((1280,720))
        camera=UsdGeom.Camera.Define(stage,'/World/IndependentViewerCamera')
        camera.CreateFocalLengthAttr(24.)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.03,100.))
        camera.MakeMatrixXform().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*c['camera']['position']),
            Gf.Vec3d(*c['camera']['target']),Gf.Vec3d(0,0,1)).GetInverse())
        viewport.camera_path=camera.GetPath()
        info=ui.Window('仿真连接状态',width=340,height=90)
        with info.frame:
            label=ui.Label('等待物理仿真连接…',word_wrap=True)
    latest=None
    latest_array=previous_array=applied_array=None
    previous=None
    sequence=None
    session=None
    channel=None
    operations={}
    class DisplayRobot:
        def __init__(self,name):self.name=name
        def get_world_pose(self):
            p=stage.GetPrimAtPath('/World/Robots/'+self.name+metadata['root_path'])
            m=UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0)
            q=m.ExtractRotationQuat()
            return np.asarray(m.ExtractTranslation()),np.array([q.GetReal(),*q.GetImaginary()])
        def get_joint_positions(self):
            return np.asarray(latest['joints'][self.name]) if latest else np.zeros(1)
    streams=None
    if not args.no_camera_images:
        import omni.replicator.core as rep
        from robot_camera_streams import RobotCameraStreams
        streams=RobotCameraStreams(stage,{r['id']:DisplayRobot(r['id']) for r in c['robots']},metadata)
        streams.scene_sample=lambda: latest_array.ravel() if latest_array is not None else np.zeros(1)
    async def capture():
        from omni.kit.viewport.utility import capture_viewport_to_file
        await capture_viewport_to_file(viewport,str(OUTPUTS/'relay_decoupled_viewport.png')).wait_for_result()
    frames=updates=0
    began=window_started=time.perf_counter()
    window_frames=0
    last_metrics=0.
    apply_seconds=render_seconds=0.
    while app.is_running() and (not args.frames or frames<args.frames):
        started=time.perf_counter()
        if args.camera_worker:
            try:
                lease=json.loads((OUTPUTS/'robot_cameras/viewers.json').read_text())
                watching=lease['count']>0 and 0<=time.time()-lease['updated_at']<4.
            except (OSError,ValueError,KeyError):watching=False
            if not watching:
                time.sleep(.2)
                continue
        if channel is None:
            try:channel=StateChannel()
            except (OSError,ValueError):pass
        snapshot=channel.read() if channel else None
        if snapshot and (snapshot['session'],snapshot['transport_sequence'])!=sequence:
            sequence=(snapshot['session'],snapshot['transport_sequence'])
            if session!=snapshot['session']:
                previous=None
                previous_array=applied_array=None
                if streams is not None:
                    streams.last_scene=None
                    streams.last_states=None
                    streams.session=snapshot['session']
                session=snapshot['session']
            else:
                previous=latest
                previous_array=latest_array
            latest=snapshot
            latest_array=np.asarray([pose for _,pose in latest['poses']],dtype=float)
            updates+=1
        if latest:
            # One sample of display delay allows interpolation without touching
            # the authoritative physics state. No extrapolation past newest pose.
            alpha=1.
            if previous and len(previous['poses'])==len(latest['poses']):
                span=latest['monotonic']-previous['monotonic']
                if 0<span<.5:
                    alpha=float(np.clip((time.monotonic()-.05-previous['monotonic'])/span,0.,1.))
            apply_started=time.perf_counter()
            poses=latest_array
            if alpha<1.:
                old=previous_array.copy()
                flipped=np.sum(old[:,3:]*poses[:,3:],axis=1)<0
                old[flipped,3:]*=-1
                poses=(1-alpha)*old+alpha*poses
                poses[:,3:]/=np.linalg.norm(poses[:,3:],axis=1)[:,None]
            changed=np.arange(len(poses)) if applied_array is None else np.flatnonzero(np.max(np.abs(poses-applied_array),axis=1)>1e-4)
            for i in changed:
                path=latest['poses'][i][0]
                pose=poses[i]
                if path not in operations:
                    prim=stage.GetPrimAtPath(path)
                    if not prim.IsValid():raise RuntimeError('Snapshot path missing: '+path)
                    xf=UsdGeom.Xformable(prim)
                    operations[path]=xf.MakeMatrixXform()
                    xf.SetResetXformStack(True)
                transform=Gf.Matrix4d(1.)
                transform.SetRotate(Gf.Quatd(float(pose[3]),Gf.Vec3d(*map(float,pose[4:]))))
                transform.SetTranslateOnly(Gf.Vec3d(*map(float,pose[:3])))
                operations[path].Set(transform)
            if applied_array is None:
                applied_array=poses.copy()
            else:
                applied_array[changed]=poses[changed]
            apply_seconds+=time.perf_counter()-apply_started
        display_runtime=SimpleNamespace(states=latest['operator'].get('states',{})) if latest and latest['operator'].get('states') else None
        if (args.camera_worker and latest and not len(changed) and streams.status is not None
                and all(c['version']>0 and not c['error'] for c in streams.status['cameras'])):
            streams.publish(latest['simulation_seconds'],runtime=display_runtime,operator=latest['operator'],read_images=False)
            if time.perf_counter()-last_metrics>=2.:
                atomic_json(OUTPUTS/'relay_camera_performance.json',dict(updated_at=time.time(),session=session,
                    frames=frames,snapshots=updates,display_fps=0.,idle=True,physics_enabled=False))
                last_metrics=time.perf_counter()
            time.sleep(max(0.,1/args.fps-(time.perf_counter()-started)))
            continue
        if timeline.is_playing():timeline.stop()
        render_started=time.perf_counter()
        if args.camera_worker:
            # A stopped timeline does not trigger RGB annotators via app.update.
            # Capture explicitly without advancing this display-only stage.
            rep.orchestrator.step(delta_time=0.0, pause_timeline=True)
        else:
            app.update()
        render_seconds+=time.perf_counter()-render_started
        if latest and streams is not None:
            streams.publish(latest['simulation_seconds'],runtime=display_runtime,operator=latest['operator'],
                            read_images=streams.prepare_render())
        frames+=1
        if frames==90 and latest and not args.camera_worker:
            asyncio.ensure_future(capture())
        window_frames+=1
        now=time.perf_counter()
        if now-last_metrics>=2.:
            age=time.monotonic()-latest['monotonic'] if latest else None
            fps=window_frames/(now-window_started)
            message=(f'显示 {fps:.1f} 帧/秒　状态延迟 {age*1000:.0f} 毫秒\n'+
                        ('物理仿真已失联，保留最后画面' if age>2 else '物理与显示独立运行')) if age is not None else '等待物理仿真连接…'
            if not args.camera_worker:label.text=message
            atomic_json(OUTPUTS/('relay_camera_performance.json' if args.camera_worker else 'relay_viewer_performance.json'),dict(updated_at=time.time(),session=session,
                frames=frames,snapshots=updates,display_fps=fps,state_age_seconds=age,
                apply_seconds=apply_seconds,render_seconds=render_seconds,physics_enabled=False,
                elapsed_seconds=now-began))
            window_started=now;window_frames=0;last_metrics=now
        time.sleep(max(0.,1/args.fps-(time.perf_counter()-started)))
finally:
    app.close()
