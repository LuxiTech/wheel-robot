"""Render RGB + optical-Z depth at a URDF sensor frame; no SDK/network access."""
import hashlib
import io
import json
import os
from pathlib import Path
import sys

sys.path.insert(0,'/workspace/tools/grasp')
sys.path.insert(0,'/workspace/grasp_core')
from runlog import RunLog, write_json

directory = Path(sys.argv[1])
os.chdir('/workspace')
log = RunLog(directory,'capture')
app = None
try:
    with log.stage('simulation_capture'):
        cfg = json.loads((directory/'runtime.json').read_text())
        with log.stage('isaac_camera_startup'):
            from isaacsim import SimulationApp
            app = SimulationApp({'headless':True,'multi_gpu':False,'renderer':'RaytracedLighting'})
            import numpy as np
            import omni.replicator.core as rep
            from pxr import Gf,UsdGeom
            from PIL import Image
            from grasp_sim_scene import build_scene
            from wheel_grasp.kinematics import RobotModel
            from isaacsim.core.utils.types import ArticulationAction
            world,stage,robot,meta,q,truth = build_scene(cfg)
            with log.stage('settle_simulation'):
                for _ in range(240):
                    robot.apply_action(ArticulationAction(joint_positions=q))
                    world.step(render=False)
                base_position,base_orientation = robot.get_world_pose()
                base_state = dict(position=base_position.tolist(),orientation=base_orientation.tolist())
                if np.linalg.norm(robot.get_linear_velocity())>.01 or np.linalg.norm(robot.get_angular_velocity())>.02:
                    raise ValueError('Simulation base did not settle before capture')
                frozen_base = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(
                    stage.GetPrimAtPath('/World/Robot'+meta['root_path']))).T
            # world.render() calls app.update(); pause the timeline so renderer
            # warmup cannot advance physics or settle the floating robot base.
            world.pause()
            camera_frames = {'head_rgb':'head_d455_color_optical_frame',
                             'left_wrist_rgb':'left_wrist_eb435i_color_optical_frame',
                             'right_wrist_rgb':'right_wrist_eb435i_color_optical_frame'}
            frame = camera_frames[cfg['camera']]
            camera = UsdGeom.Camera.Define(stage,'/World/Robot'+meta['frames'][frame]+'/GraspCamera')
            camera.AddRotateXOp().Set(180.) # ROS optical to USD camera axes.
            c = cfg['simulation']['camera'];w,h = c['width'],c['height']
            if c['cx']!=w/2 or c['cy']!=h/2 or c['fx']!=c['fy']:
                raise ValueError('Current renderer requires centered square-pixel pinhole configuration')
            camera.CreateFocalLengthAttr(24.)
            camera.CreateHorizontalApertureAttr(24*w/c['fx'])
            camera.CreateVerticalApertureAttr(24*h/c['fy'])
            camera.CreateClippingRangeAttr(Gf.Vec2f(c['near'],c['far']))
            product = rep.create.render_product(str(camera.GetPath()),(w,h))
            rgb_reader = rep.AnnotatorRegistry.get_annotator('rgb')
            depth_reader = rep.AnnotatorRegistry.get_annotator('distance_to_image_plane')
            rgb_reader.attach([product]);depth_reader.attach([product])
        with log.stage('render_rgbd'):
            # Render the same frozen physics state: no different RGB/depth poses.
            rep.orchestrator.step(rt_subframes=8,delta_time=0.,pause_timeline=True,wait_for_render=True)
            rgba = np.asarray(rgb_reader.get_data())
            if rgba.ndim != 3:
                raise ValueError('Renderer did not deliver RGB annotator data')
            rgb = rgba[:,:,:3].astype(np.uint8)
            depth = np.asarray(depth_reader.get_data(),dtype='<f4').reshape(h,w)
            if rgb.shape!=(h,w,3) or not np.isfinite(depth).any():
                raise ValueError('Empty simulator render output')
            depth[(depth<=c['near'])|(depth>=c['far'])|~np.isfinite(depth)] = np.nan
            names = list(robot.dof_names);values = np.asarray(robot.get_joint_positions())
            joint_names = ['lift_joint']+[f'openarm_{s}_joint{i}' for s in ('left','right') for i in range(1,8)]
            joints = {n:float(values[names.index(n)]) for n in joint_names}
            model = RobotModel(cfg['urdf'])
            expected = model.transform(frame,joints,cfg['base_frame'])
            cache = UsdGeom.XformCache()
            base_tf = np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/Robot'+meta['root_path']))).T
            optical_tf = np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/Robot'+meta['frames'][frame]))).T
            actual = np.linalg.inv(base_tf)@optical_tf
            if not np.allclose(base_tf, frozen_base, atol=1e-5):
                raise ValueError('Robot base moved during frozen RGB-D acquisition')
            if not np.allclose(actual,expected,atol=1e-4):
                raise ValueError('USD sensor mounting does not match URDF FK')
            jpeg=io.BytesIO();Image.fromarray(rgb).save(jpeg,format='JPEG')
            metadata=dict(schema='luxi_rgbd_v1',source='isaac_sim',mock=True,camera_id=cfg['camera'],
                pair_id=directory.name+':sim:0',frame_id=cfg['camera']+'_color_optical',
                width=w,height=h,age_ms=0.,aligned_to_color=True,calibration_available=True,
                intrinsics=dict(K=[c['fx'],0,c['cx'],0,c['fy'],c['cy'],0,0,1],D=[0]*5,distortion_model='plumb_bob'),
                color=dict(encoding='jpeg',bytes=len(jpeg.getvalue())),
                depth=dict(encoding='32FC1',unit='metre',byte_order='little-endian',step=w*4,bytes=w*h*4,value_semantics='color_optical_z'))
            with (directory/'observation.npz').open('xb') as out:
                np.savez_compressed(out,metadata=json.dumps(metadata),rgb=rgb,depth=depth)
            Image.fromarray(rgb).save(directory/'raw_rgb.png')
            write_json(directory/'joints.json',joints)
            write_json(directory/'capture_context.json',dict(source='isaac_sim',camera_frame=frame,
                pair_id=metadata['pair_id'],camera_to_base=actual.tolist(),base_to_world=base_tf.tolist(),
                base_state=base_state,hardware_access=False,
                calibration_source=c['calibration_source'],simulation_initialization_valid=True))
            # Evaluation only: inference worker never reads this file.
            target_world = np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/Target'))).T
            write_json(directory/'sim_ground_truth.json',dict(pose=(np.linalg.inv(base_tf)@target_world).tolist(),evaluation_only=True))
            files=[cfg['urdf'],cfg['model']['mesh'],cfg['model']['colliders'],
                   cfg['simulation']['robot_usd'],cfg['simulation']['robot_metadata'],cfg['simulation']['crate_usd']]
            write_json(directory/'asset_hashes.json',{p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in files})
            write_json(directory/'capture.result.json',dict(run_id=directory.name,status='completed',
                source='isaac_sim',simulation_initialization_valid=True,pair_id=metadata['pair_id']))
finally:
    if app is not None:
        # All outputs were written synchronously; there are no Replicator
        # writers to drain. Waiting here can hang after a paused zero-dt step.
        app.close(wait_for_replicator=False, exit_code=1 if sys.exc_info()[0] else 0)
