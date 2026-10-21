"""Persistent isolated RGB-D inference worker for the original relay state machine."""
import json,sys,time,traceback
from pathlib import Path
sys.path.insert(0,'/workspace/grasp_core')
from runlog import RunLog,write_json
from wheel_grasp.rgbd import load_observation
from wheel_grasp.geometry import prepare_input,mask_quality,connected_depth_mask
from wheel_grasp.pipeline import surface_quality,refine_surface_pose
from wheel_grasp.backends import NanoDetectorSegmenter,FoundationPoseBackend
from wheel_grasp.association import projected_box,associate
from wheel_grasp.visualization import draw_detections,draw_mask,draw_pose
from scipy.spatial.transform import Rotation
import numpy as np
import cv2
root=Path(sys.argv[1]);cfg=json.loads((root/'runtime.json').read_text());m=cfg['model'];log=RunLog(root,'model_worker')
with log.stage('load_models'):
 detector=NanoDetectorSegmenter(m['owl_engine'],m['sam_encoder'],m['sam_decoder'],m['prompt'])
 estimator=FoundationPoseBackend(m['foundationpose_repository'],m['mesh'],debug_dir=root/'foundationpose')
 colliders=json.loads(Path(m['colliders']).read_text())
write_json(root/'model.ready.json',dict(status='ready'))
identity=None
while not (root/'stop').exists():
 for request in sorted(root.glob('frames/*/request.json')):
  folder=request.parent
  if (folder/'result.json').exists():continue
  frame_log=RunLog(folder,'inference')
  try:
   with frame_log.stage('model_pose'):
    r=json.loads(request.read_text());obs=load_observation(folder/'observation.npz',allow_mock=True)
    if obs.metadata.get('source')!='isaac_sim':raise ValueError('Relay worker requires simulator RGB-D')
    inp=prepare_input(obs);camera=np.asarray(r['camera_world']);prior=np.asarray(r['prior_world'])
    roi=projected_box(prior,camera,inp.k,r['size'],inp.depth.shape)
    bgr=cv2.cvtColor(inp.rgb,cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(folder/'rgb.png'),bgr)
    detections=detector.detect(inp.rgb);write_json(folder/'detections.json',detections)
    cv2.imwrite(str(folder/'detection_overlay.png'),draw_detections(bgr,detections,roi))
    key=(r['robot'],r['target_id'])
    try:
     prompt,selection=associate(detections,roi)
    except ValueError:
     if identity!=key:raise
     prompt=roi
     selection=dict(method='registered_target_tracking_roi',roi=roi.tolist(),
                    reason='detector missed an already registered target; current RGB-D still required')
    write_json(folder/'association.json',selection)
    raw_mask=detector.segment(inp.rgb,prompt)
    cv2.imwrite(str(folder/'mask_raw.png'),raw_mask.astype(np.uint8)*255)
    cv2.imwrite(str(folder/'segmentation_raw_overlay.png'),draw_mask(bgr,raw_mask.astype(bool)))
    # RGB instance boundaries contain mixed foreground/background depth.
    # Remove one image pixel at the boundary before lifting the mask into 3D.
    mask=cv2.erode(raw_mask.astype(np.uint8),np.ones((3,3),np.uint8))>0
    mask,clustering=connected_depth_mask(mask,inp.depth,inp.k)
    cv2.imwrite(str(folder/'mask.png'),mask.astype(np.uint8)*255)
    cv2.imwrite(str(folder/'segmentation_overlay.png'),draw_mask(bgr,mask))
    write_json(folder/'depth_clustering.json',clustering)
    write_json(folder/'mask_cleanup.json',dict(method='one_pixel_boundary_erosion',
         raw_pixels=int(raw_mask.sum()),retained_pixels=int(mask.sum())))
    quality=mask_quality(mask,inp.depth)
    if not quality['accepted']:raise ValueError('Depth/mask rejected: '+str(quality))
    tracked=identity==key
    t=estimator.track(inp,key) if tracked else estimator.register(inp,mask,key)
    write_json(folder/'camera_pose_initial.json',t.tolist())
    quality.update(surface_quality(inp,mask,t,colliders))
    if not quality['accepted'] and tracked:
     t=estimator.register(inp,mask,key);quality.update(surface_quality(inp,mask,t,colliders));tracked=False
    if quality['surface_rms_m']>.003:
     write_json(folder/'camera_pose_before_refinement.json',t.tolist())
     t,refinement=refine_surface_pose(inp,mask,t,colliders)
     write_json(folder/'refinement.json',refinement)
     quality.update(surface_quality(inp,mask,t,colliders))
    write_json(folder/'camera_pose.json',t.tolist())
    write_json(folder/'quality.json',quality)
    cv2.imwrite(str(folder/'pose_overlay.png'),draw_pose(draw_mask(bgr,mask),t,inp.k,r['size']))
    if not quality['accepted']:raise ValueError('Pose surface quality rejected: '+str(quality))
    world=camera@t
    # The box is 180-degree symmetric. Choose the equivalent orientation
    # nearest the last observation/task prior to preserve controller continuity.
    symmetric=world.copy();symmetric[:3,:3]=world[:3,:3]@Rotation.from_euler('z',np.pi).as_matrix()
    world=min((world,symmetric),key=lambda x:Rotation.from_matrix(x[:3,:3]@prior[:3,:3].T).magnitude())
    distance=float(np.linalg.norm(world[:3,3]-prior[:3,3]))
    if distance>.12:raise ValueError('Estimated object does not match task identity: %.4fm'%distance)
    identity=key
    write_json(folder/'result.json',dict(status='completed',pair_id=obs.key[1],target_id=r['target_id'],
       pose=world.tolist(),quality=quality,tracked=tracked,prior_distance_m=distance,source='nanoowl_nanosam_foundationpose'))
  except Exception as exc:
   identity=None
   write_json(folder/'result.json',dict(status='failed',error=str(exc),traceback=traceback.format_exc()))
 if '--once' in sys.argv:break
 time.sleep(.02)
