#!/usr/bin/env python3
"""Diagnostic CAD pose and rim TCP projections in camera coordinates; no robot adapter."""
import argparse
import hashlib
import json
import logging
import os
from pathlib import Path
import sys
import uuid
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools'))
from _bootstrap import configure
configure(ROOT)
from runlog import write_json
from perceive_live import Trace,serial


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('observation')
    selection=p.add_mutually_exclusive_group(required=True)
    selection.add_argument('--target-pixel',nargs=2,type=float)
    selection.add_argument('--box',nargs=4,type=float,help='Explicit ROI for diagnostic segmentation when detector misses target')
    p.add_argument('--mesh',default='assets/objects/crate/model.obj')
    p.add_argument('--inset',type=float,default=.007,help='Unvalidated candidate offset inward from CAD outer rim, metres')
    p.add_argument('--below-top',type=float,default=.001,help='Unvalidated candidate offset below top rim, metres')
    args=p.parse_args();os.chdir(ROOT)
    for key,value in json.loads((ROOT/'.runtime/vision/native-build-env.json').read_text()).items():os.environ.setdefault(key,value)
    os.environ['PATH']=str(Path(sys.executable).parent)+':'+os.environ.get('PATH','')
    os.environ['HF_HOME']=str(ROOT/'assets/models/huggingface');os.environ['HF_HUB_OFFLINE']='1';os.environ['TRANSFORMERS_OFFLINE']='1'
    for key,sub in [('TORCH_EXTENSIONS_DIR','torch-extensions'),('XDG_CACHE_HOME','cache'),('MPLCONFIGDIR','matplotlib'),('WARP_CACHE_PATH','warp-cache')]:
        os.environ[key]=str(ROOT/'.runtime/vision'/sub);Path(os.environ[key]).mkdir(parents=True,exist_ok=True)
    out=ROOT/'outputs/grasp/camera-preview'/uuid.uuid4().hex[:10];out.mkdir(parents=True)
    print('Artifacts: '+str(out),flush=True)
    trace=Trace(out);status='failed'
    logging.basicConfig(filename=out/'models.log',level=logging.INFO,force=True)
    import cv2
    import numpy as np
    import torch
    import trimesh
    from wheel_perception_core.rgbd import load_observation,save_observation
    from wheel_perception_core.geometry import prepare_input,mask_quality,rim_targets
    from wheel_perception_core.backends import NanoDetectorSegmenter,FoundationPoseBackend
    from wheel_perception_core.visualization import draw_detections,project_points
    write_json(out/'status.json',dict(status='running',diagnostic_only=True,executable=False))
    try:
        with trace.stage('capture_replay'):
            obs=load_observation(args.observation);save_observation(out/'observation.npz',obs)
            trace.png('00_rgb',cv2.cvtColor(obs.rgb,cv2.COLOR_RGB2BGR))
            inp=prepare_input(obs);trace.record('preprocess',inp=inp)
            write_json(out/'run.json',dict(source_observation=str(Path(args.observation).resolve()),pair_id=obs.key[1],camera_id=obs.key[0],frame_id=inp.frame,diagnostic_only=True,executable=False,reasons=inp.reasons))
        with trace.stage('cad_normalization'):
            mesh=trimesh.load(args.mesh,force='mesh');height=float(mesh.extents[2])
            transform=np.diag([1.,-1.,-1.,1.]);transform[2,3]=float(mesh.bounds[:,2].sum())
            mesh.apply_transform(transform)
            mesh.export(out/'opening_up.obj')
            rules=dict(width=float(mesh.extents[0]),height=height,inset=args.inset,below_top=args.below_top,pitch_degrees=50.,heading=0.,clearance=.06)
            write_json(out/'cad_and_rules.json',serial(dict(source_mesh=str(Path(args.mesh).resolve()),source_sha256=hashlib.sha256(Path(args.mesh).read_bytes()).hexdigest(),source_to_opening_up=transform,dimensions_m=mesh.extents,rules=rules,grasp_rules_validated=False,selection='opposite X rim midpoints; not calibrated finger contacts')))
        with trace.stage('detection'):
            e=ROOT/'assets/models/engines'
            detector=NanoDetectorSegmenter(e/'owl_image_encoder.engine',e/'resnet18_image_encoder.engine',e/'mobile_sam_mask_decoder.engine',prompt='a blue plastic box')
            candidates=detector.detect(inp.rgb)
            trace.png('03_detections',draw_detections(cv2.cvtColor(inp.rgb,cv2.COLOR_RGB2BGR),candidates,args.box))
            write_json(out/'detection.json',dict(candidates=candidates,target_pixel=args.target_pixel,box=args.box,selection='explicit_roi' if args.box else 'explicit_pixel_inside_one_detection'))
            if args.box:
                box=args.box;h,w=inp.depth.shape
                if not (0<=box[0]<box[2]<=w and 0<=box[1]<box[3]<=h):raise ValueError('Invalid ROI')
            else:
                x,y=args.target_pixel
                selected=[c for c in candidates if c['box'][0]<=x<=c['box'][2] and c['box'][1]<=y<=c['box'][3]]
                if len(selected)!=1:raise ValueError('Selected pixel must lie inside exactly one detector box: '+str(len(selected)))
                box=selected[0]['box']
        with trace.stage('segmentation'):
            mask=detector.segment(inp.rgb,box);quality=mask_quality(mask,inp.depth)
            trace.record('segmentation',inp=inp,mask=mask,quality=quality)
            if not quality['accepted']:raise ValueError('Depth mask rejected: '+str(quality))
        del detector;torch.cuda.empty_cache()
        with trace.stage('pose_init'):
            backend=FoundationPoseBackend(ROOT/'third_party/FoundationPose',out/'opening_up.obj',debug_dir=out/'foundationpose')
        with trace.stage('pose'):
            camera_pose=backend.register(inp,mask,(obs.key[0],'central_crate'))
            if not np.isfinite(camera_pose).all():raise ValueError('Non-finite pose')
            write_json(out/'pose.json',serial(dict(camera_pose=camera_pose,frame_id=inp.frame,diagnostic_only=True,executable=False)))
        with trace.stage('pose_diagnostics'):
            from Utils import nvdiffrast_render
            _,render_depth,_=nvdiffrast_render(K=inp.k,H=inp.rgb.shape[0],W=inp.rgb.shape[1],ob_in_cams=torch.as_tensor(camera_pose[None],device='cuda',dtype=torch.float32),mesh=mesh,glctx=backend.est.glctx,use_light=False)
            depth=render_depth[0].detach().cpu().numpy();render_mask=depth>0
            overlap=mask & render_mask & (inp.depth>0)
            iou=float((mask & render_mask).sum()/max((mask|render_mask).sum(),1))
            residual=np.abs(depth[overlap]-inp.depth[overlap])
            metrics=dict(silhouette_iou=iou,overlap_valid_points=int(overlap.sum()),median_depth_error_m=float(np.median(residual)) if residual.size else None,p95_depth_error_m=float(np.quantile(residual,.95)) if residual.size else None,metric_depth_semantics_validated=False)
            write_json(out/'pose_quality.json',metrics)
            base=cv2.cvtColor(inp.rgb,cv2.COLOR_RGB2BGR);overlay=base.copy()
            overlay[render_mask]=(overlay[render_mask]*.75+np.array([230,160,20])*.25).astype(np.uint8)
            contours,_=cv2.findContours(render_mask.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(overlay,contours,-1,(255,180,0),2)
            cv2.putText(overlay,'CAD silhouette IoU %.3f (diagnostic)'%iou,(8,20),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
            trace.png('06_pose_overlay',overlay)
        with trace.stage('keypoints'):
            grasp,pre=rim_targets(camera_pose,**rules)
            image=base.copy()
            projected={key:project_points([value[:3,3]],np.eye(4),inp.k,image.shape)[0] for key,value in grasp.items()}
            ordered=sorted(projected,key=lambda key:projected[key]['pixel'][0] if projected[key]['pixel'] is not None else float('inf'))
            projection={label:dict(projected[key],rule_key=key) for label,key in zip(('A','B'),ordered)}
            for label,item in projection.items():
                if item['visible']:
                    xy=tuple(np.rint(item['pixel']).astype(int))
                    cv2.drawMarker(image,xy,(0,0,255),cv2.MARKER_CROSS,20,2)
                    cv2.circle(image,xy,9,(0,0,255),2)
                    cv2.putText(image,label,(xy[0]+12,xy[1]-12),cv2.FONT_HERSHEY_SIMPLEX,.75,(0,255,255),2)
            cv2.putText(image,'A / B: opposite rim candidates (not arm assignment)',(8,20),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
            cv2.putText(image,'CANDIDATE ONLY - uncalibrated TCP / depth',(8,image.shape[0]-12),cv2.FONT_HERSHEY_SIMPLEX,.48,(0,255,255),1)
            trace.png('07_grasp_candidates',image)
            write_json(out/'keypoints.json',serial(dict(frame_id=inp.frame,grasp=grasp,pregrasp=pre,projection=projection,diagnostic_only=True,executable=False,arm_assignment_validated=False,quality=metrics,rules=rules)))
            trace.png('08_comparison',np.hstack([overlay,image]))
        status='camera_preview_complete'
    finally:
        write_json(out/'status.json',dict(status=status,diagnostic_only=True,executable=False))
        trace.report(status)
        print('Report: '+str(out/'report.html'),flush=True)


if __name__=='__main__':main()
