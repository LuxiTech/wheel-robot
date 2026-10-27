#!/usr/bin/env python3
"""Read-only head RGB-D inference with durable stage diagnostics; never commands motion."""
import argparse
import datetime
import html
import importlib.metadata
import json
import logging
import os
import hashlib
from pathlib import Path
import platform
import subprocess
import sys
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools'))
from _bootstrap import configure
configure(ROOT)
from runlog import RunLog, write_json
import cv2
import numpy as np
from wheel_task_core.configuration import load_profile
from wheel_perception_core.rgbd import RGBDClient, save_observation, load_observation
from wheel_perception_core.visualization import draw_detections, draw_mask, draw_pose, draw_targets
from wheel_perception_core.pipeline import GraspPipeline


def serial(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, dict): return {k: serial(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)): return [serial(v) for v in value]
    return value


class Trace(RunLog):
    def __init__(self, directory):
        super().__init__(directory, 'vision')
        self.images = []

    def png(self, name, image):
        if not cv2.imwrite(str(self.directory/(name+'.png')), image):
            raise OSError('Cannot save diagnostic image: '+name)
        self.images.append(name+'.png')

    def record(self, name, **data):
        inp=data.pop('inp',None)
        image=cv2.cvtColor(inp.rgb,cv2.COLOR_RGB2BGR) if inp else None
        if name=='preprocess':
            self.png('01_rectified_rgb',image)
            valid=np.isfinite(inp.depth)&(inp.depth>0)
            lo,hi=np.percentile(inp.depth[valid],[2,98]) if valid.any() else (0.,1.)
            scaled=np.clip((np.nan_to_num(inp.depth)-lo)/max(hi-lo,1e-6)*255,0,255).astype(np.uint8)
            colored=cv2.applyColorMap(scaled,cv2.COLORMAP_TURBO);colored[~valid]=0
            self.png('02_depth',colored)
            data.update(K=inp.k,frame_id=inp.frame,diagnostic_only=inp.diagnostic_only,
                        reasons=inp.reasons,depth_display_range_m=[float(lo),float(hi)],valid_depth_fraction=float(valid.mean()))
        elif name=='detection':
            self.png('03_detections',draw_detections(image,data['candidates'],data['box']))
            data['selection']='explicit_roi' if data['box'] is not None else 'detector'
        elif name=='segmentation':
            mask=data.pop('mask');self.png('04_mask',mask.astype(np.uint8)*255)
            self.png('05_segmentation',draw_mask(image,mask))
        elif name=='pose':
            overlay=image
            # Show actual configured collider surfaces, preserving their centers.
            for collider in data['colliders']:
                local=np.eye(4);local[:3,3]=np.asarray(collider['position'])-np.array([0,0,collider['size'][2]/2])
                overlay=draw_pose(overlay,data['pose']@local,inp.k,collider['size'])
            self.png('06_pose_unvalidated',overlay)
        elif name=='keypoints':
            overlay,projection=draw_targets(image,data['grasp'],data['camera'],inp.k)
            self.png('07_grasp_tcp',overlay);data['projection']=projection
        write_json(self.directory/(name+'.json'),serial(data))

    def report(self, status):
        panels=''.join('<figure><figcaption>'+html.escape(name)+'</figcaption><img width="640" src="'+name+'"></figure>' for name in self.images)
        error=self.directory/'vision.error.json'
        details=error.read_text() if error.exists() else ''
        (self.directory/'report.html').write_text('<!doctype html><meta charset="utf-8"><style>body{font:16px sans-serif;margin:24px}img{max-width:100%;height:auto}pre{white-space:pre-wrap}figure{display:inline-block;margin:8px}</style><title>Vision diagnostics</title><h1>'+html.escape(status)+'</h1><p>Read-only diagnostic run. No motion commands. Pose overlays precede quality acceptance.</p><pre>'+html.escape(details)+'</pre>'+panels)


class LazySegmenter:
    def __init__(self, cfg, trace): self.cfg=cfg;self.trace=trace;self.backend=None
    def get(self):
        if self.backend is None:
            with self.trace.stage('detector_segmenter_init'):
                from wheel_perception_core.backends import NanoDetectorSegmenter
                m=self.cfg['model']
                self.backend=NanoDetectorSegmenter(m['owl_engine'],m['sam_encoder'],m['sam_decoder'],m['prompt'],threshold=m.get('threshold',.15))
        return self.backend
    def detect(self,rgb): return self.get().detect(rgb)
    def segment(self,rgb,box): return self.get().segment(rgb,box)


class LazyEstimator:
    def __init__(self,cfg,trace): self.cfg=cfg;self.trace=trace;self.backend=None
    def register(self,inp,mask,identity):
        with self.trace.stage('pose_init'):
            from wheel_perception_core.backends import FoundationPoseBackend
            m=self.cfg['model']
            if not self.cfg.get('object_geometry_validated',False):
                raise ValueError('Supply measured mesh/colliders/rim rules and set object_geometry_validated: true')
            self.backend=FoundationPoseBackend(m['foundationpose_repository'],m['mesh'],debug_dir=self.trace.directory/'foundationpose')
        return self.backend.register(inp,mask,identity)


class LazyRobot:
    def __init__(self,cfg): self.cfg=cfg
    def transform(self,frame,joints,base):
        from wheel_motion_core.kinematics import RobotModel
        if not self.cfg.get('camera_extrinsics_validated',False):
            raise ValueError('Supply real URDF/calibrated camera extrinsics and set camera_extrinsics_validated: true')
        return RobotModel(self.cfg['urdf']).transform(frame,joints,base)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default='config/robots/runtime.live.yaml')
    p.add_argument('--url',default='http://127.0.0.1:18080',help='SDK RGB-D gateway, normally through SSH tunnel')
    p.add_argument('--camera',default='head_rgb',choices=['head_rgb','left_wrist_rgb','right_wrist_rgb'])
    p.add_argument('--replay',help='Replay an observation NPZ, diagnostic only')
    p.add_argument('--capture-only',action='store_true')
    p.add_argument('--max-age-ms',type=float,default=250)
    p.add_argument('--target-id',default='crate')
    p.add_argument('--box',type=float,nargs=4)
    p.add_argument('--joints',help='JSON measured joint positions, required for moving camera chain')
    p.add_argument('--output-root',default='outputs/grasp/live')
    args=p.parse_args(argv)
    directory=Path(args.output_root).resolve()/(datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8])
    directory.mkdir(parents=True,exist_ok=False)
    trace=Trace(directory);status='failed'
    handler=logging.FileHandler(directory/'models.log')
    handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(name)s %(message)s'))
    old_level=logging.getLogger().level
    logging.getLogger().setLevel(logging.INFO)
    logging.getLogger().addHandler(handler)
    print('Artifacts: '+str(directory),flush=True)
    write_json(directory/'status.json',dict(status='running',executable=False))
    # Capture native TensorRT/CUDA diagnostics too, not only Python logging.
    sys.stdout.flush();sys.stderr.flush()
    saved_fds=[os.dup(1),os.dup(2)]
    console=(directory/'console.log').open('ab',buffering=0)
    os.dup2(console.fileno(),1);os.dup2(console.fileno(),2)
    try:
        with trace.stage('configuration'):
            native=ROOT/'.runtime/vision/native-build-env.json'
            if native.is_file():
                build_env=json.loads(native.read_text())
                for key in ('CUDA_HOME','CC','CXX','CPATH','TORCH_CUDA_ARCH_LIST','MAX_JOBS'):
                    if key in build_env:os.environ.setdefault(key,build_env[key])
                write_json(directory/'native-build-env.json',build_env)
            os.environ['PATH']=str(Path(sys.executable).parent)+':'+os.environ.get('PATH','')
            for key,subdir in [('TORCH_EXTENSIONS_DIR','torch-extensions'),('XDG_CACHE_HOME','cache'),('MPLCONFIGDIR','matplotlib'),('WARP_CACHE_PATH','warp-cache')]:
                os.environ.setdefault(key,str(ROOT/'.runtime/vision'/subdir))
                Path(os.environ[key]).mkdir(parents=True,exist_ok=True)
            cfg=load_profile(args.config)
            write_json(directory/'config.json',cfg)
            paths=[Path(__file__),ROOT/'packages/wheel_perception_core/wheel_perception_core/pipeline.py',
                   ROOT/'packages/wheel_perception_core/wheel_perception_core/backends.py',
                   ROOT/'packages/wheel_perception_core/wheel_perception_core/geometry.py']
            paths += [Path(v) for k,v in cfg['model'].items() if k in ('mesh','colliders','owl_engine','sam_encoder','sam_decoder') and v]
            hashes={}
            for path in paths:
                if path.is_file():
                    digest=hashlib.sha256()
                    with path.open('rb') as stream:
                        for chunk in iter(lambda:stream.read(1024*1024),b''):digest.update(chunk)
                    hashes[str(path.resolve())]=digest.hexdigest()
            write_json(directory/'sha256.json',hashes)
            versions={}
            for package in ('torch','torchvision','tensorrt-cu12','numpy','transformers','nanoowl','nanosam','pytorch3d','nvdiffrast','warp-lang','kornia'):
                try: versions[package]=importlib.metadata.version(package)
                except importlib.metadata.PackageNotFoundError: versions[package]=None
            sources={}
            for name in ('nanoowl','nanosam','FoundationPose','torch2trt','pytorch3d','nvdiffrast'):
                source=ROOT/'third_party'/name
                if (source/'.git').exists():
                    sources[name]=dict(commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=source,text=True).strip(),
                                       tracked_changes=subprocess.check_output(['git','status','--porcelain','--untracked-files=no'],cwd=source,text=True))
            patch=ROOT/'.runtime/vision/trt-execstack-patch.json'
            if patch.is_file():write_json(directory/'trt-execstack-patch.json',json.loads(patch.read_text()))
            write_json(directory/'run.json',dict(run_id=directory.name,source_revisions=sources,python=sys.executable,platform=platform.platform(),versions=versions,
                       git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                       git_status=subprocess.check_output(['git','status','--short'],cwd=ROOT,text=True),
                       camera=args.camera,target_id=args.target_id,replay=args.replay,box=args.box,max_age_ms=args.max_age_ms,executable=False))
        with trace.stage('capture'):
            def raw(headers,body):
                write_json(directory/'response.headers.json',headers)
                (directory/'response.multipart').write_bytes(body)
            obs=load_observation(args.replay) if args.replay else RGBDClient(args.url,max_age_ms=args.max_age_ms,on_response=raw).receive(args.camera)
            save_observation(directory/'observation.npz',obs)
            write_json(directory/'observation.json',obs.metadata)
            trace.png('00_rgb',cv2.cvtColor(obs.rgb,cv2.COLOR_RGB2BGR))
            trace.event('capture','observation',camera_id=obs.key[0],pair_id=obs.key[1])
        if args.capture_only:
            from wheel_perception_core.geometry import prepare_input
            with trace.stage('preprocess'): trace.record('preprocess',inp=prepare_input(obs))
            status='capture_only';return 0
        # Delay missing geometry errors until pose, so detection/segmentation artifacts survive.
        colliders_path=Path(cfg['model']['colliders'])
        colliders=json.loads(colliders_path.read_text()) if colliders_path.is_file() else []
        joints=json.loads(Path(args.joints).read_text()) if args.joints else {}
        write_json(directory/'joints.json',joints)
        pipeline=GraspPipeline(LazyRobot(cfg),LazySegmenter(cfg,trace),LazyEstimator(cfg,trace),cfg['rules'],colliders,base=cfg['base_frame'],trace=trace)
        result=pipeline.process(obs,joints,target_id=args.target_id,box=args.box)
        result.pop('mask');result['diagnostic_only']=True
        result['reasons']=list(result['reasons'])+['read_only_commissioning']
        write_json(directory/'result.json',serial(result));status='diagnostic_complete';return 0
    except Exception as exc:
        if not (directory/'vision.error.json').exists():
            import traceback
            write_json(directory/'vision.error.json',dict(stage='runner',error_type=type(exc).__name__,error=str(exc),traceback=traceback.format_exc()))
            trace.event('runner','failed',error=str(exc))
        return 1
    finally:
        try:
            write_json(directory/'status.json',dict(status=status,executable=False))
            trace.report(status)
        finally:
            logging.getLogger().removeHandler(handler)
            logging.getLogger().setLevel(old_level)
            handler.close()
            sys.stdout.flush();sys.stderr.flush()
            for fd,saved in zip((1,2),saved_fds):
                os.dup2(saved,fd);os.close(saved)
            console.close()
        print('Report: '+str(directory/'report.html'),flush=True)


if __name__=='__main__': sys.exit(main())
