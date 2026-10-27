#!/usr/bin/env python3
"""Run real FoundationPose on synthetic RGB-D; never a hardware accuracy claim."""
import json
import os
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools'))
from _bootstrap import configure
configure(ROOT)
sys.path.insert(0,str(ROOT/'third_party/FoundationPose'))


def main():
    os.chdir(ROOT)
    runtime=json.loads((ROOT/'.runtime/vision/native-build-env.json').read_text())
    for key,value in runtime.items():os.environ.setdefault(key,value)
    os.environ['PATH']=str(Path(sys.executable).parent)+':'+os.environ.get('PATH','')
    for key,subdir in [('TORCH_EXTENSIONS_DIR','torch-extensions'),('XDG_CACHE_HOME','cache'),('MPLCONFIGDIR','matplotlib'),('WARP_CACHE_PATH','warp-cache')]:
        os.environ.setdefault(key,str(ROOT/'.runtime/vision'/subdir))
        Path(os.environ[key]).mkdir(parents=True,exist_ok=True)
    import numpy as np
    import torch
    import trimesh
    import cv2
    import nvdiffrast.torch as dr
    from scipy.spatial.transform import Rotation
    from Utils import nvdiffrast_render
    from wheel_perception_core.geometry import ModelInput
    from wheel_perception_core.backends import FoundationPoseBackend
    from runlog import RunLog,write_json
    import uuid
    out=ROOT/'outputs/grasp/native-pose-smoke'/uuid.uuid4().hex[:8];out.mkdir(parents=True)
    log=RunLog(out,'pose_smoke')
    write_json(out/'status.json',dict(status='started',synthetic=True,real_box_accuracy_validated=False,executable=False))
    try:
        with log.stage('synthetic_fixture'):
            mesh=trimesh.creation.box(extents=[.3,.2,.15]);mesh.visual.vertex_colors=[80,150,210,255]
            mesh_path=out/'synthetic_box.obj';mesh.export(mesh_path)
            k=np.array([[260.,0,160],[0,260.,120],[0,0,1.]])
            truth=np.eye(4,dtype=np.float32);truth[:3,:3]=Rotation.from_euler('xyz',[150,10,20],degrees=True).as_matrix();truth[:3,3]=[.01,-.01,.8]
            ctx=dr.RasterizeCudaContext()
            rgb,depth,_=nvdiffrast_render(K=k,H=240,W=320,ob_in_cams=torch.as_tensor(truth[None],device='cuda'),mesh=mesh,glctx=ctx,use_light=True)
            rgb=(rgb[0].cpu().numpy()*255).astype(np.uint8);depth=depth[0].cpu().numpy()
            cv2.imwrite(str(out/'synthetic_rgb.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
            inp=ModelInput(rgb,depth,k,'synthetic_optical',True,['synthetic_fixture'])
        with log.stage('pose_init'):
            backend=FoundationPoseBackend(ROOT/'third_party/FoundationPose',mesh_path,debug_dir=out/'foundationpose')
        with log.stage('registration'):
            predicted=backend.register(inp,depth>0,('synthetic','box'))
            if not np.isfinite(predicted).all():raise ValueError('Non-finite registration')
        with log.stage('tracking'):
            tracked=backend.track(inp,('synthetic','box'))
            if not np.isfinite(tracked).all():raise ValueError('Non-finite tracking')
        write_json(out/'result.json',dict(synthetic=True,real_box_accuracy_validated=False,executable=False,
                   pose=np.asarray(predicted).tolist(),tracked_pose=np.asarray(tracked).tolist(),fixture_pose=truth.tolist(),
                   translation_error_m=float(np.linalg.norm(predicted[:3,3]-truth[:3,3]))))
        write_json(out/'status.json',dict(status='passed',synthetic=True,real_box_accuracy_validated=False,executable=False))
    except Exception:
        write_json(out/'status.json',dict(status='failed',synthetic=True,real_box_accuracy_validated=False,executable=False))
        raise
    finally:print('Artifacts: '+str(out),flush=True)


if __name__=='__main__':main()
