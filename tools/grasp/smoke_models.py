#!/usr/bin/env python3
"""Actual learned-model smoke tests, not a real-box accuracy benchmark."""
import json
import time
from pathlib import Path
import numpy as np
from PIL import Image
import torch


def timed(fn):
    torch.cuda.synchronize()
    start = time.perf_counter()
    result = fn()
    torch.cuda.synchronize()
    return result, time.perf_counter()-start


def main():
    from wheel_perception_core.backends import NanoDetectorSegmenter, FoundationPoseBackend
    from wheel_perception_core.geometry import ModelInput
    from Utils import nvdiffrast_render
    import trimesh
    import nvdiffrast.torch as dr
    out = Path('outputs/grasp/smoke');out.mkdir(parents=True,exist_ok=True)
    engines = Path('assets/models/engines')
    detector = NanoDetectorSegmenter(str(engines/'owl_image_encoder.engine'),
        str(engines/'resnet18_image_encoder.engine'),str(engines/'mobile_sam_mask_decoder.engine'), prompt='an owl')
    image = np.asarray(Image.open('/opt/nanoowl/assets/owl_glove_small.jpg').convert('RGB'))
    detections, seconds = timed(lambda: detector.detect(image))
    if not detections:
        raise RuntimeError('OWL sample produced no detections')
    best = max(detections,key=lambda x:x['score'])
    mask, sam_seconds = timed(lambda: detector.segment(image, best['box']))
    if mask.shape != image.shape[:2] or not 0 < mask.sum() < mask.size:
        raise RuntimeError('Invalid sample segmentation')
    Image.fromarray(mask.astype(np.uint8)*255).save(out/'owl_mask.png')
    del detector
    torch.cuda.empty_cache()
    # Known geometry makes a repeatable RGB-D fixture, not a sensor observation.
    mesh_path = 'assets/generated/grasp/crate.obj'
    mesh = trimesh.load(mesh_path,force='mesh')
    mesh.visual.vertex_colors = [80,150,210,255]
    k = np.array([[400.,0,160],[0,400.,120],[0,0,1.]])
    truth = np.eye(4,dtype=np.float32)
    from scipy.spatial.transform import Rotation
    truth[:3,:3] = Rotation.from_euler('xyz',[150,10,20],degrees=True).as_matrix()
    truth[:3,3] = [.01,-.01,1.0]
    ctx = dr.RasterizeCudaContext()
    rgb, depth, _ = nvdiffrast_render(K=k,H=240,W=320,
        ob_in_cams=torch.as_tensor(truth[None],device='cuda'),mesh=mesh,glctx=ctx,use_light=True)
    rgb = (rgb[0].cpu().numpy()*255).astype(np.uint8)
    depth = depth[0].cpu().numpy()
    Image.fromarray(rgb).save(out/'pose_fixture.png')
    inp = ModelInput(rgb,depth,k,'fixture_optical',True,['synthetic_fixture'])
    backend = FoundationPoseBackend('/opt/FoundationPose',mesh_path,debug_dir=str(out/'foundationpose'))
    pose, fp_seconds = timed(lambda: backend.register(inp,depth>0,('fixture','crate')))
    tracked, track_seconds = timed(lambda: backend.track(inp,('fixture','crate')))
    if not np.isfinite(pose).all() or not np.isfinite(tracked).all():
        raise RuntimeError('Non-finite pose')
    report = dict(passed=True,kind='real_model_inference_on_upstream_image_and_synthetic_rgbd',
        real_box_accuracy_validated=False,owl_detection_count=len(detections),mask_pixels=int(mask.sum()),
        cold_inference_seconds=dict(owl=seconds,sam=sam_seconds,registration=fp_seconds,tracking=track_seconds),
        estimated_pose=np.asarray(pose).tolist(),fixture_pose=truth.tolist(),
        fixture_translation_error_m=float(np.linalg.norm(pose[:3,3]-truth[:3,3])))
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
