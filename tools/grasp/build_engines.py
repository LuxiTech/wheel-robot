#!/usr/bin/env python3
"""Build local GPU-specific TensorRT engines; never download foreign engines."""
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import tensorrt as trt
import torch

ROOT = Path(__file__).resolve().parents[2]
DEST = ROOT/'assets/models/engines'


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def build(source, destination, *, fp16=False, shapes=None):
    expected = dict(gpu=torch.cuda.get_device_name(0), compute_capability=list(torch.cuda.get_device_capability(0)),
                    tensorrt=trt.__version__, torch=torch.__version__, cuda=torch.version.cuda,
                    architecture=platform.machine(), source_sha256=sha(source), fp16=fp16,
                    shapes=json.loads(json.dumps(shapes or {})))
    if destination.exists():
        metadata_path = destination.with_suffix('.engine.json')
        previous = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
        if all(previous.get(k) == v for k, v in expected.items()) and previous.get('engine_sha256') == sha(destination):
            print('verified existing engine', destination, flush=True)
            return
        raise FileExistsError(f'Archive existing engine before rebuilding: {destination}')
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, logger)
    if not parser.parse(source.read_bytes()):
        raise RuntimeError('\n'.join(str(parser.get_error(i)) for i in range(parser.num_errors)))
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 2 << 30)
    if fp16:
        config.set_flag(trt.BuilderFlag.FP16)
    profile = builder.create_optimization_profile()
    dynamic = False
    for i in range(network.num_inputs):
        tensor = network.get_input(i)
        if -1 in tuple(tensor.shape):
            dynamic = True
            if tensor.name not in (shapes or {}):
                raise ValueError(f'No shape profile for {tensor.name}: {tensor.shape}')
            profile.set_shape(tensor.name, *shapes[tensor.name])
    if dynamic:
        config.add_optimization_profile(profile)
    engine = builder.build_serialized_network(network, config)
    if engine is None:
        raise RuntimeError(f'TensorRT failed to build {source}')
    temporary = destination.with_suffix('.partial')
    temporary.write_bytes(bytes(engine))
    temporary.replace(destination)
    metadata = dict(expected, engine_sha256=sha(destination))
    destination.with_suffix('.engine.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print('built', destination, flush=True)


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--model', choices=['sam', 'owl', 'all'], default='all')
    args = p.parse_args()
    subprocess.run([sys.executable, 'tools/grasp/download_models.py', '--verify-only'], check=True)
    DEST.mkdir(parents=True, exist_ok=True)
    if args.model in ('sam', 'all'):
        build(ROOT/'assets/models/nanosam/resnet18_image_encoder.onnx', DEST/'resnet18_image_encoder.engine', fp16=True)
        build(ROOT/'assets/models/nanosam/mobile_sam_mask_decoder.onnx', DEST/'mobile_sam_mask_decoder.engine',
              shapes={'point_coords': [(1,1,2),(1,2,2),(1,10,2)], 'point_labels': [(1,1),(1,2),(1,10)]})
    if args.model in ('owl', 'all'):
        from nanoowl.owl_predictor import OwlPredictor
        onnx = DEST/'owl_image_encoder.onnx'
        if not onnx.exists():
            predictor = OwlPredictor('google/owlvit-base-patch32')
            predictor.export_image_encoder_onnx(str(onnx), onnx_opset=17)
            del predictor
            torch.cuda.empty_cache()
        build(onnx, DEST/'owl_image_encoder.engine', fp16=True,
              shapes={'image': [(1,3,768,768)]*3})


if __name__ == '__main__':
    main()
