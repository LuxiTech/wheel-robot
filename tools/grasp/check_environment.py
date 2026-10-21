#!/usr/bin/env python3
"""GPU/import/weights diagnostics. This does not send robot commands."""
import importlib
import json
import sys
from pathlib import Path


def main():
    result = {'python': sys.version, 'checks': {}, 'hardware_commands_sent': False}
    for name in ['torch','torchvision','tensorrt','torch2trt','nanoowl','nanosam.utils.predictor','nvdiffrast.torch','pytorch3d','estimater']:
        try:
            module = importlib.import_module(name)
            result['checks'][name] = {'ok': True, 'version': getattr(module, '__version__', None)}
        except Exception as exc:
            result['checks'][name] = {'ok': False, 'error': str(exc)}
    try:
        import torch
        a = torch.eye(3, device='cuda')
        assert torch.allclose(a@a, a)
        result['gpu'] = torch.cuda.get_device_name(0)
        result['visible_gpu_count'] = torch.cuda.device_count()
        result['cuda_tensor_test'] = True
    except Exception as exc:
        result['cuda_tensor_test'] = False
        result['cuda_error'] = str(exc)
    result['passed'] = result['cuda_tensor_test'] and all(c['ok'] for c in result['checks'].values())
    text = json.dumps(result, indent=2)
    print(text)
    out = Path('/workspace/outputs/grasp')
    out.mkdir(parents=True, exist_ok=True)
    (out/'perception_environment.json').write_text(text+'\n')
    sys.exit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
