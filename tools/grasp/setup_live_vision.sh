#!/usr/bin/env bash
# Isolated read-only detector/segmenter runtime for RTX 5090 (CUDA 12.8).
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$ROOT"
: "${CUDA_HOME:?Set CUDA_HOME to an existing CUDA toolkit for torch2trt setup}"
UV=${UV:-uv}
ENV_DIR="$ROOT/.runtime/vision/venv"
mkdir -p "$ROOT/.runtime/vision"
if [[ ! -x "$ENV_DIR/bin/python" ]]; then "$UV" venv --python 3.11 "$ENV_DIR"; fi
"$UV" pip install --python "$ENV_DIR/bin/python" torch==2.7.1 torchvision==0.22.1 \
  --index-url https://download.pytorch.org/whl/cu128
export NVIDIA_TENSORRT_DISABLE_INTERNAL_PIP=1
"$UV" pip install --python "$ENV_DIR/bin/python" \
  numpy==1.26.4 scipy==1.17.1 opencv-python-headless==4.11.0.86 \
  PyYAML==6.0.3 trimesh==5.1.1 transformers==4.35.2 onnx==1.17.0 \
  tensorrt-cu12==10.8.0.43 tensorrt-cu12-libs==10.8.0.43 tensorrt-cu12-bindings==10.8.0.43 \
  packaging setuptools wheel pillow patchelf==0.19.1.0 -e third_party/nanoowl -e third_party/nanosam
"$UV" pip install --python "$ENV_DIR/bin/python" --no-build-isolation -e third_party/torch2trt
"$ENV_DIR/bin/python" - <<'PYSRC'
import sysconfig
from pathlib import Path
(Path(sysconfig.get_paths()['purelib'])/'wheel_grasp_nanosam.pth').write_text(str(Path('third_party/nanosam').resolve())+'\n')
PYSRC
"$ENV_DIR/bin/python" tools/grasp/fix_trt_execstack.py
"$UV" pip freeze --python "$ENV_DIR/bin/python" > "$ROOT/.runtime/vision/installed.txt"
"$ENV_DIR/bin/python" -c 'import torch,tensorrt; print(torch.__version__,tensorrt.__version__,torch.cuda.get_device_name(0)); print(torch.ones(1,device="cuda"))'
