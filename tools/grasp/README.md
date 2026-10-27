# 感知与 ROS 工具环境

开发机使用 ROS 工具与 GPU 推理两个独立镜像；厂商 SDK 仅在开发板运行；轻量测试使用根目录 `.venv`。当前 Docker Compose 不再包含 Isaac 服务。

## 不依赖 Docker 的权重下载与校验

模型文件下载仅需网络和 Python，不需要 GPU、ROS 或 Docker。所有命令在仓库根目录执行。使用独立虚拟环境安装 `gdown`，避免改变算法测试环境：

```bash
mkdir -p outputs/grasp .runtime
uv venv --python /usr/bin/python3 .runtime/model-download-venv
UV_CACHE_DIR="$PWD/.cache/uv" uv pip install \
  --python .runtime/model-download-venv/bin/python gdown==5.2.0

# 恢复固定提交的模型源码，不下载或构建厂商 SDK
tools/grasp/environment sources

PYTHONDONTWRITEBYTECODE=1 .runtime/model-download-venv/bin/python -u \
  tools/grasp/download_models.py > outputs/grasp/model-download.log 2>&1
PYTHONDONTWRITEBYTECODE=1 .runtime/model-download-venv/bin/python \
  tools/grasp/download_models.py --verify-only > outputs/grasp/model-verify.log 2>&1
```

已有下载虚拟环境时跳过创建步骤。没有 uv 时，可用 `python3 -m venv .runtime/model-download-venv` 创建环境，再用该环境的 `python -m pip install gdown==5.2.0` 安装下载依赖。

下载器按 `config/deployments/models.lock.json` 校验每个文件，已有正确文件会复用；中断文件保留为 `.partial`。若发现校验不匹配，程序报错并保留文件用于排查，不会跳过校验。最后生成固定 revision 的 Hugging Face 离线缓存。

| 内容 | 仓库根目录下的相对路径 |
|---|---|
| NanoSAM 编码器/解码器 | `assets/models/nanosam/resnet18_image_encoder.onnx`、`mobile_sam_mask_decoder.onnx` |
| FoundationPose 两组权重 | `assets/models/FoundationPose/2023-10-28-18-33-37/model_best.pth`、`assets/models/FoundationPose/2024-01-11-20-02-45/model_best.pth`；各有对应 `config.yml` |
| OWL-ViT | `assets/models/owlvit-base-patch32/model.safetensors` 及配套配置/分词文件 |
| 离线 Hugging Face 缓存 | `assets/models/huggingface/hub/models--google--owlvit-base-patch32/` |
| 本机生成的 TensorRT 引擎 | `assets/models/engines/`；下载器不会生成引擎 |

2026-10-22 本机已下载并校验全部 14 个锁定文件及离线缓存，合计约 1.5 GiB。此次校验不需要加载 PyTorch 模型，也未运行推理。新机器克隆仓库后需要重新下载或复制这些资源，然后执行 `--verify-only`。

## 容器构建与推理验证（本机尚未完成）

先准备 Docker Engine、Compose v2 和 NVIDIA Container Toolkit。若还没有 `.env`，从 `.env.example` 复制并填写当前 UID/GID 和 `PERCEPTION_GPU_ID`；单卡为 `0`。已有 `.env` 保留，原 Isaac 变量不再被当前 Compose 使用。

```bash
mkdir -p outputs/grasp assets/models/engines .runtime/grasp .cache/grasp-wheels
tools/grasp/environment sources
tools/grasp/environment build-ros
tools/grasp/environment download
tools/grasp/environment verify-models
tools/grasp/environment build-perception
tools/grasp/environment check
tools/grasp/environment engines
tools/grasp/environment smoke
```

按顺序检查成功后继续。源码、环境和权重分别由 `config/deployments/*lock.json` 固定；既有模型环境验证基线见 `validated-environment.json`（历史 SDK 镜像记录不代表当前 ROS 工具镜像已经验证），不表示任意显卡都已通过。更换 GPU/运行库后须重新校验并构建 TensorRT 引擎，已有不匹配文件需先归档。

已通过独立下载路径准备好的文件会被容器中的下载器复用。当前开发机为 RTX 5090 Laptop GPU，历史推理基线不能替代这台机器上的兼容性测试；应以 `check`、`engines`、`smoke` 的实际结果确认能否运行。

`check` 和 `smoke` 需要真正的 GPU 推理环境，普通 `.venv` 中的逻辑测试不能替代它们。本次目录迁移后镜像需重新构建，以包含 `wheel_perception_core` 等子包。

只读采集和离线分析使用 `tools/grasp/grasp`，容器中使用 `wheel-grasp`。根据环境选择 `config/robots/runtime.yaml` 或 `config/robots/runtime.container.yaml`，配置真实 URDF、已测量的箱体网格/碰撞模型和模型引擎。输出目录和文件名由调用者明确提供，已有采集/规划文件不会覆盖。

```bash
tools/grasp/grasp doctor
docker compose -p wheel-robot-grasp -f deployment/compose.grasp.yaml run --rm ros_tools \
  wheel-grasp --config config/robots/runtime.container.yaml doctor
```

ROS 工具容器不含厂商 SDK，支持模型下载、HTTP 只读采集和离线工具，默认 `ROS_LOCALHOST_ONLY=1`。`wheel-grasp-state` 和 `wheel-grasp-rgbd` 需要开发板 SDK 提供的 `luxi_interfaces`，应在板端 source SDK 环境后运行。计划通过 SSH 管理板端调用程序，执行链路尚未实现。

`environment run` 和 `relay` 原先启动仿真，现已移除。统一任务入口见根目录 `tools/robot`。如需查阅原仿真代码和说明，可在 Git 提交 `6726ba1` 中查看；恢复方式见 `docs/MIGRATION.md`。


## 0.3.0 源码与配置边界

本目录仍是模型准备和诊断工具；核心算法已迁入 packages/，运行 CLI 属于 wheel_runtime。直接运行 Python 工具通过 tools/_bootstrap.py 发现源码。机器人 YAML 引用 recipes 中的共享规则，模型锁位于 config/deployments。历史验证记录不代表新容器已重建。


## 本目录文件结构

| 文件 | 功能 |
| --- | --- |
| `bootstrap_sources.py` | Restore locked sources without replacing locally modified checkouts. |
| `build_crate_mesh.py` | Generate a known crate mesh from existing local collider assets, in metres. |
| `build_engines.py` | Build local GPU-specific TensorRT engines; never download foreign engines. |
| `check_environment.py` | GPU/import/weights diagnostics. This does not send robot commands. |
| `container-entrypoint.sh` | 依赖声明、环境或辅助工具。 |
| `diagnose_robot.sh` | 依赖声明、环境或辅助工具。 |
| `download_models.py` | Download explicitly locked model files and verify their SHA-256 before use. |
| `environment` | Docker 环境准备、下载、校验和诊断入口。 |
| `fix_trt_execstack.py` | Clear TRT 10.8's executable-stack flag in this isolated venv only (glibc >=2.41). |
| `grasp` | 离线算法/只读采集 CLI 入口。 |
| `import_step_mesh.py` | Convert a millimetre STEP to a metre OBJ, centered in XY with bottom Z=0. |
| `perceive` | 原生 GPU 视觉诊断封装入口。 |
| `perceive_live.py` | Read-only head RGB-D inference with durable stage diagnostics; never commands motion. |
| `preview_camera_grasps.py` | Diagnostic CAD pose and rim TCP projections in camera coordinates; no robot adapter. |
| `requirements-native-pose.txt` | 依赖声明、环境或辅助工具。 |
| `requirements-perception.txt` | 依赖声明、环境或辅助工具。 |
| `requirements.txt` | 依赖声明、环境或辅助工具。 |
| `runlog.py` | Durable per-run events and atomic artifacts; standard library only. |
| `setup_live_vision.sh` | 依赖声明、环境或辅助工具。 |
| `setup_native_pose.py` | Build pinned pose extensions inside the native vision venv, without sudo. |
| `smoke_models.py` | Actual learned-model smoke tests, not a real-box accuracy benchmark. |
| `smoke_native_pose.py` | Run real FoundationPose on synthetic RGB-D; never a hardware accuracy claim. |
