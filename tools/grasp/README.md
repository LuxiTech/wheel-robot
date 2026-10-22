# 感知与 SDK 隔离环境

真机开发使用 SDK/ROS 与 GPU 推理两个独立镜像；轻量测试使用根目录 `.venv`。当前 Docker Compose 不再包含 Isaac 服务。

先准备 Docker Engine、Compose v2 和 NVIDIA Container Toolkit。若还没有 `.env`，从 `.env.example` 复制并填写当前 UID/GID 和 `PERCEPTION_GPU_ID`；单卡为 `0`。已有 `.env` 保留，原 Isaac 变量不再被当前 Compose 使用。

```bash
mkdir -p outputs/grasp assets/models/engines .runtime/grasp .cache/grasp-wheels
tools/grasp/environment sources
tools/grasp/environment build-sdk
tools/grasp/environment download
tools/grasp/environment verify-models
tools/grasp/environment build-perception
tools/grasp/environment check
tools/grasp/environment engines
tools/grasp/environment smoke
```

按顺序检查成功后继续。源码、环境和权重分别由 `config/grasp/*lock.json` 固定；既有模型环境验证基线见 `validated-environment.json`，不表示任意显卡都已通过。更换 GPU/运行库后须重新校验并构建 TensorRT 引擎，已有不匹配文件需先归档。

`check` 和 `smoke` 需要真正的 GPU 推理环境，普通 `.venv` 中的逻辑测试不能替代它们。本次目录迁移后镜像需重新构建，以包含 `wheel_grasp.vision` 等子包。

只读采集和离线分析使用 `tools/grasp/grasp`，容器中使用 `wheel-grasp`。根据环境选择 `config/grasp/runtime.yaml` 或 `runtime.container.yaml`，配置真实 URDF、已测量的箱体网格/碰撞模型和模型引擎。输出目录和文件名由调用者明确提供，已有采集/规划文件不会覆盖。

```bash
tools/grasp/grasp doctor
docker compose -p wheel-robot-grasp -f compose.grasp.yaml run --rm sdk_bridge \
  wheel-grasp --config config/grasp/runtime.container.yaml doctor
```

SDK 容器默认 `ROS_LOCALHOST_ONLY=1`，尚未实现跨机器 DDS 部署。传感器源与执行器接入需配置实际网络和时钟语义；不能把“镜像能构建”当成“机器人能执行”。

`environment run` 和 `relay` 原先启动仿真，现已移除。统一任务入口见根目录 `tools/robot`。如需查阅原仿真代码和说明，可在 Git 提交 `6726ba1` 中查看；恢复方式见 `docs/MIGRATION.md`。
