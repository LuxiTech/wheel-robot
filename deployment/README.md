# deployment

部署边界。compose.grasp.yaml 为历史 ROS 工具/GPU 推理服务，docker/ 保存其镜像定义。核心库先安装再构建 ROS 包；不在此存放机器人业务逻辑。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [docker/](docker/README.md) | Dockerfile.ros-tools 构建 Humble 工具与适配包；Dockerfile.grasp-perception 构建历史 GPU 推理环境。对应 dockerignore 限制构建上下文。当前本机原生 GPU 环境与这些历史镜像分离，迁移后镜像需重新构建验收。 |
| `compose.grasp.yaml` | 本目录配置、资源或工具文件。 |
