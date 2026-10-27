# docker

Dockerfile.ros-tools 构建 Humble 工具与适配包；Dockerfile.grasp-perception 构建历史 GPU 推理环境。对应 dockerignore 限制构建上下文。当前本机原生 GPU 环境与这些历史镜像分离，迁移后镜像需重新构建验收。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| `Dockerfile.grasp-perception` | 本目录配置、资源或工具文件。 |
| `Dockerfile.grasp-perception.dockerignore` | 本目录配置、资源或工具文件。 |
| `Dockerfile.ros-tools` | 本目录配置、资源或工具文件。 |
| `Dockerfile.ros-tools.dockerignore` | 本目录配置、资源或工具文件。 |
