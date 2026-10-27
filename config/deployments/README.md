# deployments

task.yaml 定义单箱任务与超时；各 lock.json 固定模型、源码和环境版本；validated-environment.json 为历史验证基线。锁文件迁移不改变权重与版本。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| `dependencies.lock.json` | 本目录配置、资源或工具文件。 |
| `environment.lock.json` | 本目录配置、资源或工具文件。 |
| `models.lock.json` | 本目录配置、资源或工具文件。 |
| `native-pose.sources.lock.json` | 本目录配置、资源或工具文件。 |
| `task.yaml` | 本目录配置、资源或工具文件。 |
| `validated-environment.json` | 本目录配置、资源或工具文件。 |

| `devices.yaml` | 两个 SDK 地址、unstacking 地图和导航超时；不存 SSH 密码。 |
