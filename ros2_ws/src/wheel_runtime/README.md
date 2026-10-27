# wheel_runtime

应用装配与命令入口。main.py 装配适配器，queries.py 提供状态/工序只读描述，cli.py 提供诊断和离线算法命令。业务状态机位于 wheel_task_core。未提供常驻任务调度服务。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [resource/](resource/README.md) | 实现子目录；见目录内 README.md。 |
| [wheel_runtime/](wheel_runtime/README.md) | 实现子目录；见目录内 README.md。 |
| `package.xml` | ROS 包名、依赖与构建类型。 |
| `setup.cfg` | ament 可执行脚本安装位置。 |
| `setup.py` | Python/ament 安装、资源索引与 console script 定义。 |

## 构建边界

从仓库根安装核心库到 ROS 使用的解释器，再在 ros2_ws 中 colcon build。源码工具通过统一 bootstrap 可直接运行；硬件占位接口不会伪造成功。
