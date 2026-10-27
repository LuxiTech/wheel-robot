# wheel_planning

运动规划能力适配。service.py 的 OfflinePlanningService 实际调用 wheel_motion_core.plan_pair，接收实测状态和碰撞回调，保持计划不可执行。尚无跨进程 action server。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [resource/](resource/README.md) | 实现子目录；见目录内 README.md。 |
| [wheel_planning/](wheel_planning/README.md) | 实现子目录；见目录内 README.md。 |
| `package.xml` | ROS 包名、依赖与构建类型。 |
| `setup.cfg` | ament 可执行脚本安装位置。 |
| `setup.py` | Python/ament 安装、资源索引与 console script 定义。 |

## 构建边界

从仓库根安装核心库到 ROS 使用的解释器，再在 ros2_ws 中 colcon build。源码工具通过统一 bootstrap 可直接运行；硬件占位接口不会伪造成功。
