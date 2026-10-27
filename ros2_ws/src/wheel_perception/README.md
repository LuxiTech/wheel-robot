# wheel_perception

任务侧视觉适配。service.py 定义 SynchronizedCapture/CaptureSource，将核心 GraspPipeline 转为 GraspObservation。推理工作线程串行化，默认同步源未接入。不包含已部署的 ROS action server。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [resource/](resource/README.md) | 实现子目录；见目录内 README.md。 |
| [wheel_perception/](wheel_perception/README.md) | 实现子目录；见目录内 README.md。 |
| `package.xml` | ROS 包名、依赖与构建类型。 |
| `setup.cfg` | ament 可执行脚本安装位置。 |
| `setup.py` | Python/ament 安装、资源索引与 console script 定义。 |

## 构建边界

从仓库根安装核心库到 ROS 使用的解释器，再在 ros2_ws 中 colcon build。源码工具通过统一 bootstrap 可直接运行；硬件占位接口不会伪造成功。
