# wheel_gateway

北向管理网关。service.py 汇总默认运行装配状态、配置文件名、固定目录任务结果。只读、不访问 .env、不启动任务，不将默认配置状态伪装为在线设备状态。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [resource/](resource/README.md) | 实现子目录；见目录内 README.md。 |
| [wheel_gateway/](wheel_gateway/README.md) | 实现子目录；见目录内 README.md。 |
| `package.xml` | ROS 包名、依赖与构建类型。 |
| `setup.cfg` | ament 可执行脚本安装位置。 |
| `setup.py` | Python/ament 安装、资源索引与 console script 定义。 |

## 构建边界

从仓库根安装核心库到 ROS 使用的解释器，再在 ros2_ws 中 colcon build。源码工具通过统一 bootstrap 可直接运行；硬件占位接口不会伪造成功。
