# wheel_sdk_bridge

厂商协议的唯一适配边界。sdk.py 处理升降映射、关节反馈和租期；feedback.py 对齐实测状态；ros_state/rgbd_ros 是只读 ROS 入口。厂商 SDK 本体只读放在 device/，不导入或修改其源码。ROS main 需要板端 luxi_interfaces。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [resource/](resource/README.md) | 实现子目录；见目录内 README.md。 |
| [wheel_sdk_bridge/](wheel_sdk_bridge/README.md) | 实现子目录；见目录内 README.md。 |
| `wheel_sdk_bridge/http_client.py` | 有界 JSON HTTP、禁用代理与重定向、不重试写请求。 |
| `wheel_sdk_bridge/lift_client.py` | 只读 /api/state 及底盘新鲜度/静止判断。 |
| `package.xml` | ROS 包名、依赖与构建类型。 |
| `setup.cfg` | ament 可执行脚本安装位置。 |
| `setup.py` | Python/ament 安装、资源索引与 console script 定义。 |

## 构建边界

从仓库根安装核心库到 ROS 使用的解释器，再在 ros2_ws 中 colcon build。源码工具通过统一 bootstrap 可直接运行；硬件占位接口不会伪造成功。
