# wheel_bringup

ROS 启动与资源装配包。launch/description.launch.py 启动 robot_state_publisher 和 wheel_sdk_bridge 的实测关节接收。必须显式提供 urdf 与 lift_offset_m。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [launch/](launch/README.md) | 实现子目录；见目录内 README.md。 |
| `CMakeLists.txt` | ROS 构建、接口生成或资源安装规则。 |
| `package.xml` | ROS 包名、依赖与构建类型。 |

## 构建边界

从仓库根安装核心库到 ROS 使用的解释器，再在 ros2_ws 中 colcon build。源码工具通过统一 bootstrap 可直接运行；硬件占位接口不会伪造成功。
