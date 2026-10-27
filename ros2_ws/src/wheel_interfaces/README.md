# wheel_interfaces

唯一 ROS 消息契约包，从 wheel_grasp_interfaces 迁移。msg 包含箱体位姿和双手候选，action 包含 ExecuteGrasp。CMake 生成类型，包内没有动作服务器。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [action/](action/README.md) | 实现子目录；见目录内 README.md。 |
| [msg/](msg/README.md) | 实现子目录；见目录内 README.md。 |
| `CMakeLists.txt` | ROS 构建、接口生成或资源安装规则。 |
| `package.xml` | ROS 包名、依赖与构建类型。 |

## 构建边界

从仓库根安装核心库到 ROS 使用的解释器，再在 ros2_ws 中 colcon build。源码工具通过统一 bootstrap 可直接运行；硬件占位接口不会伪造成功。
