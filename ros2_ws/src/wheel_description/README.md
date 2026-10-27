# wheel_description

机器人描述资源包。urdf/meshes 用于已验证机器人资源；当前真实 URDF 缺失，目录只保留说明，不生成仿真替代模型。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [meshes/](meshes/README.md) | 放置 URDF 引用的机器人视觉/碰撞网格。当前未提供资产。单位与缩放必须与 URDF 一致；大资产按部署流程提供。 |
| [urdf/](urdf/README.md) | 放置真实机器人 URDF/Xacro，并明确相机光学帧、升降、双臂、TCP 及关节限位。当前没有已验证文件；launch 要求调用者显式提供 URDF。 |
| `CMakeLists.txt` | ROS 构建、接口生成或资源安装规则。 |
| `package.xml` | ROS 包名、依赖与构建类型。 |

## 构建边界

从仓库根安装核心库到 ROS 使用的解释器，再在 ros2_ws 中 colcon build。源码工具通过统一 bootstrap 可直接运行；硬件占位接口不会伪造成功。
