# wheel_description

机器人描述资源包。用户提供的模型已导入仓库根目录
`assets/robots/r680_lift_openarm_claw1.4/r680_lift_openarm_robot_viewer.urdf`，网格位于其同目录 `resources/`。
CMake 在模型存在时分别安装1.3和1.4到`share/wheel_description/urdf/`下各自同名目录，
旧模型保留，保持相对网格路径，不额外维护源码副本。
launch 仍要求显式传入模型路径。本次仅导入 CAD 模型，未完成 SDK 坐标映射或真机运动验证。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [meshes/](meshes/README.md) | 预留目录；本次网格随外部模型的 resources/ 一起安装。 |
| [urdf/](urdf/README.md) | 预留目录；导入模型在构建时从 assets 安装。launch 要求显式提供 URDF。 |
| `CMakeLists.txt` | ROS 构建、接口生成或资源安装规则。 |
| `package.xml` | ROS 包名、依赖与构建类型。 |

## 构建边界

从仓库根安装核心库到 ROS 使用的解释器，再在 ros2_ws 中 colcon build。源码工具通过统一 bootstrap 可直接运行；硬件占位接口不会伪造成功。
