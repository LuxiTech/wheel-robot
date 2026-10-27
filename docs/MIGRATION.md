# 0.3.0 拆包迁移

本次在原有硬件开发工程上执行源码迁移，不修改厂商 SDK 本体、不复制模型权重、不启用真机运动。
旧单体包和旧 import 已移除，工具与测试统一使用新入口，没有并行维护旧实现。

| 旧位置 | 新位置 |
| --- | --- |
| wheel_grasp/contracts.py、task/ | packages/wheel_task_core/wheel_task_core/ |
| wheel_grasp/vision/ 中算法 | packages/wheel_perception_core/wheel_perception_core/ |
| motion/kinematics.py、planning.py、collision.py | packages/wheel_motion_core/wheel_motion_core/ |
| motion/sdk.py、feedback.py、ros_state.py、vision/ros.py | ros2_ws/src/wheel_sdk_bridge/wheel_sdk_bridge/ |
| vision/service.py | ros2_ws/src/wheel_perception/wheel_perception/service.py |
| navigation/adapter.py | ros2_ws/src/wheel_navigation/wheel_navigation/adapter.py |
| motion/executor.py | ros2_ws/src/wheel_execution/wheel_execution/executor.py |
| main.py、cli.py | ros2_ws/src/wheel_runtime/wheel_runtime/ |
| wheel_grasp_interfaces | ros2_ws/src/wheel_interfaces |
| wheel_grasp_core/launch | ros2_ws/src/wheel_bringup/launch |
| config/grasp/runtime*.yaml | config/robots/runtime*.yaml |
| runtime*.yaml 中共享 rules/model 资产引用 | config/recipes/crate.example.yaml |
| config/task.yaml | config/deployments/task.yaml |
| config/navigation | config/stations |
| config/grasp/*lock.json、验证环境 | config/deployments/ |
| config/grasp/crate-cad.json | config/recipes/crate-cad.json |
| docker/、compose.grasp.yaml | deployment/docker/、deployment/compose.grasp.yaml |

## 使用者调整

`tools/robot`、`tools/grasp/grasp`、`tools/grasp/perceive` 的命令名称保留，内部源码发现机制已更新。
console script `wheel-robot`/`wheel-grasp` 属于 wheel_runtime，`wheel-grasp-state`/`wheel-grasp-rgbd` 属于 wheel_sdk_bridge。
自有脚本应更新为 wheel_task_core / wheel_perception_core / wheel_motion_core / 对应适配包的 import。
旧虚拟环境里 `wheel_grasp_core` 的 editable 元数据可能残留；源码工具不依赖该元数据。需要 console script 时重新安装对应新包。
本次已将开发机 `.venv` 的旧安装卸载并安装 10 个新包；旧源码目录中仅剩的字节码和 egg-info 已归档到 `.runtime/refactor/legacy-wheel_grasp_core`。

Docker Compose 从仓库根运行：`docker compose --env-file .env -f deployment/compose.grasp.yaml config`。
构建上下文和挂载源目录相对于 Compose 文件修正；核心库先安装，再构建 ROS 适配包。
历史输出中的原路径/commit 作为证据保留，不回写。

## 未改变的资产和能力限制

.env、.venv、.runtime、outputs、third_party 以及被忽略的 assets 文件继续原地保留。
真实 URDF 仍未提供；wheel_description 的 urdf/meshes 是明确的资源边界，不放入假机器人模型。
导航、SDK 运动执行和同步采集源保持未就绪。网页仅提供只读管理，完整调度/权限/参数发布等待后续实现。
