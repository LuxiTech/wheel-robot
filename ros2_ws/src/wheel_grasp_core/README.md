# wheel_grasp_core

保持原 ROS/Python 包名，内部已按 `navigation`、`vision`、`motion`、`task` 重构。模块说明见项目根目录 [README](../../../README.md) 和 [架构说明](../../../docs/ARCHITECTURE.md)。

入口：

- `wheel-robot` / `tools/robot`：任务应用，`status`、`workflow`、`run`。
- `wheel-grasp` / `tools/grasp/grasp`：只读采集、检查、FK、感知、离线 IK。
- `wheel-grasp-rgbd`：`wheel_grasp.vision.ros` 的 ROS RGB-D 录制节点。
- `wheel-grasp-state`：`wheel_grasp.motion.ros_state` 的实测关节发布节点。

使用 Ubuntu 22.04 / ROS 2 Humble 的 Python 环境构建，同时提供锁定 SDK 的 `luxi_interfaces`。仓库的 SDK Dockerfile 会将 SDK 消息及本工作区一起构建。不要 source 宿主 `.venv` 来替换 Humble 的 Python。

```bash
source /opt/ros/humble/setup.bash
# source 与 SDK 固定版本匹配的 luxi_interfaces overlay
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`description.launch.py` 仍要求显式提供 URDF 和实测 `lift_offset_m`。它只发布 TF/状态。`ExecuteGrasp.action` 仍是接口契约，尚无硬件 action server；`SDKMotionExecutor` 也尚未连接自主执行入口。
