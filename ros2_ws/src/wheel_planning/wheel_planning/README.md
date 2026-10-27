# wheel_planning

运动规划能力适配。service.py 的 OfflinePlanningService 实际调用 wheel_motion_core.plan_pair，接收实测状态和碰撞回调，保持计划不可执行。尚无跨进程 action server。 本目录只放 Python 实现；安装和 ROS 元数据位于上级。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| `__init__.py` | wheel_planning: runtime adapter boundary. |
| `service.py` | Offline planning capability; never converts a plan into execution authority. |
