# wheel_motion_core

运动算法核心库。kinematics 提供 URDF FK 和 15 自由度联合 IK；collision 为保守 OBB；planning 生成离线轨迹。计划保持不可执行，不直接访问 SDK。 本目录只放 Python 实现；安装和 ROS 元数据位于上级。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| `__init__.py` | Kinematics, collision checks and offline trajectory planning. |
| `collision.py` | Conservative URDF collision OBBs; no claim of mesh-exact collision checking. |
| `kinematics.py` | URDF-only installation transforms, strict live joint requirements and pair IK. |
| `planning.py` | Continuous pair trajectories. Collision validation is an explicit dependency. |
