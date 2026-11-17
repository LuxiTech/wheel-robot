# wheel_perception_core

视觉算法核心库。RGB-D → 校正 → 检测/分割 → 位姿/质量 → 抓取候选。模型后端按需导入；没有运动指令。仅依赖任务核心及图像/数学库。 本目录只放 Python 实现；安装和 ROS 元数据位于上级。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| `__init__.py` | RGB-D, learned perception and geometric grasp targets. |
| `association.py` | Task-layout ROI association, never simulator instance labels or live object poses. |
| `backends.py` | Lazy adapters to upstream model APIs. No simulated predictions on failure. |
| `geometry.py` | Model input geometry and deterministic, object-relative grasp rules. |
| `pipeline.py` | Observation -> selected instance -> pose -> configured coordinate transform -> pair targets. |
| `extrinsics.py` | 第一套头部外参、相对升降补偿、相机来源检查及 SDK 坐标转换；不发送运动指令。 |
| `rgbd.py` | Strict reader for SDK ba0c732 atomic RGB-D, plus lossless replay storage. |
| `visualization.py` | Diagnostic overlays. All projections use the saved optical camera frame. |

`targets.py`输出视觉与运控之间的显式目标包：图像身份、SDK arm_origin、米制刚体位姿、
左右臂分配、局部箱沿顶点、滑动导轨下表面停止基准及不可丢弃的质量拒绝。
`insertion.py`仅计算同一光学坐标中的竖直净空，不直接授权闭爪或发送控制。
