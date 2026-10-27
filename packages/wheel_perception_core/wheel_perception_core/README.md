# wheel_perception_core

视觉算法核心库。RGB-D → 校正 → 检测/分割 → 位姿/质量 → 抓取候选。模型后端按需导入；没有运动指令。仅依赖任务核心及图像/数学库。 本目录只放 Python 实现；安装和 ROS 元数据位于上级。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| `__init__.py` | RGB-D, learned perception and geometric grasp targets. |
| `association.py` | Task-layout ROI association, never simulator instance labels or live object poses. |
| `backends.py` | Lazy adapters to upstream model APIs. No simulated predictions on failure. |
| `geometry.py` | Model input geometry and deterministic, object-relative grasp rules. |
| `pipeline.py` | Observation -> selected instance -> pose -> URDF transform -> pair targets. |
| `rgbd.py` | Strict reader for SDK ba0c732 atomic RGB-D, plus lossless replay storage. |
| `visualization.py` | Diagnostic overlays. All projections use the saved optical camera frame. |
