# packages

三个标准 Python 核心库，各自拥有 pyproject.toml。它们不依赖 ROS、Web 或厂商 SDK；通过统一契约和参数注入组合。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [wheel_motion_core/](wheel_motion_core/README.md) | 运动算法核心库。kinematics 提供 URDF FK 和 15 自由度联合 IK；collision 为保守 OBB；planning 生成离线轨迹。计划保持不可执行，不直接访问 SDK。 |
| [wheel_perception_core/](wheel_perception_core/README.md) | 视觉算法核心库。RGB-D → 校正 → 检测/分割 → 位姿/质量 → 抓取候选。模型后端按需导入；没有运动指令。仅依赖任务核心及图像/数学库。 |
| [wheel_task_core/](wheel_task_core/README.md) | 任务领域核心库。contracts.py 定义接口和数据，coordinator.py 编排单箱工序，state.py 保留细粒度状态工具，transforms.py 校验坐标值，configuration.py 解析共享配方。多箱调度尚未实现。 |
