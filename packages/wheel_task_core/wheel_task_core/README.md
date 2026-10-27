# wheel_task_core

任务领域核心库。contracts.py 定义接口和数据，coordinator.py 编排单箱工序，state.py 保留细粒度状态工具，transforms.py 校验坐标值，configuration.py 解析共享配方。多箱调度尚未实现。 本目录只放 Python 实现；安装和 ROS 元数据位于上级。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| `__init__.py` | Framework-independent task contracts and orchestration. |
| `configuration.py` | Resolve a recipe relative to its robot profile; never mutate input files. |
| `contracts.py` | Application contracts shared by navigation, vision and motion adapters. |
| `coordinator.py` | One transfer at a time; orchestration never owns a motor control loop. |
| `state.py` | Fine-grained grasp phase transitions, independent of hardware transport. |
| `transforms.py` | Shared validation for rigid transforms, independent of ROS and SDK. |
