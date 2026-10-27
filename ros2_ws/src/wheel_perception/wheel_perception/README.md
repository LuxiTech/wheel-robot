# wheel_perception

任务侧视觉适配。service.py 定义 SynchronizedCapture/CaptureSource，将核心 GraspPipeline 转为 GraspObservation。推理工作线程串行化，默认同步源未接入。不包含已部署的 ROS action server。 本目录只放 Python 实现；安装和 ROS 元数据位于上级。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| `__init__.py` | wheel_perception: runtime adapter boundary. |
| `service.py` | Adapt the retained RGB-D/keypoint pipeline to the task interface. |
