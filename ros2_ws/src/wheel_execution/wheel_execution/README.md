# wheel_execution

真机操作执行边界。executor.py 保留运输、规划、预抓取、接近、夹持验证、试抬、放置与保持接口。ready=False，尚未接通厂商轨迹执行。后续由本包统一管理动作反馈与取消。 本目录只放 Python 实现；安装和 ROS 元数据位于上级。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| `__init__.py` | wheel_execution: runtime adapter boundary. |
| `executor.py` | Robot-local execution boundary, separate from the existing offline planner. |
