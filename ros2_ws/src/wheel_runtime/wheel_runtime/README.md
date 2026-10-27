# wheel_runtime

应用装配与命令入口。main.py 装配适配器，queries.py 提供状态/工序只读描述，cli.py 提供诊断和离线算法命令。业务状态机位于 wheel_task_core。未提供常驻任务调度服务。 本目录只放 Python 实现；安装和 ROS 元数据位于上级。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| `__init__.py` | wheel_runtime: runtime adapter boundary. |
| `cli.py` | Read-only capture, diagnostics, calibrated FK and offline model planning. |
| `main.py` | Hardware application entry point: configure adapters, then start a task. |
| `queries.py` | Read-only runtime descriptions. No node startup or hardware access. |
