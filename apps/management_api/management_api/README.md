# management_api

FastAPI 只读管理服务。management_api/app.py 定义五类 GET 查询并挂载静态前端；__main__.py 提供 host/port/root 参数。没有登录、写参数、任务控制或 WebSocket，默认只监听回环地址。安装和启动见下文。 本目录只放 Python 实现；安装和 ROS 元数据位于上级。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| `__init__.py` | HTTP management adapter; no task execution worker or SDK dependency. |
| `__main__.py` | 本目录配置、资源或工具文件。 |
| `app.py` | Read-only management API and static console for a configured checkout. |
