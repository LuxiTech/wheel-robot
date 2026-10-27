# wheel_navigation 实现

| 文件 | 功能 |
| --- | --- |
| `atlas_client.py` | 陆图 GET 状态/点位/载体，POST 单点命名导航与停止 |
| `service.py` | A/B 请求、并发互斥、反馈关联、到达后停车、状态投影及后台监测 |
| `journal.py` | Linux 进程锁、提交前持久化、命令去重与重启后的未知状态 |
| `factory.py` | 从 devices.yaml 装配 HTTP 客户端与导航服务，默认禁用写请求 |
| `adapter.py` | 实现任务 Navigation 契约；将工位别名映射为 A/B，等待实测完成 |
| `__init__.py` | Python 包命名空间 |

仅 service 的显式导航/停车与已受理任务监测允许写 SDK API。GET 查询不触发运动、模块启动或地图准备。
命令、锁文件和运行记录位于本工程 .runtime/navigation；不导入、写入或执行 device 下的 SDK 源码。
