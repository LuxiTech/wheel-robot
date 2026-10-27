# tests

跨包算法、协议、任务、配置与管理查询回归。conftest.py 统一发现源码；grasp/ 保留原有测试，test_architecture.py 固定依赖方向与文档覆盖。测试替身不代表设备验收。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [grasp/](grasp/README.md) | 实现子目录；见目录内 README.md。 |
| `conftest.py` | 本目录配置、资源或工具文件。 |
| `test_architecture.py` | 检查核心依赖方向、README 覆盖和配方合并。 |
| `test_management.py` | 网关状态、结果目录边界、只读 HTTP 与页面集成。 |

| `test_navigation.py` | Loopback 假 SDK、A/B、幂等/并发、失联、到达/停车关联及管理 API。 |
