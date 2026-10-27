# tools

统一开发入口。robot 启动运行时，management 启动只读后台，pythonpath.sh/_bootstrap.py 发现源码，grasp/ 保存模型、CAD 与视觉诊断工具。工具不直接承载业务状态机。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [grasp/](grasp/README.md) | 实现子目录；见目录内 README.md。 |
| `_bootstrap.py` | Discover first-party source packages for uninstalled developer tools. |
| `management` | 本目录配置、资源或工具文件。 |
| `pythonpath.sh` | 本目录配置、资源或工具文件。 |
| `robot` | 本目录配置、资源或工具文件。 |
