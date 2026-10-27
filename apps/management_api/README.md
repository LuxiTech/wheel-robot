# management_api

FastAPI 管理服务，默认只观测，支持显式开启 A/B 导航。management_api/app.py 定义五类 GET 查询并挂载静态前端；__main__.py 提供 host/port/root 参数。没有登录、写参数、完整拆码垛任务控制或 WebSocket，默认只监听回环地址。安装和启动见下文。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [management_api/](management_api/README.md) | 实现子目录；见目录内 README.md。 |
| `pyproject.toml` | 独立 Python 构建、版本与依赖声明。 |

## 启动

在仓库根执行：

```bash
python3 -m venv .runtime/management/venv
.runtime/management/venv/bin/python -m pip install -r tools/grasp/requirements.txt -e apps/management_api
tools/management
```

访问 http://127.0.0.1:8088。现场环境准备后以 `tools/management --enable-navigation` 开启 A/B 和停止控制。独立安装的 API 仍需在部署环境提供核心库与适配包；tools/management 会发现本仓库源码。可用 WHEEL_MANAGEMENT_PYTHON 显式选择另一个已安装依赖的解释器。
