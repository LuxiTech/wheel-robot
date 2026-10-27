# 验证说明

## 回归范围

原有算法和任务测试迁移到新 import；增加分层依赖检查、目录 README 覆盖、配置合并、网关结果读取边界及可选 HTTP 查询测试。
新的分层检查防止三个核心包导入 ROS、Web 或 SDK，并防止感知/运动库互相依赖。

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q -rs -p no:cacheprovider
tools/robot status
tools/robot workflow
tools/grasp/grasp doctor
```

HTTP 测试需要安装管理依赖及 httpx；GPU 环境只做新模块导入/CLI 检查，不在结构重构中重建引擎。
缺外部 URDF/历史回放时跳过，不补造资产；合成测试不代表硬件验收。

## 验证边界

拆包后的 ROS 包清单/XML、Python 打包和容器 Compose 进行静态检查；完整 ROS 构建、厂商消息接入及 GPU 全量推理需要目标运行环境。
没有实施真机运动、没有验收抓取精度。具体本次运行结果由交付时的实测报告记录。


## 2026-10-27 本机实测

- 原 `.venv`（Python 3.14）：62 passed，5 skipped；其中 4 项缺外部资产，1 项是未安装 Web 依赖的可选 HTTP 测试。
- 独立 `.runtime/management/venv`（Python 3.11）：63 passed，4 skipped；含 HTTP 查询、静态页面和控制请求拒绝检查。
- 跳过项：3 项缺真实机器人 URDF，1 项缺固定历史 RGB-D 文件。
- 原有三个 CLI 查询 status/workflow/doctor 成功，新配置解析保持未标定标记与旧示例尺寸。
- 本机未安装 docker/colcon，未实施镜像构建与完整 ROS 编译，不以静态元数据校验替代真实构建。
- 三个核心库和七个 Python 适配包共 10 个 wheel 已成功构建，并在独立安装目录中通过导入检查，不依赖源码 bootstrap。
- 原 `.venv` 的旧 wheel_grasp_core 安装已替换为新 0.3.0 包，安装版 wheel-robot/wheel-grasp 查询命令通过。
- 实际启动 tools/management 后，health/status/workflow/configurations/runs 及 HTML/JS/CSS 共 8 个 HTTP 路径通过；验证进程已停止。
- GPU 环境的新包导入及两个视觉工具 --help 通过；未重新执行模型推理或构建 GPU 引擎。

## A/B 导航接入验证（2026-10-27）

- 管理环境完整回归：78 passed、4 skipped；跳过项仍为 3 个外部 URDF 用例及 1 个固定历史 RGB-D 用例。
- 新增导航测试使用 loopback 假 SDK HTTP 服务，不对实际 SDK 发运动请求；覆盖 A/B 单点、命令去重、并发、旧目标序号、版本变化、断线/响应丢失、重启、后台监测及停车回执关联。
- 浏览器通过 SSH 转发连接假 SDK 后台，实际点击 A、B 和停止按钮，页面分别显示已到达并停车/已停车；通信中断时禁用导航按钮，恢复后可继续查询。
- 实机只执行 GET：确认两个 HTTP SDK 可连接、lift_robot 载体及 unstacking 的 A/B 点位存在；定位反馈过期，未执行任何真实导航或停车。
- SDK 不可变核对：device/lucid-atlas 的 1311 个源码/配置/文档文件、device/lift_robot_ros2 的 224 个文件与接入前快照 SHA-256 一致。
- 默认 tools/management 不允许导航 POST；显式 --enable-navigation 后才开放控制。测试预览进程和 SSH 转发已停止。
