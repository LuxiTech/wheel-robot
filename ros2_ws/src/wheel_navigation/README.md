# wheel_navigation

通过 lucid-atlas 的公开 HTTP API 实现 unstacking 地图 A/B 命名点导航，不修改任何 SDK 源码。
本包负责导航请求、幂等和结果确认；规划、速度控制、载体租期与执行由陆图/机器人 SDK 负责。

## 文件与结构

| 路径 | 功能 |
| --- | --- |
| [wheel_navigation/](wheel_navigation/README.md) | API 客户端、导航服务、命令记录与任务适配 |
| [SDK_INTEGRATION_README.md](SDK_INTEGRATION_README.md) | 两个设备的仓库、地址、SSH、源码路径、API、点位与现场操作说明 |
| `package.xml`、`setup.py`、`setup.cfg` | ROS/Python 工作包安装与依赖 |
| [resource/](resource/README.md) | ament 包索引 |

## 运行方式

从仓库根使用 `tools/management` 查询双 SDK 连接和 A/B 点位；默认不允许导航写请求。
负责人准备测试环境后，显式运行 `tools/management --enable-navigation`，网页才允许前往 A/B 和停止。
`config/deployments/devices.yaml` 保存两个 SDK 的 HTTP 地址及超时，配置中没有 SSH 密码。

导航不是 Nav2/AprilTag 的本地重实现。`AtlasNavigation` 可注入现有任务编排，但完整拆码垛仍因视觉/运动执行器未接入而拒绝启动。
到达只代表地图目标及实测停车确认，不能替代箱体抓取所需的精确工位对接。

## 验证

`tests/test_navigation.py` 使用 loopback 假 SDK 验证 HTTP 契约、A/B 切换、并发、幂等、响应丢失和停车确认。
实机仅做 GET 只读核对，不在自动测试中发运动指令；具体测试记录见系统验证说明。
