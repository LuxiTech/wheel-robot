# 接口和实现范围

## 已有 Python 契约

wheel_task_core.contracts 定义 TransferJob、GraspObservation、MotionPlan、Navigation、Vision、Motion。
GraspObservation 保留 target_id/pair_id/frame_id/captured_at/quality_valid/diagnostic_only；时间属于任务进程单调时钟域。
采集适配器必须验证曝光时间与关节对齐，不能以 HTTP 接收完成时间替代曝光时间。
MotionPlan 必须绑定本次观测且通过碰撞与执行就绪检查；离线 Trajectory.executable 保持 false。

## 已有 ROS 契约

wheel_interfaces 生成 CratePose、GraspCandidate、ExecuteGrasp。它们是消息/action 定义，不包含硬件 action server。
SDK 接收端依赖开发板原有 luxi_interfaces；本仓库不生成或伪装该厂商消息包。

## 已实现 HTTP 查询

| 方法/路径 | 返回内容 |
| --- | --- |
| GET /api/v1/health | 管理 API 存活和 read_only 标记 |
| GET /api/v1/status | 默认运行装配的就绪条件，source=default_runtime_composition |
| GET /api/v1/workflow | 配置中的单箱任务和工序列表；不执行 |
| GET /api/v1/configurations | 四类配置目录中的 YAML/JSON 文件名；不读 .env |
| GET /api/v1/runs | 固定 outputs/robot 下最近 30 份有效结果的摘要 |
| GET / | 独立静态管理前端 |

完整拆码垛任务 POST、配置写入、认证与 WebSocket 仍未实现。新增 A/B 导航控制见下表；默认禁用导航写请求，未提供的控制接口不会启动任务。
API 依赖网关的查询接口，页面不直接访问 ROS 或 SDK。默认回环监听，用于本机或 SSH 转发的只读诊断。

## 后续扩展约定

增加写操作前实现身份与权限、command_id 去重、机器人本地任务唯一执行权、取消确认和状态对账。
accepted 与 completed 必须区分，超时后先查询不能盲目重发物理操作。
长动作以 ROS Action 提供进度与取消，连续状态采用 Topic；任务服务独立于 Web worker。


## A/B 导航管理接口

| 方法/路径 | 内容 |
| --- | --- |
| GET /api/v1/devices | 独立双 SDK 连接与设备反馈投影 |
| GET /api/v1/navigation | 地图版本、A/B、就绪原因、当前命令与实测状态 |
| GET /api/v1/navigation/commands/{UUID} | 查询持久化命令，提交响应丢失时使用 |
| POST /api/v1/navigation/go | {point: A或B, command_id: UUID}，202 仅表示受理 |
| POST /api/v1/navigation/stop | {command_id: UUID}，等待 stop_id 和底盘静止确认 |

默认写操作返回 423；使用 --enable-navigation 显式开启。body 禁止额外字段，跨 Origin 的控制请求返回 403。
命令状态为 SUBMITTING/MOVING/ARRIVED/STOPPING/SUCCEEDED/STOPPED/UNKNOWN/REJECTED/FAULT/CANCELLED。
GET 不发送任何运动或停车指令，已受理命令的监测线程可执行到达后停车和故障时单次停车。
双 SDK 地址与现场资料见 wheel_navigation/SDK_INTEGRATION_README.md。
