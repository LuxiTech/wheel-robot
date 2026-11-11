# deployment

部署边界。compose.grasp.yaml 为历史 ROS 工具/GPU 推理服务，docker/ 保存其镜像定义。核心库先安装再构建 ROS 包；不在此存放机器人业务逻辑。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [docker/](docker/README.md) | Dockerfile.ros-tools 构建 Humble 工具与适配包；Dockerfile.grasp-perception 构建历史 GPU 推理环境。对应 dockerignore 限制构建上下文。当前本机原生 GPU 环境与这些历史镜像分离，迁移后镜像需重新构建验收。 |
| `compose.grasp.yaml` | 本目录配置、资源或工具文件。 |

## 机器人只读关节反馈服务

`robot/joint_feedback_server.py` 在真机 ROS Humble 环境运行，直接订阅关节状态，
通过仅支持 GET 的 18032 端口提供小型快照。保持机器人原始单调时间戳；
通过驱动 GetParameters 只读接口核验 mock 参数，核验失效时不声明是真机状态。
不会打开 CAN、使能、停止或驱动电机，也不转发运动请求。

本机没有原生 ROS，故用机器人端 ROS 订阅加轻量 HTTP 传输，不要求跨主机 DDS。
真机 SDK 驱动和原相机网关没有修改或重启。原网关共用执行器和互斥锁的具体延迟来源
尚未细分；本方案隔离关节反馈工作，原 18030 网关仍服务相机及命令。

部署位置（sunrise 用户）：

- `/home/sunrise/.local/lib/wheel-robot/joint_feedback_server.py`
- `/home/sunrise/.config/systemd/user/wheel-joint-feedback.service`

服务文件中的 SDK overlay 路径、绑定 IP、端口按真实机器设置。复制这两个文件后，
使用 `systemctl --user daemon-reload` 和 `systemctl --user enable --now wheel-joint-feedback.service`。
这是用户服务，随该用户的 systemd 会话启动；不承诺未登录时启动，不自动改 linger。
服务只依赖 ROS 环境，驱动暂时未就绪时返回不可用状态，由客户端拒绝。

2026-11-09 真机地址为 `192.168.2.199`。客户端配置与用户服务的 `--host` 必须一起更新；
绑定已经失效的旧地址会报 `Cannot assign requested address`，导致18032端口无法启动。
两路ROS反馈独立到达时，只读服务最多等待80毫秒获取当前匹配数据；不修改原始时间戳，
不放宽客户端30毫秒配对差或400毫秒时效检查，未匹配或过期的数据仍由客户端拒绝。

确认反馈服务运行后，在 `config/robots/joint_control.json` 设置 `feedback_url` 和日志路径。
使用 `tools/grasp/check_joint_feedback.py` 做只读比较。服务退出或网络故障时客户端报错，
不回退旧反馈，不发送自动失能指令。重启此只读服务不会重启机械臂驱动。
若回滚，先退出使用此通道的应用，再撤销 feedback_url 配置，按需停用此用户服务；
旧通道仍有已观察到的延迟，回滚不表示恢复了可运动状态。

2026-10-28 按用户要求，关节客户端与独立反馈服务的状态有效年龄上限统一改为200毫秒。
该年龄包含完整HTTP请求耗时及电机反馈年龄；30毫秒状态配对检查、运动速度和真机驱动内部保护不变。

同日后续按用户要求，客户端与专用反馈服务有效年龄上限由200调整到400毫秒，
专用读取请求超时为0.5秒；没有更改真机驱动内部反馈看门狗或执行速度。
