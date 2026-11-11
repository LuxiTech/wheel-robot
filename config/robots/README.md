# robots

当前模型为claw1.4，由`robot_model.json`统一登记URDF、来源、SDK链、夹爪传动近似和接触几何。
三个runtime配置使用同一1.4 URDF及`sdk_arm_chains_claw14.json`；旧1.3资源、旧链快照和历史外参保留。
1.4仅更新腕部相机/支架，臂关节和夹爪安装不变。真机XML为SDK v2目录下的`openarm_claw14.xml`，
与`openarm_bimanual.xml`并存。新的`arm_origin`通过未变的左臂基座与原标定坐标注册，
两肩安装使用完整URDF；SDK腕端FK/IK相对该随升降架运动的坐标计算，升降量会在相对变换中抵消。
不以此声明整机或夹爪实物精度已验收。`gripper_claw14.json`继承旧传动配置，左爪方向矛盾仍明确保留，
没有借模型更新改写SDK电机限位、开合目标或零位。

## SDK 单次关节目标接口

`joint_control.json` 配置 SDK 地址、反馈通道和补偿协议。
已删除 `motion_enabled` / `blockers` 静态运动拦截，`JointClient.move()` 可直接提交关节目标。
提交前仍校验参数、实时反馈、使能状态、补偿交接和未完成请求，不会自动使能电机。

在项目根目录运行 `.venv/bin/python tools/grasp/joint_control.py` 可只读读取真实关节反馈。
`--positions q16.json --speed 0.02` 只预览请求，不发送；该工具没有真机执行选项。

真实协议是 `POST /api/arm_startup`，command 为 `joint_move`，detail 是 JSON 字符串，
包含 request_id、positions[16]、unit=rad、speed_rad_s。顺序为右臂七轴、右夹爪、
左臂七轴、左夹爪；速度范围 0.02～0.2 rad/s。请求被接受不代表已到位。
通过 GET /api/state 的 arm_startup.joint_control 和 arms 实际反馈确认结果。
新接口也支持 deg，但本机统一发送 rad。

此接口是在真机上执行一个目标的插值，不是连续指令流；本机断网不会自动取消已接受动作。
joint_stop 表示保持已发送姿态，不是失能；它尚无请求级控制权隔离。
本机适配器不自动重试 POST，不在超时或退出时发送 disable、归位或自动返回指令。
未知请求结果保持待核对状态，禁止紧接着提交另一个目标。

2026-10-28 只读检查发现：真机驱动的关节服务异常处理可能调用 disable_outputs，
更新异常可退出到析构函数并调用 disable_all；正常 joint_stop/到位则保持姿态。
这些是软件失能路径，不代表已观测到电源掉电。按用户“有失能风险先别执行”的约束，
本次没有真机运动测试，没有重启或修改真机服务。
当前 startup_ready 使用非零静态补偿；joint_move 接管会清除启动补偿，给未动关节填
实际角度并不等于保留其原保持目标。补偿交接也需核对后再测试。

代码快照与实际读取结果在 `outputs/sdk_joint_interface_audit/`。
单次关节执行适配为 `wheel_execution.joint_executor.SDKJointExecutor`；
自动抓取的 SDKMotionExecutor 仍未就绪，避免把接口接入误认为抓取已可用。

runtime.yaml 为默认只读/离线配置；runtime.live.yaml 为本机 GPU 视觉配置；runtime.container.yaml 为历史容器配置。recipe 相对本 YAML 解析，模型资产路径相对仓库根。相机外参/TCP/升降标定仍未完成。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| `runtime.container.yaml` | 本目录配置、资源或工具文件。 |
| `runtime.live.yaml` | 本目录配置、资源或工具文件。 |
| `runtime.yaml` | 本目录配置、资源或工具文件。 |

## 2026-10-28 关节反馈延迟修复

`joint_control.json` 的 `feedback_url=http://192.168.0.245:18032` 指向新的 ROS 只读反馈服务；
`base_url=http://192.168.0.245:18030` 继续接收关节命令。
`feedback_diagnostics` 指向本机原始反馈轮转日志。部署文件在 `deployment/robot/`。
后续按用户要求，关节客户端与专用反馈服务的门限统一调整为 400 ms；
运动准入、控制增益、保持补偿和运动速度不变。只读检测命令仍是
`tools/grasp/joint_control.py`；新旧链路对比用 `tools/grasp/check_joint_feedback.py`。

## 新版补偿接管（2026-10-28）

已核对真机新版源码 `c6adb31442d1d8a3a1082287c06171589e0ecdd2cf3603756a1394622cd9d5a7`：
普通关节接管从 `nominal_target` 开始，保留既有补偿，`support_target` 是实际电机输出。
`compensated_handover_protocol=nominal_target_v1` 允许适配器识别该协议，
检查补偿范围、夹爪无补偿、目标与输出一致性及故障状态；不是只删除旧拦截。

`hold_takeover_enabled=true` 单独允许 `takeover_hold()`，它不接受任意目标输入，
只向已使能、稳定的机械臂提交当前原目标，使用0.02 rad/s并保留补偿。
它核对前后控制代次和目标，没有自动使能、重试POST或失能。
一般运动通过 `move()` 提交；保持接管开关仅影响 `takeover_hold()`。

反馈有效年龄上限400毫秒，专用HTTP读取超时0.5秒；请求耗时仍算入有效年龄。
30毫秒状态配对检查、0.02弧度到位容差及真机驱动内部时限均不变。
机械臂驱动未因此重启，只更新并重启只读反馈服务。

## 升降控制配置

`lift_control.json` 指向真机位置控制网关 `http://192.168.0.245:8000`，
`feedback_max_age_s=0.4` 是有效反馈年龄上限，目标单位为绝对编码器米。
没有额外静态运动开关；调用 `open()` 显式获取 SDK 会话，调用 `move()` 下发目标。
`lift_mapping=null` 表示尚未标定到 IK/URDF 升降关节坐标，不影响编码器位置控制。
