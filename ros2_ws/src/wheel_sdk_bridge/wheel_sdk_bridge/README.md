# wheel_sdk_bridge

厂商协议的唯一适配边界。sdk.py 处理升降映射、关节反馈和租期；feedback.py 对齐实测状态；ros_state/rgbd_ros 是只读 ROS 入口。厂商 SDK 本体不在本仓库。ROS main 需要板端 luxi_interfaces。 本目录只放 Python 实现；安装和 ROS 元数据位于上级。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| `__init__.py` | wheel_sdk_bridge: runtime adapter boundary. |
| `feedback.py` | Timestamped measured feedback and fail-closed task transitions. |
| `rgbd_ros.py` | Optional ROS 2 atomic subscription. Records observations; never moves robot. |
| `ros_state.py` | Robot-local SDK feedback -> named measured joints. No motion publisher. |
| `sdk.py` | Real feedback mappings and local leases. This module sends no commands. |
| `joint_client.py` | 新 SDK joint_move/joint_stop HTTP 适配；16 轴弧度目标、状态配对、不重试写入。可直接提交运动请求，保留参数与实时状态校验。 |

| `http_client.py` | 有界 SDK JSON HTTP，不重试 POST。 |
| `lift_client.py` | 机器人状态 GET 与底盘反馈判断。 |

## 关节反馈专用通道

`JointClient` 的写入仍走 `base_url`，状态 GET 走 `feedback_url`。
机器人上的只读服务直接订阅 `/luxi/arms/state` 与 `/luxi/arms/startup_status`，
避免相机/录制网关的共享锁和回调排队。服务器代码与用户服务配置位于
`deployment/robot/`，没有运动发布器或运动服务客户端。

客户端保留 400 ms 有效年龄上限，额外拒绝两类状态时间差超过 30 ms 的专用通道响应。
不对齐工作站与机器人单调时钟，使用机器人内部年龄加整个请求耗时做保守检查。
新通道故障时直接报错，不自动退回旧缓存。`feedback_diagnostics.py` 保存每次
原始响应、请求耗时、字段年龄、拒绝原因，按 8 MiB × 4 文件轮转；网络失败也记录。
日志位置由 `feedback_diagnostics` 配置，相对路径相对项目根目录。
此通道只用于关节目标客户端，RGB-D/升降采集等其他接口保持原路径。

新版补偿接管使用 `nominal_target`，不把 `support_target` 当作运动目标。
`takeover_hold()` 受独立保持接管开关约束，强制目标与连续读取的原目标相同，
且必须已经使能、停稳、无锁存故障。一般 `move()` 保留参数、实时状态和请求顺序校验。
对不含新版补偿协议、输出未稳定、夹爪补偿异常的状态拒绝写入。


只读视觉 CLI 使用静止诊断的 500 ms 升降反馈门限；其他采集调用默认仍为 200 ms。保留原始 SDK valid 标记，仅允许可确认的年龄原因拒绝进入静止诊断，不输出可执行目标。

## 升降位置控制

`lift_motion_client.LiftMotionClient` 调用 `POST /api/lift` 的
`position_arm / position_heartbeat / position_move / position_stop`。
`detail` 是包含 `owner` 和递增 `sequence` 的 JSON 字符串；move 另含
递增 `task_id` 与绝对编码器位置 `target_um`。机器人端填写自己的单调时间，
工作站不伪造机器人时钟。对外 `move(position_m)` 使用米，内部转换为整数微米。

```python
from wheel_sdk_bridge.lift_motion_client import LiftMotionClient

with LiftMotionClient("http://192.168.0.245:8000") as lift:
    # 此示例会实际执行；0.100 是编码器坐标，不是距地面高度。
    command = lift.move(0.100)
    arrived = lift.wait(command["task_id"], timeout=180)
```

`open()` 显式获取控制会话并等待配置完成；独立线程持续续租，GET 不阻塞续租。
`move()` 接收回执后不算完成，`result()` / `wait()` 要求 owner、task_id、目标匹配，
且 SDK 返回 reached、实测误差不超过 10 微米、静止且底层会话已关闭。
完成后可提交下一目标；`stop()` / `close()` 停车并等待静止确认，退出 context 自动 close。
不要丢弃未关闭的客户端。SDK 当前不提供可选升降速度或定时轨迹参数。

运动 POST 不重试；回执丢失、心跳失败、反馈异常时终止续租并尝试有 owner 的停车，
不使用全局 stop 抢占其他控制者，不自动重新使能。进程退出后由 SDK 租期失效停车。
有效反馈 GET 可在 0.5 秒内等待新样本；每份被接受的反馈仍满足配置的年龄上限，
无效/过期样本从不用于目标或到位判断。run_segment 正常运动也会递增，
驱动重启识别使用 process_started_mono_ns。

`lift_client.LiftClient` 保持原有只读底盘状态职责，不影响导航模块。

机械臂 `JointClient.move(q16, speed_rad_s=.02, enable_if_needed=True)` 可显式允许
SDK 在检查姿势稳定后原地使能并执行目标；默认 False，避免调用者无意使能。
已使能时保留补偿交接检查；未使能时 SDK 从实测位置重新建立目标并重置补偿。
该选项不发送 startup/归位命令，未运动的轴仍须由调用者填入正确保持目标。

升降规划、下发前检查、到位确认仍只使用 valid=true 的实测样本。
任务进度监测单独识别 SDK 既有的 500 ms 健康窗口和最多 2 秒锁闸启动准备窗口；
此时只返回未完成状态，不把无效位置用于到位确认。motion_inhibited 或驱动故障立即报错，
不会进入无效样本等待或自动重试运动。
