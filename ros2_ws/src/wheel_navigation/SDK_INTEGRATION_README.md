# 外部 SDK 接入与现场资料

本目录仅编写 wheel-robot 的 HTTP 适配和任务协调代码。**绝不修改两个外部 SDK 的源码、配置、地图或点位文件**；也不通过导入 SDK Python 包或 SSH 执行脚本来控制设备。

## 设备连接资料

| 项目 | 陆图导航模块 | 升降机器人真机 SDK |
| --- | --- | --- |
| SDK 名称 | lucid-atlas | lift_robot_ros2 |
| 源码仓库 | https://gitee.com/fxx360724/lucid-atlas/tree/main/ | https://gitee.com/fxx360724/lift_robot_ros2 |
| wheel-robot 内只读副本 | `/home/luxi/qingyue/wheel-robot/device/lucid-atlas` | `/home/luxi/qingyue/wheel-robot/device/lift_robot_ros2` |
| 设备地址 | `192.168.4.118` | `192.168.0.245` |
| SSH | `ssh lucy@192.168.4.118` | `ssh sunrise@192.168.0.245` |
| 实机 SDK 目录 | `/home/lucy/Project/lucy-atlas-agentos/lucid-atlas` | `/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2` |
| 现有网页/API | `http://192.168.4.118:8082/` | `http://192.168.0.245:8000/` |
| 本工程用途 | 读取地图点位/定位/载体状态，提交命名点导航和停车 | 只读连接状态、底盘与设备实测反馈 |

SSH 密码使用设备负责人在本次现场会话提供的凭据，或使用 SSH 密钥；不写入版本化 README、配置或前端。应用运行不需要 SSH 密码。
历史 RGB-D 只读网关 `18030` 与当前真机控制网页 `8000` 是不同入口，双 SDK 状态查询使用当前 `8000/api/state`。

## 导航接口依据

2026-10-27 只读核对发现，实机 lucid-atlas 比 wheel-robot/device 中的副本新增命名点导航接口。使用实机公开 HTTP 契约，不同步或修改任一 SDK 副本。

| 方法和路径 | 使用方式 |
| --- | --- |
| `GET /api/status` | SDK 在线、定位、导航反馈、stop_confirmation |
| `GET /api/map/status` | nav_drive_active 等地图/导航状态 |
| `GET /api/v1/robot` | 必须为 lift_robot 载体，检查 connected/ready/estop/holder_client_id |
| `GET /api/maps/points?map_name=unstacking` | 读取 A/B 名称、坐标、map_revision、recording_ready |
| `POST /api/maps/points/navigate` | 单次提交一个已保存命名点，不传任意坐标 |
| `POST /api/nav/stop` | 请求停车，返回 stop_id，再轮询确认 |
| 机器人 `GET /api/state` | 独立读取 SDK 连接、base/base_status、arms、lift_status |

导航请求示例（这里只描述协议；不能在环境尚未准备时执行）：

```json
{
  "map_name": "unstacking",
  "map_revision": "本次 GET 点位返回的版本",
  "point_names": ["A"]
}
```

前往 B 时只将数组改为 `["B"]`。不调用 `goal_pose`，不记录/删除点位，不自动 prepare 地图、不自动启动定位/导航模块、不改变载体类型，也不直接向机器人 SDK 发送使能或速度。
陆图 SDK 原生负责导航规划、底盘控制权、速度租期与底层转发；本工程只管理 A/B 请求和结果。

## 已有地图点位的只读登记

以下是 2026-10-27 的参考快照，运行时始终从 API 重新读取，不把本文数值作为固定执行坐标：

- 地图：`unstacking`，参考版本 `d384789bf50403076ccc222f2ff1144e59d6d39c`。
- A：x=0.39625112184869227 m，y=-0.6995157019709398 m，yaw=0.02395651595433861 rad。
- B：x=-1.4890554018220095 m，y=-0.9353263084226071 m，yaw=-1.5644223899531833 rad。
- 实机地图：`maps/lidar/unstacking.pcd`；点位由 SDK 自己保存在 `maps/points/unstacking.<revision>.json`，本工程不读取或改写该文件。

只读检查时两个 SDK 的 HTTP 均可连接；载体类型为 lift_robot。地图点位存在，但定位反馈过期、recording_ready=false，因此导航就绪条件不成立。没有发送导航、停车、使能或速度请求。

## 后台启动和使用

在 `/home/luxi/qingyue/wheel-robot` 执行：

```bash
tools/management                       # 默认仅观测；导航按钮禁用
tools/management --enable-navigation   # 现场环境准备后显式开启 A/B 与停止请求
```

默认地址 `http://127.0.0.1:8088`；工作站可转发：

```bash
ssh -N -L 8088:127.0.0.1:8088 luxi@192.168.1.47
```

页面提供“前往 A 点”“前往 B 点”“停止导航”，每 2 秒读取状态。两个连接卡片分别显示 SDK 是否连通；“已连接”不等于底盘/定位/导航已就绪。
开启导航后，固定端点仅接受 A/B、UUID command_id；参数、端点、源码目录不能由网页任意指定。保持默认单 worker，使用同源控制请求。

## 到达、停车和异常

1. 前往 A/B 前，实时读取地图版本、点位、定位、导航反馈和底盘反馈。地图必须匹配、载体必须为 lift_robot，其他控制者未占用。
2. command_id 与提交意图在外部 POST 前持久化到 `.runtime/navigation/commands.json`。相同 ID、相同请求只返回既有记录；不同意图复用 ID 会被拒绝。
3. HTTP 受理只显示 MOVING。须看到大于提交前的 goal_sequence、匹配的目标位姿、goal_reached、mission_remaining=0 和新鲜底盘静止反馈，才显示 ARRIVED。
4. 导航服务随后调用陆图停车 API，解除本次导航，使下一次 A/B 切换可以重新提交。只有匹配本次 stop_id 的 stopped 回执和底盘实测静止都成立，才记为 SUCCEEDED/STOPPED。
5. 提交响应丢失不重发导航；反馈过期/任务超时会尝试一次停车并继续确认，无法确认时保留 UNKNOWN。用户可以明确再次请求停车。
6. 后台重启不恢复旧导航，旧命令保留 UNKNOWN，需明确停车对账。关闭网页/后台不能当作停车成功。精确夹取对接、AprilTag 与双臂搬运不属于本阶段。

后台的导航服务线程只推进已受理命令，网页自动刷新仅发送 GET。SDK 的实时控制循环、租期与硬件执行仍在原 SDK 中。

## 测试边界

自动测试使用本机 loopback 假 SDK HTTP 服务，覆盖 A/B 请求、幂等、并发互斥、失联、旧回执拒绝、地图版本、停车确认及 Web API。
实际设备只做只读状态和点位核对；实机 A→B/B→A 与停车验收须等负责人准备现场环境后再执行。本次不自动开启控制开关或部署 SDK。
