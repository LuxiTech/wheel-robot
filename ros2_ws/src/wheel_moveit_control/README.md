# 双臂 MoveIt 2 执行链

视觉侧复用原抓取规则、外参和夹爪定位；目标接受时检查原图时间不超过 2 秒。
2026-11-16 已部署 SDK 新协议和板端服务；完整真机抓取及 J6 制动效果尚未验证。

板端 MoveIt 2 接收腕端目标，规划双臂同一时间轴的轨迹，交给
`joint_trajectory_controller/FollowJointTrajectory`。SDKSystem 读取真实 SDK 反馈，
通过板端本地 UDP 交给 SDK；只有 SDK 访问 CAN。插件不会使能机械臂。

## 模型与规划

`tools/grasp/generate_moveit_config.py` 从选定的原 URDF、当前 SDK 限位及状态生成
独立配置目录，默认使用假硬件，不改原模型。每侧腕端目标分别换算到对应的臂根，
保持现有 SDK 角度和升降编码器到 CAD 的 `+0.2 m` 映射。升降固定，不参与规划。

- 对准：OMPL RRTConnect 双臂 14 轴联合规划，TOTG 时间参数化。
- 下插、试抬：双腕笛卡尔插值、连续 IK、全机器人碰撞检查、TOTG。
- 状态机沿用已有张开、下插停止、闭合和试抬验证。
- 碰撞模型包含原整机及选定箱体 CAD；不表示周围未建模物体已纳入。
- 试抬将箱体附着到右腕，允许两侧夹指与箱体接触，双腕保持同步路径。

规划侧直接订阅 SDK 原始关节状态，保留原始接收时间；20 Hz 发布场景状态，
与 100 Hz 控制回路分离。JTC 读取实测位置/速度，路径误差门限 0.05 rad，到位门限 0.02 rad。
这些是候选软件配置，不是电机实测制动能力。SDK 原反馈/断流 150 ms、权限、限位和
跟随保护保留。MIT 增益及前馈未改；不能宣称此改动已消除 J6 实物越限问题。

## 构建与隔离验证

板端需 ROS Humble、MoveIt 2、ros2_control、ros2_controllers、原 `luxi_interfaces`。
先 source ROS 和当前 SDK 的 `build/install/setup.bash`，在单独工作空间构建此包。

```bash
colcon build --packages-select wheel_moveit_control --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ROS_DOMAIN_ID=223 ROS_LOCALHOST_ONLY=1 ros2 launch wheel_moveit_control stack.launch.py config_dir:=/绝对路径/假硬件配置
```

规划网关仅监听板端 `127.0.0.1:18036`，通过 SSH 本地端口转发访问；不直接开放到局域网。
`GET /` 返回 mock/ready/执行权信息。`POST /plan` 保存目标和轨迹，`POST /execute`
仅执行已保存且起点仍与实测一致的计划；`POST /cancel` 必须携带本次 plan_id。

真机使用 `wheel_moveit_control/SDKSystem` 配置，必须先部署已验证的
`joint_stream_planned_trajectory_protocol=1` SDK 候选。新协议 `WJPLAN1` 的来源时间是
轨迹会话接纳时间，不伪装成图像时间；图像检查在规划接纳入口完成。
旧 SDK 缺少能力时激活会拒绝，不自动回退到旧运动回路。

## 视觉入口

`config/robots/grasp_execution.moveit.json` 为独立新配置。旧默认配置保持原执行模式。
新入口自动复用或建立 SSH 隧道：

```bash
bash tools/grasp/run_moveit_visual_grasp.sh check
bash tools/grasp/run_moveit_visual_grasp.sh grasp
```

`check` 只读；`grasp` 会实际执行，请在现场已准备好时运行。
板端服务名称 `wheel-moveit-control.service`，目前保持硬件和轨迹控制器 inactive。
重启机器人后先 `sudo systemctl start wheel-moveit-control.service`；完整原始碰撞模型首次加载约 100 秒。
这不表示完整抓取或实物制动已经验收。

每次运行仍存入 `outputs/grasp/visual-grasp-runs/`。规划输入、轨迹、JTC 实测反馈、
执行回执从板端复制到 `run/execution_trace/moveit/<plan_id>/`，耗时进入 `execution.log`。
板端原始日志保留在配置目录上一级 `executions/<plan_id>/`，复制失败会留下错误事件。

Base collision geometry: generated models now use the supplied `base_link` visual
STL with the exact original visual origin instead of its coarse collision box.
The source URDF remains unchanged. No allowed-collision pairs are added. This is
surface geometry (not a watertight solid or verified physical registration).
Source provenance and the failing-pose replay are recorded in
`outputs/grasp/base-mesh-20261116/`.

The base collision resource is now the saved derivative
`derived_collision/base/r680_base_collision.stl`, not the full visual STL.
The generator verifies source/output fingerprints and keeps the visual mesh,
all installation transforms and all other collision geometry unchanged.
See the derivative's README/provenance for its 0.5 mm sampled error budget and
`outputs/grasp/base-collision-lite-20261116/` for runtime measurements.
