# wheel_execution

当前抓取组合入口默认读取`config/robots/grasp_execution.json`，选择`TwoStageGraspExecutor`和
`SDKTrajectoryDriver`：上方对准、连续下插抓取两段主运动，内部路径点一次提交给板端SDK插补；
只在段末验位并做最多两次有界矫正。腕部停止与载荷反馈仍在运动期间读取，闭合和试抬继续分别确认。
没有SDK轨迹协议v1时不会退回逐点发送。当前板端运行版本与候选的状态须分别核实；
准备、测试和上线说明见`deployment/robot/joint_trajectory/README.md`。

真机操作执行边界。executor.py 保留运输、规划、预抓取、接近、夹持验证、试抬、放置与保持接口。ready=False，尚未接通厂商轨迹执行。后续由本包统一管理动作反馈与取消。 本目录只放 Python 实现；安装和 ROS 元数据位于上级。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| `__init__.py` | wheel_execution: runtime adapter boundary. |
| `executor.py` | Robot-local execution boundary, separate from the existing offline planner. |
| `joint_executor.py` | 独立单次关节目标适配器；提交、查询和停止均走 SDK 公共接口，ready 表示接口能力，实际状态在每次请求时校验。自动抓取 executor 仍未启用。 |

## IK 可调用的独立执行接口

- 机械臂：`SDKJointExecutor.move_joints(q16, speed_rad_s)`，弧度绝对角度，右臂 7 轴、右夹爪、左臂 7 轴、左夹爪。
- 机械臂首次未使能时可显式传入 `enable_if_needed=True`，由 SDK 稳定反馈后原地使能，不执行默认抬臂姿势。
- 升降：`SDKLiftExecutor.open()` 获取会话，`move_lift_m(encoder_m)` 发送编码器绝对位置，`wait(task_id)` 确认到位，`close()` 停车并释放会话。
- IK 升降关节：给 `SDKLiftExecutor(client, mapping=LiftMapping(...))` 传入实测标定后，调用 `move_joint_m(urdf_joint_m)`。未传 mapping 会明确报错，不将 URDF 高度直接当编码器位置。

转换关系为 `q_lift = direction * encoder_m + offset_m`。
现有 `config/robots/lift_control.json` 的 `lift_mapping` 留空，须实测方向、零点和行程后建立 `LiftMapping`。
IK 若只输出双臂 14 轴，需要保留两个夹爪目标后组装 q16；不要在有负载保持时用旧实测值覆盖现有 nominal_target。

底层为两个独立的目标控制通道；SDKIKExecutor 已把联合求解与分阶段执行串接，尚不提供升降与双臂的同步插补、原子提交、统一时钟或轨迹碰撞检查。
后续 IK 可调用这些接口，但一次求解的多轴目标不代表 SDK 会同时到达。
自动拆码垛 executor 仍未接通，不因此变为 ready。

## 联合 IK 到真机端点执行

`SDKIKExecutor` 位于 `ik_executor.py`：

```python
executor = SDKIKExecutor(joint_client, lift_client, urdf_path, sdk_chains_path)
plan = executor.plan_relative(
    left_delta_m=[0, 0.05, 0.06],
    right_delta_m=[0, 0, 0.03],
    lift_delta_bounds=(0.025, 0.035),
)
# 明确执行；scene_clear 由调用方对本次有界动作的环境检查提供。
result = executor.execute(plan["plan_id"], scene_clear=True)
```

也支持 `plan({'left': T_left, 'right': T_right}, frame='sdk_lift_zero')`。
规划只读，保存的方案有效期 30 秒；执行前重读状态并拒绝姿态、原保持目标或控制代次变化。
默认单次升降位移上限 40 mm、单关节变化上限 0.55 rad、机械臂指令速度 0.03 rad/s。
当前执行顺序为升降到位，再执行双臂关节目标；夹爪保持原 nominal_target。
它执行联合 IK 的最终目标，不重放离线规划器的带时间轨迹，不宣称笛卡尔直线插补。

执行期间轮询匹配的任务反馈；失败或 `cancel()` 后仅停止本次拥有的请求。
升降会话在成功或失败退出时关闭；机械臂成功后继续转矩保持。
结束时使用实测关节/升降重新做 FK，验收模型端点误差不超过 10 mm、姿态误差不超过 0.05 rad。
这不是相机或外部量具的末端精度验收。

### 使用已有 URDF，而不猜测整机零点

`SDKAlignedModel` 复用 claw1.3 URDF 的各臂 base→link7 几何，先逐臂验证与 SDK 模型一致，
关节限位取 URDF 与 SDK 实时报出的交集。SDK 控制点对应 link7，不使用未确认的夹爪 TCP。
双臂根变换采用 SDK 的 arm_origin 定义；升降变量直接采用编码器米。
`position_z = encoder_m + arm_relative_z`，定义虚拟固定坐标系 `sdk_lift_zero`。
该坐标定义不需要猜测地面高度或 URDF lift_joint 的绝对零点。
它不修改 CAD URDF，不宣称已完成 base_footprint/相机/夹持中心注册。
相机或地面坐标下的目标仍须先变换到此坐标系。

### 2026-10-30 实机验证结果

本机与真机 SDK 模型 SHA256 一致：`cb4aad7ef1ce36137ccc07894186f06a60fa45251f3bb94da91aa4be6bc23ae5`。
联合 IK 成功求出升降 +32.255 mm、左控制点向外 50 mm/向上 60 mm、右控制点向上 30 mm。
实际提交升降目标 137993 µm 后，升降在 move_ack 阶段出现无效/不健康反馈，客户端停车退出。
之后真机持续返回 motion_inhibited、结果 11（PROFILE）、target_status=230/0xE6（运动监控阻止）。
最终位置约 105738 µm，无升降控制会话；本次机械臂目标没有下发，双臂保留原保持。
因此完整实机联合运动验收尚未通过。不得自动清除固件保护或重放本次目标。

记录在 `outputs/ik_control/live_20261030/`：plan.json、execution.json、final_robot.json、final_arms.json。
须先诊断并恢复真机升降运动保护，再从新反馈重新求解验证，不能复用已过期方案。

### 2026-10-30 重启后联合端点复测通过

用户现场重启升降并操作按钮恢复健康后，从新反馈重新求解。
实机执行升降 73.913 → 101.163 mm，SDK 到位误差 1 µm；随后双臂关节目标返回 ready。
目标为左 SDK 控制点向外 40 mm、向上 50 mm，右侧向上 25 mm，夹爪保持原目标。
实测关节 FK 末端误差左 6.08 mm / 右 2.67 mm，姿态误差约 1.10° / 0.76°，通过端点验收。
升降停车并释放会话成功，双臂继续使能保持。本次未再次触发 motion_inhibited。
记录：`outputs/ik_control/recheck_live_20261030/` 与 `outputs/ik_control/recheck_telemetry_20261030.jsonl`。
这是一次有界、分阶段的联合端点执行验收，不是同步轨迹或外部量具精度验收。
此前发现的固件候选修正未部署，不能据这次通过宣称原固件缺陷已修复。

## 定点视觉抓取组合入口

当前仓库配置选择`laptop_visual_servo`：`LaptopVisualServoDriver`在笔记本独立10ms线程中
生成关节参考，`JointStreamClient`通过UDP发送，视觉推理和HTTP反馈不占用此循环。
状态机仍是对准→张开→下插→接触闭合→10mm试抬→箱体随动确认；只在阶段边界停稳。
`pure_visual_servo`保留为原板端IK方案，`two_stage`为完整轨迹候选，均需显式选择对应接口。
板端关节流缺失时先拒绝，不用逐点`joint_move`代替。构建、协议、时效与验收范围见
[`deployment/robot/joint_stream/README.md`](../../../../deployment/robot/joint_stream/README.md)。

`stationary_pick.create_stationary_pick(...)`将实时视觉目标源、`VisualGraspPlanner`、
SDK执行器、双腕RGB-D停止源、夹持接触控制和视觉试抬验证组合为一次定点抓取。
显式选择`two_stage`时，`TwoStageGraspExecutor.run(target_id)`保留当前开合状态，一次提交净空、平移/转向和
上方对准的完整路径；段末确认并重新观察后，完整张开、连续下插、闭合。
`SDKGripperOpener`保留手臂目标，双侧开到位后才开始连续下插；夹爪闭合提交完整目标，
接触时停止，允许先后到达的双爪分别保持。导轨停止连续三帧确认后，双侧接触闭合→10mm试抬→箱体随动确认。
对准位置按当前至张开范围内整个夹爪的最低点留出30mm箱沿净空，不再按导轨高度上移30mm。
下探总试验界限默认仍为40mm；超过时在对准动作前拒绝，不自动加深。碰撞策略由
执行配置显式决定；当前用户现场试验选择跳过，其他部署默认要求连续曲线检查器。
异常只停止本地拥有的请求并保持可能的负载；不回零、不停升降、不自动松爪，不自动重试载荷状态。

导轨停止高度由原CAD负X下表面计算，绝非SDK腕端原点。当前外参/夹爪资源的候选状态和源质量
始终随视觉目标传递；缺少已验收的实时数据或夹持策略时，不会启用运动。
定点抓取组合的软件测试通过不等于真机提箱验收。原移动运输/放置边界不因此标记ready。

旧`StationaryGraspExecutor`和逐点`SDKWaypointDriver`仅在显式选择`legacy_waypoints`时使用，
不与新两段执行的到位、停止和纠偏语义混用。

场景检查实现`wheel_motion_core.grasp_scene.GraspSceneCollision`可接入上述组合，覆盖机器人与
选定CAD箱体；每次视觉更新须update_target，每个规划/执行关节段按当前反馈刷新开合并复查。
它明确报告尚未认证的周边场景及开合精度，不能仅因局部几何返回无相交就启用整机执行。
