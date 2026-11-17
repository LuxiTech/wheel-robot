# 两段连续抓取执行

抓取的两段主运动为：移动到箱沿上方；张开后连续下插并抓取。抬高净空、平移、转向仍是第一段内部的规划节点，整段只提交一次，不在每个节点停稳等待。闭爪接触和10mm试抬仍有各自的反馈确认。

张开直接提交SDK已有的完整张开目标。闭合由`SDKContinuousContactCloser`提交完整闭合目标并持续读取接触角度和力矩；一侧先接触就停止、保持该侧，再完成另一侧，最多三次有界尝试。不会恢复每次0.01rad的闭合小步；空闭合和过载均不能算抓取成功。`joint_stop`携带本次请求ID，在板端原子核对归属，适用于臂轨迹和夹爪目标，避免停止其他控制者的动作。

`joint_path.hpp`在机器人上插补整段16轴SDK角度。保形三次插值保留输入节点、关节范围和方向，阶段首尾速度为零，中间同向节点不人为停顿。按关节速度和加速度界限局部重新分配时间，节点加速度可以跳变，不宣称限制加加速度。默认指令速度0.2rad/s，加速度界限0.5rad/s²；这是轨迹目标的界限，不是实测电机速度保证。

`JointTrajectoryClient`一次提交完整路径；没有协议v1时明确拒绝，不退回逐点发送，也不直接丢弃路径只发最后一点。两个夹爪在机械臂连续段内保留SDK当前`nominal_target`。SDK仍负责原有限位、实测反馈时效、0.25rad跟随保护、权限协调和保持补偿。

SDK只在整段末端按原0.02rad、0.08rad/s和连续0.5秒条件判断关节到位。新轨迹接口默认留2秒段末收敛时间；未收敛时保持并返回`needs_correction`，不谎报`ready`。主机从新鲜实测角重新求IK，最多两次矫正；没有改善则停止继续重发。硬故障、断流、请求被其他控制者替换仍直接中止。旧`joint_move`接口的行为不变。

## 本地代码

- `wheel_execution.phase_driver.SDKTrajectoryDriver`：整段提交、异步回执确认、实测段末验位、有界矫正和本次请求停止。
- `wheel_execution.two_stage_grasp.TwoStageGraspExecutor`：两段主运动；对准后重观测，张开后连续下插，腕部到达条件可在段内停止整段，随后确认双侧停止、接触和试抬。
- `MovingWristClearanceSource`：使用持续采样的SDK角度历史，按图像接收时间插值配对；不把接收配对冒充硬件曝光同步。配对不足、旧图或未接受的几何/深度仍拒绝。
- `config/robots/grasp_execution.json`：本机器人选择两段模式和现场看护下跳过碰撞检查，记录用户来源；此跳过不写成`collision_checked=true`。安装后的库若没有这份配置，默认仍要求连续曲线检查器，可显式传入配置文件。

旧包围盒路径的`collision_checked`不能证明新的三次曲线已检查。`required`策略要求检查器提供`validate_continuous_phase`；当前现场试验按用户明确要求选择`onsite_supervised_skip`。这项优化不改变URDF、外参、SDK零位、PID增益、补偿上限或试验下插行程。

## 真机 SDK 候选的准备和验证

只从真机当前SDK源文件生成候选，不恢复本地`device/`副本。将本目录的`prepare_sdk.py`、`joint_path.hpp`、`verify_mock.py`传到机器人上的独立目录，在source SDK环境后运行：

```bash
python3 /tmp/wheel-joint-trajectory-source-20261115/prepare_sdk.py \
  --sdk-root "$PWD" --output "$PWD/work/two-stage-trajectory-20261115/candidate"
cmake -DCMAKE_BUILD_TYPE=Release \
  -S work/two-stage-trajectory-20261115/candidate \
  -B work/two-stage-trajectory-20261115/candidate/build
cmake --build work/two-stage-trajectory-20261115/candidate/build -j2
python3 /tmp/wheel-joint-trajectory-source-20261115/verify_mock.py \
  --binary "$PWD/work/two-stage-trajectory-20261115/candidate/build/openarm_adapter_trajectory_candidate" \
  --config "$PWD/Project/robot_bringup/config/arms-kh.json" \
  --output "$PWD/work/two-stage-trajectory-20261115/mock-verification"
```

输出目录须新建。准备器不安装文件、不启动或停止服务。验证脚本固定使用模拟电机、`allow_hardware=false`和隔离ROS域，遇到与真机进程相同的域会拒绝；不会访问真实CAN。实际HTTP网关原有`/api/arm_startup`已是通用服务转发，无需为新命令修改或重启网关。

2026-11-15的最终候选位于真机SDK根目录下`work/two-stage-trajectory-20261115/candidate-complete/`，对应模拟记录`mock-verification-complete-v3/`，7项板端检查通过。本地证据目录为`outputs/grasp/two-stage-execution-20261115/`。121个上方对准节点和41个下插节点分别一次提交；0.2rad/s、0.5rad/s²条件下，离线曲线时长约37.11秒和10.51秒。旧执行器仅中间节点的0.5秒停稳条件就累计至少80秒，新版每个主运动段只需末端停稳；此比较不包括新图采集、实际电机误差、矫正和开合试抬，不是真机抓取耗时验收。

上线需要安装候选适配器并重启原机械臂适配器。当前真机源文件`Project/device_adapters/src/openarm_adapter.cpp`的析构函数会调用`disable_all()`；双臂处于使能保持时不能无支撑重启。必须先取得现场双臂已可靠支撑的确认，保留原SDK源文件、头文件和二进制备份后再切换，随后重新核对反馈和协议版本。不能把候选编译或模拟验证通过写成已经上线或真机抓箱通过。
