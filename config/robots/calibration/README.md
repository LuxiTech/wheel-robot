# 头部相机到 SDK 坐标转换

当前使用 `head_camera_j4_rezero_20261104.json`，按左右J4写零前后实测坐标差重算用户选定的fixed_height_1方案。
原始fixed_height_1.json已按用户要求移入项目，内容和SHA256保持不变，来源见同名`.source.json`。
`head_camera_left_j3_rezero_20261103.json`与`head_camera_rezero_20261102.json`保留作匹配零点的历史回放。
`head_camera_first.json`保留为标零前历史资料和显式对照，不再作为当前计算默认值。外参不替代CAD URDF。

| 文件 | 用途 |
| --- | --- |
| `head_camera_first.json` | 第一套相机外参、参考升降反馈 170000 μm、相机序列号和允许高度范围。 |
| `head_camera_rezero_20261102.json` | 11月2日标零后、11月3日左J3重标前的历史候选；参考反馈179494μm。 |
| `head_camera_left_j3_rezero_20261103.json` | 历史候选；旧包左J3按实际硬件零点变化换算后重拟合，参考反馈179494μm。 |
| `head_camera_fixed_height_1_20261104.json` | J4重标之前的用户选定候选，原文件保留供对照；strict_80pct_train/fixed_height_1，参考81814μm。 |
| `head_camera_j4_rezero_20261104.json` | 当前候选；复用同一录包、265帧训练与186帧姿态组留出，换算历史J4角度并重算FK后拟合；名义+Z补偿、单位比例及高度范围不变。 |
| `left_j3_rezero_20261103.json` | 左J3写零前后反馈与机械基准记录；不自动给当前SDK反馈加偏移。 |
| `j4_rezero_20261104.json` | 左右J4硬件写零、复测机械基准、实际坐标变化和当前临时保持服务记录；不自动给当前反馈加偏移。 |
| `sdk_arm_chains.json` | 从 SDK `openarm_bimanual.xml` 提取的双臂关节链，记录来源 SHA256；不包含惯量、碰撞或控制器。 |
| `sdk_arm_chains_claw14.json` | 当前claw1.4完整URDF在SDK兼容arm_origin下的臂链；右肩采用URDF安装位置，旧链不覆盖。 |
| `gripper_claw14.json` | 继承原有传动近似，记录来源及左爪方向未解决限制，不改电机控制配置。 |
| `claw14_contacts.json` | 对1.4 URDF和对应夹指网格重新核对的CAD接触参考；不是已标定TCP。 |

坐标以米为单位，列向量左乘：`p_arm_origin = T_origin_camera(h) @ p_camera`。
当前升降补偿为 `t(h) = t(81814) - [0,0,1] * (h-81814) * 1e-6`，旋转不变。
SDK 高度可为负值，不能当作 URDF 的 `lift_joint`；这里没有填入未经确定的升降零位。
当前文件计算范围为 -39990～171675 μm；训练高度81814μm，另有171675/−39990μm测试。
源文件仍标记physical_accuracy_accepted=false，未将录包闭环误差作为实物TCP精度验收。

2026-11-04随后已重标左右J4硬件零点，详见`j4_rezero_20261104.json`。
当前SDK反馈已经是新零点坐标，不能再叠加该记录的坐标差；历史角度需要换算时使用实际写零前后的差值，
不能使用先前被底盘支撑干涉疑问及姿态复测推翻的右J4 −1.508°作为修正量。
原fixed_height_1文件保持不变；新零点候选已重算并接入当前配置。相同新零点下，旧外参留出平移RMS10.41mm，新外参11.32mm，不能宣称精度改善。
当前抬臂叠图仍有可见偏差。高度180000μm处超原验证范围8325μm，只读叠图使用输出目录中的显式外推副本，正式外参范围未扩大。
重算及可调透明度实体叠图：`outputs/calibration-rezero/20261104/left-right-j4/report.html`。
正式SDK对J4下限0的严格比较会拒绝负半计数零反馈，当前用记录中的临时保持服务使能在0附近，
其J4会话下限为−0.003rad，正式SDK配置的限位未改；不要把当前状态记为已恢复正式服务。

以下保留2026-11-03候选的历史记录：该文件曾作为项目默认选择，现已由上述11月4日文件替换。包内全部有效升降反馈均为179494μm，
补偿应使用` t(h)=t(179494)-[0,0,1]*(h-179494)*1e-6 `，不能套第一套170000μm参考。
同样438张合格图、22组姿态，旧角度换算为`q_new[10]=q_recorded[10]+0.091554131380rad`，
其他15个角度不变。该差值来自写零前后同姿态反馈，不采用推算设计零位的5.364469°，也不叠加残余0.107875°。
换算只用于历史数据，当前SDK实测角直接使用。图像、角点、内参、时间配对、停稳门限、升降和四折姿态分组不变。
留出平移闭环RMS由14.51mm降为5.28mm，P95由24.31mm降为8.55mm，最大12.39mm；旋转RMS由3.20°降为1.52°。
这些不是独立物理计量或抓取TCP误差，不能据此宣称稳定10mm以内。跨高度方向/比例在本包中不可辨识。
检查新旧对照和叠图见`outputs/calibration-rezero/20261103/report.html`；拟合内图、逐折结果见其`left_j3/`。
可通过已有`camera_to_sdk.py --calibration config/robots/calibration/head_camera_left_j3_rezero_20261103.json`
显式选择此候选；离线需指定`--frame head_rgb_color_optical --camera-serial 261043062235 --height-um 数值`。

SDK 双臂 IK/FK 共用 `arm_origin`，给任一手臂的目标都可以表达在这个公共坐标系。
`openarm_left_base_link` / `openarm_right_base_link` 是 SDK 自身定义，不能与 CAD 的
`openarm_left_link0` / `openarm_right_link0` 直接等同。
`left_ee_control_point` / `right_ee_control_point` 是 SDK 控制参考点，不是实际夹持中心。
两侧 `ee_T_tcp` 均保留 null：棋盘格安装变换不能当作夹爪 TCP。

## 已接入的入口

在项目根目录执行离线转换（以下为示例点，不是真机抓取目标）：

```bash
.venv/bin/python tools/grasp/camera_to_sdk.py \
  --height-um 0 --camera-serial 261043062235 --point 0 0 0.8
```

输出包括公共 SDK 坐标、两侧 SDK 基座坐标的相机矩阵及点坐标。
`--pose-json` 可输入完整 `camera_T_object` 矩阵；`--joints` 可输入 SDK 原始 q[16]
（右臂 0～6、左臂 8～14，7/15 为夹爪），增加两侧 EE 相对坐标输出。
这只是坐标转换，不把输入关节当作实时反馈，不产生运动指令。

```bash
.venv/bin/python tools/grasp/camera_to_sdk.py --live --url http://192.168.0.245:18030
```

实时入口只 GET 图像和状态，输出相机矩阵。要求同一相机的彩色对齐 RGB-D、有效升降反馈，
并在图像接收前后各覆盖至少 100 ms 的静止升降窗口。时间配对全部使用机器人单调时钟；
不把主机与机器人计时相减，也不宣称得到硬件曝光同步。运动中的升降采样会被拒绝。

`tools/grasp/perceive_live.py` 的头部推理已读取 runtime 配置里的 `coordinate_conversion`，
输出 `arm_origin`，并保存 `lift_context.json`。回放图像时必须通过 `--lift-context` 提供同组
上下文。`--coordinate-frame base_footprint` 保留原 URDF 路径及其未标定检查，不能绕过检查。
`SynchronizedCapture` 已支持传递上下文，`PipelineVision` 会转交给视觉管线。
任务调度器仍拒绝坐标不一致和 diagnostic_only 结果，未改成自动执行。

## 复核

```bash
.venv/bin/python tools/grasp/verify_calibration_chain.py \
  --calibration config/robots/calibration/head_camera_first.json \
  --session /home/luxi/qingyue/biaoding/calibration_run_20261027/multiheight_restart_session
```

报告默认写入 `outputs/coordinate_chain/replay_validation.json`。检查第一套矩阵保持原值、
本机 FK 与录制时原生 SDK FK 一致，并重现 37 个多高度姿态的闭合误差。
原数据 RMS 约 8.396 mm，最大约 17.074 mm；软件回放一致不等于实物抓取验收。
右臂转换沿用 SDK 公共根坐标，未新增右臂物理精度验证。

## 2026-10-28 真机只读检查与剩余工作

真机 SDK `ik.hpp`、安装目录的 `openarm_bimanual.xml` 与标定时文件 SHA256 一致。
新 CAD 包 README 说明使用 OpenArm 1.0；同数值关节零位下，扣除安装偏航后前五关节轴向
基本一致，但第六、第七轴不同。SDK 与 CAD 基座原点间距分别约 62 mm 和 180 mm；
原点可能定义在不同位置，不能单凭间距判定实际肩宽，也不能直接认作同一个坐标系。
尚需确认实际关节零位/方向映射、安装关系及实际 EE 到夹持中心的偏移，才能接通整机规划。
当前没有发布假的 `base_link -> camera` 固定 TF。

头部硬件为 D455F，支持 RGB-D。当前实际运行 `luxi-rgb-head_rgb.service`，其 drop-in
`90-rpent-rgb-only.conf` 选择 640×360、30 Hz 的 RGB。
`luxi-depth-head_rgb.service` 已安装但 inactive/disabled；右腕深度服务正在运行。
头部深度服务已配置正确序列号和 `ALIGN_TO_COLOR=true`，默认 640×480、15 Hz。
因此需要切换相机服务，而不是重新实现深度驱动或仅修改说明性 JSON。

相机服务切换的具体操作尚未执行：先确认没有依赖原 RGB 分辨率的录制任务，检查
`luxi-robot.service` 状态，再启动 `luxi-depth-head_rgb.service`；它的 Conflicts 会停止
头部 RGB-only 服务，且也与 `luxi-robot.service` 冲突，不能未经检查直接启动。
切换后检查 `/api/cameras/head_rgb/rgbd`、相机序列号、640×480 对应的 CameraInfo 和深度有效像素。
新分辨率必须使用新流内参；同一彩色镜头的外参无需仅因分辨率变化而重算。
深度值当前标注 `source_depth_z_resampled_to_color`，已有视觉诊断标志仍保留。
需要恢复原 RGB 时停止头部深度服务并启动原头部 RGB 服务。

本次没有修改真机服务、SDK、驱动，也没有发送运动指令。实时 RGB-D 验证、整机坐标注册和
实际 TCP 未完成，因此不能把“相机到 SDK 坐标的软件转换已接入”表述为“自动抓取已可用”。

## 静止视觉诊断的简化时效策略（2026-10-28）

`camera_to_sdk.py --live` 和头部 `perceive_live.py` 采用 500 ms 升降总年龄上限，默认采集类仍为严格 200 ms。SDK `valid=false` 仅在确认是年龄原因、UART 来源正确、位置有效、无故障/运动/保护标志时用于只读静止诊断；不改写 SDK 标记。上下文记录 `freshness_policy`、门限、观测最大年龄及原始 `sdk_valid` 列表，结果仍 `executable=false`。

接收链路 150 ms、图像默认 250 ms、拍照前后各 100 ms 的静止窗口、高度变化 200 μm、反馈间隔 200 ms 检查保留。故障、断流和无法证明静止仍拒绝；500 ms 不表示忽略所有过期错误。

已撤销并删除独立板端配对服务及固件准备副本，没有更新升降固件或重启控制服务。保留限时重试和 RGB-D 二进制解析提速。

本轮真机连续 100 次采集：69 次完成坐标转换，31 次被剩余检查拒绝；详见 `outputs/visual_grasp_verification/lift_timing_diagnosis/static500_100/summary.json`。本次只解决静止诊断的升降年龄限制，不宣称采集链路已稳定通过全部请求。

静止诊断现最多等待 8 秒，遇到短暂网络超时或尚未静止时丢弃整组观测并重试；任何成功结果仍需同时通过 500 ms 升降年龄、250 ms 图像年龄和完整静止窗口检查。持续运动仍超时退出，故障、错误相机身份等立即拒绝。严格模式默认仍为 3 秒。

8 秒重试版真机连续 100 次：97 次完成坐标转换，3 次在限定时间内未通过检查；结果见 `outputs/visual_grasp_verification/lift_timing_diagnosis/static500_retry8_100/summary.json`。这是当前现场静止诊断结果，不是运动执行验收，也不保证其他网络条件下的成功率。

## claw1.3 接触几何与安装检查（2026-10-29）

当前只使用 claw1.3。单臂相对基座的 SDK FK 与 CAD link7 按同值关节角一致，
左右各100组随机姿态的位置误差小于 3e-15 m，不需要旧模型的第5关节角偏移。
这只是模型对应，不是实物安装验收。

使用视觉环境运行 `tools/grasp/inspect_claw13_contacts.py --output <新目录>`，
从实际 STL 三角面提取长指凸台与短指平面的一对候选接触点，检查点在面内、
法向相反及21个开口下的公式一致性，生成 JSON 和测量参考图。
CAD 净开口 W(mm)=2*q(mm)-25.72，单指 q 范围约12.86～44 mm；
对应净开口约0～62.28 mm。这不是电机角度映射，也不是已确认的箱沿接触方式。
长指后退24 mm的指腹平面不能混作同一个宽度基准。

分别以左右臂基座对应推导的 CAD→SDK 整机注册存在120 mm平移差异，
因此不能将左臂对应直接复用于右臂。需要独立核实实物双肩间距和两边基座定义。
现有单画面的腕部角度拟合未通过整夹爪对位，不写入安装校正。

SDK 既有开闭记录仅含电机角度；当前左右反馈0 rad也不代表接触面间距0 mm。
缺少同一接触面基准下、带对应反馈的多开口实测宽度，不能拟合或验收角度—宽度映射。
`ee_T_tcp`保持未标定，接触点输出只供诊断，不发送运动。

## 2026-10-30 链路复核：尚未完成视觉抓取验收

只读实测关节反馈两通道各一次通过；头部 RGB-D 与静止升降配对转换成功，
高度 101163 μm，记录在 `outputs/visual_grasp_verification/chain_status_20261030.json`。
此结果仍使用候选外参和静止诊断时效策略，不代表运动所需精度已验收。

修正 `check_claw13_alignment.py` 的左右臂坐标混用：现在分别用 SDK 实测关节 FK
得到各侧腕端，再乘 claw1.3 的腕端到完整夹爪链，统一在 `arm_origin` 投影。
不再把左肩推导的 CAD 整机注册用于右侧；原方法会在右侧引入约120 mm的模型平移差异。
`SDKKinematics.gripper_transform` 拒绝其他手臂的帧、腕端模型不一致、缺少手指位移。
它使用真实第七轴反馈，不置零、不反号，也不拟合未经测量的固定偏移。

历史图像重新投影见 `outputs/grasp/alignment-check/sdk-per-arm-20261030/overlay.png`。
左右各100组腕端模型一致性检查通过，但整夹爪轮廓仍与实物不一致。
所以此次修正解决诊断工具的坐标混用，未解决实物安装/TCP标定。
尚缺多姿态夹爪参考点观测、同一接触面基准的开口与电机反馈配对测量，以及外参独立验收。
第七轴固定零偏与绕该轴的固定安装偏转可能等价，不能仅凭一幅轮廓确定物理原因。

已有 SDKIKExecutor 的真机端点复测通过不等于视觉抓取通过：控制点仍为 link7，
执行为升降和机械臂分阶段到位，不能直接当作钩尖竖直插入轨迹。
当前尚未执行或验收“箱沿上方对位→向下插入→闭合→试抬”的视觉抓取全过程。

## 本地 SDK 副本删除（2026-10-30）

用户要求删除 `device/lift_robot_ros2` 与 `device/lucid-atlas`。运行适配器不导入或链接这些副本；真机 SDK 与板端 ROS 工作空间保持原状。`sdk_arm_chains.json` 是实际使用的独立运动学快照，继续保留；其中 source.path 仅为历史来源记录，不是运行时加载路径。

`verify_calibration_chain.py` 不再自动查找历史副本。需要核对 SDK 模型指纹时，显式传入 `--sdk-model <实际核对的XML文件>`；不传时报告 `sdk_source_hash_check.checked=false`，不宣称本次已核对真机源码。

## 当前 claw1.3 与原始外参兼容性复核（2026-10-30）

原始 `extract_recorded_calibration.py` 使用 SDK `sdk_fk_batch` 与 OpenArm v2
`cell/cell.xml` 生成 A（arm_origin 到 left_ee_control_point），并非使用旧夹爪 URDF。
标定方程为 `A_i @ Y = X @ B_i`：X 为 arm_origin 到相机，Y 为腕端到棋盘格。
当前 URDF 按 `SDK arm_origin_T_left_base @ URDF left_base_T_link7` 重算同一组实测角度。
这种坐标转换只复用单臂结构，不假定 URDF 整机根与 arm_origin 相同。

本次核对原始候选文件和其6个输入文件 SHA256，全数一致；原始 X 与项目外参矩阵逐元素一致。
原始标定562帧及多高度录制1488帧，当前 URDF 与记录的 SDK FK 最大矩阵元素差分别
约1.47e-15、2.05e-15。原始545对棋盘格闭合 RMS 5.204 mm、最大15.992 mm；
多高度37姿态闭合 RMS 8.396 mm、P95 12.701 mm、最大17.074 mm，角度 RMS 2.242°。
替换为当前 URDF 重算未改变这些误差，因此没有发现因更换 URDF 导致该外参失效的证据。

原始录制42帧左臂joint2超过当前URDF上限0.17453 rad，最大记录0.205806 rad。
审计仅在内存中的离线几何副本里不施加命令限位，以原始角度计算并逐项记录超限；
没有裁剪反馈，没有修改URDF、运行限位、外参或关节零位。此回放不授权重放历史运动。

报告：`outputs/coordinate_chain/current_urdf_extrinsics_audit_20261030.json`。
复现：

```bash
.venv/bin/python tools/grasp/verify_calibration_chain.py \
  --session /home/luxi/qingyue/biaoding/calibration_run_20261027/multiheight_restart_session \
  --original-session /home/luxi/qingyue/biaoding/calibration_run_20261027/continuous_session \
  --output outputs/coordinate_chain/current_urdf_extrinsics_audit_20261030.json
```

边界：本次复用录制的棋盘格PnP结果，未重新检测所有原始图像，也未重读真机SDK模型或执行新物理验收。
原多高度数据的棋盘格安装Y已在重装后冻结；原始拟合使用原始Y，不混用两次安装。
残差包含相机观测、关节/时间配对、棋盘格安装与模型等误差，不能全归为外参误差。
X可沿用在SDK arm_origin坐标链中，不能直接当作URDF base_footprint或夹爪基座到相机的变换。
保持候选状态，当前误差记录不能证明钩尖毫米级插入精度。

## 夹爪传动资料核对（2026-10-30）

本轮检查了用户claw1.3原包、`custom_grippers.yaml`、`validation.json`，以及标定目录中已有的
SDK接口历史证据，未恢复已删除的SDK副本。结果保存于 `outputs/grasp/mechanism-audit-20261030/audit.json`。

- 用户模型的夹指为prismatic（米），validation明确说明省略内部闭环曲柄连杆。
- 标定时SDK v2 XML的默认夹指为hinge（弧度），不能直接代替本夹爪的平移位移；单臂FK一致不证明夹指模型相同。
- 历史SDK反馈为电机角减cfg.offset，目标下发加回cfg.offset。2026-10-21零位记录是软件偏置，未重置电机编码器。
- 历史开口端点记录于10月8日，无配对的毫米位移；不能与10月21日零位记录未经核对就拼接线性映射。
- `set_position(target+offset, 50., 1./4.5)` 中第三参数尚未读取到定义，不能把它认作机械传动比。

本轮真机SSH只读访问被拒绝，未核实上述历史源码与当前部署的一致性。下一步读取
`device/openarm/openarm_can` 的夹爪set_position定义及对应机械机构文件。
优先用机械参数推导，不要求先做精细实测拟合；只有驱动/结构资源确实缺少必要参数时才具体说明缺什么。

## 真机SSH已核实的夹爪驱动与IK范围（2026-10-30）

用户提供访问凭据后成功只读登录，结果见 `outputs/grasp/mechanism-audit-20261030/verified_conclusions.json`。
密码不存入文件。保存所查源文件的片段、SHA256和完整夹爪驱动小文件作为审计证据，未恢复SDK本地副本。

- 真机 `gripper_component.cpp` 直接令 `target_motor_pos = position`；没有角度到毫米的转换。
- `set_position`第三参数是 `torque_pu` 电流限制比例，`1/4.5`不是传动比。
- 真机 `ik.hpp` 只同步14个机械臂关节，FK输出SDK腕端控制点。夹爪目标直接放回q[7]/q[15]，不参与夹指运动学。
- 因此现有腕端FK/IK不需要为定制夹爪修改XML；定制夹爪几何沿用用户URDF，在腕端后接入。
- 真机当前arms-kh配置的开闭目标已包含软件坐标偏移；不应把读到的0 rad直接称为闭合或URDF位移0。
- 真机所检查的openarm目录只有openarm_can及openarm_mujoco，未找到对应平行夹爪kinematics_link.yaml/xacro；本轮所查驱动中也没有该机构位移公式。仍需匹配的完整机构定义，不能凭外形或电流参数补造传动关系。

本次未修改真机源码、配置、XML或下发运动命令。

## 找到官方夹爪角度—位移近似（2026-10-30）

OpenArm官方ROS2已合并PR #49（2025-07-24）：
https://github.com/enactic/openarm_ros2/pull/49/files
固定提交：f2177fd3e2dc41aa636ed5589c4ca134c5f38855。
其v10_simple_hardware.cpp使用单指位移 `g(m)=0.044*theta(rad)/(-1.0472)`，
反向 `theta=-1.0472*g/0.044`；官方代码明确标为近似，非精确闭环机构解。
因此此前“没找到映射”只适用于已查真机驱动，不代表官方没有提供。

官方1.0夹爪规格给出最大净开口88mm、闭合到张开转动60°：
https://docs.openarm.dev/hardware/specifications/gripper/
官方parallel_link xacro采用两个相反轴的平移关节和mimic，与用户模型的表示方法一致。
但匹配的模型表示不证明内部传动完全相同；实际部署前仍须核对是否保留v1平行夹爪机构、
左右电机方向及零位偏置。用户定制指尖的净间隙不能直接沿用官方88mm；现有CAD公式仍为2*g-0.02572m。
本轮已保存来源与公式至 `outputs/grasp/mechanism-audit-20261030/official_mapping.json`，未修改运行配置或真机。

## 官方近似映射接入诊断（2026-10-30 16:01）

用户确认保留1.0完整开合传动机构。新增 `gripper_v1.json` 保存真机已查偏置与开合方向，
`wheel_motion_core.gripper.finger_displacement` 按 `raw=SDK反馈+offset`、
`g=0.044*opening_sign*raw/1.0472` 计算，明确假设原始电机零角对应官方位移零。
左右方向取已查开闭配置的增减方向；未另外拟合或更改零位。映射仅用于诊断，不授权运动。

新采样 `outputs/grasp/v1-mapping-live-20261030-retry`：右手估计g=15.363mm，
左手g=-15.716mm超出URDF范围。说明左侧原始角参考与官方位移零参考的直接对应尚不成立，
不能据此断定实际机构负开口，也不能强制取绝对值或裁剪；需核对机械装配相位/官方零参考。
右侧在范围内也仅表示公式可求值，不证明已实物验收。
当前图像两夹爪主体大部分位于画面下缘，无法可靠验收完整轮廓。
报告保留两指4×4位姿与CAD钩部接触面参考点；该参考点不是已确认的钩尖最末端。

16:06新采样：`outputs/grasp/v1-mapping-live-recheck-20261030-b/`。
本次双夹爪清晰可见；已生成完整叠图与body-only固定主体叠图。
右侧估计15.684mm；左侧-15.427mm拒绝绘制活动手指。固定主体轮廓仍有明显方向差异，
该部分不随开口变化，因此不能继续单独归因于开合映射。钩部输出为已有CAD接触面参考点，
不是已确认的最末端插入点。当前没有通过实物对齐验收，未发运动命令。

## 分段偏差定位（2026-10-30）

`tools/grasp/localize_alignment.py` 离线绘制link4/5/6碰撞STL、腕端原点与固定夹爪主体。
结果 `outputs/grasp/segment-audit-20261030/report.html`，对比16:06新帧和10月29日历史帧。
原SDK双臂根坐标下，右腕与其上游臂段整体向内偏；左腕位置相对接近。
使用左基座对应注册、CAD双肩相对安装作为右根假设，两帧右腕位置均明显改善。
这说明此前切换至逐臂SDK根只能证明坐标模型一致，不能证明实物双臂安装已正确注册。
在该根假设下，固定夹爪角度仍偏；复用历史常量J7偏移假设后方向明显改善。
目前定位到右根注册和J7末端角参考两段，而非单纯开口问题。
J7零偏与绕其轴的固定安装偏转在数学上等价，当前定性叠图不能区分两者。
未修改运行根坐标、角偏移或相机外参；碰撞STL只是臂段近似轮廓，不是量具验收。

## 修正候选交叉检查未通过（2026-10-30）

复现脚本、人工粗标点与结果：`outputs/grasp/correction-validation-20261030/`。
使用两帧实拍图的可见腕部圆盖中心作粗略投影参照（人工标注约±5px，不是实测坐标标记）：
右侧原SDK根偏差约122.85/94.78px；CAD相对肩部修正候选后约14.55/13.92px。
这支持根注册修正方向，但残差及圆盖中心与URDF原点的对应误差均尚未分离，不能接受为精确位姿标定。
左侧相同参照偏差约5.10/9.20px，不支持“前六轴与相机完全无误”的结论。
历史J7角偏移仍只有定性轮廓改善，未独立拟合或验收。
所有候选保持仅离线诊断；未修改真机/运行参数，未下发运动。
要进一步定准，需要固定夹爪上与CAD坐标一一对应的可见刚性特征（例如已识别的螺丝中心），
在多个独立姿态下记录对应像素与关节角，再用保留姿态验证；不能以同一图的轮廓拟合自证正确。

## 用户授权全零目标验证中止（2026-10-30 16:37～16:40）

用户明确要求发送所有关节零目标。双臂q[16]全0、0.03rad/s，经SDK公共接口执行；
不重置编码器或改零位偏置。日志位于 `outputs/zero-verification/20261030T083719Z/`。
首请求因反馈请求延迟约905ms超过400ms门限，客户端停止，SDK确认保持。
重读反馈确认停止、无锁存故障且使能后，从当前姿态提交新的全零请求；
第二请求被SDK joint_following_error阻断，未再重试。结束实测J7约左+8.84°/右-8.58°，
J4双侧约4.29°，未达到全零，不能据此认定已观察到机械零位姿态。
升降前检一度因反馈时效拒绝，之后只读检查恢复；未发送任何升降移动命令。
最终joint_control为fault，双臂仍使能；保存了末态反馈、前中后RGB-D、两次请求及停止回执。

## 完整URDF统一模型候选（2026-10-30）

模型：`outputs/unified-model/claw13-20261030-obj/robot.xml`及同目录meshes/manifest。
转换器为`tools/grasp/build_unified_model.py`，反馈适配为`wheel_motion_core.unified_model`。
MuJoCo3.3.7真实加载通过；30组随机关节输入、54个link，共1620项URDF/MuJoCo比较，
最大矩阵差约1.22e-15。机身固定相机在多高度转换后的差约1.11e-16。
16:06实测角度回放见`outputs/unified-model/validation-20261030-b/`；无J7角度修正。

模型使用完整URDF双肩安装，SDK保留关节名称和腕端site；新arm_origin是升降架，
不能与旧SDK arm_origin混用。已有外参通过左基座对应显式转到base_footprint，
注册假设与0.2m升降零偏候选均未标记为物理验收。绝对升降偏置在共同坐标变换下
会从相对投影中抵消，不能用叠图一致反过来证明0.2m零偏正确。

同目录`ik_unified.hpp`和`ik_lift_adapter.patch`是基于已读取SDK头文件生成的审阅候选，
增加`sync_lift_encoder(height_um, direction, offset_m)`；调用方必须在配对反馈时调用它。
仅替换XML而不调用该入口，旧SDK仍只同步14轴，不能宣称已接入升降。
候选头文件尚未在目标SDK构建，未部署或重启真机服务。

整机统一后的实物叠图仍有固定夹爪方向差异，左侧开口零参考问题仍在；
没有将历史±30多度偏移写入新模型。本轮未发送运动或清除之前的跟随误差故障。

## 近似闭合零点后实拍检查（2026-10-30 17:18）

用户澄清闭合SDK0是近似位置，可能略有偏差。已取消精确零参考的表述。
新采样 `outputs/grasp/closed-reference-check-20261030-b`，高度175840um，
统一URDF/MuJoCo叠图 `outputs/grasp/closed-reference-unified-overlay-20261030/overlay.png`。
两侧夹爪SDK反馈0，按名义闭合几何估计每指位移12.86mm，左侧不再因旧raw-zero假设产生负位移。
但完整夹爪叠图仍有明显方向差异，不能宣称已对齐或已满足抓取条件；没有补J7角度，也没有发运动。
同帧实测J7左约0.650607rad、右约-0.599870rad，作为当前姿态输入保留，不因零点口头参考而置零。

## J7逐级调试与绘图修复（2026-10-30）

入口 `tools/grasp/debug_joint7.py`，使用17:18的配对图像/反馈，结果在
`outputs/grasp/j7-debug-20261030/trace.json`。右J7数组索引6、左14，单位rad；
左0.650606546rad(37.277°)、右-0.599870298rad(-34.370°)。
URDF两侧J7都是link6局部X轴、关节origin为单位矩阵，因此link6_T_link7=Rx(q7)。
手写Rx与URDF结果误差约1.11e-16，URDF/MuJoCo腕端最大差约4.44e-16；
手写针孔投影与OpenCV相差约2.27e-13px。J7增加0.01rad不改变腕端原点位置，符合零平移关节定义。
这些检查不验证编码器与实际装配角参考，也不独立验证已有相机外参。

发现小的绘图问题：一次fillPoly填充所有重叠三角面会按奇偶规则产生孔洞。
已统一改为逐三角形fillConvexPoly取并集，新增重复/交叠三角面回归测试。
原/修复后的固定主体外轮廓IoU左约0.99798、右0.99883；修复不消除大角度失配。
修复后叠图在 `outputs/grasp/j7-debug-20261030/unified-fixed-raster/overlay.png`。
未为消除误差改J7、取反号或套用历史角度，未发送动作。

## 不依赖外参的导轨相对夹角检查（2026-10-30）

入口 `tools/grasp/check_relative_rail_orientation.py`；证据 `outputs/grasp/relative-rail-audit-20261030/`。
使用同一17:18配对RGB-D与关节反馈，人工ROI选择两侧白色导轨长条，以有效深度点的主轴估计方向。
27组分割/深度门限下左右锐夹角11.86～16.04°，中位14.08°；完整URDF按同一组关节角计算为64.00°。
模型固定主体长导轨面主轴与URDF夹指平移轴差约0.018°，提供特征方向对应依据。
共同刚体相机外参不能改变两方向的夹角，因此在当前深度解释与特征识别假设下，
主要矛盾已出现在“SDK角度→已安装夹爪相对方向”的链，不能仅归因于相机外参。
尚不能唯一分离J7零参考、同轴安装参考、上游方向误差或CAD实物对应，不能直接采用历史偏移。
深度语义与特征身份未经独立计量验收；27组范围是参数稳定性，不是物理精度置信区间。
新证据已加入 gripper-explained-view 的原公式页面；本次没有修改运动参数或真机。

## 撤回未核实特征的导轨夹角定位（2026-10-30）

用户指出白色长条可能是相机外壳。本次没有先核实零件身份就把该图像主轴当作导轨轴，
与URDF夹指平移轴比较，属于无效的跨特征比较。
因此上述“12～16度对64度”不能用于定位夹爪位姿误差，也不能据此排除相机外参。
relative-rail-audit的report/summary已标记retracted，原网页结论已替换；原始数据保留供追溯。
URDF默认姿势与真机姿势不同是正常的，必须使用配对反馈驱动模型，再核实同一物理部件的对应关系。
