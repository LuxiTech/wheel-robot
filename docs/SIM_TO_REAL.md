> 当前客户演示已切换为升降版地面三箱任务，入口与配置见 [CUSTOMER_DEMO.md](CUSTOMER_DEMO.md)。本文保留原单箱/固定立柱方案说明。

# 固定立柱搬箱：仿真到真机

默认模型是三全向轮底盘 + 原始 OpenArm 固定立柱 + 双臂双夹爪 + 三相机。没有升降关节，也没有独立转向关节。19 个独立驱动 = 3 轮 + 14 臂 + 2 夹爪，另有 2 个 mimic。

## 模型与实际安装必须一致

当前采用仓库原始 `r680_openarm.urdf` 的固定立柱，肩轴离 `base_footprint` 1.00312 m、离 `base_link` 0.966 m；这不是把升降柱锁死在最高位。`openarm_mount_link` 相对 `base_link` 为 (0.102000202, -0.000000143, 0.268) m。原无相机模型 59.150 kg，加入现有三相机后 59.562 kg。底盘 30 kg、轮子各 1 kg 是估计，不是实测。

`python3 scripts/build_fixed_urdf.py` 可重建固定立柱加相机 URDF。原始两套模型均保留。腕部相机安装沿用升降版，修正了两套模型 EE 参考框架的 20.5 mm 差异；头部相机安装沿用升降版。支架是否与实物一致尚未确认。轮中心取原始 CAD 坐标，半径 76.12 mm；用户的 504.1 mm 轮距与 280 mm 中心距不能同时视为等边轮心三角形的精确尺寸，应实际测量轮心位置。

## 当前任务

300×240×220 mm、1 kg 箱子，从 0.75 m 高 Pick_A 搬到同高 Place_A；箱子预置在供料台靠通道的可达边缘。240 mm 短边超过约 155 mm 单夹爪开口，采用微张双夹爪从左右两侧摩擦夹抱。每侧主动指保持约 0.25 rad（14.3°）张角，让两指接触点分开；双臂向内夹抱，夹爪本身不环抱整个箱子。没有吸附或绑定约束。载箱平移指令 0.16 m/s，双臂抬箱高度增量指令 0.16 m。放置时按桌沿测距停在约 0.22 m；底盘和桌子几何变化时必须重新检查这一接近距离。展开姿态使用固定立柱专用 IK 分支，连续路径拒绝超过 0.35 rad 的单步关节跳变，并按关节限速延长执行时间；这些检查不等价于碰撞规划。双臂命令限速 0.6 rad/s；这只是当前仿真配置，不是硬件批准参数。

```bash
./scripts/simctl stop
python3 scripts/build_fixed_urdf.py
./scripts/simctl build-robot
./scripts/simctl test
./scripts/simctl transport --manipulation-test
./scripts/simctl transport
./scripts/simctl console
```

全流程验收要求放箱后箱体完整落在目标台内、接触支撑、释放、稳定保持，再后退且不再接触箱子。`--manipulation-test` 只验证夹起与放回，不计完整搬运成功。每次运行覆盖同名报告，重复实验请逐次归档。

## 真机接入还缺什么

当前 `TransportRuntime` 是 Isaac 专用适配器，直接使用 USD 驱动、PhysX 接触、射线传感器和仿真状态监控，不能直接连接机器人执行。尚无真实 SDK/ROS 2 驱动、急停反馈或硬件 watchdog。不要把 `transport_telemetry.jsonl` 当成可直接回放的真机轨迹。

需要实物固定肩高、相机内外参、箱重/尺寸/摩擦材料、工位位置和桌高，以及底盘与双臂 SDK 或 ROS 2 的接口定义。确认指令单位、关节名称/正负号、反馈频率、超时停止和急停状态后，才能实现硬件适配器。任务中的仿真接触、箱体离台和支撑确认也必须替换为真实视觉/力反馈；不能仅凭动作计时就判定抓稳并松手。

当前 SLAM 使用额外的仿真 2D 激光（并未证明真机有雷达），箱体定位使用理想实例标签的射线深度。实机只有三相机时需接入 RGB-D 定位/检测及相应定位方案，重新评估导航。内部自碰撞当前关闭，数值 IK 也不做全身避碰；真机执行前需要按实际碰撞模型检查轨迹。摩擦参数和驱动增益是调试值，不能移植为已标定的力控制参数。

## 如何量化 gap

先固定硬件安装、箱子和桌面条件，分三步收集相同任务的数据：空载底盘前后/横移/原地旋转，底盘静止的夹起放回，最后完整搬运。首次实机执行应由具备急停操作能力的现场人员按硬件允许的低速、低负载范围进行，验证反馈和超时停止；具体范围由实测能力决定。

仿真报告记录模型、质量、成功状态、`duration_s`、`placement_error_m`（成功时箱心到目标桌中心的平面距离）、`joint_tracking_rmse_rad`（10 Hz 采样，14 臂关节和 2 个主动夹爪的发送目标与反馈误差）、`localization_final_error`（评估真值与定位估计的平面距离）、运输阶段的最大箱体倾角和最低箱体底边高度。控制只使用估计位姿；日志中 `evaluation_*` 明确为评估真值，不得作为真机控制输入。

保存每次仿真的 `transport_report.json` 和 `transport_telemetry.jsonl`。真机按相同定义生成 JSON 摘要，如：

```json
{"passed": false, "duration_s": 150.2, "placement_error_m": null,
 "joint_tracking_rmse_rad": 0.025, "localization_final_error": null}
```

上面只是格式示例，不是真机数据。没有外部定位基准时定位误差保持 null，失败时放置误差保持 null；不要填零。实际测量应统一坐标、关节顺序、时间区间和放置成功标准。至少做多次重复实验，包含失败样本，并记录每轮失败阶段与原因。不同箱重、摩擦和速度的试验分组比较，不能混算。

```bash
python3 scripts/compare_sim_real.py \
  --sim outputs/trials/sim_*.json \
  --real outputs/trials/real_*.json \
  --output outputs/sim_real_gap.json
```

工具输出成功率、各项已测样本数和均值，以及 real − sim 差值；缺失数据保留 null。它不凭空生成真机结果，也不把单次成功当作低 gap 或对差异进行统计显著性判断。
