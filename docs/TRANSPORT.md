> 当前客户演示已切换为升降版地面三箱任务，入口与配置见 [CUSTOMER_DEMO.md](CUSTOMER_DEMO.md)。本文保留原单箱/固定立柱方案说明。

# SLAM + IK 搬箱

启动命令：

```bash
./scripts/simctl test
./scripts/simctl transport --manipulation-test  # 停靠区域起步：验证抓起与放回
./scripts/simctl transport --navigation-test    # 验证两工位导航
./scripts/simctl transport                     # 完整搬运，无界面
./scripts/simctl console                       # 四画面操作台，点击 START 才开始
./scripts/simctl transport --gui               # 同上
```

`transport` 会停止本项目已有仿真进程，避免两个 Isaac 进程争用缓存。关闭实时窗口或执行 `./scripts/simctl stop` 可停止演示；运行时可随时点击 RESET 复原。操作台打开后暂停物理仿真，等待 START；不会自动执行搬运。START 每次只执行一轮，完成或失败后需 RESET 才能重新开始。RESET 会中断动作、恢复底盘/关节/箱子的初始状态，双臂回到默认姿态，同时重新初始化 SLAM、接触监听和任务判定；复原后仍等待 START。修改只写入 USD 会话层，不改变原始机器人、车间或常规 GUI 的初始场景。任务开始时，将原来的 300×240×220 mm、1 kg 箱子放到供料台靠通道的边缘；桌面中央超出当前双臂的实用可达范围。完整任务从原始机器人出生点开始，使用固定立柱，不执行升降动作。

流程：在线建图 → 导航至供料工位 → 在桌外展开双臂 → 停靠 → 深度观测/视觉微调 → 双臂 IK 夹抱 → 验证离台 → 全向底盘运输 → 承托台停靠 → 降箱 → 检查桌面接触 → 松开 → 连续稳定判定 → 向后退约 0.5 m → 停止。退让距离见 `manipulation.retreat_distance`；后退采用 0.1 m/s、5 秒的开环指令，不做距离反馈或校准。

## 模块

- `scripts/lidar_slam.py`：编码器里程计预测、粗到细扫描匹配、log-odds 占据地图、膨胀障碍和 A*。规划失败、连续扫描匹配失败和前方距离过近时停止。
- `scripts/arm_ik.py`：直接读取当前 URDF 的链、轴、安装变换与限位，使用有界数值优化求 TCP 的位置/姿态 IK。
- `scripts/box_perception.py`：已知尺寸箱体的深度拟合，处理箱顶不可见、视野裁切和观测不足。
- `scripts/transport_runtime.py`：Isaac 传感器适配、动作执行、接触监听、地图与结果输出。底盘只下发三轮速度目标，机械臂只下发关节驱动目标。
- `scripts/operator_console.py`：2×2 实时渲染视窗：头部 D455、左右腕部 EB435i、固定全局俯视（置于顶梁下方以避免遮挡）。机载视窗绑定 URDF 光学坐标系，随底盘/手腕运动；ROS 光学轴转换为 USD 相机轴。机载显示为 640×360 预览（全局俯视为 640×480），采用标称视场角，无畸变/标定模型。多视窗使用 RaytracedLighting + TAA，避免小尺寸预览依赖 DLSS 缩放。腕部初始折叠姿态可能看到自身或地面，属于实际安装视角。
- `scripts/run_transport.py`：启动、显式开始/复原和失败处理。
- `scripts/transport_task.py`：可重复执行的顺序任务；原始箱体尺寸超过单夹爪开口，因此采用双臂微张夹爪从两侧摩擦夹抱。无吸附、焊接或父子绑定抓取。
- `config/transport_config.json`：扫描、导航、停靠与操作参数。

## 传感器与适用范围

当前增加**仿真二维激光**（0.71 m 高、240 束、5 Hz），用 PhysX 场景射线生成测距；这不意味着实物机器人已有激光雷达。SLAM 算法只接收扫描和轮速编码器，不读取仿真机器人位姿或预先加载车间碰撞清单。地图坐标由配置的初始位姿锚定，任务工位坐标是已知先验。当前动作几何针对南侧 Pick_A → Place_A 两工位；北侧工位需要重新规划进场方向。当前是局部扫描匹配建图，尚无回环检测/位姿图优化，不是 ROS 2 Nav2/slam_toolbox 集成。

近距离箱子定位使用头部相机标称外参/FOV生成的射线深度和仿真实例标签，并使用已知箱体尺寸。激光/射线深度使用理想化自体过滤。它处理箱顶不在视野内和部分边缘截断的情况；这仍是理想分割的仿真感知，没有实现真实 RGB 检测网络、相机噪声模型或纯视觉 SLAM。仿真真值仅在传感器生成、结果评估、安全监控和展示相机中使用。

夹爪摩擦系数是仿真估计；保留原始力矩限位。当前机器人资产仍关闭内部自碰撞，不能据此宣称任意关节轨迹对实机安全。任务不具有动态人群避障、未知物品识别或任意尺寸箱子的通用抓取能力。

## 验收输出

- `outputs/transport_report.json`：完整搬运的结果；只有释放后完整箱体位于目标桌面内、有目标接触且稳定保持规定时间，并完成后退、无机器人接触，才成功。报告包含名义指令距离 `retreat_nominal_distance` 和 `robot_box_contact`。
- `outputs/transport_manipulation_report.json`、`transport_navigation_report.json`：两个局部测试，明确区别于完整搬运成功。
- `outputs/slam_map.png`、`slam_map.npz`：扫描建图结果、分辨率、原点与估计轨迹。
- `outputs/transport_*.png`：抓取、抬升、目标工位、完成或失败截图。

失败会停止底盘、保留当前机械臂目标，输出原因，不能把到达目标附近或执行完指令视为搬运成功。

## 固定立柱验证与真机对比

当前已切换为固定立柱模型，升降版历史结果不能当作本模型验证。最新本机结果见 `outputs/transport_report.json`，模型由报告的 `source_urdf` 标识。报告同时记录关节跟踪误差、运行时间和放置误差，10 Hz 指令/反馈见 `outputs/transport_telemetry.jsonl`。真机缺失接口、测量定义和重复实验比较命令见 [SIM_TO_REAL.md](SIM_TO_REAL.md)。

2026-09-18 固定立柱版：30 项单元测试及局部抓放通过，完整 10 m 搬运通过，仿真耗时约 155 s，最终稳定放箱并脱离接触。详细数值和适用边界见 [验证记录](../VALIDATION.md)。本次未重新运行 GUI 自动回归。
