# 三箱演示提速验证（2026-09-18）

- 空载/载箱 0.65/0.35 m/s，返程同步下降升降柱，停靠 0.14 m/s，到位停顿 0.25 s。
- 物理步长保持 1/120 s；演示会话位置/速度求解迭代 32/8。全局主画面默认开启，可切换四路相机，原生 Viewport 保留。
- 完整 GUI 三箱通过：210.100 s 仿真时间，335.007 s 现实时间。旧版仿真耗时 253.892 s，缩短 17.2%。
- 最大载箱倾角 3.284°、最低箱底 0.85237 m，三箱均稳定释放且最终无机器人接触。
- 33 项单元测试、中途 RESET、复位后完整批次及完成后 RESET 通过。
- 报告：`outputs/demo_trials/passed_fast_gui_batch.json`、`outputs/demo_trials/passed_fast_console_controls.json`；截图：`outputs/demo_trials/fast_complete.png`。

以下为提速前验证记录。

---

# 地面三箱客户演示验证（2026-09-18）

当前默认模型改为 `r680_lift_openarm/r680_lift_openarm_sim.urdf`，0.4 m 升降机构、20 个独立驱动，质量约 55.672 kg。固定立柱配置保留于 `config/profiles/robot_fixed.json`。

- 33 项单元测试通过（`outputs/demo_unit_tests.log`），新增连续地面抓取至台面 IK、实际第二箱停靠偏差的轨迹回归、三箱布局与分区边界检查。
- 图形操作台完整地面三箱任务通过（`outputs/demo_report.json`），253.892 s 仿真时间。三个 300×240×220 mm、1 kg 箱子均释放于 0.75 m 工作台的独立区域，连续稳定且最终与机器人无接触。
- 载箱指令 0.25 m/s，空载指令 0.45 m/s；升降仍遵守原模型 0.05 m/s 限速。原关节 effort 限值及 IK 跳变保护保留。
- 运输最大倾角 1.743°，最小箱底高度 0.85698 m。物体仅在初始化/RESET 设置位姿，运行中无绑定抓取或箱体瞬移。
- 操作台回归全部通过：待机不动；第一箱完成、第二箱抓取时中断复原；复原后完整三箱；完成后再次复原。报告为 `outputs/demo_console_test_report.json`，通过结果已归档于 `outputs/demo_trials/`。
- 台面停靠距离改为 0.32 m，TCP 前伸 0.52 m，释放后收手再反馈后退。箱位中心间距 0.55 m、验收区域宽 0.50 m，区域不重叠且包含在实际桌面内。所有箱子持续更新接触和速度反馈，不以已完成计数替代整批验收。

这是固定场景的仿真展示验证，不是随机扰动统计或真机验证。使用与配置见 [客户演示说明](docs/CUSTOMER_DEMO.md)。

---

以下为原固定立柱及早期版本的历史验证记录。

# 固定立柱版本验证（2026-09-18）

默认模型为 `r680_openarm/r680_fixed_openarm_sim.urdf`，原始固定立柱加三个相机，无升降关节。19 个独立驱动、2 个 mimic、72 个被动滚子；URDF 与 USD 总质量一致（约 59.562 kg）。肩轴离地 1.00312 m，尚待实物安装尺寸核对。

- 30 项单元测试通过（`outputs/fixed_unit.log`），覆盖固定模型拓扑/相机资产、连续 IK 分支、三轮运动学、感知、稳定放置判定及 sim/real 数据比较。
- 加强后的局部抓放测试通过（`outputs/transport_manipulation_report.json`）：抬离台面、放回、完整箱体在源台范围内、稳定保持、与机器人脱离接触。
- 完整 Pick_A → Place_A 搬运通过（`outputs/transport_report.json`）：1 kg、300×240×220 mm 箱子，两工位相隔 10 m；完成放稳、释放、名义后退 0.5 m，最终无机器人接触。仿真用时 154.775 s。
- 运输最低箱体底边 0.84894 m，高于 0.75 m 桌面约 98.9 mm；最大运输倾角约 12.67°。关节跟踪 RMSE 0.01423 rad，最终定位平面误差 0.04552 m（仿真真值仅供评估）。这不是实机精度。
- 箱子最终中心约 (5.03470, -3.16133, 0.86000) m。已检查实际仿真截图 `outputs/transport_complete.png`，图中箱子释放在承托台上。
- 机器人 PhysX 回归 通过（`outputs/robot_sim_report.json`）：17 项检查，覆盖前进/横移/偏航、14 臂关节跟踪、夹爪/mimic、静态障碍阻挡及复位；93 个 PhysX 自由度包含被动滚子和 mimic。
- 本次未重新执行四画面 GUI 自动回归。此前 GUI/升降版报告不能当作当前版本 GUI 验收。

关键适配：固定立柱专用展开分支、拒绝 IK 大幅跳变、按关节限速延长路径时间、64/16 次关节求解迭代；双夹爪保持约 0.25 rad 张角，以分开的指端接触抑制箱体转动。抬箱指令增量 0.16 m，放置测距目标 0.22 m，未增加关节 effort 限值。曾出现放置越界，失败报告保存在 `outputs/fixed_trials/failed_placement.json`；没有放宽成功判定。

这是当前配置的一轮完整成功，尚未做随机初态、摩擦/载荷扰动统计或真机验证。真实相机感知、定位传感器、控制驱动、夹持力和内部碰撞检查仍待接入/标定。比较工具及测量定义见 [SIM_TO_REAL.md](docs/SIM_TO_REAL.md)。

---

以下为历史版本记录，包含先前升降版；请按模型和日期区分。

# 本机验证记录

验证日期：2026-09-16。工作目录：`/home/wwd/projects/wheel-robot`。

- 宿主机 Ubuntu 26.04，Docker 29.6.1，Compose 5.3.1。
- NVIDIA 580.178.04，RTX 3090 24 GB；只向 Isaac 容器分配宿主 GPU 1。
- Blender 4.5.3 官方归档 SHA256 校验通过，独立镜像构建通过。
- Isaac Sim 6.0.1 镜像固定到 `sha256:783444c706538aa76cf5126e911ddc5e618779e6105305ad4af4260362a30aa9`。
- Compose 配置、shell 语法、Ruff 检查通过。
- Blender 完成 `.blend` 与 USD 导出；119 个静态碰撞体、一个动态箱子。
- USD 装配、重开、引用解析、米制检查通过。
- 8 项任务判定单元测试通过：正常放稳、无接触、未释放/未知释放、越界、角速度过大、连续保持、复位、NaN。
- PhysX 仿真执行 960 步、8 秒模拟时间，6 项检查全部通过：源台落稳、源台不误判、目标接触、目标放稳、复位回源台、复位清除成功。
- 最后复位后的箱子中心约 `[-4.999999, -3.499999, 0.860000]` 米，符合 0.75 米台高及 0.22 米箱高。
- 已查看 `outputs/isaac_showcase.png`、`outputs/isaac_task_view.png`：模型、箱子、材质、照明和标线可见。Blender 预览另存为 `outputs/blender_preview.png`。
- 测试结束后容器退出，未保留持续占用 GPU 的仿真进程。

详细证据：`outputs/init.log`、`outputs/assemble.log`、`outputs/task-tests.log`、`outputs/smoke.log`、`outputs/simulation_report.json`。

当前边界：未接入实际机器人、移动与抓取控制器；目标台落箱测试使用位置初始化，不代表完成机器人搬运。未接入商业工业设备模型或外部 PBR 贴图。默认以无界面方式运行，不开放 GUI/串流端口。

可选 OmniHub 缓存服务有启动失败日志，随后回退读取本地资产；实际 USD 加载、渲染和 PhysX 验证均通过。性能日志含相机读回和历史变换缓存警告，当前静态截图可用；未验收运动模糊或多相机实时吞吐量。

## 实时窗口补充验证（2026-09-16）

已增加 `compose.gui.yaml`、`scripts/view_scene.py`、`scripts/prepare_display.py` 和 `simctl gui`。

- 在本机 `DISPLAY=:0` 的 Xwayland 桌面成功创建 `Wheel Robot - Industrial Simulation 6.0.1` 窗口。
- 日志出现 `GUI_READY` 和 `GUI_VIEWPORT_CAPTURED`；已查看真实视口截图 `outputs/gui_viewport.png`，确认车间、台面与箱子正常显示。
- 新增暂停/继续、时间线复位、整体/箱子视角控制面板。
- Compose 覆盖文件验证、shell 语法和 Ruff 检查通过。
- 实际容器配置：用户 1000:1000、network=none、privileged=false、仅 GPU 1。
- 显示认证目录权限 700、文件权限 600；未执行 xhost 放宽操作。
- 此次验证后保留 GUI 容器运行，供用户直接操作；关闭窗口或 `./scripts/simctl stop` 可停止。

前文“未开放 GUI”描述的是初始无界面版本；现已支持本机实时窗口，仍未配置 WebRTC 网络串流。

## 实际机器人补充验证（2026-09-16）

已接入 `assets/robots/r680_openarm/` 的原始 URDF 与网格，生成 `assets/robots/generated/robot.usdc`。本节取代前文“未接入实际机器人”的历史描述。

- 13 项单元测试通过：原有任务判定及新增三全向轮运动学、限幅与异常输入检查。
- 17 项 PhysX 机器人检查全部通过，详见 `outputs/robot_sim_report.json` 与 `outputs/robot-test.log`。
- 总质量约 59.150 kg；19 个独立驱动、2 个 mimic 手指、72 个被动滚子。底盘由轮电机与滚子接触驱动，不写入根刚体位置/速度来产生运动。
- 低速测试前进约 0.435 m、横移约 0.424 m、原地偏航约 42.08°，均保持直立。
- 14 个臂关节目标跟踪最大误差约 0.00825 rad；左右夹爪开合与物理 mimic 联动通过。
- 临时静态障碍阻挡底盘，复位恢复初始状态；测试障碍不保存到场景。
- 接入机器人后重新执行原有箱子回归，6 项检查通过，报告 `outputs/simulation_report.json` 中 `robot_loaded=true`。落箱仍是测试初始化，不代表机器人抓取搬运。
- 实时 GUI 已显示机器人、右侧关节/底盘控制面板与底部场景控制标签；已查看 `outputs/robot_gui_window.png` 和 `outputs/robot_preview.png`。Kit 布局恢复后显式停靠控制面板，防止面板被隐藏。
- Ruff、shell 语法、GUI Compose 合并配置检查通过。沿用非 root、无网络、仅 GPU 1 的容器隔离；原始 URDF 与网格保持不变。
- 本次验证结束后已停止 GUI 和测试容器，未保留运行中的仿真进程。

轮距采用 URDF 相邻轮心距离约 485.8 mm，与文本 504.1 mm 的差异待实机确认。滚子及驱动参数为近似值；自碰撞暂时关闭，负载、夹持力及自动抓取未验证。完整参数与边界见 `docs/ROBOT.md`。

## 升降版替换验证（2026-09-17）

本节为当前状态，取代前述无升降机器人数据。

- 输入 `r680_lift_openarm.rar`；保留原始模型，仿真副本将 `lift_joint` 原点 Z 改为 −0.03 m，匹配底板参考以上 106–506 mm 肩高。
- 升降初值为 0 m；双臂收拢姿态写入 USD 初始刚体变换，避免低位零角度下垂穿地导致倾覆。
- 新质量 55.672 kg；20 个独立驱动、2 个 mimic、72 个被动滚子，共 94 个仿真关节自由度。
- 16 项单元测试通过，覆盖新模型网格引用、肩高/限位、升降米制命令/限速/限幅/回零以及已有运动学和任务判定。
- 27 项 PhysX 检查全部通过，包含最低位启动、完整升降往返、肩高、相机随动、底盘、双臂、夹爪、障碍与复位。
- 升降最高实测 0.398416 m、最低 −0.0000011 m；峰值速度 0.0500004 m/s。最高位相对底板肩高 0.504396 m；与几何目标 0.506 m 的差主要来自重力下的驱动弹性。
- 前进约 0.4363 m、横移约 0.4264 m、偏航约 0.7599 rad；均保持直立。
- 已查看新 `outputs/robot_preview.png`，确认升降柱、低位双臂和相机外观正常。

证据：`outputs/build_lift_robot.log`、`outputs/test_lift_robot.log`、`outputs/robot_sim_report.json`。保留此前自碰撞关闭、载荷/抓取未认证等边界；相机目前保留几何和参考框架，未启用图像采集。

原有场景 smoke 回归 6 项通过，960 个物理步；证据 `outputs/smoke_lift_robot.log` 与 `outputs/simulation_report.json`。

GUI 使用与物理回归一致的 World 初始化、TGS CPU 求解器和显式关节默认状态。新窗口已在 DISPLAY=:0 启动，日志出现 `ROBOT_GUI_PANELS_READY`、`GUI_READY`、`GUI_VIEWPORT_CAPTURED`，本次启动无 Invalid PhysX transform/Traceback。已查看 `outputs/gui_viewport.png`，确认机器人保持直立，升降处于最低位；保留窗口运行。证据：`outputs/gui_lift_robot.log`。

## SLAM + 双臂 IK 搬箱验证（2026-09-17）

- 新增在线二维激光扫描匹配建图、轮速里程计、障碍膨胀/A*、URDF 双臂 IK、头部射线深度箱体拟合和顺序搬运任务。
- 24 项单元测试通过：已有机器人/任务判定，以及 IK 可达/不可达/限位/升降、里程计漂移修正、障碍路径、无效扫描、相机视野裁切下的箱体定位。
- 静止工位抓取测试通过：同尺寸 1 kg 箱子由两侧闭合夹爪摩擦夹抱，抬升约 81 mm 后放回供料台。没有创建吸附/固定抓取关节。
- **实时 GUI 完整搬运通过**，用时 168.117 秒仿真时间：原始出生点 → Pick_A → 夹起 → 通道运输 → Place_A → 释放 → 稳定验收。
- 箱子 X 方向位移 9.9628 m，最终中心 `[4.962750434875488, -3.192885160446167, 0.85999995470047]` m，底面与 0.75 m 目标桌面对齐。
- 运输中箱体中心最低 0.9381 m，对应底面距桌面约 78.1 mm；双侧接触监测累计 12102 个物理步。
- 在线 SLAM 累计 821 帧扫描；仿真真值仅用于传感器生成、评估、安全监控和展示，导航未使用真值位姿反馈。评估得到最大位置误差约 12.8 cm，因此末端靠台使用实时测距，源台抓取前使用深度视觉微调。
- 新增虚拟激光，不代表实机具有该传感器；无回环/位姿图优化。箱子深度使用仿真实例标签和已知尺寸，不是已训练的 RGB 检测系统。现有自碰撞关闭、实机载荷未认证等边界保持。
- 搬运前仅在会话层把箱子初始化到供料台靠通道可达位置；任务运行中箱子位姿写入次数为 0，底盘只受轮电机驱动。
- 已查看 `outputs/transport_complete.png` 与 `outputs/slam_map.png`；GUI 留在完成状态，可通过关闭窗口或 `./scripts/simctl stop` 停止。

最终证据：`outputs/transport_report.json`、`outputs/transport_gui.log`、`outputs/autonomy_unit_tests.log`、`outputs/transport_complete.png`。此前停靠保护触发与过严距离容差的调试报告另行保留，不作为最终成功报告。
