# 双机接力码垛环境

当前场景已将源垛、单箱朝向和目标托盘绕各自中心旋转 90°；世界坐标下每层为三排两列，垛占地 0.6×1.2 m，托盘占地 0.82×1.32 m。配置中的行列和尺寸仍表示托盘局部坐标，通过 `pallet.yaw_degrees=90` 转换。历史单箱/技能通过记录属于旋转前布局，不能作为新布局的自动接力验收。新布局已适配初始位姿、选箱目标、侧向接近、抓点和推入方向；完整物理接力仍需重新验收。


本阶段搭建独立环境：Robot A 从源托盘取料到中转桌，Robot B 从中转桌取料到目标托盘。
本文说明基础物理环境与手动调试。自动取放技能、导航接力及任务调度已另行接入，运行入口和验收状态见 [自动接力说明](RELAY_DEMO.md)；力闭环尚未实现。
旧的纸箱搬运场景与原机器人配置保持独立。

## 启动

```bash
./scripts/simctl relay --gui
./scripts/simctl relay-test
./scripts/simctl build-relay
```

GUI 提供 A/B 两机各自左右 AG95 的开合、升降柱高度和环境复位。
`relay --gui` 运行基础物理与手动控制；`relay-demo --gui` 启动自动接力调试。头部/腕部相机保留在机器人资产中，新增的理想实例检测与合成深度位姿拟合不等同于这些相机上的真实 RGB 检测模型。
无界面预览可用 `./scripts/simctl relay --steps 240 --capture`。

## 资产

| 文件 | 内容 |
|---|---|
| `config/relay_config.json` | 双机位置、托盘、中转桌、箱体参数、源垛排列 |
| `config/relay_robot_config.json` | AG95 版机器人驱动与初始姿态 |
| `assets/generated/relay/main.usda` | Isaac 可直接打开的完整环境 |
| `assets/generated/relay/tote.blend` | Blender 可编辑的单个开口周转箱 |
| `assets/generated/relay/relay_environment.blend` | Blender 工位场景（机器人与动态箱在 USD 装配阶段加入） |
| `assets/generated/relay/crate.usda` | 真实刚体、41 个组合盒碰撞，箱口保持开放 |
| `assets/robots/relay/relay_robot.urdf` | 原移动双臂机器人更换左右 AG95 后的 URDF |
| `assets/robots/vendor/dh_ag95/ag95.urdf` | 独立展开的夹爪 URDF |
| `assets/robots/generated/relay/robot.usdc` | PhysX 驱动、联动和全向轮处理后的机器人 |

## 参考和边界

- 视频：https://www.bilibili.com/video/BV1BA756EEVc/ 。箱口夹持结构参考 26–29 秒，退爪参考 42–43 秒。
- 未找到能确认属于视频专用夹具的公开 URDF。采用 DH AG95 连杆双指夹爪作为**替代模型**，不是宣称同款。
- 模型来源：https://github.com/ian-chuang/dh_ag95_gripper_ros2 ，固定提交 `fc4f80fdfb3acae5626df4359aec1401cb71a9a3`。模型包 Apache-2.0 许可及仓库 MIT 许可已分别保存。
- 原厂 ROS 仓库：https://github.com/DH-Robotics/dh_gripper_ros 。替代模型保留上游质量、惯量、关节限位与 STL；去掉 Gazebo 闭环片段，使用 PhysX mimic 联动树。没有额外增加约束闭环。
- 腕部转接盘为仿真近似，不是实机加工图。每台 20 个主动驱动、10 个 mimic、72 个被动轮滚子，总计 102 DOF。
- 箱子为视频风格的参数化重建：400×300×220 mm、1 kg、灰色开口、外筋、箱沿、底部支撑条。尺寸与质量是仿真设定，不是从视频测量所得。
- 托盘 1320×820×130 mm，中转桌面 750 mm；源端每层 2 行×3 列，共 3 层、18 箱，中转桌两个空箱位，目标托盘初始为空。
- 箱体外沿紧贴排列：X/Y 中心距分别为 400/300 mm，层距 220 mm，不额外预留水平或竖直间隙；箱垛外包络 1200×600×660 mm。
- 托盘外观是简化蓝色格纹，承重碰撞为连续甲板；不是叉车插齿/网格穿透测试模型。
- 原机器人内部自碰撞配置仍关闭；箱体、工位和两机器人之间有实际碰撞。抓取路线/自碰撞规划尚未验收。
- 箱体惯量为组合几何体积加权近似，筋条少量重叠，不是精密 CAD 质量参数。

## 验证

`relay-test` 对两个 articulation、配置指定数量的动态箱、真实支撑接触、整垛静置稳定、顶箱中央向下射线穿过箱口、四个夹爪独立闭合/张开、mimic 联动、底盘稳定和复位进行检查。
结果写入 `outputs/relay_environment_report.json`，截图为 `relay_overview.png`、`relay_totes.png`、`relay_gripper.png`。
测试通过仅说明环境和驱动可用，不代表夹持箱沿、带载搬运或完整接力已经通过。

2026-09-20 初版（8 箱）：无界面和桌面 GUI 环境验收均通过，18 项检查全部通过；现有 33 项单元回归通过。两机各 102 DOF，四个主动夹爪闭合目标 0.65 rad 的反馈约为 0.6493–0.6495 rad。

下一阶段在这个环境上接入箱沿接近/下探、两夹爪闭合、试提、落箱卸载、张爪上撤，以及双机中转箱位互斥。

## 中转桌自动推箱

桌面增加橙色推板、直线滑台及两侧低导向条。推板是质量 2 kg 的动态刚体，通过沿 X 的移动关节连接世界；力驱动上限 80 N，推进速度 0.12 m/s，退回速度 0.20 m/s。参数在 `config/relay_config.json` 的 `handoff.pusher`。

程序读取箱体位姿作为理想化区域传感器：左区仅一个箱子、放置居中并静止至少 0.5 秒、右区为空时，推板沿桌面将箱子推到右区，停留 0.5 秒后退回。右区已有箱则等待；推进中检测到另一箱进入右区则暂停，清空后继续。箱子通过真实碰撞和摩擦移动，控制器不修改箱子的位姿。运动状态显示在 GUI 的 `Pusher` 行。

GUI 的 `Demo: load left` / `Demo: load right` 仅用于手动测试上料，会将源垛顶层一只现有箱子直接摆到对应桌面区域；不是机械臂抓取。左区测试按钮上料后即自动推箱。`Reset environment` 恢复 18 箱三层源垛、推板原位和空桌面。自动接力模式隐藏手动上料按钮，并使用 A/B/推板独占互锁，要求推板回原点、机械臂退出后交接控制权。

`relay-test` 同时检查空桌等待、右区占用禁止启动、物理接触推箱、连续运动且箱底不离桌、右区到位、推板回原点及防止重复触发。`relay_handoff.png` 为推箱完成后的中转桌截图。

移动关节及力驱动参考 [NVIDIA Omni Physics joints](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/rigid_bodies_articulations/joints.html)。

2026-09-20 推箱机构验收：26 项环境及推箱检查全部通过，3305 个物理步；右区目标 X=0.34 m，箱子实际 X=0.33737 m，误差约 2.6 mm。确认推板/箱体真实接触、运动连续、箱底保持桌面高度、完成后推板回原点。
