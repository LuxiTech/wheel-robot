> 当前客户演示已切换为升降版地面三箱任务，入口与配置见 [CUSTOMER_DEMO.md](CUSTOMER_DEMO.md)。本文保留原单箱/固定立柱方案说明。

# R680 三全向轮底盘 + 固定立柱 + OpenArm 2.0

## 使用

现有工业场景已引用 `assets/robots/generated/robot.usdc`。启动：

```bash
./scripts/simctl gui
```

默认聚焦机器人，新增 **R680 + OpenArm controls** 面板：

面板停靠在窗口右侧；向下滚动可查看右臂与右夹爪。底部 **Wheel Robot Controls** 标签页提供暂停、复位和场景视角按钮。

- Forward / Backward：机器人本体 X 方向前后移动。
- Left / Right：机器人本体 Y 方向左右横移。
- Yaw + / Yaw -：逆/顺时针转动。
- 方向按钮每次给出 0.8 秒命令，含加速度限制；默认平移 0.25 m/s、偏航 0.45 rad/s。
- STOP BASE：立即将轮速度目标设为零；这是仿真命令停止，不是实机急停接口。
- 左右臂 J1–J7 滑条：单位为度，范围取自 URDF，命令按 0.6 rad/s 渐变。
- Open / Close：分别控制左右夹爪，另一手指由 PhysX mimic 约束联动。
- 默认无升降滑条；仅切回升降版模型时显示。
- Robot home：双臂回到默认姿态，同时停止底盘。
- Robot view：重新聚焦机器人当前位置。
- 原有面板的 Reset scene：停止并重新启动物理时间线，恢复机器人和箱子初始状态。

方向速度都是 **base_link 坐标系**，并非固定世界坐标。绕 Z 转动后，前进方向也随车体转动。

重新生成和测试（先停止 GUI）：

```bash
./scripts/simctl stop
./scripts/simctl build-robot                 # 从原始 URDF 重新导入并配置物理
./scripts/simctl build-robot --reuse-import  # 仅重做物理/驱动，复用已导入网格
./scripts/simctl build-scene                 # 修改场景配置后重新装配
./scripts/simctl robot-test                  # 实际 PhysX 机器人回归与截图
./scripts/simctl smoke                       # 保留原有箱子落台回归
./scripts/simctl test                        # 运动学与任务判定单元测试
```

`robot-test` 将报告保存到 `outputs/robot_sim_report.json`，机器人图片保存到 `outputs/robot_preview.png`。它会使用测试专用临时障碍验证环境碰撞；不会将障碍写入主场景。

## 输入与输出

当前使用 `assets/robots/r680_openarm/r680_fixed_openarm_sim.urdf`，由原始无升降 URDF 和升降版三相机合成。运行 `python3 scripts/build_fixed_urdf.py` 可重建。原始资产保留，详细安装关系见 [SIM_TO_REAL.md](SIM_TO_REAL.md)。

| 文件 | 内容 |
|---|---|
| `config/robot_config.json` | 驱动增益、速度/加速度上限、滚子近似参数、默认姿态与参数来源 |
| `assets/robots/generated/imported/` | Isaac 官方 URDF importer 输出，保留可追溯的导入资产 |
| `assets/robots/generated/robot.usdc` | 工业场景引用的最终机器人资产 |
| `assets/robots/generated/robot_metadata.json` | 轮几何、驱动关节、mimic、质量与坐标框架映射 |
| `scripts/import_robot.py` | 固定关节合并与 URDF 导入 |
| `scripts/robot_asset.py` | PhysX 滚子、mimic、增益与参考框架装配 |
| `scripts/robot_control.py` | 全向轮逆运动学、限速、加速度限制和关节目标接口 |
| `scripts/robot_ui.py` | GUI 手动控制面板 |
| `scripts/test_robot_sim.py` | 真实接触驱动、双臂、夹爪、复位与障碍测试 |

最终资产默认 prim 为 `/Robot`，场景路径为 `/World/Robot`。固定关节的质量与惯量由 importer 合并至所属活动刚体。固定参考框架按原始变换重建，具体路径见 metadata 的 `frames`，不会重复增加质量。`openarm_mount_link` 与底盘刚性连接，双臂安装在原始 `openarm_body_link0` 上。肩轴离地 1.00312 m。头部相机固定，腕部相机随手腕运动。

## 默认姿态

双臂默认收拢：左/右 J2 为 −0.6/+0.6 rad，双侧 J4 为 2.1 rad，用于紧凑的启动姿态。该初始姿态写入 USD 刚体变换，驱动目标与之保持一致。GUI 通过 World 和显式关节默认状态初始化/复位，采用与物理测试一致的 TGS CPU 求解器。固定立柱版关节求解使用 64 次位置迭代、16 次速度迭代；这用于降低接近限位姿态下的数值跟踪误差，未提高电机 effort 限值。


## 物理实现

三轮径向轴分别接近 0°、120°、240°；无独立转向关节。使用 URDF 轮心坐标和半径建立每轮的运动学矩阵：

```text
滚动方向 t_i = 轮轴 a_i × [0,0,1]
轮速度 qdot_i = (t_ix*vx + t_iy*vy + (-t_ix*y_i+t_iy*x_i)*wz) / r_i
```

命令限幅后只施加 **三个轮关节速度目标**，不设置根刚体位置或速度来制造底盘运动。底盘确实依靠电机、接触和摩擦移动，能与场景静态物体碰撞。

普通圆柱碰撞已关闭，替换为每轮双排、每排 12 个被动滚子，共 72 个无驱动 revolute 关节。每个滚子以切向 capsule 近似，滚动自由度允许横向运动。原始轮外观不变，滚子代理只用于碰撞。

每轮总质量仍为 1 kg，分成 0.7 kg 轮毂和 0.3 kg 滚子。新滚子/轮毂惯量为近似值，不代表已识别的真实惯量。全车质量为 **59.562 kg**，按固定立柱和三相机 URDF 所有惯性质量求和校验。

独立驱动数为 **19**：3 轮 + 14 臂 + 2 夹爪。另有 2 个 mimic 手指和 72 个被动滚子，因此 PhysX 关节列表中显示 **93 个自由度**；这不意味着实机有 93 个电机。

Isaac 6 导入器使用 Newton mimic 描述。此项目额外为两侧从动手指创建 `PhysxMimicJointAPI`，并移除其独立位置驱动，避免两个控制器与联动约束互相对抗。[官方迁移说明](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/urdf_mjcf_importer_exporter_pipeline.html)

## 参数取舍与当前边界

- **几何以实际 URDF 为准**：轮径 152.24 mm，轮心半径约 280.48 mm；相邻轮心距离约 485.8 mm。文本的 504.1 mm 与这些坐标不一致，本版未按该值缩放网格。应结合实机测量确认“轮距”的定义。
- **限位沿用 URDF**：J1–J7 的位置、effort、velocity，及夹爪 ±45° 范围均保留。夹爪右侧指关节为负角开口，mimic multiplier=1。
- **保守控制**：平移/偏航上限 1 m/s、1.5 rad/s来自用户估计；加速度限制、12 N·m 轮电机上限和 PD 增益均为仿真调参值。机械臂/夹爪 maxForce 不超过 URDF 的 effort。速度上限并不等于经过测试的实机安全速度。
- **尚未验证负载**：4.1/6.0 kg 额定/峰值含末端执行器，不能直接作为额外箱子质量；此项目没有做负载认证、指尖夹持力标定或真机控制。
- **自碰撞暂时关闭**：保留机器人与地面、箱子和设施的碰撞；本版未校准各个简化碰撞体的自碰撞过滤。GUI 滑条允许的关节范围不保证姿态无自碰撞，不应据此直接下发真机。
- **滚子是工程近似**：轮上实际滚子尺寸、摩擦、轴承阻力和电机扭矩需要实测校准，当前验证为平坦地面低速运行。
- **单夹爪无法直接夹住当前箱体**：默认箱体为 300×240×220 mm，夹爪最大开口估计 155 mm。自动搬运入口使用双臂微张夹爪从两侧摩擦夹抱；仍不支持用单夹爪直接捏住整箱。
- **指定工位搬运**：新增 [SLAM + IK 搬箱入口](TRANSPORT.md)，使用虚拟激光、深度观测和双臂夹抱；未接 ROS 2/MoveIt，不能替代通用导航、感知和抓取系统。
- **任务判定仍保持未知释放不成功**：机器人存在时，常规环境运行不会假设它已释放箱子；实际抓取控制器仍需提供释放状态。

## Python 控制接口

在 Isaac 已加载主场景后创建控制器，在每个物理步调用 `update(dt)`：

```python
from robot_control import RobotControl
controller = RobotControl(stage)
controller.set_twist(0.2, 0.0, 0.0)             # 本体前进，m/s、rad/s
controller.set_twist(0.0, 0.2, 0.0, duration=1) # 横移命令，1秒后减速
controller.set_joint('openarm_left_joint4', 0.6) # 臂关节目标使用 rad
controller.gripper('left', True)                # 左夹爪打开
controller.update(dt)
controller.stop()                              # 停止底盘目标
```

每次方向按钮命令超时后按配置减速；直接 `stop()` 立即清零轮速度目标，实际车体仍由物理求解器减速。
