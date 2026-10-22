# Wheel Robot：真机拆码垛开发

项目按 **导航 → 视觉关键点生成 → 运动控制** 三个模块组织，通过任务状态机协调。当前保留了 RGB-D 接收、检测/分割、箱体位姿、抓取规则、URDF 运动学、联合 IK、轨迹规划和 SDK 状态映射；导航与真机执行器尚待接入。

**目前可以运行只读诊断、离线感知/规划和逻辑测试，不能直接执行真机拆码垛。** SDK 子模块固定在 `ba0c7320cd1499995cfc614e83c0c3963343e15a`。本次结构重构没有发送设备指令。

## 目录

```text
wheel-robot/
├── ros2_ws/src/
│   ├── wheel_grasp_core/
│   │   ├── wheel_grasp/
│   │   │   ├── main.py              # 初始化适配器、启动任务、保存结果
│   │   │   ├── contracts.py         # 三个模块的数据及异步接口
│   │   │   ├── task/                # 工序状态机、超时与取消
│   │   │   ├── navigation/          # 预留：Nav2 + AprilTag 工位对接
│   │   │   ├── vision/              # RGB-D → 箱体位姿 → 双手抓取目标
│   │   │   ├── motion/              # FK/IK、碰撞、轨迹、SDK 映射与执行接口
│   │   │   └── cli.py               # 只读采集、感知和离线规划命令
│   │   └── launch/                 # 实测关节与 URDF TF
│   └── wheel_grasp_interfaces/      # ROS 消息/action 契约
├── config/
│   ├── task.yaml                   # 单箱任务及阶段超时
│   ├── navigation/                 # 地图工位、Tag 与对接位姿配置工作目录
│   └── grasp/                      # 模型锁、SDK 锁、视觉/机器人参数
├── tools/robot                     # 统一应用入口
├── tools/grasp/                    # 模型下载、环境、诊断和离线工具
├── docker/                         # SDK/ROS 和 GPU 推理独立镜像
├── compose.grasp.yaml
├── tests/grasp/                    # 纯逻辑、算法及任务接口测试
├── docs/ARCHITECTURE.md             # 模块职责、执行流程、后续开发边界
├── docs/MIGRATION.md                # 旧文件 → 新文件、保留和归档清单
└── third_party/                    # 固定 SDK 子模块及外部模型源码
```

保留 `wheel_grasp_core` ROS 包名和 `wheel_grasp` Python 包名，内部模块已经重新划分；不再维护一套平铺旧模块作为兼容副本。

## 环境隔离与本地验证

轻量算法和任务测试使用项目虚拟环境；ROS/SDK 使用独立容器，GPU 模型推理使用另一独立容器。宿主无需安装 ROS、PyTorch 或 CUDA Toolkit。ROS Humble 使用其容器里的 Python，不能复用宿主 Python 3.14 的虚拟环境。

```bash
git submodule update --init --recursive
python3 -m venv .venv
.venv/bin/python -m pip install -r tools/grasp/requirements.txt
# 统一入口通过项目源码路径运行，无需先构建 ROS
tools/robot status
tools/robot workflow
.venv/bin/python -m pytest -q
```

已经存在 `.venv` 时直接复用。若使用 uv，可执行 `UV_CACHE_DIR="$PWD/.cache/uv" uv pip install --python .venv/bin/python -r tools/grasp/requirements.txt`。

```bash
tools/robot run
```

当前 `run` 会记录 `NOT_READY` 并以退出码 **2** 结束，因为导航、同步观测源和硬件执行器未配置；它不会自动启用仿真、发送运动指令或伪造成功。结果位于 `outputs/robot/<运行ID>/result.json`。`workflow` 只展示工序，不代表执行或仿真。

测试使用明确的替身验证模块协调，不代表硬件或模型推理通过。外部 URDF、录制数据缺失时对应测试会跳过。

## 视觉及运动算法入口

“视觉关键点”目前采用已有的检测/分割与位姿估计，再按箱沿几何生成左右手 **4×4 抓取位姿及预抓取位姿**；不是新增的关键点神经网络，也不只输出像素坐标。

```bash
tools/grasp/grasp doctor
# 配置真实网关后，执行只读 GET 采集
mkdir -p outputs/observations
tools/grasp/grasp capture --url http://ROBOT_IP:18030 \
  --output outputs/observations/box.npz
tools/grasp/grasp inspect outputs/observations/box.npz
# 其余参数见各命令 --help
tools/grasp/grasp perceive --help
tools/grasp/grasp ik --help
```

先核对 `config/grasp/runtime.yaml`：机器人 URDF、相机、SDK URL、TCP、升降映射及实物箱模型。当前示例 TCP 尚未真机验证，`lift_mapping: null` 表示未标定，不得自动填零。

箱体资源改为独立于仿真的路径：

```text
assets/objects/crate/model.obj
assets/objects/crate/colliders.json
```

`model.obj` 和碰撞几何应对应实物箱；已有数据不会自动搬移或替换。如果提供了实测长方体组合定义，可运行 `tools/grasp/build_crate_mesh.py` 生成网格。该工具保留，但不再依赖 `build-relay` 生成资产。

模型镜像、权重和引擎准备见 [环境说明](tools/grasp/README.md)。SDK/ROS 接口及构建见 [核心包说明](ros2_ws/src/wheel_grasp_core/README.md)。

## 后续接入

1. 在 `navigation/` 实现地图导航、AprilTag 对接、到位/静止反馈及停止确认。
2. 为 `vision/PipelineVision` 提供同步 RGB-D/实测关节源，保留采集时间、frame、pair_id 和目标身份。
3. 在 `motion/executor.py` 实现 SDK 控制权仲裁、双臂轨迹、夹爪、升降、抓持/试抬/支撑反馈；底盘导航控制由导航模块管理。
4. 在 `main.create_application()` 装配实际适配器，再按只读观测、预抓取、单箱试抬、单箱搬放、多箱工序依次验收。

`main` 只负责装配和启动；业务顺序在任务状态机中，持续的传感器和电机控制由独立节点/进程承担。详见 [架构说明](docs/ARCHITECTURE.md)。
