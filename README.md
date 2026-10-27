# Wheel Robot

双臂轮式机器人的拆码垛工程，采用“核心算法库 → 运行适配 → 任务编排 → 网页管理”的分层架构。
当前版本为 0.3.0。原单体 `wheel_grasp_core` 已拆分，旧 Python import 不再提供兼容副本。

**当前支持只读诊断、离线感知/规划、任务逻辑测试及双 SDK 状态网页与可显式开启的 A/B 导航。导航已提供陆图 API 适配；完整抓取执行闭环尚未接入，不能直接执行自主拆码垛。**

## 项目结构

```text
wheel-robot/
├── packages/                         # 不依赖 ROS 的 Python 核心库
│   ├── wheel_task_core/              # 契约、状态机、配置与坐标值校验
│   ├── wheel_perception_core/        # RGB-D、检测分割、位姿、抓取候选
│   └── wheel_motion_core/            # FK、IK、碰撞、轨迹
├── ros2_ws/src/
│   ├── wheel_interfaces/             # ROS 消息与 action 定义
│   ├── wheel_description/            # URDF/网格资源边界；真实资产待提供
│   ├── wheel_sdk_bridge/             # SDK 反馈映射、实测关节/RGB-D ROS 接收
│   ├── wheel_navigation/             # 陆图 unstacking 地图 A/B 导航
│   ├── wheel_perception/             # 同步采集契约及视觉任务适配
│   ├── wheel_planning/               # 离线规划服务封装
│   ├── wheel_execution/              # 真机操作执行边界；待接入
│   ├── wheel_runtime/                # 装配、CLI、任务结果记录
│   ├── wheel_gateway/                # 对管理端提供只读查询
│   └── wheel_bringup/                # ROS 启动与参数装配
├── apps/
│   ├── management_api/               # FastAPI 查询与显式导航控制
│   └── management_web/               # 独立 HTML/CSS/JS 管理页面
├── config/
│   ├── robots/                      # 机器人/相机/TCP/推理引擎配置
│   ├── stations/                    # 工位、Tag 与对接位姿
│   ├── recipes/                     # 箱体模型、几何抓取规则与 CAD 登记
│   └── deployments/                 # 任务参数、依赖锁与环境验证基线
├── assets/                          # 外部资产及 CAD 原件
├── deployment/                      # Dockerfile、Compose 与部署说明
├── tools/                           # 统一入口、模型与诊断工具
├── tests/                           # 算法、任务、边界与管理接口测试
└── docs/                            # 架构、迁移、接口和验证说明
```

每个受维护的源代码/配置/部署目录均包含 `README.md`，说明本目录的文件、职责、依赖和实现状态。
`.venv`、`.runtime`、缓存、历史输出和外部 `third_party` 不属于逐目录文档维护范围。

## 分层约束

- `wheel_task_core` 定义业务契约并通过注入的 Navigation/Vision/Motion 接口执行工序，不导入 ROS、SDK、Web 或数据库。
- 两个算法库只依赖核心值类型及各自数学/图像库；感知与运动算法之间没有相互 import。
- ROS 工作包是独立安装边界，已有算法通过适配器调用；新目录不等于已实现对应 ROS action server。
- `wheel_runtime` 是任务装配入口；网页、网关不启动任务控制循环，不直接发送设备指令。
- 厂商 SDK 实机部署在开发板，本地只读副本位于 device/；本工程只调用公开 API，不导入、同步或修改两套 SDK 的源码。

## 本地运行与回归

在仓库根目录执行；已存在 `.venv` 时直接复用，不向系统 Python 安装依赖。

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r tools/grasp/requirements.txt
tools/robot status
tools/robot workflow
.venv/bin/python -m pytest -q -rs
```

工具和测试自动发现 `packages/`、`ros2_ws/src/` 中的源码，无需先构建 ROS。
三个核心库也可独立安装：

```bash
.venv/bin/python -m pip install -e packages/wheel_task_core \
  -e packages/wheel_perception_core -e packages/wheel_motion_core
```

本机现有 `.venv` 不含 pip，已复用原算法依赖并安装新工作包，无需重复初始化；若需更新安装，可使用管理环境的 pip：
`.runtime/management/venv/bin/python -m pip --python .venv/bin/python install ...`。
源码入口 `tools/robot`、`tools/grasp/grasp` 始终使用当前工作区；安装版 console script 使用已安装版本，修改代码后需要重新安装。

`tools/robot run` 在默认适配器未接入时记录 `NOT_READY`，返回退出码 2，结果保存到 `outputs/robot/<运行ID>/result.json`。
`workflow` 只描述工序；`status` 查询默认装配状态，不代表已连接机器人。缺少外部 URDF、历史采集文件的测试明确跳过。

## 网页后台

提供双 SDK 连接、unstacking 地图 A/B 点位及导航状态，同时保留默认模块、单箱工序、配置目录和历史结果。默认仅观测；现场准备后使用 `tools/management --enable-navigation` 开启 A/B/停止控制。尚无账号权限、配置写入或完整拆码垛调度。
前后端分别位于 `apps/management_api`、`apps/management_web`，后端通过 `wheel_gateway` 查询运行层。

```bash
python3 -m venv .runtime/management/venv
.runtime/management/venv/bin/python -m pip install \
  -r tools/grasp/requirements.txt -e apps/management_api
tools/management
```

浏览器访问 `http://127.0.0.1:8088`。默认只监听回环地址；部署账号权限、TLS 和写入控制后再作为共享管理服务使用。
Windows 工作站可用 `ssh -N -L 8088:127.0.0.1:8088 luxi@192.168.1.47` 转发后访问。
管理后台独立环境不会改变 `.runtime/vision/venv` 的 GPU 依赖。

## 配置与真机视觉

- 默认机器人配置：`config/robots/runtime.yaml`；本机 GPU 诊断：`config/robots/runtime.live.yaml`。
- 机器人配置的 `recipe` 相对于当前 YAML 所在目录解析；箱体规则集中于 `config/recipes/crate.example.yaml`，机器人字段可覆盖配方字段。
- 资产路径仍相对于仓库根目录，不随配方文件位置改变。
- 依赖、模型权重和原生位姿源码锁位于 `config/deployments/*lock.json`。
- 工位示例位于 `config/stations/stations.example.yaml`，未测量位置保留 `null`。

```bash
tools/grasp/grasp doctor
tools/grasp/grasp perceive --help
tools/grasp/grasp ik --help
.venv/bin/python tools/grasp/perceive_live.py --capture-only
```

检测/分割/位姿推理继续使用 `.runtime/vision/venv`。模型准备与校验见 [工具说明](tools/grasp/README.md)，
真实图像、CAD 和候选点预览见 [视觉诊断说明](docs/LIVE_VISION.md)。本次拆包不重新下载模型或重建 GPU 引擎。

## 已实现能力与待接入部分

| 部分 | 当前状态 |
| --- | --- |
| RGB-D 协议、几何预处理、检测/分割/位姿封装 | 已迁移，保留质量门限与诊断标记 |
| FK、双臂及升降联合 IK、保守包围盒碰撞、轨迹 | 已迁移，输出仍不可直接执行 |
| 单箱任务状态机、超时、取消、可能持载处理 | 已迁移，有替身回归测试 |
| SDK 反馈映射、ROS 实测关节与 RGB-D 接收 | 已迁移；ROS 接收依赖开发板 `luxi_interfaces` |
| 规划适配、网关、网页查询 | 已实现离线规划封装与本地只读管理 |
| 导航 | 已实现陆图命名点 API 适配，默认不启用；实机运动待现场验收 |
| 同步实机观测源、真机操作执行 | 未接入，默认 `ready=False` |
| 多箱调度、任务持久化调度服务、用户权限、配置发布 | 后续开发范围 |

当前 CAD 为约 445×310×180 mm，源开口朝 -Z；历史示例规则仍为 400 mm 宽、220 mm 高。
独立相机预览脚本翻转 CAD 并从模型推导尺寸，不能将其结果直接当作基座系执行目标。
TCP、相机外参、深度度量语义、箱沿夹取偏移及升降映射仍需实测验证；所有离线计划与预览保持 `executable=False`。

已有权重、OBJ、引擎、虚拟环境和历史输出继续原地保留。CAD 原件 `assets/objects/crate/source/box.step` 已被 Git 跟踪；
其他大体积资产默认忽略，仅资产 README 进入版本管理。

## 部署与开发说明

历史 ROS/GPU Docker 构建入口迁入 `deployment/`，路径已更新；需在目标环境重新构建验证。
不复用宿主 Python 3.14 的虚拟环境到 ROS Humble 容器，也不将历史 GPU 镜像当作本机原生环境。

- [架构与依赖方向](docs/ARCHITECTURE.md)
- [迁移对照与兼容边界](docs/MIGRATION.md)
- [接口与当前管理范围](docs/INTERFACES.md)
- [验证记录与复现方法](docs/VALIDATION.md)
- [双 SDK 连接与 A/B 导航说明](ros2_ws/src/wheel_navigation/SDK_INTEGRATION_README.md)
