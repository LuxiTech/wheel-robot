# Wheel Robot

LuxiTech 轮式双臂机器人的仿真与任务控制代码，基于 Isaac Sim 和 Blender，面向 R680 全向底盘、升降机构和 OpenArm 双臂，包含搬箱、双机接力码垛、固定工位导航及网页控制台。

本仓库只管理业务代码和配套配置、测试、文档。机器人模型和生成资产需单独准备；仅克隆仓库不能直接启动完整仿真。当前默认运行纯仿真；真机 SDK 已固定为子模块，尚未接入真机自主运动执行。

本文对应 **`dev-real` 分支**（复现说明更新于 2026-10-21）。首次部署按“环境准备 → 外部资源准备 → RGB-D 模型环境（按需）→ 测试与运行”的顺序执行；所有命令均在仓库根目录运行。

## 入库范围

| 内容 | 是否上传 | 说明 |
|---|---|---|
| `scripts/` | 是 | 控制、感知、规划、任务状态机、场景生成、运行与诊断脚本；`web/` 为网页控制台 |
| `config/` | 是 | 场景、机器人、任务、导航及传感器参数 |
| `tests/` | 是 | 业务逻辑回归测试，部分需要模型或生成资产 |
| `docker/`、`compose*.yaml` | 是 | 仿真与可选定位服务的构建、启动配置 |
| `README.md`、`docs/`、`VALIDATION.md` | 是 | 使用说明、设计与历史验证记录 |
| `.env.example`、`.gitignore`、`.dockerignore`、`ruff.toml` | 是 | 环境模板与开发配置 |
| `assets/` | 否 | 外部 URDF、网格、模型压缩包、生成的 USD/Blender 文件 |
| `video/`、`outputs/` | 否 | 录屏、截图、日志及运行报告 |
| `ros2_ws/src/`、`tools/grasp/` | 是 | 抓箱核心、ROS 接口、环境与模型工具 |
| `third_party/lift_robot_ros2` | 子模块引用 | Git 固定 SDK 提交，克隆时需初始化子模块 |
| `third_party/` 中其他源码 | 否 | 按依赖锁文件下载，不随主仓库上传 |
| `.env`、`.cache/`、`.runtime/`、Python 缓存和虚拟环境 | 否 | 本机环境、下载缓存和运行数据 |

排除项由 `.gitignore` 管理，保留在本机，不随代码推送。文档中提到的资产和 `outputs/` 报告也是本地文件，不是仓库内附件。

## 主要模块

| 模块 | 入口与职责 |
|---|---|
| 机器人控制与 IK | `robot_control.py`、`arm_ik.py`、`whole_body_control.py`、`gpu_whole_body_ik.py` |
| 搬运与放置判定 | `transport_task.py`、`transport_runtime.py`、`floor_demo.py`、`setup_task.py` |
| 双机接力 | `relay_task.py`、`relay_runtime.py`、`relay_contacts.py`、`relay_perception.py`、`relay_pusher.py` |
| 导航与定位 | `grid_navigation.py`、`navigation_runtime.py`、`lidar_slam.py`、`livo_*.py` |
| 网页控制台与相机 | `navigation_console_server.py`、`robot_camera_server.py`、`robot_camera_streams.py`、`web/` |
| 场景与模型处理 | `build_scene.py`、`assemble_scene.py`、`build_relay_scene.py`、`import_robot.py`、`prepare_relay_robot.py` |
| 统一命令入口 | `scripts/simctl` |

表中 Python 文件和 `web/` 均位于 `scripts/`。

## 环境准备

需要 Linux、NVIDIA GPU 及驱动、NVIDIA Container Toolkit、Docker Engine、Docker Compose v2，以及宿主机 Python 3.10 或更新版本、Git、curl。GUI 模式还需要本机 X11/Xwayland 桌面会话。

项目配置使用 Isaac Sim **6.0.1**（`.env.example` 固定镜像摘要）和 Blender **4.5.3**。Isaac/Blender 运行在容器中；首次初始化需要联网下载 Blender、构建镜像，并在运行前获取 Isaac 镜像。

```bash
git clone --branch dev-real --single-branch --recurse-submodules https://github.com/LuxiTech/wheel-robot.git
cd wheel-robot
git submodule status
./scripts/simctl init
```

已克隆该分支但未下载子模块时，执行 `git submodule update --init --recursive`。SDK 固定提交为 `ba0c7320cd1499995cfc614e83c0c3963343e15a`；不要用 `git submodule update --remote` 替换锁定版本。GitHub 仓库公开不代表外部 SDK、镜像与模型下载源一定可访问，首次部署需要分别连通这些来源。

`init` 会创建本地目录，从 `.env.example` 生成 `.env` 并填写当前用户 UID/GID。随后编辑 `.env`：

- `GPU_ID`（仿真）默认是 `1`，`PERCEPTION_GPU_ID`（模型推理）默认是 `0`。单显卡机器将两者都改为 `0`。
- 核对 `LOCAL_UID`、`LOCAL_GID` 与当前用户一致。
- 使用 NVIDIA 镜像前确认接受其许可条款；模板中的 `ACCEPT_EULA=Y` 会传入容器。

单显卡 `.env` 的关键项如下，UID/GID 保留 `init` 自动填入的值：

```dotenv
GPU_ID=0
PERCEPTION_GPU_ID=0
```

```bash
# 显式准备固定版本的 Isaac 镜像，再检查 Docker、GPU 和磁盘
docker compose pull isaac-sim
./scripts/simctl doctor
```

### GPU 分工与单卡复现

当前默认分工如下（宿主机 GPU 编号以 `nvidia-smi` 为准）：

| 工作负载 | 配置入口 | 默认宿主 GPU |
|---|---|---|
| Isaac 物理仿真、相机渲染、接力任务 CUDA IK | `.env` 的 `GPU_ID` → `compose.yaml` | 1 |
| NanoOWL、NanoSAM、FoundationPose 模型推理及 TensorRT 引擎构建 | `.env` 的 `PERCEPTION_GPU_ID` → `compose.grasp.yaml` | 0 |

接力 IK 在 `config/relay_demo_config.json` 中使用 `ik_backend=cuda`、`ik_device=cuda:0`，由仿真进程内的 `WholeBodyIK` 调用 Warp CUDA 求解。这里的 `cuda:0` 是**容器内可见设备编号**：仿真容器仅挂载宿主 GPU 1，因此它对应宿主 GPU 1，不能据此判断 IK 在宿主 GPU 0 上运行。这个分工描述针对接力任务，不将单箱测试入口的规划实现混为同一套 IK。

单 GPU 机器在 `.env` 中同时设置 `GPU_ID=0` 和 `PERCEPTION_GPU_ID=0`，保留 IK 的 `cuda:0`。两个容器会共享同一张卡，不需要第二张卡才能配置启动，但尚未完成单卡完整接力验收。若 shell 中导出了同名变量，会优先于 `.env` 生效，应一并核对。可检查 Compose 最终解析结果：

```bash
nvidia-smi --query-gpu=index,name,memory.total --format=csv
docker compose -p wheel-robot-grasp -f compose.grasp.yaml --profile sim config
```

检查 `sim` 与 `perception` 的 `deploy.resources.reservations.devices.device_ids`；单卡应均为 `0`。修改配置后需重新启动任务，已有容器不会自动切换 GPU。

`environment run` 按采集 → 感知/规划 → 仿真执行串行启动并退出阶段容器；`environment relay` 则先启动常驻模型进程，再启动仿真进程，两者同时占用显存。推理时暂停物理推进不等于释放 Isaac 的显存，因此不能通过“暂停仿真”解决显存不足。先通过 `check`、`engines`、`smoke`，再尝试单箱接力并观察显存和日志；单独推理通过不代表同卡接力能够运行。更换显卡后按下文重新构建引擎，当前没有经验证的单卡最低显存指标。

`doctor` 应能在容器中列出所选 GPU；它不校验机器人资产或模型权重。`init` 会下载 Blender 并构建场景工具镜像，不能只复制 `.env` 就跳过初始化。

默认仿真容器不联网，缓存保存在项目 `.runtime/`，不会随代码提交。GUI 会额外挂载当前桌面的显示认证信息。网页服务仅监听本机。

## 外部资源准备

模型不在此仓库中，请向项目维护者获取与当前配置匹配的资源包，并保留包内相对路径：

```text
assets/robots/
├── r680_lift_openarm/
│   ├── r680_lift_openarm_sim.urdf
│   └── resources/                 # URDF 引用的完整网格目录
├── r680_lift_openarm_20261019/     # 当前拆码垛模型：r680_lift_openarm(1).rar（完整解压）
│   └── r680_lift_openarm/
│       ├── r680_lift_openarm_robot_viewer.urdf
│       └── resources/
└── r680_openarm/                  # 使用固定立柱配置时需要
```

默认机器人入口由 `config/robot_config.json` 的 `source_urdf` 决定，路径相对 `assets/robots/`。历史固定立柱配置位于 `config/profiles/robot_fixed.json`；不要混用不同模型的配置和生成结果。

拆码垛使用 `r680_lift_openarm(1).rar` 中自带的 OpenArm 夹爪，保留原始网格、关节限位和 mimic 联动，不再依赖 AG95 资源。左右夹爪张开角分别为 +π/4、−π/4，闭合角均为 0。

资源准备好后生成基础场景和机器人：

```bash
./scripts/simctl build-robot
./scripts/simctl build-scene
```

接力与导航还需要生成双机环境：

```bash
./scripts/simctl build-relay
```

生成结果写入 `assets/generated/` 和 `assets/robots/generated/`。`build-relay` 使用独立的 `config/profiles/robot_relay.json`，基于 2026-10-19 提供的 `r680_lift_openarm(1).rar` 原始 URDF，保留固定于 `base_link` 的 D455 相机及其安装标定，相机不随升降移动，并使用 URDF 原装 OpenArm 夹爪；也会更新 `config/relay_robot_config.json`，提交前检查其差异。修改模型或场景配置后应重新构建；不要在仿真运行期间同时执行装配。旧模型的抓取验证记录仅适用于当时的模型，新的视觉及完整拆码垛任务需要重新验收。

## RGB-D 模型环境与首次运行

仅运行基础仿真可跳过本节；`tools/grasp/environment run` 和 `relay` 都需要本节的模型环境。基础仿真与感知使用独立容器，宿主机无需安装 ROS、PyTorch 或 CUDA Toolkit。

版本依据为 [环境锁](config/grasp/environment.lock.json)、[源码锁](config/grasp/dependencies.lock.json)、[模型锁](config/grasp/models.lock.json)。[历史验证环境](config/grasp/validated-environment.json)记录的是 RTX 3090、推理 Python 3.8.18、PyTorch 2.0.0+cu118、TensorRT 8.6.1；这是已有验证基线，不是最低配置或任意 GPU 的兼容保证。Isaac 使用独立镜像中的 Python。宿主驱动需同时支持两套镜像；先通过 `doctor` 和下方 `check` 验证。

完成上文初始化及资产生成后，依次执行；每一步成功后再继续：

```bash
tools/grasp/environment sources           # 恢复固定提交的 SDK 与模型源码
tools/grasp/environment build-sdk         # 同时提供模型下载工具所用镜像
tools/grasp/environment download          # 权重和离线 Hugging Face 缓存
tools/grasp/environment verify-models     # 核对锁文件 SHA-256
tools/grasp/environment build-perception
tools/grasp/environment check             # 模块导入、CUDA 和 GPU 隔离
tools/grasp/environment engines           # 在目标 GPU 构建 TensorRT 引擎
tools/grasp/environment smoke             # 实际模型推理冒烟检查
```

模型保存在 `assets/models/`，引擎在 `assets/models/engines/`，都不在 Git 中。更换 GPU、TensorRT 或构建参数后，应先归档旧引擎和配套 JSON 元数据，再重新生成；工具会拒绝覆盖不匹配的已有引擎。首次下载和构建需要较多磁盘空间，FoundationPose 基础镜像仅压缩层就约 11 GB，还需容纳解压镜像、构建缓存、Isaac、权重及日志。

感知还需要箱体网格。`build-relay` 生成 `crate_colliders.json` 后，在独立虚拟环境中生成它，并准备轻量测试依赖：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r tools/grasp/requirements.txt
.venv/bin/python tools/grasp/build_crate_mesh.py
```

输出为 `assets/generated/grasp/crate.obj`。脚本拒绝覆盖已有文件；首次复现执行一次即可。箱体碰撞几何更改后，应归档旧网格并重新生成。若系统缺少 `venv`，先安装系统对应 Python 版本的 venv 包。

选择与资产相匹配的入口：

| 入口 | 必需资产 / 配置 | 当前验证范围 |
|---|---|---|
| `environment run` | 基础 URDF、`build-robot` 生成的机器人、`build-relay` 生成的箱体、`runtime.sim.json` | 单箱 RGB-D、模型感知、IK、预抓取/抓取位姿接近；不含夹紧提箱 |
| `environment relay` | 2026-10-19 原装夹爪 URDF、`build-relay` 生成的双机场景、`robot_relay.json` | 原接力状态机接入模型感知；完整 18 箱仍未验收 |

```bash
# 无需 GUI：先确认相机采集和感知链路
tools/grasp/environment run --perception-only
# 单箱接近动作验证（使用基础机器人模型）
tools/grasp/environment run
# 双机模型感知模式：先检查单箱抓取，桌面环境可增加 --gui
tools/grasp/environment relay --boxes 1 --skill-test grasp
```

单显卡可将两个 GPU ID 设为 0，但 `relay` 的感知进程与仿真会同时驻留，需实际确认显存足够；单箱 `run` 按阶段串行运行。不要用较旧的单箱模型替代接力模型，也不要把单箱接近完成当作接力搬运完成。

单箱日志在 `outputs/grasp/runs/<ID>/`，接力日志在 `outputs/grasp/relay_runs/<ID>/`。先看 `manifest.json` 的状态及错误，再查看对应阶段日志；接力还要检查 `relay_report.json` 中的 `passed` / `skill_passed`。`outputs/` 的历史报告不随仓库提供，需要自行运行生成。

## 常见复现问题

| 现象 | 检查与处理 |
|---|---|
| 无效 GPU 编号、容器看不到 GPU | 核对 `.env` 两个 GPU ID、宿主 `nvidia-smi`、NVIDIA Container Toolkit；重新运行 `doctor` |
| URDF、网格或 USD 不存在 | 获取外部机器人资源，按配置的相对路径解压；执行对应 `build-robot` / `build-relay`；模型感知另外需要 `crate.obj` |
| SDK 子模块为空或构建找不到源码 | 执行 `git submodule update --init --recursive` 和 `environment sources`，检查 Gitee/GitHub 下载访问 |
| 权重缺失、校验失败或离线缓存不完整 | 重新检查 `download` / `verify-models` 输出，保留错误文件用于排查；不要跳过校验 |
| 引擎不兼容或显存不足 | 归档旧引擎后在目标 GPU 重建；检查其他占用 GPU 的进程，确认双机模式同时驻留的显存需求 |
| SSH 中 GUI 启动失败 | 先使用不带 `--gui` 的入口；GUI 需要可用的 X11/Xwayland 会话和显示认证 |
| 挂载目录无写权限 | 核对 `.env` 的 UID/GID 与 `id -u` / `id -g`，以及 `assets/`、`outputs/`、`.runtime/` 的属主 |

迁移时除了源码与子模块，还需保存机器人资产、模型权重/缓存、构建镜像及校验值。在线重建的间接 apt/pip 依赖可能变化；精确复现历史环境应使用归档镜像。归档/加载命令见[迁移与离线复现](tools/grasp/README.md#7-迁移与离线复现)。公开源码本身不足以完整复现未发布的机器人资源。

## 常用运行命令

交接台固定分工：Robot A 从后侧长边（+Y）向左槽放箱，推箱机构将箱子移至右槽，Robot B 从前侧长边（−Y）取箱。两台机器人的初始位置分别位于这两侧；B 每轮在距右槽中心 0.8 米处正对箱子等待，完成视觉观察后再驶入取箱停靠点。控制台第一视角与 RGB-D 感知使用同一 URDF D455 安装位姿及 640×480 仿真内参。`scripts/validate_handoff_view.py` 可渲染右槽放箱工况；完整物理抓取需要另行验收。

自动启动时，两台机器人先保持臂姿及朝向退到各自的外侧通道，再收臂；B 收臂后返回观察点。B 取箱前先在观察点取得视觉位姿，再后退到前侧通道展开机械臂，之后保持准备姿态接近箱子。启动退让与收臂严格串行，台面碰撞检测始终开启。

交接台当前高度为 0.38 米，桌腿、护栏、推箱机构和槽位坐标均从台面高度生成。搬运目标为台面上方 0.12 米（当前 0.50 米），中间准备高度为台面上方 0.07 米与当前箱底高度的较大值，不再使用固定的 0.67/0.72 米要求。B 提箱指令由 0.15 米降为 0.08 米；对右槽的视野和原装夹爪接近轨迹有回归检查。`outputs/pinch_grasp_validation/lowered_handoff_38_pick_8cm/lower_table_pick_summary.json` 记录了 B 在新台面上的夹紧、提箱和摆正通过，后续运输触发倾斜保护；整套物理接力仍未通过。

源垛退让先保持双臂夹持，通过底盘外移使箱体完全越过垛边并留出 10 厘米余量，再水平收回 3 厘米，不额外抬高。额外外移前检查底盘扫掠范围与托盘、交接台、另一台机器人及房间边界；感知确认净空不足时禁止收臂。首箱物理诊断见 `outputs/pinch_grasp_validation/short_level_retreat_20261019/retreat_summary.json`；此阶段通过不代表后续交接姿态或完整 18 箱已通过。

所有命令在仓库根目录执行，按需选择模式。

```bash
# 基础场景
./scripts/simctl gui
./scripts/simctl smoke

# 三箱地面上料
./scripts/simctl console
./scripts/simctl demo

# 双机接力
./scripts/simctl relay --gui
# 原拆码垛状态机 + URDF RGB-D / NanoOWL / NanoSAM / FoundationPose（新感知集成）
tools/grasp/environment relay --gui --boxes 18
./scripts/simctl camera-console          # 本机网页 http://127.0.0.1:8766
./scripts/simctl relay-demo --gui --boxes 1

# 原装 OpenArm 夹爪：密集源垛首箱的真实接触抓取验证
./scripts/simctl camera-console --auto --boxes 1 --skill-test grasp --source-box 12 \
  --task-profile /workspace/config/profiles/relay_pinch_grasp.json

# 固定工位导航
./scripts/simctl navigation --gui
./scripts/simctl navigation-console     # 本机网页 http://127.0.0.1:8765

# 日志与停止
./scripts/simctl logs
./scripts/simctl stop
```

`camera-console`、`navigation-console` 等入口会停止当前仿真实例再启动对应模式。GUI 需要桌面会话，不能直接在没有显示环境的 SSH 终端启动。

FAST-LIVO2 是可选的调试定位链路，首次使用先构建镜像：

```bash
./scripts/build_fast_livo.sh
./scripts/simctl navigation-livo --gui
```

构建脚本自动获取 `U-AMC/FAST-LIVO2-ROS2` 的固定版本 `a09599a288c56b832114d7ad9e898248a4111d9e` 到 `third_party/`，该目录不入库。

## 测试与验证边界

不依赖模型与 Isaac 的放置判定测试可直接运行：

```bash
python3 -B -m unittest discover -s tests -p 'test_task.py' -v
```

已按上文建立 `.venv` 后，可运行抓箱核心测试：

```bash
.venv/bin/python -m pytest tests/grasp -q
```

缺少外部 URDF 或回放数据时，部分用例会跳过；检查测试输出中的 skipped，不能据此认定资产或真实模型推理已通过。

准备好模型、生成资产和容器环境后再执行完整测试及物理验证：

```bash
./scripts/simctl test
./scripts/simctl smoke
./scripts/simctl robot-test
./scripts/simctl relay-test
```

完整测试集包含读取外部 URDF、机器人元数据和生成碰撞布局的用例，空资产目录下不能视为可运行的纯单元测试。物理运行的报告和截图保存在 `outputs/`。

历史记录包含三箱上料、单箱接力及部分摆放策略的通过结果，但完整 **18 箱接力尚未验收通过**，FAST-LIVO2 融合导航仍处于调试验收阶段。A* 真值定位通过不代表融合定位通过；单元测试通过也不代表完成物理或真机验收。历史结果需在匹配资源与环境下复验。

原装 OpenArm 夹爪的 `relay_pinch_grasp.json` 保留双侧夹取顺序，校正夹爪姿态并从上方接近箱沿，提起时向前退让。手指碰撞实例被显式展开，使用原网格的凸分解并绑定摩擦材质，避免凸包填平凹槽以及实例遍历遗漏。原 URDF 网格、关节限位与 7 N·m 力矩上限保持不变，不使用吸附。首箱完全悬空十秒保持验收见 `outputs/pinch_grasp_validation/native_side_free_hold/relay_demo_report.json`，可视仿真复跑及额外三十秒保持检查见 `outputs/pinch_grasp_validation/live_native/hold_summary.json`。`skill_passed` 仅表示该抓取技能通过，不代表完整接力的 `passed`。早期 `front_rim_*` 的短时结果不作为稳定验收。默认完整接力参数未被该配置覆盖；交接、退场及全垛仍需另行验收。

## 详细文档

纯仿真抓取联调入口：`tools/grasp/environment run`。默认按真实 URDF 安装位置渲染 RGB-D、读取仿真关节，调用分割/位姿模型、生成抓取点并联合 IK 执行；不请求真机或依赖 SDK 反馈。日志保存在 `outputs/grasp/runs/`，`--perception-only` 可单独诊断感知。当前执行到双臂/升降接近目标，夹紧提箱尚未接入。参数、回放和错误定位见 [运行说明](tools/grasp/README.md)。

- [RGB-D 自主抓箱开发方案与 SDK 集成](docs/RGBD_GRASP_DEVELOPMENT.md)
- [RGB-D 抓箱核心运行与当前实现范围](ros2_ws/src/wheel_grasp_core/README.md)
- [抓箱 Docker 环境、固定 SDK、模型下载与迁移复现](tools/grasp/README.md)
- [客户演示与三箱上料](docs/CUSTOMER_DEMO.md)
- [双机接力操作与边界](docs/RELAY_DEMO.md)
- [双机环境与模型说明](docs/RELAY_ENVIRONMENT.md)
- [导航模式与定位验证](docs/NAVIGATION.md)
- [机器人与驱动配置](docs/ROBOT.md)
- [搬运任务](docs/TRANSPORT.md)
- [仿真到真机的参数边界](docs/SIM_TO_REAL.md)
- [历史验证记录](VALIDATION.md)

详细文档中的历史路径、截图和测试数量用于追溯当时状态；首次部署以本文资源准备步骤和当前配置为准。
