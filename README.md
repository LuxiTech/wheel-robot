# Wheel Robot

LuxiTech 轮式双臂机器人的仿真与任务控制代码，基于 Isaac Sim 和 Blender，面向 R680 全向底盘、升降机构和 OpenArm 双臂，包含搬箱、双机接力码垛、固定工位导航及网页控制台。

本仓库只管理业务代码和配套配置、测试、文档。机器人模型和生成资产需单独准备；仅克隆仓库不能直接启动完整仿真。当前未接入真机驱动。

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
| `third_party/` | 否 | 外部源码；需要时按脚本指定版本获取 |
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

需要 Linux、NVIDIA GPU 及驱动、NVIDIA Container Toolkit、Docker Engine、Docker Compose v2，以及宿主机 Python 3、Git、curl。GUI 模式还需要本机 X11/Xwayland 桌面会话。

项目配置使用 Isaac Sim **6.0.1**（`.env.example` 固定镜像摘要）和 Blender **4.5.3**。Isaac/Blender 运行在容器中；首次初始化需要联网下载 Blender、构建镜像，并在运行前获取 Isaac 镜像。

```bash
git clone https://github.com/LuxiTech/wheel-robot.git
cd wheel-robot
./scripts/simctl init
```

`init` 会创建本地目录，从 `.env.example` 生成 `.env` 并填写当前用户 UID/GID。随后编辑 `.env`：

- `GPU_ID` 默认是 `1`，单显卡机器通常改为 `0`。
- 核对 `LOCAL_UID`、`LOCAL_GID` 与当前用户一致。
- 使用 NVIDIA 镜像前确认接受其许可条款；模板中的 `ACCEPT_EULA=Y` 会传入容器。

```bash
./scripts/simctl doctor
```

默认仿真容器不联网，缓存保存在项目 `.runtime/`，不会随代码提交。GUI 会额外挂载当前桌面的显示认证信息。网页服务仅监听本机。

## 外部资源准备

模型不在此仓库中，请向项目维护者获取与当前配置匹配的资源包，并保留包内相对路径：

```text
assets/robots/
├── r680_lift_openarm/
│   ├── r680_lift_openarm_sim.urdf
│   └── resources/                 # URDF 引用的完整网格目录
├── r680_openarm/                  # 使用固定立柱配置时需要
└── vendor/dh_ag95/                # 双机接力模式需要
    ├── ag95.urdf
    ├── meshes/
    ├── urdf/
    └── ...                       # 来源和许可证说明
```

默认机器人入口由 `config/robot_config.json` 的 `source_urdf` 决定，路径相对 `assets/robots/`。历史固定立柱配置位于 `config/profiles/robot_fixed.json`；不要混用不同模型的配置和生成结果。

AG95 替代夹爪的本地来源记录为 `ian-chuang/dh_ag95_gripper_ros2`，固定版本 `fc4f80fdfb3acae5626df4359aec1401cb71a9a3` 的 `dh_ag95_description` 包。接力脚本使用经过路径与材质处理的独立 `ag95.urdf`；若仅准备原始 Xacro，需提供完整 `urdf/`、网格与宿主机 `xacro` 依赖，由 `prepare_relay_robot.py` 展开。资源包应一并保留来源及许可证文件。该模型是仿真替代件，不能视为实体夹具的精确模型。

资源准备好后生成基础场景和机器人：

```bash
./scripts/simctl build-robot
./scripts/simctl build-scene
```

接力与导航还需要生成双机环境：

```bash
./scripts/simctl build-relay
```

生成结果写入 `assets/generated/` 和 `assets/robots/generated/`。`build-relay` 也会按当前机器人配置更新 `config/relay_robot_config.json`，提交前检查其差异。修改模型或场景配置后应重新构建；不要在仿真运行期间同时执行装配。

## 常用运行命令

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
./scripts/simctl camera-console          # 本机网页 http://127.0.0.1:8766
./scripts/simctl relay-demo --gui --boxes 1

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

准备好模型、生成资产和容器环境后再执行完整测试及物理验证：

```bash
./scripts/simctl test
./scripts/simctl smoke
./scripts/simctl robot-test
./scripts/simctl relay-test
```

完整测试集包含读取外部 URDF、机器人元数据和生成碰撞布局的用例，空资产目录下不能视为可运行的纯单元测试。物理运行的报告和截图保存在 `outputs/`。

历史记录包含三箱上料、单箱接力及部分摆放策略的通过结果，但完整 **18 箱接力尚未验收通过**，FAST-LIVO2 融合导航仍处于调试验收阶段。A* 真值定位通过不代表融合定位通过；单元测试通过也不代表完成物理或真机验收。历史结果需在匹配资源与环境下复验。

## 详细文档

- [客户演示与三箱上料](docs/CUSTOMER_DEMO.md)
- [双机接力操作与边界](docs/RELAY_DEMO.md)
- [双机环境与模型说明](docs/RELAY_ENVIRONMENT.md)
- [导航模式与定位验证](docs/NAVIGATION.md)
- [机器人与驱动配置](docs/ROBOT.md)
- [搬运任务](docs/TRANSPORT.md)
- [仿真到真机的参数边界](docs/SIM_TO_REAL.md)
- [历史验证记录](VALIDATION.md)

详细文档中的历史路径、截图和测试数量用于追溯当时状态；首次部署以本文资源准备步骤和当前配置为准。
