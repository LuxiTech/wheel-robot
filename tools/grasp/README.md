# 抓箱开发环境与迁移复现

本目录默认运行 **仿真 RGB-D／关节反馈 → 模型感知／关键点 → 联合 IK → 仿真执行**。真机故障期间无需启动或连接真机，SDK 仅保留为后续迁移接口。SDK 与模型源码固定提交，基础镜像固定 digest，权重固定 SHA-256，TensorRT 引擎在目标 GPU 上构建。环境准备不等于抓取闭环已经验收。

## 1. 隔离结构

| Compose 服务 | 内容 | 网络与设备 |
| --- | --- | --- |
| `sim` | 复用现有 Isaac Sim 镜像、URDF、场景和控制器 | 默认断网，宿主 GPU 默认 1，包含接力 CUDA IK；需显式启用 `sim` profile |
| `perception` | NanoOWL、NanoSAM、FoundationPose、抓取核心 | 运行时断网，GPU 默认 0；不挂载串口、CAN 或 USB |
| `sdk_bridge` | ROS 2 Humble、固定 SDK 的 `luxi_interfaces`、项目 ROS 包与 HTTP 接收工具 | 普通容器网络可访问 SDK HTTP；默认仅执行 doctor |
| `download` | 复用桥接镜像的临时下载工具 | 仅 `tools` profile 启用，有网络，可写模型目录 |

启动脚本显式使用独立项目名 `wheel-robot-grasp`，避免被原项目 `.env` 的 `COMPOSE_PROJECT_NAME` 覆盖。不会清理或重启已有仿真容器。

这是一套 Compose 项目，不是把三套 Python/CUDA 依赖合并。模型依赖不安装进 Isaac Sim、宿主 Python 或 ROS Python。FoundationPose 使用上游基础镜像内的 Python 环境；桥接使用 Ubuntu 22.04 系统 Python 3.10。

`perception` 和 `sdk_bridge` 的根文件系统只读，以宿主用户 UID/GID 运行。代码与原始权重只读挂载；仅 outputs、推理缓存和引擎目录可写。模型构建在 GPU 容器内执行。容器不会自动连接电机或执行硬件驱动。

统一入口 `tools/grasp/environment run` 默认串联 Isaac RGB-D 快照、模型感知、联合 IK 和 Isaac 接近动作，容器间使用本次运行目录交换文件。`--cycles N` 顺序执行 N 次新快照；它不是实时视觉伺服。`docker compose up` 默认仍只做环境诊断。

## 连续流程与每次运行日志

### 原双机拆码垛任务接入模型感知

```bash
# 原来的 18 箱场景、URDF 原装 OpenArm 夹爪、拆码垛状态机；仅替换感知入口
tools/grasp/environment relay --gui --boxes 18
# 先验证第一只箱子的原有夹紧／辅助拉出／双手接入／试抬逻辑
tools/grasp/environment relay --gui --boxes 1 --skill-test grasp
```

此入口运行 `run_relay_scene.py --perception rgbd`，复用 `RelayRuntime` 的动作、抓取规则和 IK；不使用单箱测试场景的 pinch TCP。NanoOWL、NanoSAM、FoundationPose 在独立无网络容器中常驻。`observe()` 通过原场景 URDF 的头部相机采集冻结同帧 RGB-D，再读取模型估计的世界坐标箱体位姿。推理期间暂停仿真物理，属于同步快照集成，不是实时视觉伺服。内参为显式仿真参数，安装变换逐帧与 URDF 校验。

同形箱体使用任务编号、已知箱垛布局及历史观测／关节 FK 预测的 ROI 做关联；ROI 只用于选择检测和提示分割，不作为测量结果返回。每帧保存检测、分割、配准质量和关联依据；首次注册必须有 RGB 检测支持；已注册目标在检测短暂漏检时使用上次估计的 ROI 进行当前帧分割与 FoundationPose 跟踪，仍需通过深度和表面质量检查。无法注册、质量拒绝或超时会停止任务，不回退真值。原任务的底盘定位、接触、交接区占用和成功验收仍包含仿真专用反馈，本次只替换视觉位姿来源。日志在 `outputs/grasp/relay_runs/<ID>/`，含 `simulation.log`、`model_worker.log`、`relay_report.json` 和 `frames/<序号>/`。GUI 打开后自动启动任务；旧入口的真值模式仍保留作对照，不代表新模型模式已通过整垛验收。

位姿质量使用掩码内全部有效深度点计算点到模型表面 RMS，保留 6 mm 门限，避免固定抽样放大少数边缘背景点。RMS 高于 3 mm 时尝试局部几何精配准，平移／旋转修正分别限制在各轴 4 cm／0.2 rad 内，再用全部点复验；不能通过校验则停止。`camera_pose_initial.json`、`camera_pose.json` 和可选 `refinement.json` 保存修正前后结果。CPU 图像预处理会隔离 FoundationPose 对 PyTorch 默认 CUDA 张量类型的影响，支持同进程连续分割和跟踪。

三维提取前对 NanoSAM 掩码做一像素边界腐蚀（3×3），排除 RGB 轮廓处混合的背景深度。`mask_raw.png`、`mask.png`、`mask_cleanup.json` 分别记录原始掩码、实际输入掩码和保留点数；该步骤不移动深度点，也不把背景点投放到箱体上。

边界清理后，按相邻像素反投影三维点的距离（不超过 25 mm）提取最大连通区域，要求至少保留一半有效点，去掉从箱沿空隙看到的后方箱体。`depth_clustering.json` 记录区域数量和保留比例；连通区域不明确时拒绝该帧。2026-10-16 的 42 帧实际抓取前缀回放（包含单侧抬出及双手试抬后的观测）全部通过此感知链路，最大表面 RMS 约 2.53 mm；这项结果是离线感知验证，不等于整垛物理执行成功。

#### 原拆码垛逐帧可视化与抓取目标追溯

`relay_runs/<ID>/frames/<帧号>/` 保留原始输入和各阶段产物；失败前已生成的文件不会删除。没有某阶段产物时，结合 `inference.error.json`、`result.json` 的异常和调用栈判断是否在该阶段之前终止，不能将缺失产物当成成功。

| 文件 | 内容 |
| --- | --- |
| `observation.npz` / `rgb.png` | 原始 RGB-D、内参与原始 RGB 图 |
| `detections.json` / `detection_overlay.png` | NanoOWL 所有检测框与分数；黄色框为检测，紫色框为任务先验 ROI |
| `association.json` | 实际目标选择或跟踪 ROI 的依据 |
| `mask_raw.png` / `segmentation_raw_overlay.png` | NanoSAM 原始分割及绿色半透明叠加 |
| `mask.png` / `segmentation_overlay.png` | 边缘与深度连通域清理后实际使用的分割 |
| `camera_pose*.json` / `quality.json` / `pose_overlay.png` | 相机坐标位姿、质量指标、三维箱体外框投影；`result.json.pose` 为最终世界坐标位姿 |
| `grasp_targets.jsonl` | 当前帧关联的全部目标计算历史，每行含机器人、源箱体编号、pair_id、动作阶段、仿真时间、4×4 世界坐标目标和像素投影 |
| `rule_targets_latest.json` / `rule_targets_overlay.png` | 最近一次几何规则计算结果，包括候选、接近或预测目标 |
| `motion_targets_latest.json` / `motion_targets_overlay.png` | 最近一次传给 `hands()` 运动规划入口的目标；不代表 IK 已通过或执行成功 |

运行根目录另有汇总 `grasp_targets.jsonl`，按序号保存上述全部数值历史。叠加图和 `*_latest.json` 只保留该帧对应类别的最后一次结果，历史坐标不会覆盖。图中左右标记为期望 TCP，不是测得的手指接触点；目标在相机后方或画面外时只记录状态，不强行夹到图像边界，遮挡不在该投影中判定。记录包含原图采集时间、计算时间和观测年龄：预测移动目标投影在历史图像上不等于当时物体实际所在位置。仅经过感知而尚未计算手部目标的帧没有抓取点图。

2026-10-16 起新启动的 `relay` 运行启用这些产物。旧进程不会热加载新代码，旧日志也不会伪造缺失的抓取点历史。

### 单箱接近测试入口（与原拆码垛任务分开）

以下 `run` 命令及本节 TCP、动作范围描述仅适用于单箱接近测试；原拆码垛任务请使用上面的 `relay` 入口。

```bash
# 默认纯仿真流程：不启动 SDK 接收器，不访问真机
tools/grasp/environment run
# 可选：人工指定当前仿真图像中的目标框；每轮重新渲染
tools/grasp/environment run --box 100 80 500 450 --cycles 3
# 只运行仿真相机与模型感知；跳过 IK 和动作执行
tools/grasp/environment run --perception-only
# 使用一次通过反馈校验的历史采集重新推理、规划、执行仿真
tools/grasp/environment run --replay outputs/grasp/runs/<运行ID>
# 感知诊断的采集也可以回放，但仍只允许感知模式
tools/grasp/environment run --replay outputs/grasp/runs/<感知诊断ID> --perception-only
```

框坐标只是参数格式示例，必须替换为实际箱体框。多轮中的场景应保持目标位置稳定，旧框不会自动跟踪目标身份。默认输入 `--source sim`，运行配置默认 `config/grasp/runtime.sim.json`（JSON，也是 YAML 子集），可用 `--config` 指定；每个容器阶段默认超时 600 秒，可用 `--timeout` 调整。任一阶段失败立即退出，不继续后续动作，不自动重试或覆盖错误现场。

仿真相机挂在 URDF 对应的 `*_color_optical_frame`，运行时校验 USD 安装变换与 URDF FK 一致；RGB 和 `distance_to_image_plane` 深度来自同一渲染产品、同一冻结物理状态，深度为光学 Z、单位米。URDF 不包含完整相机内参，因此分辨率和针孔内参在 `simulation.camera` 中明确配置，并记录为仿真参数，不冒充硬件标定值。关节从 Isaac articulation 读取，不需要 SDK 反馈、升降零点标定或本机 ROS。

`simulation.initial_joints`、箱体位置和角度定义测试场景。先保持关节等待底盘沉降稳定，再暂停物理，用 Replicator 的 `delta_time=0` 同帧采集；`capture_context.json` 同时保存基座位姿，执行端恢复该位姿，避免采集/执行坐标不一致。感知只读取图像、深度、内参和关节，不读取 `sim_ground_truth.json`；箱体真值仅留作独立评估。NanoOWL → NanoSAM → FoundationPose → 几何抓取规则 → 联合 IK 均调用现有实现，不使用真值分割或真值位姿旁路。执行阶段重新建立同一个配置场景，不按感知结果移动箱体。当前为串行快照流程，尚不是持续视觉伺服或夹紧提箱闭环。

SDK 模式只在未来显式指定 `--source sdk` 时启用，使用 `runtime.container.yaml`；本次仿真开发不运行该模式。SDK 模式才需要实际双臂/升降反馈和 `lift_mapping`，原校验仍保留。仿真与推理服务均 `network_mode: none`，默认流程不会启动有网络的 `sdk_bridge`，也不要求构建该镜像。

当前仿真使用 `tool_convention=pinch_y`：根据提供的 pinch gripper URDF，将工具 Y 轴（夹爪闭合方向）对准箱沿法向，接近方向位于箱沿切平面。仿真 TCP 为 `[-0.02193, 0, -0.15] m`，X 来自指根位置，Z 选在模型手指的夹持区域；这是仿真调试值，`tcp_hardware_validated=false` 保持不变。接近倾角为 30°，预抓取距离为 0.14 m、轨迹插值 120 段，避免原姿态与较粗采样导致的碰撞和关节跳变。迁移真机时须核验 TCP 与工具坐标约定。

采集时升降保持低位，避免双臂遮挡箱体。规划先执行 `clearance`：保持双臂关节角，将升降抬至 0.15 m，再执行 `pregrasp` 和 `grasp` 接近。这里抬高的是机器人升降机构，不代表已经夹起箱子。双手工具 +X 均朝上，给 URDF 中的腕部相机留出空间；碰撞检查保留相机外壳，禁止通过删除传感器碰撞体规避错误。

仿真双臂驱动使用 `simulation.arm_drive_gains`（刚度 2000 N·m/rad、阻尼 70 N·m·s/rad），降低重力下的位置跟随偏差，仍保留原 URDF 的力矩和速度上限。这是仿真调参，不是实测电机控制参数。末端到位检查仍要求位置误差不超过 10 mm、姿态误差不超过 0.06 rad。

2026-10-16 验证记录：`outputs/grasp/runs/20261016T074118Z-922ae379/manifest.json` 状态为 `completed`，实际执行模型推理和 6515 个物理步，准备抬升、预抓取及抓取位姿接近均通过。最终左右 TCP 位置误差分别为 4.77 / 4.74 mm，姿态误差为 0.00713 / 0.00707 rad；机器人与目标、支撑台无报告接触。该结果仅覆盖当前配置场景，未验收自碰撞、夹紧、动态提箱和搬运，也不代表真机精度。相关测试 41 项通过。

每次启动打印 `RUN_DIRECTORY`，在 `outputs/grasp/runs/<UTC时间-随机ID>/` 保存：

| 文件 | 定位用途 |
| --- | --- |
| `manifest.json` | 运行模式、当前阶段、最终状态、最深层失败原因与 traceback |
| `*.events.jsonl` | 阶段开始/结束/失败、UTC/单调时间、耗时、run_id 和相关 pair_id |
| `capture.log` / `perception_plan.log` / `simulate.log` | Docker 启动、模型、CUDA、Isaac 的完整 stdout/stderr |
| `*.error.json` | 具体子阶段、异常类型、完整 Python 调用栈 |
| `sim_ground_truth.json` / `sim_scene_reuse.json` | 仅用于评估的场景真值，以及执行未按估计位姿挪箱的记录 |
| `state_before.json` / `state_after.json` | 仅 SDK 模式：SDK 原始反馈 |
| `rgbd_response.mime` / `rgbd_headers.json` | 仅 SDK 模式：解码/过期检查前的原始响应 |
| `observation.npz` / `joints.json` / `capture_context.json` | RGB、浮点深度、标定元数据、关节及时间关联方式 |
| `detections.json` / `mask.png` / `rgb.png` / `camera_pose.json` | 检测、分割和位姿中间结果；后续质量拒绝时仍保留已生成结果 |
| `perception.json` / `plan_*.json` / `trajectory.json` | 箱体位姿、双手抓取点/预抓取点、联合 IK 轨迹 |
| `sim_initialization.json` / `sim_steps.jsonl` | 仿真初始化假设，以及每个物理步的目标、实际关节、误差、接触和底盘漂移 |
| `tcp_errors.json` | 各动作阶段结束时，基于实际 USD 末端位姿计算的位置／姿态误差 |
| `contact_failure.json` | 发生碰撞时的动作阶段、物理步、刚体与具体碰撞体路径，便于区分腕部相机和夹爪 |
| `runtime.*` / `*lock.json` / `source_hashes.json` / `asset_hashes.json` / `container-images.json` | 配置快照、依赖与源码/资产哈希、实际模型镜像信息 |
| `*.result.json` | 阶段完成凭据；进程成功退出且凭据匹配，编排器才进入下一步 |

定位顺序：先看 `manifest.json` 的 `failure.stage` 和 `failure.error`，再看对应 `*.error.json`、阶段日志和输入。没有运行结束凭据的目录不能当作成功。进程被强制杀死/机器断电可能留下 `running` 状态；结合最后一个事件判断中断点。日志不自动清理；原始 RGB-D 占用磁盘，长期运行应归档运行目录。回放重新使用当前代码/模型，记录新的版本哈希，校验原采集几何资产未改变；逐版本复现仍需保存对应源码与 Docker 镜像。

若 SDK 返回 `arms=null`，在机器人端运行 `bash diagnose_robot.sh <SDK根目录>`（脚本来自本项目 `tools/grasp/diagnose_robot.sh`），收集 systemd 状态/日志、实际进程 ROS 环境、CAN 统计和 ROS 话题。脚本不启动、重启或使能设备，各检查有 12 秒超时；结果保存在打印的 `/tmp/wheel-feedback-diagnostic-*` 目录。服务显示 running 不等于消息已到网关。升降 SDK 的 250 ms 有效性阈值与本项目更严格的 150 ms 规划输入门限也应分别检查；URDF 不提供实时关节测量。

单箱 `run` 入口的仿真执行范围：使用原始轮式 USD/URDF，使用仿真采集时的关节状态初始化，底盘不规划移动；按共同时间推进升降和双臂到预抓取/抓取目标，检查关节跟随、末端残差、底盘漂移和目标接触。纯仿真模式保持采集场景中的地面、支撑台和静态箱体，不能用感知位姿修正箱体真值。夹爪明确初始化为仿真开位，未使用未经标定的真实夹爪角度；自碰撞继承原 USD 设置，其他真实障碍物尚未重建。轨迹 `executable=false` 保持不变，仅在此隔离仿真验证，不发送 SDK 控制请求。**此版本不执行夹紧、提箱，也不代表完整搬箱或碰撞安全已验收。**

此前 SDK 模式的历史排查（不阻塞当前纯仿真）：真实 SDK 可获取 RGB-D，但 `/api/state` 返回 `arms=null`、升降反馈不可用，亦捕获到 RGB-D 超过 250 ms 的失败；对应目录保留具体证据。另一次真实 RGB-D 已进入 NanoOWL，检测返回 0 个周转箱候选；原图中没有目标周转箱，流程正确停止。Isaac 独立合成关节运动夹具已验证联合 IK 输出可驱动双臂与升降到达目标；该测试不使用真实箱体感知，不能替代完整链路验收。

```bash
# 生成明确标记 synthetic 的运动夹具；输出其运行目录
.venv/bin/python tests/grasp/make_sim_fixture.py
# 将下一行路径替换为上一步输出
docker compose -p wheel-robot-grasp -f compose.grasp.yaml --profile sim run --rm -T \
  sim /workspace/scripts/run_grasp_snapshot.py /workspace/outputs/grasp/runs/<夹具ID>
```

## 2. 固定 SDK

SDK 位于 `third_party/lift_robot_ros2`，使用 Git 子模块，固定：

```text
https://gitee.com/fxx360724/lift_robot_ros2.git
ba0c7320cd1499995cfc614e83c0c3963343e15a
```

不要运行 `git submodule update --remote` 自动追随最新版本。更新 SDK 时需要显式选择新提交，同时更新依赖锁、接收协议测试及镜像标签。本次集成完整保存 SDK 源码与许可证，但桥接镜像只编译消息定义和项目接收节点；硬件驱动仍应在机器人本机按 SDK 文档部署。

其他源码通过 `config/grasp/dependencies.lock.json` 恢复；下载后保存在忽略入库的 third_party。源码许可证保留在各自仓库中。

## 3. 首次准备

宿主机需要 Linux x86_64、Git、Python 3（仅用于源码初始化）、Docker Engine／Compose 和 NVIDIA Container Toolkit。模型环境无需宿主安装 CUDA Toolkit；宿主需要兼容镜像 CUDA 的 NVIDIA 驱动。为镜像、构建缓存、权重和 Isaac 资源预留充足磁盘空间，FoundationPose 基础镜像压缩层约 11 GB。

从项目根目录执行：

```bash
git submodule update --init --recursive
tools/grasp/environment sources
tools/grasp/environment build-sdk
tools/grasp/environment download
tools/grasp/environment verify-models
tools/grasp/environment build-perception
tools/grasp/environment check
tools/grasp/environment engines
tools/grasp/environment smoke
```

这些命令应依次成功后再继续。已存在且校验正确的权重会复用；校验失败不会覆盖原文件。已有引擎只有在源文件、构建参数、GPU／软件版本及引擎 SHA-256 全部一致时才复用；不匹配时拒绝覆盖。升级环境或更换 GPU 时先将旧引擎及其 JSON 元数据归档，再重建。

下载工具验证 `models.lock.json` 中的 SHA-256，并准备固定 revision 的离线 Hugging Face 缓存。NanoSAM 原 Google Drive 链接本次不可下载，替代来源及引用依据在锁文件中记录，不把社区镜像误写成原始官方发布。FoundationPose 的模型文件来自其 README 指向的 Google Drive；OWL-ViT 来自 Google 的 Hugging Face 模型仓库。

`.env` 中可以设置 `LOCAL_UID`、`LOCAL_GID`、`GPU_ID`、`PERCEPTION_GPU_ID`。只有一张 GPU 时将两个 GPU ID 都设为 0，接力配置中的 `ik_device=cuda:0` 保持不变；该编号是容器内设备编号。`run` 的阶段容器按顺序退出，但 `relay` 的模型进程与仿真进程同时驻留，需要容纳两者的显存；暂停物理仿真不会释放显存。单卡完整接力尚未验收，不能把单独模型 smoke 通过当作同卡接力通过。详见[GPU 分工与单卡复现](../../README.md#gpu-分工与单卡复现)。

## 4. 数据与感知调用

只读采集真实 SDK 的成对 RGB-D：

```bash
tools/grasp/environment capture \
  --url http://192.168.0.245:18030 \
  --camera head_rgb --output outputs/grasp/head_sample.npz
```

这条命令只执行 HTTP GET。机器人暂时不能移动时，可以回放已有 NPZ；不会自动连接或使能机械臂。相机流必须由机器人侧 SDK 正常提供。

模型推理使用容器配置：

```bash
docker compose -p wheel-robot-grasp -f compose.grasp.yaml run --rm perception \
  -m wheel_grasp.cli --config config/grasp/runtime.container.yaml \
  perceive outputs/grasp/head_sample.npz \
  --joints outputs/grasp/measured_joints.json \
  --target-id crate_1 --output outputs/grasp/targets.json
```

`measured_joints.json` 必须来自实际反馈，尤其腕部相机需要与拍摄时刻对应的升降／双臂状态。多目标情况下需要显式传入 `--box x0 y0 x1 y1`，坐标对应模型输入适配后的图像。实际 URDF、箱体网格／collider 不随源码自动下载，需要迁移已有资产并确认尺寸；已有几何资产可通过 `build_crate_mesh.py` 生成试验网格。

真实传感器画面对应实物场景。仿真应使用估计出的箱体位姿和机器人实际初始姿态验证规划；虚拟机器人运动不会使真实腕部相机移动，因此回放验证不能冒充同步视觉闭环。SDK 已对齐的 RGB-D 不重复做跨传感器对齐；现有输入适配负责畸变／投影语义检查。源深度 Z、TCP 和升降映射的未验证状态仍保留诊断标记。

## 5. ROS 与仿真的边界

镜像构建会编译 `luxi_interfaces`、`wheel_grasp_interfaces` 和 `wheel_grasp_core`，宿主不需另装 ROS。HTTP 接收无需 DDS。默认 `ROS_LOCALHOST_ONLY=1`，不能据此宣称已打通跨机器 ROS；若以后切换 DDS，应单独配置发现、网络、domain 和 QoS。

Isaac 环境保持原有启动方式：

```bash
docker compose -p wheel-robot-grasp -f compose.grasp.yaml --profile sim run --rm sim
```

此命令运行现有仿真入口，不代表自动消费感知输出。GUI、具体搬箱任务继续使用项目原有启动脚本。SDK 双臂自主控制仲裁、真机轨迹执行仍是后续迁移工作；本阶段不会自动启用。

## 6. 验证结果在哪里

2026-10-16：SDK 桥接镜像已完成 colcon 构建与实际 ROS 消息导入，容器内 21 项抓箱测试通过；原有相关测试与新增测试合计 31 项通过。14 个模型／配置文件及离线缓存已校验；推理镜像的全部模块导入、实际 CUDA 运算和单 GPU 隔离检查通过。三个 TensorRT 引擎已构建并通过重复运行时的复用校验；NanoOWL／NanoSAM 示例图推理及 FoundationPose 合成 RGB-D 注册／跟踪通过。推理容器内 21 项核心测试也通过。固定版本与镜像 ID 见 [`validated-environment.json`](../../config/grasp/validated-environment.json)。这些结果不代表实物箱精度或自主抓取闭环已经验收。

- `outputs/grasp/perception_environment.json`：模块导入与实际 CUDA 张量运算。
- `outputs/grasp/smoke/report.json`：上游示例图上的真实检测／分割推理，以及合成 RGB-D 上的真实 FoundationPose 注册／跟踪；不是实物箱精度或抓取成功率测试。
- `assets/models/engines/*.engine.json`：GPU、计算能力、CUDA、TensorRT、源 ONNX 与引擎校验值。
- 镜像中的 `/opt/perception-packages.txt`、`/opt/sdk-system-packages.txt`、`/opt/sdk-python-packages.txt`：构建后的实际依赖清单。

核心测试仍可在项目独立 `.venv` 中运行；它只是轻量单元测试环境，不是模型推理环境。新增 ROS 构建结果、模型检查结果以实际运行报告为准，不能仅凭 Dockerfile 存在宣称通过。

## 7. 迁移与离线复现

迁移时保存以下内容：

1. 项目源码及 Git 子模块提交；没有外网时还要复制已检出的 third_party 源码。
2. `assets/robots`、实际使用的箱体几何资产，以及 `assets/models`（包含离线 HF 缓存）。
3. 所需输入 NPZ／实测关节记录；outputs 默认是本机数据，不纳入 Git。
4. 本次构建的镜像归档及校验文件。

```bash
mkdir -p outputs/grasp/migration
docker save -o outputs/grasp/migration/grasp-images.tar \
  wheel-robot-sdk-bridge:ba0c732 wheel-robot-grasp-perception:fp-a1b694b
sha256sum outputs/grasp/migration/grasp-images.tar \
  > outputs/grasp/migration/grasp-images.tar.sha256
# 目标机校验后加载
sha256sum -c outputs/grasp/migration/grasp-images.tar.sha256
docker load -i outputs/grasp/migration/grasp-images.tar
```

Isaac 镜像按原项目 README 单独保存／准备。基础镜像 digest 固定，但 apt/Python 间接依赖的在线源可能变化，**完全复现已验证环境应加载归档镜像**，不要把重新在线构建称为逐字节相同。更换 GPU／架构／TensorRT 后重新生成引擎并跑 check、smoke；x86_64 镜像不能直接当作 Jetson ARM64 镜像使用。

## 上游依据

- [NanoOWL 安装与引擎构建](https://github.com/NVIDIA-AI-IOT/nanoowl)
- [NanoSAM 安装与 ONNX](https://github.com/NVIDIA-AI-IOT/nanosam)
- [FoundationPose 环境与权重](https://github.com/NVlabs/FoundationPose)
- [NanoSAM 替代下载来源的容器脚本](https://github.com/dusty-nv/jetson-containers/blob/master/packages/vit/nanosam/Dockerfile)
