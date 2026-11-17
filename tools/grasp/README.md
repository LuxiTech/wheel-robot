# 感知与 ROS 工具环境

开发机使用 ROS 工具与 GPU 推理两个独立镜像；厂商 SDK 仅在开发板运行；轻量测试使用根目录 `.venv`。当前 Docker Compose 不再包含 Isaac 服务。

## 不依赖 Docker 的权重下载与校验

模型文件下载仅需网络和 Python，不需要 GPU、ROS 或 Docker。所有命令在仓库根目录执行。使用独立虚拟环境安装 `gdown`，避免改变算法测试环境：

```bash
mkdir -p outputs/grasp .runtime
uv venv --python /usr/bin/python3 .runtime/model-download-venv
UV_CACHE_DIR="$PWD/.cache/uv" uv pip install \
  --python .runtime/model-download-venv/bin/python gdown==5.2.0

# 恢复固定提交的模型源码，不下载或构建厂商 SDK
tools/grasp/environment sources

PYTHONDONTWRITEBYTECODE=1 .runtime/model-download-venv/bin/python -u \
  tools/grasp/download_models.py > outputs/grasp/model-download.log 2>&1
PYTHONDONTWRITEBYTECODE=1 .runtime/model-download-venv/bin/python \
  tools/grasp/download_models.py --verify-only > outputs/grasp/model-verify.log 2>&1
```

已有下载虚拟环境时跳过创建步骤。没有 uv 时，可用 `python3 -m venv .runtime/model-download-venv` 创建环境，再用该环境的 `python -m pip install gdown==5.2.0` 安装下载依赖。

下载器按 `config/deployments/models.lock.json` 校验每个文件，已有正确文件会复用；中断文件保留为 `.partial`。若发现校验不匹配，程序报错并保留文件用于排查，不会跳过校验。最后生成固定 revision 的 Hugging Face 离线缓存。

| 内容 | 仓库根目录下的相对路径 |
|---|---|
| NanoSAM 编码器/解码器 | `assets/models/nanosam/resnet18_image_encoder.onnx`、`mobile_sam_mask_decoder.onnx` |
| FoundationPose 两组权重 | `assets/models/FoundationPose/2023-10-28-18-33-37/model_best.pth`、`assets/models/FoundationPose/2024-01-11-20-02-45/model_best.pth`；各有对应 `config.yml` |
| OWL-ViT | `assets/models/owlvit-base-patch32/model.safetensors` 及配套配置/分词文件 |
| 离线 Hugging Face 缓存 | `assets/models/huggingface/hub/models--google--owlvit-base-patch32/` |
| 本机生成的 TensorRT 引擎 | `assets/models/engines/`；下载器不会生成引擎 |

2026-10-22 本机已下载并校验全部 14 个锁定文件及离线缓存，合计约 1.5 GiB。此次校验不需要加载 PyTorch 模型，也未运行推理。新机器克隆仓库后需要重新下载或复制这些资源，然后执行 `--verify-only`。

## 容器构建与推理验证（本机尚未完成）

先准备 Docker Engine、Compose v2 和 NVIDIA Container Toolkit。若还没有 `.env`，从 `.env.example` 复制并填写当前 UID/GID 和 `PERCEPTION_GPU_ID`；单卡为 `0`。已有 `.env` 保留，原 Isaac 变量不再被当前 Compose 使用。

```bash
mkdir -p outputs/grasp assets/models/engines .runtime/grasp .cache/grasp-wheels
tools/grasp/environment sources
tools/grasp/environment build-ros
tools/grasp/environment download
tools/grasp/environment verify-models
tools/grasp/environment build-perception
tools/grasp/environment check
tools/grasp/environment engines
tools/grasp/environment smoke
```

按顺序检查成功后继续。源码、环境和权重分别由 `config/deployments/*lock.json` 固定；既有模型环境验证基线见 `validated-environment.json`（历史 SDK 镜像记录不代表当前 ROS 工具镜像已经验证），不表示任意显卡都已通过。更换 GPU/运行库后须重新校验并构建 TensorRT 引擎，已有不匹配文件需先归档。

已通过独立下载路径准备好的文件会被容器中的下载器复用。当前开发机为 RTX 5090 Laptop GPU，历史推理基线不能替代这台机器上的兼容性测试；应以 `check`、`engines`、`smoke` 的实际结果确认能否运行。

`check` 和 `smoke` 需要真正的 GPU 推理环境，普通 `.venv` 中的逻辑测试不能替代它们。本次目录迁移后镜像需重新构建，以包含 `wheel_perception_core` 等子包。

只读采集和离线分析使用 `tools/grasp/grasp`，容器中使用 `wheel-grasp`。根据环境选择 `config/robots/runtime.yaml` 或 `config/robots/runtime.container.yaml`，配置真实 URDF、已测量的箱体网格/碰撞模型和模型引擎。输出目录和文件名由调用者明确提供，已有采集/规划文件不会覆盖。

```bash
tools/grasp/grasp doctor
docker compose -p wheel-robot-grasp -f deployment/compose.grasp.yaml run --rm ros_tools \
  wheel-grasp --config config/robots/runtime.container.yaml doctor
```

ROS 工具容器不含厂商 SDK，支持模型下载、HTTP 只读采集和离线工具，默认 `ROS_LOCALHOST_ONLY=1`。`wheel-grasp-state` 和 `wheel-grasp-rgbd` 需要开发板 SDK 提供的 `luxi_interfaces`，应在板端 source SDK 环境后运行。计划通过 SSH 管理板端调用程序，执行链路尚未实现。

`environment run` 和 `relay` 原先启动仿真，现已移除。统一任务入口见根目录 `tools/robot`。如需查阅原仿真代码和说明，可在 Git 提交 `6726ba1` 中查看；恢复方式见 `docs/MIGRATION.md`。


## 0.3.0 源码与配置边界

本目录仍是模型准备和诊断工具；核心算法已迁入 packages/，运行 CLI 属于 wheel_runtime。直接运行 Python 工具通过 tools/_bootstrap.py 发现源码。机器人 YAML 引用 recipes 中的共享规则，模型锁位于 config/deployments。历史验证记录不代表新容器已重建。


## 双臂弧度控制页面

在仓库根目录运行（Python标准库，无需额外安装Web框架）：

```bash
python3 tools/grasp/arm_joint_web.py
```

浏览器打开 `http://127.0.0.1:8091`。页面持续读取16个电机的实测角度和SDK限位，
目标输入框初次按实测值填入，刷新不会覆盖正在编辑的目标。J1～J7为臂关节，J8为夹爪电机；
目标单位rad，速度单位rad/s，默认0.02，允许范围取SDK与客户端共同支持的0.02～0.2。
修改输入后，点击某一行的“发送”、单臂8轴发送或双臂16轴发送。
双臂失能时，勾选“失能时允许SDK使能并执行”才允许发送；打开页面、填值和刷新不会使能。

单关节/单臂提交仍按SDK协议发送完整16轴：已使能时，其余轴保留SDK的`nominal_target`；
失能时，其余轴使用提交时的新鲜实测位置。SDK会使能双臂，不是仅使能选中的电机。
未选轴仍受SDK原有保持补偿控制；本脚本不额外加偏移、反号或置零。
J8输入为电机弧度，不是夹指米制位移；现有SDK左爪正向open配置与已确认的实物打开方向相反，
页面保留原SDK限位，不提供未经验收的一键开合，不擅自改写SDK配置。

“停止本页面运动”仅在SDK当前请求ID仍属于本进程时可用，发送`joint_stop`后等待反馈。
SDK的停止接口本身是全局接口，归属检查和发送之间无法原子隔离其他控制者，因此使用时避免
同时从其他SDK页面/脚本操作。关闭网页、Ctrl+C或网络断开不会停止已提交的真机运动。
回执不确定时不重发、不自动使能/恢复；页面锁住新运动，需在SDK侧核对实际状态。
到位需匹配请求ID、SDK处于ready、实测最大误差≤0.02rad且关节速度<0.08rad/s。
日志保存在 `outputs/arm-joint-web/<UTC时间>/`，包含命令意图、SDK回执和原始反馈。
脚本不读取或控制升降，不改硬件零点、URDF、外参或SDK文件。

默认读取 `config/robots/joint_control.json`，命令走18030，实测反馈走18032；可用`--config`更换。
运行不需要SSH密码。其他局域网设备访问时，使用 `--host 0.0.0.0 --port 8091`，
浏览器访问运行脚本的电脑IP；该选项仅适用于可信现场网络。

2026-11-03通过SSH直接核对真机部署的
`Project/device_adapters/src/openarm_adapter.cpp:137`：`joint_move`接收16轴目标，
右8+左8，原生支持rad与0.02～0.2rad/s；失能时从反馈接续，停稳后使能，不经startup/home。
`Project/arm_motion/include/luxi/ik.hpp`索引映射与实时反馈的`right_then_left`一致；
部署XML SHA256为`cb4aad7ef1ce36137ccc07894186f06a60fa45251f3bb94da91aa4be6bc23ae5`。
本功能只使用SDK电机关节坐标，不涉及URDF或相机换算；软件/页面检查不等同于实机运动验收。

## 本目录文件结构

| 文件 | 功能 |
| --- | --- |
| `joint_control.py` | 新 SDK 关节接口只读检查与请求预览；没有真机执行选项。 |
| `arm_joint_web.py` / `arm_joint_web.html` | 本地双臂关节控制页面：左右各8轴，输入rad，支持单关节、单臂、双臂发送和停止本页面的运动。使用真机SDK的joint_move/joint_stop与独立实测反馈。 |
| `zero_arms_check_j2.py` | 双臂及夹爪16轴回零；默认只读预览，`--execute`发送一次SDK零目标并允许SDK按实测位置使能。保留机械臂到位检查和运动JSON日志，不读取或控制升降、不采集图像、不叠图，不依赖URDF或外参。 |
| `set_arm_encoder_zero.py` | 仅将已经固定、失能的当前位置写为硬件零点，不执行官方机械限位自动标定。默认SSH预览，`--side left --joints 2`可只写左J2，`--execute --pose-supported`才写入。使用真机OpenArm CLI、停止臂服务隔离CAN、检查原始反馈；执行后保留失能及服务停止，不控制升降。 |
| `verify_supported_wrist.py` | 有真实运动能力：仅限用户明确授权并确认机械支撑的左腕 J6/J7 单关节 +2°及稳定回位试验；使用进程内有限范围授权，全局运动配置不变。不得当作一般运动入口。 |
| `inspect_claw13_contacts.py` | 用视觉环境提取 claw1.3 接触面、净开口公式与双臂安装差异，输出候选点和测量参考图；不发送运动。 |
| `check_claw13_alignment.py` | 对已配对的实物采集进行 claw1.3 全夹爪叠图；直接使用关节角度，检查单臂 FK，并遵守新模型的安装旋转与开口范围。不复用 1.1 钩点、不发送运动。 |
| `render_gripper_capture.py` | 对 `current_gripper_pose.py` 保存的配对数据只读生成完整夹爪轮廓；可用 `--calibration` 显式比较候选，不改运行配置，不发送运动。 |
| `evaluate_overlay_robustness.py` | 复算2026-11-03指定多高度/小幅变姿样本，比较冻结修正与共同相机刚体候选，分离拟合和留出轴心观测；依赖该次试验和2026-11-02定位记录，只作诊断，不部署候选或运动。 |
| `camera_to_sdk.py` | 将相机点/姿态转换到带升降补偿的 SDK 坐标；支持 GET-only 实时相机矩阵检查。 |
| `verify_calibration_chain.py` | 用已有多高度录制复核第一套外参和 SDK FK，输出可复核 JSON。 |
| `bootstrap_sources.py` | Restore locked sources without replacing locally modified checkouts. |
| `build_crate_mesh.py` | Generate a known crate mesh from existing local collider assets, in metres. |
| `build_engines.py` | Build local GPU-specific TensorRT engines; never download foreign engines. |
| `check_environment.py` | GPU/import/weights diagnostics. This does not send robot commands. |
| `container-entrypoint.sh` | 依赖声明、环境或辅助工具。 |
| `diagnose_robot.sh` | 依赖声明、环境或辅助工具。 |
| `download_models.py` | Download explicitly locked model files and verify their SHA-256 before use. |
| `environment` | Docker 环境准备、下载、校验和诊断入口。 |
| `fix_trt_execstack.py` | Clear TRT 10.8's executable-stack flag in this isolated venv only (glibc >=2.41). |
| `grasp` | 离线算法/只读采集 CLI 入口。 |
| `import_step_mesh.py` | Convert a millimetre STEP to a metre OBJ, centered in XY with bottom Z=0. |
| `perceive` | 原生 GPU 视觉诊断封装入口。 |
| `perceive_live.py` | Read-only head RGB-D inference with durable stage diagnostics; never commands motion. |
| `preview_camera_grasps.py` | Diagnostic CAD pose and rim TCP projections in camera coordinates; no robot adapter. |
| `requirements-native-pose.txt` | 依赖声明、环境或辅助工具。 |
| `requirements-perception.txt` | 依赖声明、环境或辅助工具。 |
| `requirements.txt` | 依赖声明、环境或辅助工具。 |
| `runlog.py` | Durable per-run events and atomic artifacts; standard library only. |
| `setup_live_vision.sh` | 依赖声明、环境或辅助工具。 |
| `setup_native_pose.py` | Build pinned pose extensions inside the native vision venv, without sudo. |
| `smoke_models.py` | Actual learned-model smoke tests, not a real-box accuracy benchmark. |
| `smoke_native_pose.py` | Run real FoundationPose on synthetic RGB-D; never a hardware accuracy claim. |

## 反馈链路验收（只读）

`preview_camera_grasps.py <observation.npz> --box x1 y1 x2 y2 --lift-context <同次采集的lift_context.json>`
同时生成 `sdk_candidates.json`：校验图像 pair_id、相机身份和静止升降时间配对后，
将候选抓取/预抓取姿态转换到 SDK `arm_origin`。保存所用外参及升降上下文，
仍为不可执行的诊断候选；不补造夹爪 TCP、不将规则左右标签视为已验证的机械臂分配。

在项目根目录运行：

```bash
.venv/bin/python tools/grasp/check_joint_feedback.py --samples 120 --output outputs/feedback_check_new
```

输出目录必须尚不存在；逐次保存新旧端点原始响应、拒绝原因、耗时，汇总中位数、P95 和最大值。
脚本只调用 GET。默认比较 `joint_control.json` 的 `base_url` 和真机 18032 反馈服务，
可用 `--feedback-url` 指定后者。新通道有任一拒绝时返回非零退出码。


`camera_to_sdk.py --live` 与 `perceive_live.py` 的相机到 SDK 转换采用静止诊断模式：升降总年龄最多 500 ms，接收时效、拍照前后静止、高度变化、故障和断流检查仍保留。原始 SDK valid 不改写，诊断上下文记录门限及原始标志；结果不可用于自动执行。图像默认 250 ms 门限不变，静止诊断采集限时 8 秒重试（严格模式仍为 3 秒）。无需新增板端服务或更新固件。

## 升降位置控制

```bash
# 只读检查
.venv/bin/python tools/grasp/lift_control.py
# 实机：获取会话，提交当前编码器位置，确认到位后停车释放
.venv/bin/python tools/grasp/lift_control.py --hold-current
# 实机：移动到绝对编码器 100 mm，确认到位后停车释放
.venv/bin/python tools/grasp/lift_control.py --target-m 0.100
```

可用 `--output outputs/lift_control/result.json` 保存新结果文件。
目标是绝对编码器位置，不是增量、距地高度或未标定的 URDF 升降关节值。
SDK 当前接口没有升降速度参数；心跳续租由客户端自动维护。

## 联合 IK 端点控制

```bash
# 只求解并保存：左 SDK 控制点向外 5 cm、上移 6 cm；右侧上移 3 cm
.venv/bin/python tools/grasp/ik_control.py \
  --left-delta 0 0.05 0.06 --right-delta 0 0 0.03 \
  --lift-delta-bounds 0.025 0.035 --output outputs/ik_control/preview_new
# 确认现场空间与设备状态后，加 --execute 执行，output 必须使用新的目录
```

坐标是 `sdk_lift_zero`，控制点为 SDK EE / URDF link7；不是地面或相机坐标、也不是已标定指尖。
求解使用已有 URDF 单臂链和 SDK 根定义、真实关节限位；执行分为升降到位和双臂到位两个阶段。
2026-10-30 首次实机测试被升降固件 motion_inhibited 保护阻断；用户重启恢复后重新求解，联合端点复测通过。
详见 wheel_execution 的执行接口说明及 outputs/ik_control/recheck_live_20261030。

## 完整夹爪坐标复核

```bash
.runtime/vision/venv/bin/python tools/grasp/check_claw13_alignment.py \
  --capture outputs/grasp/alignment-check/20261029T023624Z-21f908 \
  --calibration config/robots/calibration/head_camera_first.json \
  --output outputs/grasp/alignment-check/sdk-per-arm-new
```

输出目录须不存在。使用各侧 SDK 腕端和 claw1.3 腕端到夹爪链，不复用左侧整机注册到右臂。
保留实测第七轴；开口尚无电机映射，因此叠图画出 CAD 允许开口的两端，仅供诊断。

## 当前真机夹爪位姿（只读）

```bash
.runtime/vision/venv/bin/python tools/grasp/current_gripper_pose.py \
  --output outputs/grasp/current-pose-new
.runtime/calibration/venv/bin/python tools/grasp/render_gripper_capture.py \
  --capture outputs/grasp/current-pose-new --output outputs/grasp/current-pose-overlay-new \
  --omit-wrist-cameras --show-unclipped-fingers
```

contacts 来自 `inspect_claw13_contacts.py`，须匹配当前 URDF 与接触面网格指纹。
工具通过真机 GET 采集关节、RGB-D 与升降，要求图像接收时刻前后200 ms内有有效关节样本，
控制代次一致、关节变化不超过0.006 rad、速度不超过0.08 rad/s；不宣称硬件曝光同步。
输出 `pose_report.json` 保存各臂腕端和固定夹爪主体的4×4矩阵、毫米坐标及xyzw四元数。
SDK夹爪电机反馈保留为rad，实际手指位移未知时明确为null，另外给出URDF允许开口两端的候选位姿。
接触面中心是CAD几何参考，不冒充钩尖插入TCP；完整叠图也是开口范围而非实测开口。
输出目录必须不存在；失败也保留 `pose_report.json` 与关节采样。

2026-10-30 14:36（北京时间）实测记录：`outputs/grasp/current-pose-20261030-a/`。
同帧视觉结果：`outputs/grasp/camera-preview/843bbc05a8/`，检测得到一个箱，显式像素选择该检测框。
箱沿候选到两臂根的水平距离1.003～1.146 m，大于URDF机械臂长度上界与完整夹爪半径之和约0.786 m。
`reachability.json` 记录逐臂/逐候选结果；此上界忽略限位和碰撞，仅用于排除可达性。
当前视觉候选不支持固定底盘抓取；未发送任何运动命令。相机度量及活动夹指映射仍保留各自的验证边界。

## 真机夹爪驱动定义只读检查

```bash
.venv/bin/python tools/grasp/inspect_gripper_sdk.py \
  --host sunrise@192.168.0.245 \
  --output outputs/grasp/mechanism-audit-new/live_sdk_inspection.json
```

支持已配置SSH别名及 `--sdk-root`。只读取真机 openarm_can、相关机构定义和夹爪配置，
保存带行号的源码片段及SHA256；不复制整套SDK、不写真机文件、不发运动命令。
使用现有非交互SSH认证；认证失败也保存原因。输出文件必须不存在。

`current_gripper_pose.py` 现接入 `calibration/gripper_v1.json` 官方近似映射。
`actual_finger_displacement_m`仍为空（没有直接位移传感器），估计值位于 `official_approximation`；
有效时 `estimated_fingers` 保存位姿和CAD钩部接触参考点，无效时为null。
叠图使用映射估计开口；若超限，省略该侧活动夹指并标注原因，不再默认画允许开口两端。
首次接入发现左侧零参考不一致，不能以取绝对值掩盖。固定主体仍照实测七轴绘制。

## 从用户完整 URDF 生成统一 MuJoCo 模型

当前默认模型为claw1.4，选择见`config/robots/robot_model.json`。历史采样仍按其模型来源回放。
已生成`outputs/unified-model/claw14-20261104/robot.xml`及`claw14_meshes/`，旧1.3模型和XML保留。
SDK兼容转换命令如下；输出目录必须不存在：

```bash
.runtime/vision/venv/bin/python tools/grasp/build_unified_model.py \
  --urdf assets/robots/r680_lift_openarm_claw1.4/r680_lift_openarm_robot_viewer.urdf \
  --sdk-reference-chains config/robots/calibration/sdk_arm_chains.json \
  --mesh-directory claw14_meshes --output outputs/unified-model/claw14-new
```

`--sdk-reference-chains`保留旧标定的左基座坐标对应；整机双肩、腕部相机和夹爪均采用新URDF。
`arm_origin`随升降架运动，SDK原十四轴腕端FK/IK在此相对坐标下与升降位置无关，已做多高度数值验证。
因此无需把尚未精确验收的绝对升降偏置写入SDK。原SDK不计算夹爪开合，XML手指qpos0为CAD名义闭合；
不把电机弧度直接塞给米制手指关节，不宣称SDK已经实时同步了夹指或完整碰撞规划。
新XML放在真机v2目录旧`openarm_bimanual.xml`旁，名为`openarm_claw14.xml`，安装目录同样并存。
teleop/program启动入口的model默认值选择新XML，可显式传旧路径回退；没有为切换模型启动或重启运动服务。

以下为此前不带SDK兼容选项的完整模型验证示例，其arm_origin定义与上述兼容产物不同：

```bash
# 可在已有视觉隔离环境安装 requirements-unified-model.txt 中的额外依赖。
.runtime/vision/venv/bin/python tools/grasp/build_unified_model.py \
  --output outputs/unified-model/claw13-new
.runtime/vision/venv/bin/python tools/grasp/verify_unified_model.py \
  --model outputs/unified-model/claw13-new/robot.xml \
  --capture outputs/grasp/v1-mapping-live-recheck-20261030-b \
  --calibration config/robots/calibration/head_camera_first.json \
  --lift-offset-m 0.2 --output outputs/unified-model/validation-new
```

输出目录须不存在。模型保留全部URDF层级、joint名、平移/旋转限位和mimic约束，
将含镜像缩放的DAE/STL按实际变换导出OBJ，生成可迁移的XML与meshes。
`left_ee_control_point`/`right_ee_control_point`放在原link7；新模型`arm_origin`明确定义为升降架，
与旧SDK的双臂根坐标不同，不能只替换XML而沿用旧目标坐标。
`UnifiedModel.update`显式接入height_um和升降方向/偏置，并分别填入两个mimic手指qpos。
未测得轮子转角按0作静态FK对比，不用于移动底盘估计。

示例中的0.2m是“编码器零为URDF行程中点”的待验收候选，不能当作实测标定。
相机转换为 `URDF base_T_left_base(q_lift) @ inverse(oldSDK origin_T_left_base) @ oldX(h)`，
明确保留左基座对应假设；不能声称已独立测得base_footprint外参。
校验包含30组随机姿态、全部link及多高度下机身固定相机的不变性，并生成相同实测角度的叠图。

该产物为FK/IK几何模型，接触计算关闭，原始惯量保留；URDF中无惯量的活动虚拟frame加入
明示的微小质量/惯量以满足MuJoCo加载要求，见manifest。这不是动力学或碰撞规划验收。
缺少有效手指位移时，不把任意默认开口画成实测结果。

## 直观检查夹爪三维姿态

```bash
.runtime/vision/venv/bin/python tools/grasp/view_gripper_pose.py \
  --capture outputs/grasp/closed-reference-check-20261030-b \
  --unified-report outputs/grasp/j7-debug-20261030/unified-fixed-raster/report.json \
  --output outputs/grasp/pose-3d-inspection-new
```

生成离线可旋转的report.html（内嵌Plotly和图片，无需联网），显示计算的整机坐标三维姿态，
以及同帧实物照片、相机视角模型和叠图。三维图不使用相机外参；相机视角面板使用待核对的注册外参。
内部从原始URDF手写RPY/Rodrigues和mimic计算，不复用生产关节映射或FK实现，
逐部件对照生产FK，并核对已保存MuJoCo腕端结果。此检查验证软件计算，不替代实物坐标测量。

`check_relative_rail_orientation.py` 在固定保存画面的人工ROI内提取导轨白色长条的RGB-D主轴，
比较两导轨的三维夹角与模型夹角，不使用外参。当前ROI专用于17:18画面，换画面必须重新核对，
不能直接当作自动识别或精度标定工具。输出包含27组门限结果、输入指纹、选择区域和证据图。

`recalibrate_recorded_head.py` 离线重算标零后头部眼在手外标定，阶段为`extract`、`detect`、
`solve`、`report`，每阶段显式传入`--source 录包根目录 --output 分析目录`。
新增`rebase`阶段用于左J3硬件重标后的历史角度换算：`--source`传原分析目录，
`--zero-record config/robots/calibration/left_j3_rezero_20261103.json`提供写零前后证据，
`--output`为不存在的新目录。仅换算旧数据q[10]并重算缓存FK，原包不改，禁止重复换算。
随后对新目录运行`solve`与`report`。当前选用`head_camera_j4_rezero_20261104.json`，参考升降81814μm；
历史`check_claw13_alignment.py`、`localize_alignment.py`、`evaluate_overlay_robustness.py`
需显式传入`--calibration`，保证与采集时的关节零点匹配。
完整帧提取使用`extract --image-period-s 0`。需要独立Python3.12环境及
`requirements-recorded-calibration.txt`，不依赖本机ROS，不连接真机或发送运动命令。
使用包中CameraInfo和原始关节反馈；按停稳、时效、棋盘格质量筛选，以姿态组做交叉验证，
输出带升降补偿的候选、分组误差、角点叠图及HTML报告。当前求解器明确处理单高度包，
保留名义+Z相对高度公式，但不会将单高度重算标记为跨高度精度验收。

`recalibrate_j4_head.py`用于本次左右J4写零后的旧数据重算。显式传入旧外参`--calibration`、
J4事件`--zero-record`、原冻结训练/测试划分`--split`和新目录`--output`。
仅对历史q[3]/q[11]加真实写零前后坐标差并重算FK，不给当前SDK反馈加偏移，不改原包、角点或分组。
`render_standard_gripper_overlay.py --capture 已采集目录 --output 新目录`可输出实体RGBA模型和45%半透明叠图，
使用原图坐标、模型内部深度缓冲、透视正确深度和2倍抗锯齿；不连接或控制真机。

`audit_camera_height.py --output 新目录`仅GET读取当前SDK升降反馈，对照URDF的
`head_d455_color_optical_frame`与已选外参给出的相机相对J1高度、位置和朝向。
默认`--lift-offset-m 0.2`是现有待验收绝对零点映射；输出另列0/0.1/0.2m对照。
超外参记录高度时仅记录原公式的诊断外推，不修改正式范围；不采图、不运动、不写零或改外参。

`fit_overlay_height.py --train 配对采集目录 --test 另一配对采集目录 --calibration 诊断外参 --output 新目录`
离线扫描一个固定相机Z偏差，以真实红色夹指的分割区域和模型投影区域IoU选择训练最优值，
在未参与选择的照片上保持该值不变。保存三组颜色门限、固定ROI和分块分割原图供检查；
输出候选始终`diagnostic_only=true/deployable=false`，不修改源外参、角度、开合值或执行配置。
仅有颜色轮廓改善不能验收整臂或三维TCP；须结合独立姿态与高度检验，不能逐图调值冒充泛化。

`fit_linear_camera_height.py --output 新目录`在已选外参的可追溯J4换算数据上比较固定修正与
`ΔZ=a*(SDK高度−参考高度)+b`，只拟合新增高度参数，冻结其余外参和棋盘格安装。
复用带指纹的姿态留出集，并对新增参数做留一高度检验；输出自定义离线线性模型schema，
不修改生产外参、单位升降轴、SDK比例或机器人。历史数据来源可用`--calibration`显式指定。

`split_recorded_lift_levels.py BAG目录 输出JSON`可在真机ROS环境中读取一轮连续录制的
`/luxi/lift/state`，按机器人单调时间和SDK实测升降高度生成`height_segments.json`。
仅使用有效、无故障、零速且在窗口内高度跨度≤0.5mm的停稳区间；反馈长时间中断会切段，
同一高度反复出现会归到同组。它不改写原始包，不发送ROS消息；机械臂停稳及棋盘格质量
仍需在标定提取阶段另行检查。连续多高度采集不需要在每次换高度时停止录包。

`compare_residual_models.py --output 新目录`离线比较25种残差模型：固定/线性/二次高度、
相机整体平移和刚体关系、相机与棋盘安装、左臂逐关节常数/线性/sin以及整体腕端岭回归/RBF。
按已冻结姿态组划分训练与测试，并做训练内三折姿态验证；每组最多3帧用于优化、全部帧用于评估。
输入哈希、逐帧误差、参数、边界与秩保存在各模型JSON；不会向真机或正式外参写入补偿。

`collect_overlay_residuals.py --output 新目录`默认只生成SDK可接受的小幅采样计划。
现场确认运动范围已清空后，原命令加`--execute`，在当前保持姿态附近依次改变双臂J1～J7各±0.06rad，
速度0.03rad/s，每次停稳采图，结束后返回原始目标。不发送升降指令，不写零，不自动使能。
过期或异常反馈立即中止；只有明确未提交的本地静止门限拒绝可等待重试，未知写入不重发。
`--resume`仅接受相同计划、原请求已确认停止或到位且没有故障锁存的记录，不是清故障开关。
所有失败目录保留，采图现在还保存`capture_state.jsonl`中的原始SDK状态，便于定位门限拒绝。

`validate_residual_models.py --fits 拟合目录 --captures 清单JSON --output 目录`
进一步检验新增修正的留一高度表现、真实夹指轮廓，分别拟合左右臂关节及腕端安装假设。
清单每项包含`name/path/split`；新去畸变照片、固定ROI、三组颜色门限、完整网格验证均保留。
不将左臂修正复制到右侧；棋盘安装修正不作用于夹爪。投影完全出视野时像素距离记为null，
不会把OpenCV空距离图的哨兵值当成实测误差。轮廓拟合仅作诊断，不能认定为硬件零点或TCP精度。
`report_residual_models.py --root 本轮目录`可复建11月5日的交互报告；目录组织见
`outputs/calibration-rezero/20261105/model-comparison/`。

`fit_current_pose_sine.py --capture 配对目录 --calibration 诊断外参 --output 新目录`
离线查看单个位姿下`q_virtual=q+a+b*q+c*sin(q)`的局部贴合效果，沿用轮廓弱先验及参数边界。
当前入口仅针对11月13日画面已核对的可见红色内指（左ee_link2、右ee_link1）计分，
其余网格仍绘入实体叠图供交叉观察；不能将未计分部分记为已对齐。
一个姿态对每关节的常数/线性/sin基仅有秩1，21个参数不唯一，不提供多姿态鲁棒性结论。
本轮单独保存左爪负向开合诊断副本，不改原采集、生产映射、SDK、URDF或控制目标。
实际回放见`outputs/grasp/fixed-height-sine/20261113T085657Z/fit-visible-red/report.html`。

腕部下探停止的纯几何函数位于`wheel_perception_core/insertion.py`：在同一光学坐标中，
将导轨下表面与局部箱沿顶点投影到竖直方向，扣除显式净空/不确定性余量。
返回的是几何接受状态，不能代替完整控制器的时效、运动/碰撞、钩尖插入和闭合保护。
11月13日腕部叠图不符的诊断见`outputs/grasp/wrist-insertion-20261113/20261113T091309Z/report.html`；
原始资源与帧已保留。该函数未接入真机动作，不会因为高度条件成立自动闭爪或试抬。

## 视觉目标到传统运控（固定升降）

`plan_visual_grasp.py --preview 已完成的camera-preview目录 --capture 同期关节采集目录 --calibration 本次外参 --output 新目录 --steps 40`
将真实视觉估姿转换为`wheel_visual_grasp_target_v1`，按SDK +Y分配左右臂，明确区分箱沿点、
CAD夹持参考和SDK link7腕端。按“两指滑动导轨下表面”计算目标高度；本次试验采用10mm
名义净空；对准时整个夹爪最低点高于箱沿30mm，覆盖当前及张开宽度。
路径先竖直留净空，再等高平移/转向，最后竖直到对准位置。这些参数不等于物理精度或验收结论。
升降固定在同期反馈值。
输出`target.json`、`resolved_tool_targets.json`、`wrist_targets.json`、`plan.json`及抓取点报告。
该工具只规划，不发送机器人命令；场景碰撞检查未配置时不会将规划标为可执行。

视觉入口`preview_camera_grasps.py --clean-depth`可保留原SAM掩码并使用既有3D连通域清理前景干扰。
掩码清理后仍保留完整性、深度和外参质量拒绝，不能为执行而删掉拒绝原因。
本次真实数据入口为`outputs/grasp/pipeline-20261113/plan-final/report.html`。

实际执行组合入口为`wheel_execution.stationary_pick.create_stationary_pick`，接收实时目标源、
SDK关节/升降只读客户端、场景碰撞检查器和已验收的夹持接触策略。它组合了：

- `PlanningTargetSource(PipelineVision(...), rules, ...)`：保留同帧升降、图像身份、源时间和质量状态；
- `VisualGraspPlanner`：固定升降、箱沿/夹持点到腕端转换、分段IK及连续性/碰撞检查；
- `SDKWaypointDriver`：SDK公共关节接口逐段到位，保持原升降，不把离线五次插值误称为SDK流式控制；
- `SDKGripperOpener`：对准且停稳后才张开，保持手臂目标，双侧开到位后才允许下插；
- `WristClearanceSource`：新鲜双腕RGB-D在预期局部箱沿附近测深，按导轨下表面判断净空；
- `SDKContactCloser`：小步闭合、双侧接触与硬负载保护，夹爪到0不等于抓住箱体；
- `VisualTestLiftVerifier`：手臂试抬10mm后要求箱体视觉随动且双侧接触保持。

本次尚未把该组合部署为可用的真机抓取服务：当前腕部几何/深度、视觉质量、夹持策略和
场景碰撞检查尚未共同通过。移动搬运的原`SDKMotionExecutor`仍保留未就绪状态，不能把定点
抓取组合当作已实现运输/放置。禁止清空blockers、伪造接触策略或把回放时间改成当前时间来执行。

`visual_grasp_flow.py`提供一个只读组合入口，将新采图、视觉推理、固定高度IK和场景碰撞检查串起来：

```bash
.venv/bin/python tools/grasp/visual_grasp_flow.py \
  --calibration outputs/grasp/fixed-height-sine/20261113T085657Z/diagnostic_camera.json \
  --box 205 210 420 385 --clean-depth \
  --output outputs/grasp/my-new-flow
```

上述外参和ROI仅对应本次固定高度诊断，不是任意场景的默认配置。省略ROI时要求检测器恰好返回
一个目标；检测失败不会自动选框。`--capture/--preview`可复用同一图像对的记录，保留回放属性。
`flow.json`中的completed指只读处理完成，不能当作抓取成功；该入口不提供运动开关。

真机试验可同时启动只读记录器，保留10Hz关节反馈、2Hz SDK整机状态、三路2秒一次的原子RGB-D、
相机时间戳和失败原因：

```bash
.venv/bin/python tools/grasp/record_pick_run.py --output outputs/grasp/my-pick/telemetry --duration 60
```

记录器不发动作；实际动作的请求ID、目标和返回值另存事件日志。2026-11-15实测记录入口为
`outputs/grasp/live-pick-20261115-01/report.html`，包含0.2rad/s抬臂的主动停止及随后到位全过程，
不能把该次抬臂成功记成抓取成功。

2026-11-15用户允许放宽后，视觉抓取位置门限默认3mm，`--position-tolerance-mm`可显式指定
不超过5mm的值；其他IK入口仍默认2mm，SDK原关节限位不变。
`plan_visual_grasp.py --mesh-collision`在包围盒筛选后使用原始collision网格/实体复核，
保留原有余量、材料内部包含及碰撞拒绝；依赖可选`python-fcl`。
本次隔离环境`.runtime/collision/venv/bin/python`已具备依赖，具体版本保存在
`outputs/grasp/live-pick-20261115-02/collision_environment.txt`。该参数不绕过视觉/反馈验收，
也不提供运动开关。三段纯IK通过仍不代表完整抓取路径或真机执行通过。

`plan_visual_grasp.py`现在默认连接`GraspSceneCollision`：机器人用原URDF局部包围盒，
箱体用原CAD材料体素（5mm），保留开放空腔；不将整个箱体外包围盒填成实体。
同一刚体和直接关节连接的刚体组按拓扑排除内部相交；三轮采用覆盖任意转角的球包络外接盒，
没有虚构轮子反馈角。每个待执行关节段还会从实测起点重新检查碰撞，拒绝沿用已失效的检查结果。
当前检查只覆盖机器人和选定箱体，不代表周边环境/实物注册已验收。`--kinematics-only`只生成
离线IK诊断，不赋予执行权限。最新报告见`outputs/grasp/pipeline-20261113/flow-current/plan/report.html`。


## 2026-11-15 当前现场运行入口：复用已有资源、笔记本关节流

用户已明确确认：复用已有外参、箱体CAD抓取规则和夹爪定位逻辑，先看效果，不要求重新验收。
当前`grasp_execution.json`选择`resource_policy=user_confirmed_reuse`，这些历史验收状态仅记录到
`grasp_readiness.json.resource_warnings`，不再阻断演示；原资源文件中的标记和数值不伪改。

在笔记本仓库执行：

```bash
cd /home/luxi/qingyue/wheel-robot
bash tools/grasp/run_current_visual_grasp.sh check  # 只读视觉检查
bash tools/grasp/run_current_visual_grasp.sh align  # 仅对准，不执行后续开合
bash tools/grasp/run_current_visual_grasp.sh grasp  # 对准→张开→下插→闭合→10mm试抬
```

三种入口均复用现有现场外参副本（同一个已选矩阵及升降补偿）、已选箱子ROI、CAD与规则。
腕端目标到雅可比/限速均在笔记本内部完成；经有线UDP18034发送连续关节参考。
真机已部署joint_stream v1并返回图像门限2000ms。现有板端适配器周期10ms；
驱动器内部伺服频率没有在这次工作中实测，不将其写成已实现1kHz。

夹爪直接使用真实SDK已配置的open/closed端点及原POS_FORCE限流控制，
不再要求另外提供一份接触策略文件，不改真机电流限制。
闭合过程中先停止的一侧保持，另一侧继续闭合；这里只报告实测闭合/受阻状态，
最终效果由10mm试抬后箱体是否随动判断，不把到达闭合0位当作抓箱成功。
既有10mm导轨停止净空与5mm不确定性余量保留；下插最大行程200mm对应当前180mm箱体，
替代旧40mm临时调试边界，仍受实际关节限位与下插超越检查约束。

固定升降图像配对只把该帧实际前后样本用于间隔检查，较早样本用于确认高度/速度稳定，
避免旧采样缺口反复否决新图。图像仍按原始时间戳2s过期；关节反馈/断流150ms、
权限心跳500ms、原限位和跟随保护均保留。当前复用模式使用头部箱沿深度、
同期实测关节角与既有夹爪几何计算下插净空；strict模式才要求腕部50ms关节配对。
RGB/depth同设备时间差仍最多250ms。运行时图像质量不良、深度缺失、目标丢失、
控制权变化和SDK故障仍会停止，不因复用资源而放行这些真实错误。

每次执行打印新目录`outputs/grasp/visual-grasp-runs/时间-随机值/`，包括：

- `execution.log`：每次结束后自动生成的可读执行日志。该入口指向同目录`run/execution.log`，
  开头直接给出结果、停止位置、触发原因和SDK首个观测停止原因；随后按时间排列执行逻辑、
  模型加载/准备/状态机各阶段耗时、逐帧视觉耗时、IK与100Hz统计、停止回执和异常栈。
  无须事后手动运行统计命令。正常结束、Python异常或退出码130的中断退出均会汇总；
  即使在模型加载前失败，入口脚本退出处理仍会生成日志并保留原失败退出码。
- `launch.json`与`process_exit.json`：本次模式、启动时间及入口退出码。历史记录缺少退出码时
  显示未记录，不从成功/失败结果臆造进程退出码。
- `console.log`与`run/summary.json`：原始输出、完整异常栈、结果、停止回执及最终实测状态。
- `run/grasp/execution.json`：抓取状态机阶段。
- `run/execution_trace/events.jsonl`：资源指纹、运行ID、事件序号、图像及目标、SDK请求/回执、
  高频关节参考、实测角/速度/力矩、板端接收序号和故障原因。
- `run/execution_trace/frames/*.multipart`：包括被拒绝帧在内的解压后原始RGB-D multipart，
  对应事件记录原HTTP头、长度和SHA256；回放时直接解multipart，不再次按压缩头解压。

高频线程只入有界队列，独立线程写盘。日志失败停止发送，按归属停止自己的请求；
停止命令不依赖日志可用。UDP发送与板端真正接收分开记录。
`strict`策略保留给需要正式验收门槛的其他部署，不是本次用户确认的现场模式。

完整抓取状态机向同一事件流记录进入对准、张开、下插、闭合、试抬和停止的时间。
高频回路出错时另存`reference_loop_failed.detail.failed_tick`，记录失败当次dt、处理阶段、
目标、反馈与图像年龄，覆盖尚未生成`reference_send_attempt`就抛出异常的情况。
可读日志在运动线程退出后汇总，不在100Hz回路里同步生成或打印。
如果原始事件末行未写完，汇总器保留其余记录并指出损坏行；没有证据时明确写未记录。

历史或异常断电后的日志可只读重建：

```bash
python3 tools/grasp/execution_log.py \
  --run outputs/grasp/visual-grasp-runs/20261115T133207Z-2sJ1lp/run
```

对准的完整IK在独立进程低频求解，100Hz线程使用已求出的目标并限制关节速度、
加速度与实测跟随误差；等待新IK时保留旧图原始时间戳，不延长其2s有效期。
`/api/stream_state`返回各主题的原始接收时间，避免等待低频状态主题拖慢反馈读取。

2026-11-15实机已多次进入对准，尚未完成完整抓取。最近一次记录
`outputs/grasp/visual-grasp-runs/20261115T133207Z-2sJ1lp/run/`显示主机生成217个参考、
板端只接受到203；反馈本身仍新鲜。SDK接收器误把参考与滞后的内部插值输出比较，
导致合法连续参考被丢弃，最终显示反馈超时。修正候选改为检查相邻已接收参考的跳变，
首包仍检查当前位置，实际板端输出仍独立限制速度、加速度和原始关节限位。
候选验证通过与实际部署记录需分别核对，不能将软件测试当作已经抓箱。

2026-11-16按用户明确指示，左右J7在真机SDK中统一使用固定±52°运行限位
（±0.9075712110370514rad），不设置相邻腕姿态关联的范围。当前视觉IK和100Hz参考回路
直接读取SDK报告的限位，保留已有内侧减速余量；原URDF几何、关节零点和外参保持。
现场指令与部署记录见`config/robots/calibration/wrist_camera_interference_20261116.json`，
修改日志在`outputs/grasp/visual-grasp-runs/20261116T014656Z-j7-limit52/execution.log`。

### 执行耗时统计

下面命令只分析日志，不连接真机或发送运动命令：

```bash
.venv/bin/python tools/grasp/summarize_execution_timing.py \
  outputs/grasp/visual-grasp-runs/*/run \
  --output outputs/grasp/timing-analysis-20261115
```

输出`report.md`和`timing.json`，按运行/请求分别计算视觉处理、发送间隔、SDK往返和中断原因。
同一pair重复取图时只匹配质量事件之前的响应；不同运动会话之间的暂停不计入发送间隔。
旧日志的正文到质量事件包含解码、预处理、分割、位姿和质量检查，不能标成纯模型推理时间；
图像年龄还包含上游等待与传输，也不是识别耗时。

当前入口新增`vision_quality.detail.timing_s`，分别记录取图解码、预处理、分割与深度处理、
位姿注册/跟踪及质量检查；`pose_mode`区分首帧与连续跟踪。实际结果返回CPU数组后才计时，
不是仅记录CUDA异步提交。`alignment_endpoint_ik.detail.solver.solve_wall_s`记录完整IK求解，
`reference_sent`新增`udp_send_s`和`compute_to_send_s`。不改变运动参数、图像原时间戳或控制周期。

只读实时采样可使用原`check`入口。它能测视觉，不会生成运动参考；关节发送间隔必须使用
既有真实运动日志，不能把只读采样中的零条指令当成回路卡顿。没有逐包板端到达时间，
本统计不宣称已测得单向网络延迟或电机驱动器内部伺服周期。

2026-11-16 默认入口更新：`run_current_visual_grasp.sh`默认配置现与
`run_moveit_visual_grasp.sh`一致，均走板端MoveIt/JTC。启动会输出实际后端；
网关未就绪则退出并保留日志，不回退笔记本关节流。旧后端仅在显式指定
`--execution-config config/robots/grasp_execution.laptop-legacy.json`时使用，
该文件为历史配置，不代表已解决J6实物跟随问题。
