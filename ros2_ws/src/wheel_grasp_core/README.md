# RGB-D 抓箱核心实现

实现入口位于 `wheel_grasp`，可脱离 Isaac 和 ROS 使用。先集中实现可测试的核心，再按进程边界拆分；开发文档中的多个逻辑模块并不要求一开始各复制一套数据结构。

当前默认开发模式为纯仿真：使用真实 URDF 的传感器安装参数，在 Isaac 渲染 RGB-D 并读取仿真关节，再运行模型感知、抓取点生成、联合 IK 与仿真接近动作。入口 `tools/grasp/environment run` 不请求真机；SDK 保留为未来显式 `--source sdk` 的接口。日志、环境与迁移方式见 [隔离 Docker 环境](../../../tools/grasp/README.md)。夹紧/提箱尚未接入。

## 已实现

- 最新 SDK `ba0c732` HTTP 原子 RGB-D 接收、严格 MIME 解码、组身份、单位、大小、时效校验与去重。
- 无损 NPZ 回放，保留 metadata、RGB 和 float32 深度；回放始终为诊断输入。
- 具名 URDF 正向运动学、固定/活动相机安装链和缺失反馈拒绝。
- 14 轴双臂与升降共 15 自由度的有界联合 IK，双手完整姿态约束。
- 连续目标插值、共同时间参数化、限速、IK 分支检查和碰撞回调。
- URDF 包围盒保守碰撞检查，可加载碰撞网格；不会把“未检查”当成通过。
- NanoOWL/NanoSAM 和 FoundationPose 原版的延迟加载适配器；缺权重报错，不输出假推理。
- mask 深度质量、箱体表面残差、跟踪跳变检查，以及箱沿抓取/预抓取规则。
- SDK 双臂顺序、升降坐标映射、本机外部租期包构造及任务状态转换。
- ROS 成对 RGBDFrame 录制节点、具名关节发布节点、URDF TF launch 和自定义消息/action 契约。

## 本机运行

从项目根目录执行：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r tools/grasp/requirements.txt
tools/grasp/grasp doctor
tools/grasp/grasp capture --output outputs/my_observation.npz
tools/grasp/grasp inspect outputs/my_observation.npz
.venv/bin/python -m pytest tests/grasp -q
```

输出目录需先创建，已有文件不会覆盖。capture 只执行 GET，不发送运动请求。网关可由 `--url` 修改。配置为 `config/grasp/runtime.yaml`，相对路径按仓库根目录解析。

`frames --joints measured.json` 计算三台相机的基座变换；measured.json 为关节名到实际位置的 JSON，至少包含 lift_joint 与两臂各 7 关节。未知/无效关节不允许补零；若仅使用头部固定变换，可直接调用 RobotModel.transform(head_frame, {})。

`ik --joints measured.json --targets targets.json --fixed-lift 0.15 --output plan.json` 生成诊断轨迹。targets.json 包含 left、right 两个 4×4 基座坐标目标矩阵。省略 fixed-lift 可进行联合升降规划，但不代表 SDK 能同步执行。所有 CLI 规划结果目前 executable=false；无硬件轨迹 action server。

## 模型环境

`config/grasp/dependencies.lock.json` 固定外部源码版本，源码放在忽略入库的 third_party。模型运行还需在兼容 CUDA/TensorRT/PyTorch 的独立环境按各上游说明安装依赖和权重；本地 Python 基础测试环境不等于模型运行环境。

Docker 配置 `config/grasp/runtime.container.yaml` 已填写三个引擎路径和容器内 FoundationPose 路径。若选择自行配置本机模型环境，再填写 runtime.yaml 的 owl_engine、sam_encoder、sam_decoder，并准备兼容依赖。可设置 PYTHONPATH 包含 nanoowl、nanosam 源目录，或按上游安装。当前适配器为 FoundationPose 原版接口，尚未实现 C++/TensorRT 位姿部署后端。

已知箱体几何可从本地 collider 资产生成试验网格：

```bash
.venv/bin/python tools/grasp/build_crate_mesh.py
```

生成模型保留箱底坐标参考，不代表已验证与所有实物箱一致。正式感知通过 `perceive observation.npz --joints measured.json --target-id crate_1 --box x0 y0 x1 y1 --output result.json` 运行。box 指经过输入适配后的图像；省略时只在恰好一个检测结果时选择目标，多箱不擅自选取。

## ROS 环境

在 Ubuntu 22.04 / ROS 2 Humble 上构建，并在同一 overlay 中提供最新版 luxi_interfaces。不要用当前主机的 Python 3.14 虚拟环境替换 Humble 的系统 Python。

```bash
source /opt/ros/humble/setup.bash
# 先 source 与 SDK ba0c732 一致、按当前机器架构编译的 luxi_interfaces overlay
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 run wheel_grasp_core wheel-grasp-rgbd --ros-args -p camera:=head_rgb
```

`description.launch.py` 需要显式传入 urdf 绝对路径和实测 lift_offset_m；状态桥位于机器人本机，使用本机 monotonic 核对 SDK 时间。这里只发布实际 joint_states/TF，不提供电机控制。腕部动态 TF 的完整在线时间验证尚待 ROS 环境完成。

## 当前执行边界

- 独立 `wheel-robot-sdk-bridge:ba0c732` 镜像已完成 colcon 构建、SDK／项目消息导入和 21 项抓箱核心测试；跨机器 ROS 通信尚未验收。
- NanoSAM ONNX、OWL-ViT 权重及 FoundationPose 两组权重已下载并通过锁文件校验；独立推理镜像、三个 TensorRT 引擎及真实模型冒烟测试已通过，详见环境 README 与运行报告。合成几何测试只验证模块衔接，不是模型效果证明。
- 真实头部 SDK 数据是 realsense_2 且保留源深度 Z 语义。已实现逆 Brown 数值映射，仍保留 device-reference/metric-validation 诊断标记，不能据此宣称抓取精度合格。
- 基线 TCP 是仿真参数；lift_mapping 未知时保留 null。不可猜测设备升降零点。
- URDF OBB 碰撞是保守近似，需提供完整碰撞链关节与场景障碍。现有 SDK 没有轮角反馈，整机碰撞链需要按真实可观测状态或经过验证的保守包络接入；默认 CLI 不宣称已检查碰撞。
- 双臂自主控制权与轨迹入口需要 SDK 端配套实现。LocalLease 只构造底盘/升降协议包，不发送它们，也没有自动续租旧远端目标。

任务保持、取消、抓持验证与低层指令的真实闭环绑定尚未完成。该实现支持只读数据和离线规划，不能直接作为整机自主抓取已经交付的证明。
