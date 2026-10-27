# 真机头部视觉诊断

`tools/grasp/perceive_live.py` 是独立的只读入口，复用 `GraspPipeline` 的检测、分割、位姿质量校验和箱沿抓取规则，不启动任务状态机或运动控制。模型和标定缺失会报错，不输出伪造抓取点。

## SSH 取图

先在一个终端建立隧道，密码在 SSH 提示中输入，不写入配置或脚本：

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:18080:127.0.0.1:18030 sunrise@192.168.0.245
```

板端 SDK 的 RGB-D 网关负责调用相机驱动；本机经隧道只读 GET `/api/cameras/head_rgb/rgbd`。RGB、深度、内参来自同一 `pair_id` 的原子响应。采集程序不自动启停板端服务。特别注意板端 `luxi-depth-head_rgb.service` 声明与 `luxi-robot.service` 互斥，需按板端部署安排相机进程。

在仓库根目录执行：

```bash
.venv/bin/python tools/grasp/perceive_live.py --capture-only
```

默认校验源帧龄加完整 HTTP 往返时间不超过 250 ms。SSH 首次实测出现约 320 ms，默认拒绝并保留原始响应。仅在静止场景做只读诊断时，可显式使用 `--max-age-ms 1000`；这不会修改运动流程门限。图像有深度不意味着坐标已经具备执行条件：当前头部网关提供 `source_depth_z_resampled_to_color` 和 `realsense_2` 畸变，相关度量语义必须继续验证。

## 推理与资源

使用隔离的 `.runtime/vision/venv`，配置是 `config/robots/runtime.live.yaml`。模型权重沿用已校验的 `assets/models`，本机引擎由 `tools/grasp/build_engines.py` 构建，不能借用其他 GPU 的 engine。

本机 RTX 5090 使用 Python 3.11、PyTorch 2.7.1/cu128、TensorRT 10.8；与历史容器环境分开。安装脚本仅准备检测/分割，不宣称 FoundationPose 的 CUDA 扩展已可用：

```bash
# 先按模型环境文档准备 third_party 与模型权重。
mkdir -p .runtime/vision
CUDA_HOME=/path/to/cuda UV=uv tools/grasp/setup_live_vision.sh \
  > .runtime/vision/setup.log 2>&1
HF_HOME="$PWD/assets/models/huggingface" HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 \
  .runtime/vision/venv/bin/python tools/grasp/build_engines.py
```

Ubuntu 26.04 实测遇到 TensorRT 10.8 构建库的可执行栈加载错误，参见 [NVIDIA 问题记录](https://forums.developer.nvidia.com/t/linux-gblic-2-41-unable-to-load-library-libnvinfer-builder-resource-so-10-7-0/323729)。安装脚本调用 `fix_trt_execstack.py`，仅修正 `.runtime/vision/venv` 内该库：先复制备份并断开缓存硬链接，再清除标志；原件和前后 SHA256 保存在 `.runtime/vision/`，不修改系统库。NanoSAM 上游缺少包初始化文件，脚本同时在此 venv 写入源目录 `.pth`。

封装入口会设置本仓库的离线模型缓存：

```bash
tools/grasp/perceive --max-age-ms 1000
# 使用保存的同一帧重复定位模型问题（必定标记为诊断）
tools/grasp/perceive \
  --replay outputs/grasp/live/RUN_ID/observation.npz
```

真机诊断配置使用 `a blue plastic box` 和 0.15 门限，针对当前画面的中央蓝色箱子；换物体应修改 `model.prompt`。首次 PyTorch 预检中，原 `a plastic crate` 对该箱分数约 0.075，新提示词约 0.413；这不是跨场景精度验收。

检测为零个或多个目标时会保存候选框并停止。检查 `03_detections.png` 后，可用 `--box x1 y1 x2 y2` 明确选择**校正图**中的 ROI。该方式跳过自动检测选择，日志标记 `explicit_roi`，不能当作自动检测成功。

需要实物资产才能继续位姿和抓取点：

- `model.mesh`：实物箱网格，单位米；坐标原点和朝向须与箱沿规则一致。
- `model.colliders`：对应网格的表面几何，用于深度点到模型表面的残差验证。
- `rules`：实测箱宽、高、箱沿内缩、夹取深度等。示例参数不能自动视为实测参数。核验后才设置 `object_geometry_validated: true`。
- `urdf`：真实机器人及头部相机光学帧安装变换，核验后设置 `camera_extrinsics_validated: true`。可动链需要 `--joints` 提供对应实测关节，固定链不需要无关的机械臂关节。

所有结果均为 `executable: false`；本入口完成后仍是诊断结果，不向 SDK 发送运动指令。

## 日志和图片

每次独立创建 `outputs/grasp/live/<UTC时间>-<随机ID>/`：

| 文件 | 用途 |
| --- | --- |
| `run.json`, `config.json` | Python/依赖版本、Git commit/工作区状态、运行配置、ROI和帧龄门限 |
| `vision.events.jsonl` | 每阶段开始、结束、耗时、失败及 UTC/单调时钟 |
| `sha256.json`, `joints.json`, `models.log`, `console.log` | 算法与模型文件指纹、使用的关节值、模型 Python 日志及原生 TensorRT/CUDA 控制台输出 |
| `vision.error.json` | 最内层失败阶段、异常、调用栈；后续阶段不会覆盖 |
| `response.multipart`, `response.headers.json` | 网关原始响应；解码/过期失败时也保留 |
| `observation.npz`, `observation.json` | RGB/浮点米制深度和相机内参、pair_id、时间信息 |
| `00_rgb.png`, `01_rectified_rgb.png`, `02_depth.png` | 原图、校正图、深度有效区域；深度显示范围记于 `preprocess.json` |
| `03_detections.png`, `detection.json` | 所有候选框、分数或人工 ROI |
| `04_mask.png`, `05_segmentation.png` | 二值掩膜、叠加结果；深度质量见 `segmentation.json` |
| `06_pose_unvalidated.png`, `pose*.json` | 模型表面投影、相机坐标位姿、残差校验；投影生成不代表校验通过 |
| `07_grasp_tcp.png`, `keypoints.json` | 左右手 TCP 投影及抓取/预抓取变换；不判断遮挡 |
| `status.json`, `result.json`, `report.html` | 最终状态、完整结果（成功时）、已完成阶段的图片报告 |

出错时只展示已完成阶段的图，不补造后续图片。`capture_only` 只说明取图和预处理完成，`diagnostic_complete` 才说明整个算法路径完成。目录含真实现场图像，默认不进 Git；目前不自动删除历史运行，需按项目保留期限手动归档。

## 原生 FoundationPose 环境

先准备上述检测环境，原生位姿源版本单独锁在 `config/deployments/native-pose.sources.lock.json`（PyTorch3D 0.7.9、nvdiffrast 0.3.3）。`setup_native_pose.py` 在同一隔离环境构建 CUDA 扩展和上游 `mycpp`，不安装系统包。需要 CUDA 12.8 完整开发工具链以及兼容的 GCC/G++（本机使用已有 GCC 13.4）。`CUDA_HOME` 应具有 `bin/nvcc`、`include` 和 `lib64`；Conda 分离布局可在 `.runtime/vision/cuda` 建立指向现有工具链的符号链接，不能改动原环境。

本机 Ubuntu 26.04 的 Eigen/Boost 头文件仅下载并解包到项目目录：

```bash
mkdir -p .runtime/vision/debs .runtime/vision/sysroot
(cd .runtime/vision/debs && apt-get download libeigen3-dev libboost1.90-dev)
for package in .runtime/vision/debs/*.deb; do
  dpkg-deb -x "$package" .runtime/vision/sysroot
done
.runtime/vision/venv/bin/python tools/grasp/setup_native_pose.py \
  --cuda-home .runtime/vision/cuda \
  --cc /path/to/gcc-13 --cxx /path/to/g++-13 \
  --headers .runtime/vision/sysroot/usr/include
.runtime/vision/venv/bin/python tools/grasp/smoke_native_pose.py
```

`mycpp` 使用上游两个源文件及头文件直接编译；其当前代码只用 Boost 头文件，不需要上游通用 CMake 配置中额外要求的 Boost system/program_options 库。编译和 JIT 环境保存在 `.runtime/vision/native-build-env.json`，推理入口加载它；Torch、Warp、Matplotlib 缓存也保存在 `.runtime/vision`。实际安装清单在 `.runtime/vision/installed.txt`。

`smoke_native_pose.py` 使用程序生成的长方体与渲染 RGB-D，执行真实 FoundationPose 注册与跟踪，仅用于验证模型运行环境。数据保存在 `outputs/grasp/native-pose-smoke/`，明确标记 `synthetic: true`、`real_box_accuracy_validated: false`，不能替代真实箱体网格或抓取精度验收。

## 本机验证记录（2026-10-23）

- 真实头部 RGB-D：`outputs/grasp/live/20261023T071458Z-b8e8b07d`，640×480。
- 保存真机图像的自动检测/分割：`20261023T073259Z-a4435062`，中央箱分数约 0.412，掩膜有效深度比例约 89.3%；随后因实物几何未配置停在 `pose_init`。
- 新鲜在线画面：`20261023T073321Z-e5fbf7dd` 检出两个箱子，按预期拒绝歧义；显式中央 ROI 的 `20261023T073447Z-de69c0de` 分割通过，仍停止于实物几何要求。人工 ROI 的运行不算自动目标选择成功。
- FoundationPose：`outputs/grasp/native-pose-smoke/9ed3bd75` 完成合成 RGB-D 注册和跟踪。该项仅证明模型可运行，不证明实物箱位姿或抓取点精度。

这些目录都是本机产物，不随 Git 分发。当前没有生成真实箱体的已验证位姿或抓取点，也没有执行任何机械臂/夹爪运动。

## 已导入的实物 CAD（2026-10-23）

用户提供 `/home/luxi/下载/码垛.STEP`，原件复制至 `assets/objects/crate/source/box.step`；米制网格为配置已引用的 `assets/objects/crate/model.obj`，尺寸约 445×310×180 mm，源文件包含两个实体。转换保留原坐标方向，并将 XY 中心、最低 Z 作为网格原点；清理 4 个零面积三角形后网格封闭、法向绕序一致。原始转换网格也保存在 `source/model.raw.obj`。来源、转换指纹见 `model.cad.json`，项目内登记见 `config/recipes/crate-cad.json`，预览见 `cad_preview.png`。

**当前 CAD 开口朝 -Z，原抓取规则假定开口朝 +Z。** 必须先统一开口朝向、重新提取抓取几何及表面校验模型；旧示例中的 0.4 m 宽、0.22 m 高不能继续当作该箱参数。尚未自动将 `object_geometry_validated` 设为 true。仅看相机坐标下的抓取点投影不需要整机 URDF；转换到机器人基座、驱动夹爪时才需要真实安装变换、TCP 和相应关节状态。深度值语义仍需校验。

转换环境与视觉环境独立：

```bash
uv venv --python 3.11 .runtime/cad/venv
uv pip install --python .runtime/cad/venv/bin/python cadquery-ocp==8.0.1.1.0 trimesh==5.1.1
.runtime/cad/venv/bin/python tools/grasp/import_step_mesh.py \
  assets/objects/crate/source/box.step --output assets/objects/crate/model.obj
```

工具拒绝覆盖已有模型；重转换时指定新输出文件。STEP 原件已在后续提交中纳入 Git；OBJ、权重和历史图片仍被忽略，克隆代码不会自动获得这些生成资产。

## 相机图上的抓取候选点预览（已运行）

用户授权先查看图像投影后，新增 `tools/grasp/preview_camera_grasps.py`。它在独立输出目录将当前 CAD 从开口 -Z 翻转到 +Z，实际调用 NanoOWL、NanoSAM 和 FoundationPose，并直接在相机坐标生成箱沿候选 TCP；不使用虚拟 URDF、不设置实物/外参已验证标志、不调用机械臂。该入口专供当前开口朝 -Z、原点位于最低 Z 的 CAD，不能不检查朝向就替换其他模型。

```bash
.runtime/vision/venv/bin/python -u tools/grasp/preview_camera_grasps.py \
  outputs/grasp/live/20261023T091358Z-18088eab/observation.npz \
  --target-pixel 325 225
```

指定像素必须落在唯一检测框内，用于从多箱场景明确选择中央箱。当前规则使用 CAD 的 445 mm 宽、180 mm 高，候选点设在相对两侧箱沿中点，默认从外沿内缩 7 mm、低于顶面 1 mm；这些夹取偏移和 TCP 朝向尚未验证。A/B 按图像从左到右编号，不代表已分配机械臂。

结果目录：`outputs/grasp/camera-preview/a1e74aa8c4/`。`06_pose_overlay.png` 为 CAD 渲染轮廓，`07_grasp_candidates.png` 为 A/B 候选点，`08_comparison.png` 为并排对比，`report.html` 汇总完整阶段图片。`pose.json`、`keypoints.json`、`cad_and_rules.json` 保存相机位姿、点坐标、投影和使用的模型变换/规则。该帧轮廓 IoU 约 0.936；这是模型轮廓与分割结果的重合指标，不是抓取成功率或毫米精度保证。深度与外参未校准，全部输出保持 `diagnostic_only: true`、`executable: false`。

自动检测漏检时，可显式指定校正图中的 ROI 继续诊断，并在 `detection.json` 中记录 `explicit_roi`，不计作自动检测成功：

```bash
.runtime/vision/venv/bin/python -u tools/grasp/preview_camera_grasps.py \
  outputs/grasp/live/20261023T092211Z-58243ce4/observation.npz \
  --box 212 228 424 384
```

该次近距离、夹爪遮挡画面的输出为 `outputs/grasp/camera-preview/6de135db8b/`，轮廓 IoU 约 0.822，投影点与夹爪区域重叠；不能据此认定实际夹取准确。


## 0.3.0 结构迁移

本文保留历史实验结果。当前算法在 packages/wheel_perception_core，入口仍为 tools/grasp；runtime.live.yaml 位于 config/robots，共享箱体规则位于 config/recipes。运行工具已适配新导入，不修改既有结果目录或 GPU 引擎。
