# 本次结构迁移清单

迁移前基线为 `6726ba1`。重构在当前 `dev-real` 工作区进行，没有删除外部资源、虚拟环境、已下载源码或运行结果。

## 保留并移动

以下旧路径相对 `ros2_ws/src/wheel_grasp_core/wheel_grasp/`：

| 旧文件 | 新位置 | 保留内容 |
|---|---|---|
| `rgbd.py`、`ros.py` | `vision/` | 成对 RGB-D、严格解码、录制/回放、ROS 接收 |
| `backends.py` | `vision/` | NanoOWL/NanoSAM/FoundationPose 适配 |
| `geometry.py`、`pipeline.py` | `vision/` | 输入几何、箱体位姿、深度质量、箱沿抓取规则 |
| `association.py`、`visualization.py` | `vision/` | 目标 ROI 关联、投影和结果可视化 |
| `kinematics.py`、`planning.py`、`collision.py` | `motion/` | URDF FK、双臂升降 IK、轨迹与碰撞 |
| `sdk.py`、`ros_state.py` | `motion/` | SDK 顺序/坐标/租期、实测关节发布 |
| `state.py` 中的 `JointHistory` | `motion/feedback.py` | 曝光时刻关节插值、拒绝外推 |
| `state.py` 中的 `TaskState` | `task/state.py` | 细粒度抓取阶段校验，供动作实现复用 |
| `tools/grasp/pipeline_worker.py` 中的 `measured_state` | `motion/feedback.py` | 真机反馈健康度、时效和升降映射检查 |

新增 `contracts.py`、`task/coordinator.py`、`vision/service.py`、`motion/executor.py`、`navigation/adapter.py`、`main.py` 和 `tools/robot`。

Python import 改为 `wheel_grasp.vision.*`、`wheel_grasp.motion.*`、`wheel_grasp.task.*`。旧平铺 import 不再提供兼容层。SDK 和感知镜像需重新构建，打包通过 `find_packages()` 包含子模块。

## 已删除的旧仿真内容

旧仿真代码曾临时集中到 `legacy` 目录，现已按清理要求删除本地归档。历史实现仍保存在 Git 提交 `6726ba1`。

| 原内容 | 删除原因 |
|---|---|
| 整个 `scripts/`，含双机接力、网页、录像和实验导航 | 不作为真机启动路径 |
| `relay_runtime.py`、`relay_task.py` 等 | 依赖仿真接触、世界位姿、固定库存与推台 |
| `arm_ik.py`、`whole_body_control.py`、`gpu_whole_body_ik.py` | 主路径统一使用 `motion/kinematics.py`；旧 CPU/CUDA IK 可从 Git 历史恢复 |
| `run_pipeline.py`、`pipeline_worker.py`、`run_relay_model.py`、`relay_model_worker.py` | 原流程最终调用仿真，不是硬件执行器；反馈检查已迁入 `motion/feedback.py` |
| 原场景、接力、导航及仿真 profile | 实机参数按工位、机器人和实物箱建立 |
| 仿真 compose、Blender/FAST-LIVO Dockerfile | 不进入真机容器链路 |
| 原非抓箱测试、抓箱仿真专用测试 | 当前仅保留主工程算法与任务测试 |
| 原 README、历史设计/验收记录 | 历史结论不代表真机已验收 |

另外清理了 Blender 下载缓存、Isaac 运行目录、空的旧生成资产目录及 Python/pytest 缓存。虚拟环境、模型源码、SDK、现有运行报告和 uv 安装缓存继续保留。

若要复跑完整旧版本，请创建原提交的独立工作区并按其 README 准备外部资产：

```bash
git worktree add --detach ../wheel-robot-sim-reference 6726ba1
```

## 继续使用

`third_party/lift_robot_ros2`、模型锁与源码锁、`tools/grasp` 模型下载/校验/引擎工具、`runlog.py`、两个 SDK/推理 Dockerfile、`compose.grasp.yaml`、ROS interfaces 和纯核心测试继续保留。

`compose.grasp.yaml` 已移除仿真服务；`tools/grasp/environment run|relay` 会明确报入口已移除的提示。真正任务入口为 `tools/robot`，但硬件适配器尚未实现。

外部箱体模型配置由 `assets/generated/...` 改为 `assets/objects/crate/`，不再需要先构建双机场景才能开发视觉。配置中的现有 TCP 和升降映射验证标志仍然保留，不能把迁移视为标定完成。
