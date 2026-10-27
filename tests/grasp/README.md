# 抓箱核心测试

运行 `.venv/bin/python -m pytest -q`。默认仅收集主工程 `tests/`；旧仿真专用测试已移除，可从 Git 历史查阅。

测试覆盖损坏/重复 MIME、实际 HTTP 解析、缺测与回放标记、左右关节顺序、升降映射、租期中断、URDF 活动相机、15 自由度 IK、不可达、轨迹及保守碰撞、状态转换和合成几何集成。任务测试另外覆盖三模块调用顺序、重新观测、过期/错误目标拒绝、未校验轨迹拒绝、超时、取消、可能持载时保持，以及未接入硬件时拒绝启动。

URDF 用例需要项目本地外部资产；缺失时明确 skip。真实图像回放用例需要 outputs/grasp/head_rgb_20261016.npz，本文件为只读网关抓取，不入库。合成后端明确为测试替身，不替代 NanoSAM/FoundationPose 模型验收。


## 本目录文件结构

| 文件 | 功能 |
| --- | --- |
| `test_association.py` | 对应功能的算法/协议/任务回归测试。 |
| `test_coordinator.py` | Application integration with fake ports; no device commands or model inference. |
| `test_core.py` | 对应功能的算法/协议/任务回归测试。 |
| `test_depth_components.py` | 对应功能的算法/协议/任务回归测试。 |
| `test_download.py` | Model transfer resumes safely even when a server ignores Range. |
| `test_live_trace.py` | Diagnostic artifacts and failure localization, independent of GPU models. |
| `test_pipeline.py` | Synthetic geometry verifies integration, explicitly not model accuracy. |
| `test_run_logging.py` | 对应功能的算法/协议/任务回归测试。 |
| `test_surface_refinement.py` | 对应功能的算法/协议/任务回归测试。 |
| `test_vision_service.py` | Task adapter preserves the retained pipeline's identity and diagnostic gate. |
| `test_visualization.py` | 对应功能的算法/协议/任务回归测试。 |
