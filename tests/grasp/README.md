# 抓箱核心测试

运行 `.venv/bin/python -m pytest tests/grasp -q`。

测试覆盖损坏/重复 MIME、实际 HTTP 解析、缺测与回放标记、左右关节顺序、升降映射、租期中断、URDF 活动相机、15 自由度 IK、不可达、轨迹及保守碰撞、状态转换和合成几何集成。

URDF 用例需要项目本地外部资产；缺失时明确 skip。真实图像回放用例需要 outputs/grasp/head_rgb_20261016.npz，本文件为只读网关抓取，不入库。合成后端明确为测试替身，不替代 NanoSAM/FoundationPose 模型验收。
