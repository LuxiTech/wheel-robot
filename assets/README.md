# assets

模型、CAD、机器人和生成资产工作目录。README 被跟踪，大资产默认忽略；STEP 原件已在 Git 中。此轮不移动或重建任何现有二进制资产。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [models/](models/README.md) | 放置 NanoSAM/FoundationPose/OWL-ViT 权重、Hugging Face 离线缓存和 GPU 引擎。模型由 config/deployments/models.lock.json 校验，README 以外大文件默认不入库。 |
| [objects/](objects/README.md) | 对象模型工作目录。crate 保存箱体 CAD 原件和本地转换资产；各对象的坐标方向、单位和抓取规则需一致。 |
| [robots/](robots/README.md) | 外部机器人 URDF 与配套网格的现有资产位置。真实文件缺失时运动学测试跳过；已验证资产可在后续规范到 wheel_description，不能在此次迁移中补造。 |
