# models

放置 NanoSAM/FoundationPose/OWL-ViT 权重、Hugging Face 离线缓存和 GPU 引擎。模型由 config/deployments/models.lock.json 校验，README 以外大文件默认不入库。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [engines/](engines/README.md) | 本机 TensorRT 引擎输出位置。引擎依赖 GPU/驱动/运行库，不能把另一设备的引擎当作可移植模型。重构不重新构建或替换现有文件。 |
