# config

配置分为机器人、工位、工艺与部署四类。机器人 profile 引用相对 recipe，配置只读合并。未验证值保持 false/null，历史结果中的旧配置不回写。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [deployments/](deployments/README.md) | task.yaml 定义单箱任务与超时；各 lock.json 固定模型、源码和环境版本；validated-environment.json 为历史验证基线。锁文件迁移不改变权重与版本。 |
| [recipes/](recipes/README.md) | 箱型与工艺。crate.example.yaml 集中历史示例的箱体模型引用和抓取规则；crate-cad.json 记录已导入 STEP 的尺寸/来源。CAD 尺寸与历史示例不同，尚未将示例视为已验证实物规则。 |
| [robots/](robots/README.md) | runtime.yaml 为默认只读/离线配置；runtime.live.yaml 为本机 GPU 视觉配置；runtime.container.yaml 为历史容器配置。recipe 相对本 YAML 解析，模型资产路径相对仓库根。相机外参/TCP/升降标定仍未完成。 |
| [stations/](stations/README.md) | 工位与导航参数。stations.example.yaml 登记地图坐标、Tag 和对接变换；null 表示尚未配置，不能用于实际导航。 |
