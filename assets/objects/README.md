# objects

对象模型工作目录。crate 保存箱体 CAD 原件和本地转换资产；各对象的坐标方向、单位和抓取规则需一致。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [crate/](crate/README.md) | 箱体资产。source/box.step 已被跟踪；model.obj、colliders.json、model.cad.json 与预览图为本地生成/外部文件。CAD 源开口朝 -Z，预览脚本独立翻转，不代表主执行链已完成标定。 |
