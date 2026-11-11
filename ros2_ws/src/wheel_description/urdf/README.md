# urdf

预留描述文件目录。用户模型原件保存在仓库根 `assets/robots/r680_lift_openarm_claw1.3/`，
CMake 将其安装到本包 share 下的 `urdf/r680_lift_openarm_claw1.3/`，无需重复维护源码副本。
launch 要求调用者显式提供 URDF；模型导入不代表 SDK 坐标和 TCP 已完成真机验证。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
