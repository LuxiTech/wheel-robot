# 导航工作目录

本目录预留给真机 Nav2 地图导航/定位及 AprilTag 工位对接。`adapter.py` 目前只定义拒绝执行的占位适配器，`ready=False`。

实现目标：

1. 根据 `config/navigation/` 中的工位 ID 获取地图目标，发起导航并等待实际结果。
2. 在工位附近检测 AprilTag，使用实测 Tag/底盘变换完成精确对接。
3. `go_to()` 在对接完成且确认底盘静止后返回。
4. `stop()` 取消导航/对接，并等待停止确认；断流由机器人本地控制租期保护。

尚未选择或实现 Nav2 定位配置、传感器驱动、AprilTag 节点和底盘速度适配。底盘控制须经 SDK 支持的仲裁入口，不能假定现有 SDK 已订阅 `/cmd_vel`。

可在本目录继续添加 `nav2_client.py`、`apriltag_docking.py` 等实现；完成后在 `main.create_application()` 注入适配器。地图、Tag 标定和机器人局部资源由实际部署提供。
