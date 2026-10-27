# wheel_sdk_bridge

厂商协议的唯一适配边界。sdk.py 处理升降映射、关节反馈和租期；feedback.py 对齐实测状态；ros_state/rgbd_ros 是只读 ROS 入口。厂商 SDK 本体不在本仓库。ROS main 需要板端 luxi_interfaces。 本目录只放 Python 实现；安装和 ROS 元数据位于上级。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| `__init__.py` | wheel_sdk_bridge: runtime adapter boundary. |
| `feedback.py` | Timestamped measured feedback and fail-closed task transitions. |
| `rgbd_ros.py` | Optional ROS 2 atomic subscription. Records observations; never moves robot. |
| `ros_state.py` | Robot-local SDK feedback -> named measured joints. No motion publisher. |
| `sdk.py` | Real feedback mappings and local leases. This module sends no commands. |

| `http_client.py` | 有界 SDK JSON HTTP，不重试 POST。 |
| `lift_client.py` | 机器人状态 GET 与底盘反馈判断。 |
