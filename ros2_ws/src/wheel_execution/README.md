# wheel_execution

真机操作执行边界。executor.py 保留运输、规划、预抓取、接近、夹持验证、试抬、放置与保持接口。ready=False，尚未接通厂商轨迹执行。后续由本包统一管理动作反馈与取消。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [resource/](resource/README.md) | 实现子目录；见目录内 README.md。 |
| [wheel_execution/](wheel_execution/README.md) | 实现子目录；见目录内 README.md。 |
| `package.xml` | ROS 包名、依赖与构建类型。 |
| `setup.cfg` | ament 可执行脚本安装位置。 |
| `setup.py` | Python/ament 安装、资源索引与 console script 定义。 |

## 构建边界

从仓库根安装核心库到 ROS 使用的解释器，再在 ros2_ws 中 colcon build。源码工具通过统一 bootstrap 可直接运行；硬件占位接口不会伪造成功。
