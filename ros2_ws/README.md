# ros2_ws

ROS 2 工作空间；src 为受维护源码，build/install/log 为生成目录并被 Git 忽略。核心 Python 库需先安装到 ROS 使用的解释器。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [src/](src/README.md) | 十个独立 ROS 工作包。interfaces 定义通信，description/bringup 管资源与装配，其余包按功能边界适配核心算法和设备。目录存在不代表所有硬件能力已接入。 |
