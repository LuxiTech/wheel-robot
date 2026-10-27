# wheel_gateway

北向管理网关。service.py 汇总默认运行装配状态、配置文件名、固定目录任务结果。只读、不访问 .env、不启动任务，不将默认配置状态伪装为在线设备状态。 本目录只放 Python 实现；安装和 ROS 元数据位于上级。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| `__init__.py` | wheel_gateway: runtime adapter boundary. |
| `service.py` | Northbound read-only projection. Task execution remains owned by runtime. |
