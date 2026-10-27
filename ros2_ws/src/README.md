# src

十个独立 ROS 工作包。interfaces 定义通信，description/bringup 管资源与装配，其余包按功能边界适配核心算法和设备。目录存在不代表所有硬件能力已接入。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [wheel_bringup/](wheel_bringup/README.md) | ROS 启动与资源装配包。launch/description.launch.py 启动 robot_state_publisher 和 wheel_sdk_bridge 的实测关节接收。必须显式提供 urdf 与 lift_offset_m。 |
| [wheel_description/](wheel_description/README.md) | 机器人描述资源包。urdf/meshes 用于已验证机器人资源；当前真实 URDF 缺失，目录只保留说明，不生成仿真替代模型。 |
| [wheel_execution/](wheel_execution/README.md) | 真机操作执行边界。executor.py 保留运输、规划、预抓取、接近、夹持验证、试抬、放置与保持接口。ready=False，尚未接通厂商轨迹执行。后续由本包统一管理动作反馈与取消。 |
| [wheel_gateway/](wheel_gateway/README.md) | 北向管理网关。service.py 汇总默认运行装配状态、配置文件名、固定目录任务结果。只读、不访问 .env、不启动任务，不将默认配置状态伪装为在线设备状态。 |
| [wheel_interfaces/](wheel_interfaces/README.md) | 唯一 ROS 消息契约包，从 wheel_grasp_interfaces 迁移。msg 包含箱体位姿和双手候选，action 包含 ExecuteGrasp。CMake 生成类型，包内没有动作服务器。 |
| [wheel_navigation/](wheel_navigation/README.md) | 工位导航与 AprilTag 对接边界。adapter.py 的 ready=False，导航/停止确认抛出 NotImplementedError。后续在此接入 Nav2，不在任务状态机中嵌入导航代码。 |
| [wheel_perception/](wheel_perception/README.md) | 任务侧视觉适配。service.py 定义 SynchronizedCapture/CaptureSource，将核心 GraspPipeline 转为 GraspObservation。推理工作线程串行化，默认同步源未接入。不包含已部署的 ROS action server。 |
| [wheel_planning/](wheel_planning/README.md) | 运动规划能力适配。service.py 的 OfflinePlanningService 实际调用 wheel_motion_core.plan_pair，接收实测状态和碰撞回调，保持计划不可执行。尚无跨进程 action server。 |
| [wheel_runtime/](wheel_runtime/README.md) | 应用装配与命令入口。main.py 装配适配器，queries.py 提供状态/工序只读描述，cli.py 提供诊断和离线算法命令。业务状态机位于 wheel_task_core。未提供常驻任务调度服务。 |
| [wheel_sdk_bridge/](wheel_sdk_bridge/README.md) | 厂商协议的唯一适配边界。sdk.py 处理升降映射、关节反馈和租期；feedback.py 对齐实测状态；ros_state/rgbd_ros 是只读 ROS 入口。厂商 SDK 本体不在本仓库。ROS main 需要板端 luxi_interfaces。 |
