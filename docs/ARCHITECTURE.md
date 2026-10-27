# 分层架构

## 依赖方向

```text
management_web → HTTP → management_api → wheel_gateway → wheel_runtime
wheel_runtime → wheel_task_core + navigation/perception/execution 适配器
    management_api → wheel_navigation 服务 → wheel_sdk_bridge HTTP 传输 → 外部 SDK API
wheel_perception → wheel_perception_core → wheel_task_core
wheel_planning → wheel_motion_core → wheel_task_core
wheel_execution → wheel_sdk_bridge + wheel_planning（真机接入待实现）
wheel_sdk_bridge → 厂商协议/ROS 消息（ROS 在 main 内按需导入）
wheel_bringup → launch 参数 → description / SDK state bridge
```

核心业务契约通过 Protocol 注入，不知道具体适配器。感知与运动算法共用 `wheel_task_core.transforms.pose`，不相互导入。
`packages/` 下三个包是标准 Python 库，`ros2_ws/src/` 下各包是 ROS 构建与部署边界。包不等于进程，也不等于容器。

## 当前实现

任务协调器保持“空载运输 → 源工位 → 观测/规划/预抓取 → 再观测/再规划/接近 → 夹紧确认 → 试抬 → 持载运输 → 目标工位 → 放置退出”。
检查 target_id、pair_id、基座 frame、质量、诊断标记、观测年龄；耗时规划完成后再次检查年龄。
夹紧前置位 may_be_holding；异常分别尝试底盘停止和机械臂保持，单路失败不屏蔽另一路，且不隐式释放载荷。

`wheel_runtime.main.create_application` 默认不注入导航服务，视觉与运动执行器仍未就绪。`wheel_planning` 只提供实际可调用的离线规划封装。
`wheel_gateway` 查询默认装配、固定配置目录和本地任务结果；不是在线 ROS 状态订阅器。网页/API 不创建第二个拆码垛任务状态机。新增独立 NavigationService 管理 A/B 请求和反馈，CLI 显式开启后才运行导航监测；该服务由网页和可注入的任务适配器共同复用，设备控制仍由 SDK 执行。

`wheel_interfaces` 已迁移已有消息/action 定义；新增各能力 action server、北向任务写入接口以及运行时常驻调度服务均未实现。

## 部署边界

机器人本机部署 SDK 适配、执行、任务运行时和网关；GPU 主机部署视觉/规划；管理 API、前端与数据库可独立部署。
当前代码仍以开发机源码工具运行，上述跨机任务通信拓扑是接入目标，不能据目录齐全认定已实现。
模型推理不进入 Web worker；设备控制与断流保持由机器人本地执行层负责。

## 配置与资产

robots 保存设备/相机/TCP/模型引擎引用，recipes 保存箱体几何与抓取规则，stations 保存工位，deployments 保存任务与锁定环境。
机器人 profile 引用一个相对 recipe；深合并后 profile 字段优先。禁用嵌套 recipe 包含，避免隐式依赖链。
未验证参数继续保留 false/null，不在重构时标记为已标定。历史输出、SDK 本体、模型、CAD 与环境不迁移或重建。

## 后续扩展

能力调用落地为 ROS Action 时保留进度、取消和实测完成语义。连续反馈走 Topic，短时查询走 Service。
上层以 task_id/command_id 幂等提交，运行时持有唯一任务执行权；管理数据库保存投影视图。
配置发布、权限、审计与在线事件流在独立管理层扩展，不能绕过运行时直接控制 SDK。
