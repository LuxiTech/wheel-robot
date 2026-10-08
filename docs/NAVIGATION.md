# 固定取物点导航：A* 与 FAST-LIVO2

原接力控制器按固定安全通道和航点移动，没有全局路径搜索。新增导航模式专门验证定位和路径规划：双臂收起，A 依次到达源垛左前、右前、右后、左后、交接桌前和待命点；B 原地停放。此模式不执行抓箱，不代表完整拆码垛已切换到新导航。

```bash
# 独立浏览器控制页面 + Isaac 仿真窗口
./scripts/simctl navigation-console
# 浏览器访问 http://127.0.0.1:8765

# 已知工位地图 + 仿真真值定位，独立检查 A* 与底盘跟踪
./scripts/simctl navigation --gui

# 实际 FAST-LIVO2 进程输出定位 + 同一个 A* 规划器
./scripts/simctl navigation-livo --gui

# 无界面，只到一个指定取物点
./scripts/simctl navigation-livo --navigation-goal source_front_left

# 当前机器的 FAST-LIVO2 镜像已构建；其他机器先执行
./scripts/build_fast_livo.sh
```

点击 `START navigation tour` 依次巡航；空闲时也可点击单个站点直接前往。黄色标记为固定站点，绿色线为当前规划路径。固定点和速度在 `config/navigation_demo_config.json` 修改。导航点是接近工位的安全站位，抓取前仍需单独精对位。

独立页面提供实时工位地图、单点导航、六点巡航、停止任务、环境复位及执行记录；该模式不再显示 Isaac 内嵌操作面板。页面只监听本机 `127.0.0.1:8765`，通过会话隔离的指令队列控制仿真，按钮发送后等待执行确认。3 秒未收到仿真心跳则禁用控制，5 秒前的积压指令不再执行。停止是仿真软件停止任务，不是硬件急停。

## 原载箱路线如何生成

原 `RelayRuntime.dock()` / `retreat()` 按取物侧和工位类型拼接固定航点，并非 A* 搜索。普通前排工位按箱子坐标减去停靠距离生成站位；交接桌先在 `y=-0.7 m` 公共通道横向对齐，再直线靠近。后排工位先到 `x=-3.2 m`（A）或 `x=3.2 m`（B）的外侧通道，再绕到后方通道（A 为 `y=2.1 m`，B 为 `y=1.85 m`），横移到箱子对应位置后靠近。退回时先退出到后通道，检查载箱清空源垛边缘，再按外侧通道返回。

`navigate()` 只负责追踪当前航点：根据位置、朝向误差计算底盘速度，变换到机器人坐标系后下发轮速。载箱时并行执行腰部与双臂 IK 姿态调整、保持双手相对位置；IK 不负责搜索底盘绕障路线。这种规则有意避开托盘、桌腿和载箱转弯扫掠区域，但不能选择全局更短的路径。

## 路径规划

`scripts/grid_navigation.py` 执行八邻域 A*，网格分辨率 5 cm，禁止对角穿角；按收臂机器人半径 0.43 m 加 0.06 m 跟踪余量膨胀障碍，再以碰撞检查合并可直达路段。不能穿过托盘或狭窄缝隙时才绕行。跟踪过程中检查剩余线段；新增障碍导致线段不安全、停滞或定位过期时停车。当前不会自动绕过突发动态障碍。

本轮规划地图采用已有工位的静态碰撞几何，另一台机器人作为占据区域加入。**尚未把 FAST-LIVO2 点云转换成在线占据地图**，也没有对载箱和双臂展开的扫掠体做新规划验收。原接力任务仍保留原有载箱路径。

## FAST-LIVO2 接入

- 使用 [HKU MARS FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) 的 [U-AMC ROS 2 移植](https://github.com/U-AMC/FAST-LIVO2-ROS2)，源码提交 `a09599a288c56b832114d7ad9e898248a4111d9e`。
- Isaac 输出同步的 3D PhysX 射线点云、120 Hz 原生 IMUSensor 和 10 Hz 实际渲染 RGB；点云为瞬时扫描，每点时间偏移为 0，不伪造旋转扫描时序。
- 相机为 480×360、焦距 320 px 的无畸变针孔模型。传感器共址于底盘上方 1.25 m，外参配置在 `config/fast_livo_sim.yaml`，内参在 `config/fast_livo_camera.yaml`。
- 当前工位墙面和箱体纹理较少，采用较低视觉权重（`vio.img_point_cov: 1000`）并关闭曝光估计，定位以激光惯性约束为主；视觉更新仍开启。该调参针对本仿真工位，不是通用传感器标定结果。
- `scripts/livo_ros_bridge.py` 发布 `/sim/lidar`、`/sim/imu`、`/sim/image`；真实 FAST-LIVO2 进程订阅并运行视觉、激光和惯性估计，桥接器读取 `/aft_mapped_to_init`。
- 原 ROS 2 移植以系统时间发布里程计。本地 Docker 构建补丁改用 `LidarMeasures.last_lio_update_time`，保证定位新鲜度检查采用测量时刻。
- 对与图像同一时刻的瞬时点云，分组边界采用 `<=`，避免把整帧相同时间戳的点推迟到下一次更新。RGB 在点云采样对应的物理步用零时间步刷新，避免 annotator 返回上一渲染帧；报告记录实际渲染位姿与射线原点偏差。
- 原始点云检测到机器人周围 0.48 m 内障碍时停车。该保护独立于定位结果；不代替载箱时的完整扫掠体碰撞检查。
- 导航仅使用估计位姿和初始化工位坐标对齐；传感器生成、其他机器人已知障碍位置及验收读数才读取仿真真值。失去新鲜估计时不回退到真值导航。
- Isaac 和 ROS 容器通过带会话编号的原子文件交换数据，保持 `network_mode: none`。任务换目标保留单调传感器时间；环境复位建立新会话并通知 ROS 侧重启估计器，拒绝旧会话里程计。

## 验收

- 独立页面控制链路已实测单点到达、停止任务和环境复位，见 `outputs/navigation_console_acceptance.json`；另有 4 项接口/过期指令测试，以及页面 DOM 交互检查 `outputs/navigation_console_ui_check.json`。本次无可用浏览器自动化连接，未完成浏览器截图外观验收。
- A* 已知地图、真值定位最终回归通过 6 个站点，86.725 s 仿真时间、13.863 m 规划路径，到点物理误差 1.1–2.0 cm；证据 `outputs/navigation_astar_final_pass_report.json`，截图 `outputs/navigation_astar_final.png`。
- 68 项回归测试通过，包含障碍膨胀、拐角、不可达目标和失效定位拒绝。
- FAST-LIVO2 的激光惯性对照（关闭视觉更新）通过全部 6 点，83.225 s 仿真时间，到点物理误差 0.7–8.7 cm；证据 `outputs/navigation_lio_tour_report.json`，报告明确标记 `LIO_only_diagnostic`。可用 `LIVO_DIAGNOSTIC_NO_VISION=1 ./scripts/simctl navigation-livo` 复现，不能将该结果称为三传感器融合通过。
- **FAST-LIVO2 三传感器融合尚未通过整条路线。** 当前保留参数的一次试验通过 5 个工作站点，但返回待命点的物理误差为 14.3 cm，超过 10 cm 阈值后判失败，见 `outputs/navigation_livo_home_failure_report.json`。继续降低视觉权重、收紧 IMU 协方差的对照均未解决问题，未保留这些调参。融合模式标为 experimental；不要将激光惯性对照或真值导航的成功混为融合成功。
- 最新导航报告：`outputs/navigation_report.json`；ROS 日志：`docker compose -f compose.livo.yaml logs fast-livo`。导航报告与拆码垛报告分开保存。
