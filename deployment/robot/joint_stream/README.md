# 笔记本视觉伺服与关节流

当前仓库的 `config/robots/grasp_execution.json` 选择 `laptop_visual_servo`。
笔记本保留视觉目标和抓取状态机，对准的完整IK在独立进程求解，独立线程执行参考更新、
下插微分IK及速度与加速度限制，目标周期10ms。视觉推理与HTTP反馈采集独立运行，不在关节循环中等待图像、
做HTTP请求或重新规划整条路径。目标更新时保留当前关节参考和速度，不逐点停稳。
`reference_timing`记录实际生成数、最大周期、计算发送耗时和跳过的期限；100Hz是目标，
不是普通Linux/Python线程的硬实时保证，也不是电机驱动器内部闭环频率。

运动学复用当前选择的claw1.4原URDF和SDK链快照。解析空间雅可比由原关节轴和变换构造，
对应SDK右8+左8；实测角度直接使用，不写零、不改符号、不加安装偏移。
实际误差和跟随保护使用实测关节；IK在名义指令参考上积分，避免把负载下的偏转不断积入目标。
夹爪轴7/15在臂运动中保持，开合使用单独的SDK端点请求。当前用户确认复用模式观察
双侧实测闭合状态，再由10mm试抬观察箱体随动；不将其报告为已标定力接触。

## 会话与数据

HTTP `/api/arm_startup`只用于显式`joint_stream_start`和带请求ID的`joint_stop`。
启动须已经使能、无故障、停稳且控制台持权，不隐式使能或清故障。
SDK必须报告`joint_stream_protocol=1`、`joint_stream_port`和原配置的
`joint_stream_speed_limits`；未部署时在任何运动/开爪请求前拒绝，不退回高频`joint_move`。

关节数据通过UDP发送。每包336字节，按网络字节序排列：

| 字段 | 大小 |
| --- | --- |
| `WJSTRM1\0` | 8字节 |
| 启动时随机生成的会话key（不在状态接口公开） | 32字节ASCII |
| 递增序号 | uint64 |
| 真机实测反馈接收时间 | uint64 ns |
| 原始图像接收时间 | uint64 ns |
| 原始pair标识SHA256前16字节 | 16字节 |
| 16个关节位置，rad | 16×float64 |
| 16个关节参考速度，rad/s | 16×float64 |

相同图像可以用于多个10ms控制周期，原始时间戳/pair不刷新；新图须时间递增且pair改变。
反馈和图像时间都来自机器人；不比较笔记本和机器人的单调时钟，也不为旧帧生成新时间戳。
UDP不重传、网络发送不阻塞，接收器按序号采用最新有效参考。旧会话、重复、乱序、畸形、
超限、非有限数、夹爪变动、速度超限及过大位置跳变均拒绝，拒绝包不刷新有效期。

板端每10ms检查权限、实际反馈、原限位和0.25rad跟随保护，并对网络抖动保留最终速度、加速度限制。
它不运行IK、不规划路径。两端限制的是关节参考，不构成对实测速度、加速度的保证。
首包前只保持，最长等待1秒；首包后命令/来源反馈150ms、图像2000ms超时即停止本次请求并
沿用SDK保持语义。迟到包不能恢复超时会话，必须显式重新开始。
笔记本控制周期超过30ms时不补发追赶；反馈缓存失效或工作线程异常时立即停止发送。
停止不松爪、不回零、不控制升降、不自动失能。

顺序为对准、张开、下插、闭合及10mm试抬。当前现场配置复用已有资源，最大下插200mm，
旧资源验收说明仅作日志信息；原始限位、视觉质量、时效及真实故障仍生效。
当前现场碰撞跳过配置不会被记录成碰撞检查通过。完整抓箱是否成功按实测结果报告。

## 从真机当前SDK准备独立候选

将本目录`prepare_sdk.py`、`adapter_members.inc`和`joint_stream.hpp`传到真机临时目录。
在真机SDK根目录并加载现有构建环境后运行，输出目录必须新建：

```bash
python3 /tmp/wheel-joint-stream/prepare_sdk.py \
  --sdk-root "$PWD" --output "$PWD/work/laptop-joint-stream/candidate"
cmake -DCMAKE_BUILD_TYPE=Release \
  -S work/laptop-joint-stream/candidate \
  -B work/laptop-joint-stream/candidate/build
cmake --build work/laptop-joint-stream/candidate/build -j2
```

将`verify_mock.py`也传入同一临时目录后，可在SDK环境中验证独立候选：

```bash
python3 /tmp/wheel-joint-stream/verify_mock.py \
  --candidate "$PWD/work/laptop-joint-stream/candidate" \
  --config "$PWD/Project/robot_bringup/config/arms-kh.json" \
  --output "$PWD/work/laptop-joint-stream/mock-verification" --domain 224
```

验证器拒绝被其他ROS进程占用的域，强制`mock=true`、`allow_hardware=false`和回环UDP地址，
只启动和清理自己的独立候选进程；合成参考、图像及模拟使能均不会访问真实CAN。

准备器只读取当前真实SDK，严格匹配原适配器锚点并记录SHA256，生成独立二进制
`openarm_adapter_stream_candidate`。若SDK源码已变化或另装其他候选，会明确拒绝，
不能用旧本地SDK副本代替、放宽锚点或混装两个候选。默认监听127.0.0.1:18034；
部署到网线接口必须显式设置`joint_stream_bind`为真机对应IPv4地址。
此准备步骤不安装、不启动、不停止或重启任何服务。

仓库本地验证命令：

```bash
.venv/bin/python -m pytest tests/grasp/test_laptop_joint_servo.py -q
```

这些检查使用原模型、合成反馈、回环UDP和本机编译的C++接收协议，不连接CAN。
完整SDK候选须另在真机隔离ROS域、`mock=true`、`allow_hardware=false`下验证，之后才考虑部署。
SDK退出时可能失能，双臂使能保持时切换服务仍须遵守已有现场支撑要求。

2026-11-15首次通过SSH直接核对当前SDK并完成上述独立编译与6项模拟联合检查。
实际候选为SDK根目录下`work/laptop-joint-stream-20261115/candidate/`，
最终记录为同目录旁的`mock-verification-timed/`。100个合成关节参考和30个合成图像，
回环发送约99.84Hz、最大周期约11.91ms；此处不包含笔记本推理或跨网线传输，不是真机运动实测。
源文件原始字节SHA256和换行归一化SHA256分别登记，避免CRLF/LF差异混淆。
该次候选验证尚未安装或替换生产SDK，未重启生产服务；本地归档见
`outputs/grasp/laptop-joint-servo-20261115/board-verification/`。

后续已部署v1并按用户要求更新为2s图像门限，见开发依据文档同日部署记录。
实机对准又定位出接收器把新参考与滞后的插值输出比较，误丢连续包的问题。
修正候选检查相邻参考，首包仍与当前输出比较；实际输出仍独立限制速度、加速度和限位。
本地合成回放80包，旧逻辑仅接受22包后超时，修正后全部接收且输出限制通过。
证据在`outputs/grasp/joint-stream-deploy-20261115/reference-continuity-verification/`；
候选路径为真机`work/laptop-joint-stream-20261115/candidate-reference-continuity/`。
`update_candidate.py`只接受已核对的运行版本及通过隔离验证的候选，安装前再次确认失能，
备份原件后原子替换；双臂仍使能时拒绝切换。

2026-11-16 固定对准目标模式：SDK需额外公布`joint_stream_fixed_target_protocol=1`。
数据包布局不变，固定模式magic为`WJFIXD1\0`。首包图像仍须在2秒内，后续固定
同一原始图像时间和pair，禁止中途切换模式或图像；已接受目标不再随图像年龄停止。
命令/反馈150ms、权限、跟随误差、原限位和速度加速度约束保持原逻辑。
笔记本仅在align阶段采用固定模式：接受后不再更新目标或等待视觉/IK，以实测末端误差
和停稳时间判定完成；下插及其他阶段维持原逻辑。未支持该能力的SDK明确拒绝启动固定模式。
