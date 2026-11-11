2026-11-09 陆图连接与现行 SDK 网页协议适配

现场入口：http://192.168.4.118:8082/#/device
机器人 SDK：http://192.168.2.199:8000
机器人本地中继：TCP 18031 / UDP 18032

根因：陆图 config/local/robot.yaml 仍指向 192.168.0.245；旧中继把 shared_enable
直接当作 Quest 占用，且使用不带 owner/sequence 的旧键盘协议。
直接核对真机 r680_adapter.cpp 后，按当前 console 权限、接管许可、反馈有效性
接入带独立 owner 和递增 sequence 的网页协议。遥操、外部占用和过期反馈仍拒绝接管；
停止按本会话 owner 发送，未知使能回执进入本会话清理，不自动重发使能。
采用 SDK 返回的 web_keyboard_linear_limit，当前为 0.05 m/s，网页显示和请求同步限幅。
没有更改 SDK 驱动、权限协调器、运动参数、零点或模型。

prepare_compatibility.py：从两端实际源文件按唯一锚点生成候选及原始 SHA256。
prepare_web_limits.py：从实际前端生成显示/请求速度上限候选，支持显式指定已有依赖目录。
test_scoped_carrier.py：对执行严格 owner/sequence 校验的模拟 HTTP SDK 验证协议和权限。
test_scoped_sdk_ros.py：扩展机器人既有测试，在隔离 ROS 域 178 验证真实中继与模拟底盘。
install_candidate.py：核对指纹、空闲状态，备份并安装已验证候选；只重启网页/中继，
恢复部署前已运行的雷达/建图进程。它不接管控制权或提交行驶请求。
Carrier 类布局有变化，必须同时重编 Web API 消费方，不能只替换共享库。

两端原件、候选、编译日志、安装回执和可回退备份：
陆图：/home/lucy/Project/lucy-atlas-agentos/lucid-atlas/.local/atlas-link-repair-20261109T100455Z/
机器人：/home/sunrise/.local/share/luxi-atlas-carrier/work/atlas-link-20261109/
本地状态、日志、指纹和浏览器截图：outputs/atlas-link-repair-20261109/
没有恢复本地 device/ SDK 副本，也没有将登录密码写入文件。

验证：14项原有协议、4项新增会话/权限/超时、2项隔离ROS集成、48项网页控制测试通过。
TypeScript检查、前端构建及本目录ruff检查通过。实际Chrome DOM确认接管按钮可用，
无连接失败提示，显示“底盘就绪，等待接管”和0.05m/s。连续12次GET均connected/ready，
无持有者且armed=false；雷达/建图恢复，定位/导航/录制保持停止。
真机未发接管或运动请求，不把模拟协议验证记为实机行驶验收。
部署前后机械臂使能/失能计数均0；升降在任务开始和部署前读数不同，但部署前后均175109μm，
本次未调用升降接口，不能用任务起止读数推断期间保持位置。

后续：2026-11-09 移动速度滑条修复
用户要求恢复控制台调速。0.05m/s网页调试限速使滑条min=max；现在底盘支持启动参数
web_keyboard_linear_limit，默认仍0.05，非法值/超过0.50拒绝，实际有效上限仍取它与max_linear的较小者。
当前真机服务显式配置为0.50m/s，Atlas沿已有动态能力反馈恢复0.05–0.50m/s范围。
机器人原生网页的限速说明也读取实际反馈；Quest/导航限速、加速度、会话保护未改。
仅重启空闲底盘服务；控制协调器、升降、机械臂监测服务PID未变，保留原teleop权限选择。
需要行驶时，操作者先选择console并显式接管。未发送真机行驶、接管或权限切换请求。

prepare_speed_limit.py和install_speed_limit.py分别准备/核对/备份/部署这一后续修改。
check_speed_slider.py使用Chrome实测四档滑条选择，屏蔽控制接口并检查没有修改请求。
底盘模拟回归在ROS域97下以默认和0.50两种配置运行，共22项通过；3个非法参数及5项中继协议检查通过。
实际浏览器验证0.05/0.10/0.30/0.50显示一致，截图停在0.30m/s，不能当作真机轮速验收。
原件、二进制、回退备份和安装日志位于机器人SDK work/web-speed-20261109/。
本地证据：outputs/atlas-speed-repair-20261109/；登录凭据未保存。
