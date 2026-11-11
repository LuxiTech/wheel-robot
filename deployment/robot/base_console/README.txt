网页与Quest底盘控制互斥开关

prepare_sdk.py 从真机SDK当前源码生成独立候选及原文件指纹，不安装、不重启。
install_sdk.py 只安装已经通过模拟检查且指纹匹配的候选，要求真机空闲；备份原程序和网页后重启底盘与遥操协调程序，不操作机械臂和升降驱动，不发送运动请求。
test_base_console.py 必须在机器人ROS2环境运行，使用mock底盘及独立ROS域97，参数为候选底盘程序、候选管理器程序、arms-teleop.json。
test_console_browser.py 使用headless Chrome及页内模拟API验证真实DOM，不发送真机控制请求。

本次完整差异与验证回执位于 outputs/base-console-diagnosis/20261106T073324Z/change-report.txt。
真实SDK备份与候选位于机器人 SDK/work/web-base-control-20261106/。
SDK源码保持在真机，不恢复本地device目录副本。

2026-11-06 后续更新：prepare_retained_control.py 针对已部署初版网页开关的SDK生成手动释放/500ms停车策略候选。两个准备脚本对应不同起点，锚点不匹配时会拒绝写入。当前test_base_console.py和浏览器测试验证此后续策略。install_sdk.py --base-only只更换底盘程序与网页，不重启遥操程序；仍要求底盘空闲及统一遥操未运行。
本次回执：outputs/base-console-diagnosis/manual-release-20261106T084824Z/report.txt。

原地转向更新：prepare_turn_controls.py 基于当前手动释放版本生成Q/E和页面长按按钮，仅涉及网页资源；无需编译或重启SDK服务。浏览器测试和mock纯偏航测试覆盖新入口。
