# 既有保持补偿候选工具

`hold_trim.hpp` 保存补偿实现，`test_hold_trim.cpp` 为对应 C++ 测试。
`prepare_sdk.py` 核对指定 SDK 源文件指纹后生成隔离候选，不覆盖运行中的 SDK，不重启服务。
本目录与头部 RGB-D/升降配对服务独立；此次反馈修复未修改这里的控制代码。
