2026-11-10 陆图建图异常定位（只诊断，未部署或重启生产）

真机：lucy@192.168.4.118，SDK根目录 /home/lucy/Project/lucy-atlas-agentos/lucid-atlas
原始输入录包、原始历史日志备份、诊断二进制及完整证据保留在真机 .local/mapping-cause-20261110/。
本地报告与轻量证据：outputs/lutu-mapping-check-20261110/cause/cause-analysis.json。
未复制SDK源码到本地device目录；所有候选和读取均基于真机当前部署源。

已确认的缺陷：publish_map无界追加扫描，并在同一个SingleThreadedExecutor内转换/发布全量地图；
IMU订阅深度10（200Hz约50ms）。439,507,008字节快照的转换+序列化实测238ms，尚不含网络发布。
ImuProcess::Process在空IMU组上直接返回而保留旧feats_undistort，调用方仍继续滤波和建图更新。
同一14.5秒录包：无阻塞诊断实例零空IMU更新，位移模长最大0.052m；每秒250ms阻塞的对照出现
16个空IMU组，其中12组仍使用已有点云更新。只读进程采样亦观察到空IMU与旧点云共存的测量状态。
这复现了错误处理路径，不把它冒充已完整重放历史公里级发散；无限大地图的阻塞触发因素有强证据支持。

另有网页侧大包best-effort接收丢包，以及传感器共同epoch落后主机约52843.6秒的时基问题。
实际查询网页最小地图频率是0.8Hz，启动参数覆盖了YAML默认1.5Hz，因此频率阈值不是本次原因。
雷达/IMU共同旧epoch不等于两者相对失步。未修改标定、模型、任何生产进程或运动控制。

build_audit_mapper.py：用实际生产编译/链接参数构建诊断实例，只添加trace和可控阻塞。
ROOT_DIR重定向到独立audit-root，避免覆盖生产日志。生产二进制不替换。
replay_audit.py：在独立ROS域98、DDS仅lo上回放原始输入，仅播放雷达/IMU，不回放生产Odometry。
ROS_LOCALHOST_ONLY设0，由CycloneDDS显式lo与域98隔离；设1会与显式lo重复而无法启动，失败记录已保留。
analyze_recording.py：只读SQLite录包，核对时序、缺失、数值范围。
read_live_memory.py：需要进程读取权限，使用process_vm_readv，只读已核对ABI的全局测量，既不暂停也不写入进程。
benchmark_snapshot.cpp：不创建ROS节点，仅测量实际PCL/ROS序列化库对同尺寸快照的处理成本。
诊断录包进程已退出，两个隔离建图实例已停止；现有生产建图和地图原件保持原状。
