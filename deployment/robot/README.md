# 机器人侧辅助工具

| 文件 | 用途 |
| --- | --- |
| `joint_feedback_server.py` | 已有 ROS 关节反馈到 GET-only HTTP 服务。 |
| `wheel-joint-feedback.service` | 已有关节反馈服务模板。 |
| `hold_trim/` | 既有保持补偿工具。 |
| `console_authority/` | 控制台与遥操显式权限的候选生成、构建、隔离验证和空闲部署工具。 |
| `check_model_ik.cpp` | 在真机SDK头文件和MuJoCo库上只读加载新模型，检查FK与IK；不创建ROS节点或硬件控制器。 |
| `install_claw14_model.py` | 仅用于本次用户明确授权的1.4模型部署：先检查原模型和已通过的SDK计算记录，再并列安装新XML/网格，备份并更新启动model参数；不启动服务或发送运动。 |

头部独立配对服务方案已撤销，不需要新增服务或修改固件。当前只读视觉工具使用静止诊断 500 ms 升降反馈门限，见 `tools/grasp/README.md`。原有机械臂关节反馈和保持补偿功能保留。
