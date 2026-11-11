# robots

外部机器人 URDF 与配套网格的现有资产位置。

当前使用用户确认的 `r680_lift_openarm_claw1.4.zip`，原样保存在
`r680_lift_openarm_claw1.4/`；入口为 `r680_lift_openarm_robot_viewer.urdf`，网格使用同目录
`resources/` 下的相对路径。三个 runtime 配置均已指向此文件。

来源及 SHA256 见 `config/robots/r680_lift_openarm_claw1.4.source.json`。1.3目录和来源文件原样保留。
当前模型统一选择见`config/robots/robot_model.json`；保存采样记录其模型版本，历史采样仍使用原模型。
此目录依照仓库原有规则
不纳入 Git，部署时需要一起复制整个模型目录，不能只复制 URDF。

该资产提供 OpenArm2.0 与自定义夹爪 CAD 安装关系。单臂相对基座的 FK 已做模型对照，
但整机安装、夹爪接触 TCP 和电机角度到手指开口的映射尚未实物验收。
不含 SDK `arm_origin`；相机安装值不是标定外参，升降零位映射仍需核对。
没有为匹配旧程序而伪造坐标系或修改原始关节。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
