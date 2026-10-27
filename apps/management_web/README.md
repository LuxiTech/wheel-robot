# management_web

独立静态前端。index.html 定义模块状态、工序、配置和任务记录；app.js 请求 /api/v1 并使用 textContent 渲染；styles.css 提供响应式布局。不依赖 Node 构建，不直接连接 ROS/SDK。navigation.js 通过后台获取双 SDK 连接、点位和执行状态，只有用户点击才发送 A/B 或停止 POST。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| `app.js` | 本目录配置、资源或工具文件。 |
| `navigation.js` | 双 SDK 卡片、A/B 控制、命令关联、自动 GET 刷新与失联禁用。 |
| `index.html` | 本目录配置、资源或工具文件。 |
| `styles.css` | 本目录配置、资源或工具文件。 |
