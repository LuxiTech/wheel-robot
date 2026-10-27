# apps

网页前后端边界。management_api 依赖北向网关，management_web 只通过 HTTP 查询后台；不包含模型推理和设备控制。

## 文件与子目录

| 路径 | 功能 |
| --- | --- |
| [management_api/](management_api/README.md) | FastAPI 只读管理服务。management_api/app.py 定义五类 GET 查询并挂载静态前端；__main__.py 提供 host/port/root 参数。没有登录、写参数、任务控制或 WebSocket，默认只监听回环地址。安装和启动见下文。 |
| [management_web/](management_web/README.md) | 独立静态前端。index.html 定义模块状态、工序、配置和任务记录；app.js 请求 /api/v1 并使用 textContent 渲染；styles.css 提供响应式布局。不依赖 Node 构建，不直接连接 ROS/SDK。 |
