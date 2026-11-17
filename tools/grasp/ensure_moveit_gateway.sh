#!/usr/bin/env bash
set -euo pipefail
# 网关只监听机器人本地。已有隧道可直接复用；否则使用正常 SSH 认证建立。
if ! python3 - <<'PY'
import socket
try:
    with socket.create_connection(('127.0.0.1', 18036), timeout=1): pass
except OSError:
    raise SystemExit(1)
PY
then
    ssh -fN -o ExitOnForwardFailure=yes -o ServerAliveInterval=10 -o ServerAliveCountMax=3 \
        -L 127.0.0.1:18036:127.0.0.1:18036 "${WHEEL_ROBOT_SSH:-sunrise@192.168.2.199}"
fi
python3 - <<'PY'
import json
import urllib.request

# 本地 SSH 监听存在，不代表板端 HTTP 服务仍在线；远端服务退出时
# SSH 会重置已接收的连接。预检失败只退出，绝不继续执行或自动使能。
try:
    with urllib.request.urlopen('http://127.0.0.1:18036/', timeout=3) as response:
        state = json.load(response)
except (OSError, ValueError) as error:
    raise SystemExit(
        'MoveIt网关连接失败，尚未启动视觉识别或抓取。'
        '\n本地隧道端口存在不代表板端服务在线；请检查SSH转发及板端服务：'
        '\nsystemctl status wheel-moveit-control.service luxi-openarm-monitor.service'
        '\n若SDK日志出现 USB CAN absent，请先恢复USB设备连接。'
        '\n原始错误：' + str(error)
    ) from None
if state.get('mock') is not False or state.get('protocol') != 1 or not state.get('ready'):
    raise SystemExit('MoveIt板端服务尚未就绪；不要使能，先检查 wheel-moveit-control.service。')
PY
