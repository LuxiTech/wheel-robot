#!/usr/bin/env bash
# 同一视觉入口与统一日志目录；只切换到板端 MoveIt/JTC 执行配置。
set -euo pipefail
cd "$(dirname "$0")/../.."
mode=${1:-check}
if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in check|align|grasp) ;; *) echo 'Usage: run_moveit_visual_grasp.sh [check|align|grasp]' >&2; exit 2 ;; esac
exec bash tools/grasp/run_current_visual_grasp.sh "$mode" \
    --execution-config config/robots/grasp_execution.moveit.json "$@"
