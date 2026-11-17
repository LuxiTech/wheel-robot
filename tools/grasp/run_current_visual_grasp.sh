#!/usr/bin/env bash
# 当前现场入口：check 只读检查；align 仅对准；grasp 执行正常完整抓取组合。
# 每次使用新目录，Python 原始输出与结构化执行日志均保留。
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$repo_root"
mode=${1:-check}
if (( $# > 0 )); then
    shift
fi
# 当前升降位置复用已存在的同矩阵升降补偿副本；所有模式使用同一份选定资源。
calibration=outputs/grasp/visual-grasp-runs/20261116T022850Z-jfVTAo/run/current_height_calibration.json
entry=tools/grasp/run_visual_grasp.py
motion_args=()
case "$mode" in
    check|align)
        entry=tools/grasp/commission_visual_alignment.py
        if [[ "$mode" == align ]]; then
            motion_args+=(--execute)
        fi
        ;;
    grasp)
        # 直接复用用户确认的资源；执行时仍检查真实视觉、反馈和限位。
        motion_args+=(--execute)
        ;;
    *)
        printf 'Usage: %s [check|align|grasp] [additional Python options]\n' "$0" >&2
        exit 2
        ;;
esac

preview=outputs/grasp/joint-stream-deploy-20261115/closer-box/vision
vision_python=.runtime/vision/venv/bin/python
base=outputs/grasp/visual-grasp-runs
mkdir -p "$base"
run_parent=$(mktemp -d "$base/$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")
# 退出后自动汇总，原进程的失败退出码照常返回。启动阶段失败也有execution.log。
finalize_run_log() {
    local execution_status=$?
    trap - EXIT
    python3 tools/grasp/execution_log.py \
        --run "$run_parent/run" --exit-code "$execution_status" || {
        printf 'Execution log generation failed: %s\n' "$run_parent" >&2
    }
    return "$execution_status"
}
trap finalize_run_log EXIT
python3 tools/grasp/execution_log.py --start --run "$run_parent/run" --mode "$mode"
ln -s run/execution.log "$run_parent/execution.log"
printf 'Mode: %s\nLogs: %s\nExecution log: %s/execution.log\n' "$mode" "$run_parent" "$run_parent"
[[ -x "$vision_python" && -f "$calibration" && -f "$preview/detection.json" ]] || {
    printf 'Current-scene runtime/resources unavailable; inspect paths in %s\n' "$0" \
        | tee -a "$run_parent/console.log" >&2
    exit 2
}
# 打印并检查实际生效的后端；默认只走 MoveIt，不因服务离线回退旧回路。
execution_config=config/robots/grasp_execution.json
extra_args=("$@")
for ((index=0; index<${#extra_args[@]}; index++)); do
    case "${extra_args[index]}" in
        --execution-config)
            execution_config=${extra_args[index+1]:?Missing execution config path}
            ;;
        --execution-config=*) execution_config=${extra_args[index]#*=} ;;
    esac
done
execution_mode=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["execution_mode"])' "$execution_config")
printf 'Execution backend: %s\nExecution config: %s\n' "$execution_mode" "$execution_config" | tee -a "$run_parent/console.log"
if [[ "$execution_mode" == moveit_trajectory ]]; then
    bash tools/grasp/ensure_moveit_gateway.sh 2>&1 | tee -a "$run_parent/console.log"
fi
"$vision_python" "$entry" \
    --calibration "$calibration" \
    --cad-preview "$preview" \
    --output "$run_parent/run" \
    "${motion_args[@]}" "$@" 2>&1 | tee -a "$run_parent/console.log"
