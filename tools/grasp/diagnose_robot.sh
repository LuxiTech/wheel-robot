#!/usr/bin/env bash
# Run ON THE ROBOT. Read-only checks: no start/restart/enable/control commands.
set -uo pipefail
sdk_root="${1:-/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2}"
report_dir="${2:-/tmp/wheel-feedback-diagnostic-$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "$report_dir"
exec > >(tee "$report_dir/robot-diagnostic.log") 2>&1
check() {
  printf '\nCHECK:'
  printf ' %q' "$@"
  printf '\n'
  timeout 12 "$@"
  printf 'EXIT_CODE=%s\n' "$?"
}
check date -u --iso-8601=ns
check systemctl show luxi-openarm-monitor.service luxi-rgbd-gateway.service luxi-lift.service \
  -p Id -p ActiveState -p SubState -p MainPID -p NRestarts -p ExecMainStatus -p ExecStart -p Environment
check journalctl -u luxi-openarm-monitor.service -n 120 --no-pager
check journalctl -u luxi-rgbd-gateway.service -n 60 --no-pager
check journalctl -u luxi-lift.service -n 60 --no-pager
check ip -details -statistics link show
check ps -eo pid,ppid,stat,wchan:28,comm
check python3 - <<'PY'
from pathlib import Path
import json
allowed={'ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','ROS_DISCOVERY_SERVER','FASTRTPS_DEFAULT_PROFILES_FILE','CYCLONEDDS_URI'}
for directory in Path('/proc').iterdir():
    if not directory.name.isdigit():
        continue
    try:
        command=(directory/'cmdline').read_bytes().replace(b'\x00',b' ').decode(errors='replace')
        if not any(name in command for name in ('/openarm_adapter','/web_ctrl','/lift_adapter')):
            continue
        env={}
        for item in (directory/'environ').read_bytes().split(b'\x00'):
            key,_,value=item.partition(b'=')
            if key.decode(errors='replace') in allowed:
                env[key.decode()]=value.decode(errors='replace')
        print(json.dumps(dict(pid=directory.name,command=command,ros_environment=env),ensure_ascii=False))
    except (OSError,PermissionError) as exc:
        print('PROCESS_READ_ERROR',directory.name,str(exc))
PY
if [[ -f /opt/ros/humble/setup.bash && -f "$sdk_root/build/install/setup.bash" ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  source "$sdk_root/build/install/setup.bash"
  set -u
  export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
  check ros2 node list --no-daemon
  check ros2 topic info /luxi/arms/state --verbose
  check ros2 topic echo /luxi/arms/state --once
  check ros2 topic hz /luxi/arms/state
  check ros2 topic echo /luxi/faults --once
  check ros2 topic echo /luxi/lift/state --once
else
  printf 'SDK ROS workspace unavailable: %s\n' "$sdk_root"
fi
printf '\nREPORT_DIRECTORY=%s\n' "$report_dir"
