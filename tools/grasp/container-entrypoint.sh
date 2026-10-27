#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/wheel_ws/install/setup.bash
export PATH="/opt/wheel_ws/install/lib/wheel_runtime:/opt/wheel_ws/install/lib/wheel_sdk_bridge:$PATH"
if [[ -f /workspace/tools/pythonpath.sh ]]; then
  ROOT=/workspace
  source "$ROOT/tools/pythonpath.sh"
fi
exec "$@"
