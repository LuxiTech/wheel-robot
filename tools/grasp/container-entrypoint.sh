#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/wheel_ws/install/setup.bash
export PATH="/opt/wheel_ws/install/lib/wheel_grasp_core:$PATH"
exec "$@"
