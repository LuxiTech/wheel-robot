#!/usr/bin/env bash
set -eo pipefail
sdk_root=/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2
workspace="$sdk_root/work/moveit-integration-20261116"
source /opt/ros/humble/setup.bash
source "$sdk_root/build/install/setup.bash"
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
# The generated real configuration starts hardware and arm controller INACTIVE.
# No motor enable, pose reset, gripper command or trajectory is sent at launch.
exec ros2 launch wheel_moveit_control stack.launch.py config_dir:="$workspace/config-board-real"
