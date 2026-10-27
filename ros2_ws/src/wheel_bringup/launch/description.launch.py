"""Robot-local TF from supplied URDF. Requires measured lift offset explicitly."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    urdf=Path(LaunchConfiguration('urdf').perform(context)).read_text()
    offset=float(LaunchConfiguration('lift_offset_m').perform(context))
    return [Node(package='robot_state_publisher',executable='robot_state_publisher',
                 parameters=[{'robot_description':urdf}]),
            Node(package='wheel_sdk_bridge',executable='wheel-grasp-state',
                 parameters=[{'lift_offset_m':offset}])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('urdf'),DeclareLaunchArgument('lift_offset_m'),
                              OpaqueFunction(function=setup)])
