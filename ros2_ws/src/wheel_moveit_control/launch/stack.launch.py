"""Launch fake hardware by default; real hardware requires generated SDK plugin config."""

import json
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    folder = Path(LaunchConfiguration("config_dir").perform(context))

    def load(name):
        return yaml.safe_load((folder / name).read_text())

    real = (
        json.loads((folder / "context.json").read_text())["hardware"]
        != "mock_components/GenericSystem"
    )
    parameters = {
        "robot_description": (folder / "robot.urdf").read_text(),
        "robot_description_semantic": (folder / "robot.srdf").read_text(),
        "robot_description_kinematics": load("kinematics.yaml"),
        "robot_description_planning": load("joint_limits.yaml"),
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": load("ompl_planning.yaml"),
        **load("moveit_controllers.yaml"),
        "trajectory_execution.allowed_start_tolerance": 0.02,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 3.0,
    }
    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[parameters],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                parameters,
                str(folder / "controllers.yaml"),
                *([{"hardware_components_initial_state.inactive": ["WheelSDK"]}] if real else []),
            ],
        ),
        *(
            []
            if real
            else [
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster", "--controller-manager-timeout", "30"],
                )
            ]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller", "--controller-manager-timeout", "30"]
            + (["--inactive"] if real else []),
        ),
        Node(package="moveit_ros_move_group", executable="move_group", parameters=[parameters]),
        Node(package="wheel_moveit_control", executable="wheel_retime", parameters=[parameters]),
        Node(
            package="wheel_moveit_control",
            executable="planning_gateway.py",
            arguments=["--config-dir", str(folder)],
            output="screen",
        ),
    ]

    # A dead gateway/planner must not leave a controller executing unattended.
    critical = [nodes[1], *nodes[-3:]]
    return nodes + [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[
                    EmitEvent(event=Shutdown(reason="Critical MoveIt/control process exited"))
                ],
            )
        )
        for node in critical
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("config_dir"), OpaqueFunction(function=setup)])
