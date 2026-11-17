#!/usr/bin/env python3
"""Generate a separate MoveIt model from supplied resources and live SDK limits."""

import argparse
import copy
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure

configure(ROOT)
from wheel_motion_core.grasp_scene import fixed_body_exclusions
from wheel_motion_core.kinematics import RobotModel

BASE_COLLISION_RELATIVE = Path("derived_collision/base/r680_base_collision.stl")


def replace_base_collision(robot, mesh_filename=None):
    """Use the validated collision-only base mesh at its original visual pose.

    Only the derived MoveIt model changes. Keep the supplied URDF, other links,
    joint origins and allowed-collision matrix intact. The source STL is a
    surface mesh, not proof that all internal chassis material is represented.
    """
    base = robot.find("link[@name='base_link']")
    visual = base.find("visual[@name='r680_base_visual']")
    collisions = base.findall("collision")
    if visual is None or visual.find("geometry/mesh") is None or len(collisions) != 1:
        raise ValueError("Unexpected base geometry; refusing an ambiguous replacement")
    collision = collisions[0]
    for tag in ("origin", "geometry"):
        old = collision.find(tag)
        if old is not None:
            collision.remove(old)
        source_element = visual.find(tag)
        if source_element is not None:
            collision.append(copy.deepcopy(source_element))
    collision.find("geometry/mesh").set("filename", str(mesh_filename or BASE_COLLISION_RELATIVE))
    collision.set("name", "r680_base_mesh_collision")


def generate(output, state, *, hardware="mock_components/GenericSystem", asset_root=None):
    selected = json.loads((ROOT / "config/robots/robot_model.json").read_text())
    source = ROOT / selected["urdf"]
    model = RobotModel(source)
    robot = ET.parse(source).getroot()
    collision_path = source.parent / BASE_COLLISION_RELATIVE
    collision_record = json.loads((collision_path.parent / "provenance.json").read_text())
    if hashlib.sha256(collision_path.read_bytes()).hexdigest() != collision_record["output_sha256"]:
        raise ValueError("Base collision model fingerprint mismatch")
    if (
        hashlib.sha256((ROOT / collision_record["source"]).read_bytes()).hexdigest()
        != collision_record["source_sha256"]
    ):
        raise ValueError("Base collision source changed; regenerate and verify the derived model")
    replace_base_collision(robot)
    sdk = state["arm_startup"]["joint_control"]
    names = [f"openarm_{side}_joint{i}" for side in ("right", "left") for i in range(1, 8)]
    indices = [*range(7), *range(8, 15)]
    limits = {}
    initial = {}
    for name, index in zip(names, indices):
        item = robot.find(f"joint[@name='{name}']/limit")
        lower = max(float(item.get("lower")), sdk["lower"][index])
        upper = min(float(item.get("upper")), sdk["upper"][index])
        if not lower < upper:
            raise ValueError("Invalid SDK/URDF limit intersection: " + name)
        item.set("lower", str(lower))
        item.set("upper", str(upper))
        item.set("velocity", str(min(float(item.get("velocity")), 0.2)))
        limits[name] = dict(
            has_velocity_limits=True,
            max_velocity=0.2,
            has_acceleration_limits=True,
            max_acceleration=0.5,
        )
        initial[name] = float(state["arms"]["position"][index])
    # Reuse existing GraspSceneCollision encoder-to-URDF mapping; lift stays fixed.
    passive = {"lift_joint": state["lift"]["current_um"] * 1e-6 + 0.2}
    grippers = json.loads((ROOT / "config/robots/calibration/gripper_claw14.json").read_text())[
        "sides"
    ]
    for side, index in [("right", 7), ("left", 15)]:
        g = grippers[side]
        passive[f"openarm_{side}_finger_joint1"] = (
            g["closed_displacement_m"]
            + 0.044
            * g["opening_sign"]
            * (state["arms"]["position"][index] - g["closed_feedback_rad"])
            / 1.0472
        )
    # Static non-arm joints must still be present in MoveIt's complete state.
    # Mimic fingers are derived by robot_state_publisher, not independently driven.
    for joint in robot.findall("joint"):
        name = joint.get("name")
        if (
            name not in names
            and name not in passive
            and joint.get("type") != "fixed"
            and joint.find("mimic") is None
        ):
            passive[name] = 0.0
    for mesh in robot.findall(".//mesh"):
        filename = mesh.get("filename")
        if not filename.startswith(("package://", "file://", "/")):
            mesh.set(
                "filename",
                "file://" + str((Path(asset_root) if asset_root else source.parent) / filename),
            )
    control = ET.SubElement(robot, "ros2_control", name="WheelSDK", type="system")
    hw = ET.SubElement(control, "hardware")
    ET.SubElement(hw, "plugin").text = hardware
    for name, index in zip(names, indices):
        joint = ET.SubElement(control, "joint", name=name)
        ET.SubElement(joint, "param", name="sdk_index").text = str(index)
        for interface in ("position", "velocity"):
            ET.SubElement(joint, "command_interface", name=interface)
        for interface in ("position", "velocity", "effort"):
            element = ET.SubElement(joint, "state_interface", name=interface)
            ET.SubElement(element, "param", name="initial_value").text = str(
                initial[name] if interface == "position" else 0.0
            )
    semantic = ET.Element("robot", name=robot.get("name"))
    for side in ("right", "left"):
        group = ET.SubElement(semantic, "group", name=side + "_arm")
        ET.SubElement(
            group, "chain", base_link=f"openarm_{side}_base_link", tip_link=f"openarm_{side}_link7"
        )
    group = ET.SubElement(semantic, "group", name="both_arms")
    for side in ("right", "left"):
        ET.SubElement(group, "group", name=side + "_arm")
    for first, second in fixed_body_exclusions(model.joints):
        ET.SubElement(
            semantic,
            "disable_collisions",
            link1=first,
            link2=second,
            reason="Adjacent or same rigid body",
        )
    for name, j in model.joints.items():
        if name not in names and j["kind"] != "fixed":
            ET.SubElement(semantic, "passive_joint", name=name)
    controller = dict(
        joints=names,
        command_interfaces=["position", "velocity"],
        state_interfaces=["position", "velocity"],
        open_loop_control=False,
        allow_partial_joints_goal=False,
        allow_nonzero_velocity_at_trajectory_end=False,
        state_publish_rate=100.0,
        action_monitor_rate=50.0,
        constraints=dict(
            stopped_velocity_tolerance=0.03,
            goal_time=3.0,
            **{n: dict(trajectory=0.05, goal=0.02) for n in names},
        ),
    )
    configs = {
        "joint_limits.yaml": dict(joint_limits=limits),
        "kinematics.yaml": {
            side + "_arm": dict(
                kinematics_solver="kdl_kinematics_plugin/KDLKinematicsPlugin",
                kinematics_solver_timeout=0.1,
            )
            for side in ("left", "right")
        },
        "controllers.yaml": dict(
            controller_manager={
                "ros__parameters": dict(
                    update_rate=100,
                    arm_controller=dict(
                        type="joint_trajectory_controller/JointTrajectoryController"
                    ),
                    joint_state_broadcaster=dict(
                        type="joint_state_broadcaster/JointStateBroadcaster"
                    ),
                )
            },
            arm_controller={"ros__parameters": controller},
        ),
        "moveit_controllers.yaml": dict(
            moveit_controller_manager="moveit_simple_controller_manager/MoveItSimpleControllerManager",
            moveit_simple_controller_manager=dict(
                controller_names=["arm_controller"],
                arm_controller=dict(
                    type="FollowJointTrajectory",
                    action_ns="follow_joint_trajectory",
                    default=True,
                    joints=names,
                ),
            ),
        ),
        "ompl_planning.yaml": dict(
            planning_plugin="ompl_interface/OMPLPlanner",
            request_adapters="default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            start_state_max_bounds_error=0.0,
            path_tolerance=0.0,
            resample_dt=0.01,
            min_angle_change=0.001,
            planner_configs=dict(
                RRTConnectkConfigDefault=dict(type="geometric::RRTConnect", range=0.0)
            ),
            both_arms=dict(
                planner_configs=["RRTConnectkConfigDefault"], longest_valid_segment_fraction=0.005
            ),
        ),
    }
    output.mkdir(parents=True, exist_ok=False)
    ET.indent(robot)
    ET.ElementTree(robot).write(output / "robot.urdf", encoding="unicode")
    ET.indent(semantic)
    ET.ElementTree(semantic).write(output / "robot.srdf", encoding="unicode")
    for name, value in configs.items():
        (output / name).write_text(yaml.safe_dump(value, sort_keys=False))
    (output / "context.json").write_text(
        json.dumps(
            dict(
                sdk_state=state,
                passive_joint_positions=passive,
                hardware=hardware,
                source_urdf=str(source),
                source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                collision_scope="Full robot collision geometry, except base_link uses a separately validated, component-preserving simplified surface at the original visual origin. High-resolution visual mesh unchanged; no added collision exclusions.",
                base_collision_mesh_sha256=collision_record["output_sha256"],
                base_collision_sampled_error_budget_m=collision_record["sampled_error_budget_m"],
                base_collision_source_sha256=collision_record["source_sha256"],
            ),
            indent=2,
        )
    )
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--hardware", default="mock_components/GenericSystem")
    parser.add_argument("--asset-root", type=Path)
    args = parser.parse_args()
    print(
        generate(
            args.output,
            json.loads(args.state.read_text()),
            hardware=args.hardware,
            asset_root=args.asset_root,
        )
    )
