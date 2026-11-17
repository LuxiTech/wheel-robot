#!/usr/bin/env python3
"""Connect saved real vision inference to fixed-lift SDK trajectory planning.

No POSTs, enables, zeros, lift sessions or robot motion. Execution is handled
separately by StationaryGraspExecutor with live perception/verification sources.
"""

import argparse
import hashlib
import html
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure
from _robot_model import capture_model

configure(ROOT)
from wheel_motion_core.visual_grasp import VisualGraspPlanner
from wheel_perception_core.extrinsics import CalibratedHeadTransform, HeadExtrinsics
from wheel_perception_core.rgbd import load_observation
from wheel_perception_core.targets import visual_target
from wheel_perception_core.visualization import draw_targets


def read(path):
    return json.loads(path.read_text())


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preview", type=Path, required=True)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=40)
    parser.add_argument("--position-tolerance-mm", type=float, default=150.0)
    parser.add_argument("--mesh-collision", action="store_true")
    parser.add_argument(
        "--kinematics-only",
        action="store_true",
        help="Skip robot/crate collision diagnosis; never permits execution",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    capture = read(args.capture / "pose_report.json")
    state = read(args.capture / "paired_joint_state.json")
    obs = load_observation(args.capture / "observation.npz")
    preview_obs = load_observation(args.preview / "observation.npz")
    if obs.key != preview_obs.key:
        raise ValueError("Inference and measured-joint capture belong to different image pairs")
    calibration = HeadExtrinsics(args.calibration)
    camera, reasons = CalibratedHeadTransform(calibration).resolve(
        obs, read(args.capture / "lift_context.json")
    )
    resources = capture_model(ROOT, capture)
    contacts = read(resources["contacts"])
    source = read(resources["source"])
    cad = read(args.preview / "cad_and_rules.json")
    quality = read(args.preview / "pose_quality.json")
    pose_camera = np.asarray(read(args.preview / "pose.json")["camera_pose"])
    blockers = [reason for reason in reasons if "not_registered" in reason]
    if calibration.config.get("deployable") is not True:
        blockers.append("selected_extrinsics_not_hardware_accepted")
    if source.get("physical_gripper_alignment_accepted") is not True:
        blockers.append("physical_gripper_alignment_not_accepted")
    if contacts.get("hardware_validated") is not True:
        blockers.append("hook_reference_not_hardware_accepted")
    if cad.get("grasp_rules_validated") is not True:
        blockers.append("rim_rules_not_hardware_accepted")
    target = visual_target(
        target_id="central_crate",
        pair_id=obs.key[1],
        object_pose_origin=camera @ pose_camera,
        rules=cad["rules"],
        height_um=capture["height_um"],
        quality=quality,
        source_blockers=blockers,
    )
    save(args.output / "target.json", target)
    limits = state["arm_startup"]["joint_control"]
    planner = VisualGraspPlanner(
        resources["urdf"],
        resources["sdk_chains"],
        contacts,
        limits["lower"],
        limits["upper"],
        gripper_config=read(resources["gripper"])["sides"],
        position_tolerance_m=args.position_tolerance_mm * 0.001,
    )
    target = planner.resolve_target(target, state["arms"]["position"])
    save(args.output / "resolved_tool_targets.json", target)
    height_m = capture["height_um"] * 1e-6
    wrist = {
        stage: {
            side: value.tolist()
            for side, value in planner.wrist_targets(
                target, stage, height_m, q16=state["arms"]["position"]
            ).items()
        }
        for stage in ("pregrasp", "grasp")
    }
    save(
        args.output / "wrist_targets.json",
        dict(frame_id="sdk_lift_zero", endpoint="SDK EE control point / link7", stages=wrist),
    )
    kinematic_plan = planner.plan(target, state["arms"]["position"], height_m, steps=args.steps)
    save(args.output / "kinematics_plan.json", kinematic_plan)
    if args.kinematics_only:
        plan = kinematic_plan
    else:
        from wheel_motion_core.grasp_scene import CADMaterial, GraspSceneCollision

        material = CADMaterial(
            Path(cad["source_mesh"]), cad["source_sha256"], cad["source_to_opening_up"]
        )
        opening = read(resources["gripper"])["sides"]
        checker = GraspSceneCollision(
            resources["urdf"],
            resources["sdk_chains"],
            material,
            target,
            opening,
            mesh_narrow_phase=args.mesh_collision,
        )
        planner.collision_check = checker
        plan = planner.plan(target, state["arms"]["position"], height_m, steps=args.steps)
    save(args.output / "plan.json", plan)
    import cv2
    from wheel_perception_core.geometry import prepare_input

    prepared = prepare_input(obs)
    photo = cv2.cvtColor(prepared.rgb, cv2.COLOR_RGB2BGR)
    image, projected = draw_targets(
        photo, {side: np.asarray(t) for side, t in target["grasp"].items()}, camera, prepared.k
    )
    cv2.imwrite(str(args.output / "assigned_grasp_targets.png"), image)
    path_image = photo.copy()
    inverse_camera = np.linalg.inv(camera)
    for side, color in (("left", (0, 230, 120)), ("right", (0, 190, 255))):
        pixels = []
        for stage in kinematic_plan["stages"].values():
            for point in stage["points"]:
                wrist_pose = planner.ik.fk(point["joints"])[side]
                optical_point = (wrist_pose @ planner.mounts[side])[:3, 3].copy()
                optical_point[2] -= height_m
                optical_point = (inverse_camera @ np.r_[optical_point, 1])[:3]
                if optical_point[2] > 0.01:
                    screen = prepared.k @ optical_point
                    xy = screen[:2] / screen[2]
                    if np.isfinite(xy).all() and np.abs(xy).max() < 10000:
                        pixels.append(np.rint(xy).astype(np.int32))
        if len(pixels) > 1:
            cv2.polylines(path_image, [np.asarray(pixels)], False, color, 2)
    cv2.putText(
        path_image,
        "MODEL PATH ONLY - no robot motion",
        (8, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        2,
    )
    cv2.imwrite(str(args.output / "trajectory_preview.png"), path_image)
    current_mapping_sha256 = hashlib.sha256(resources["gripper"].read_bytes()).hexdigest()
    capture_mapping_sha256 = capture.get("source_sha256", {}).get(
        str(resources["gripper"].relative_to(ROOT))
    )
    result = dict(
        capture_gripper_mapping_sha256=capture_mapping_sha256,
        current_gripper_mapping_sha256=current_mapping_sha256,
        mapping_reprocessed_from_unchanged_measured_feedback=(
            capture_mapping_sha256 is not None and capture_mapping_sha256 != current_mapping_sha256
        ),
        status=(
            "scene_collision_rejected"
            if "collision_path_rejected" in plan["execution_blockers"]
            else "vision_to_motion_planned"
            if "grasp" in plan["stages"]
            else "vision_to_motion_IK_rejected"
        ),
        kinematic_stage_count=len(kinematic_plan["stages"]),
        position_tolerance_m=planner.ik.ptol,
        collision_check=plan.get("collision_check"),
        commands_sent=False,
        execution_ready=plan["execution_ready"],
        target_id=target["target_id"],
        pair_id=obs.key[1],
        projection=projected,
        execution_blockers=plan["execution_blockers"],
        errors=plan["errors"],
        sources={
            str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [
                args.preview / "pose.json",
                args.preview / "pose_quality.json",
                args.capture / "paired_joint_state.json",
                resources["urdf"],
                resources["sdk_chains"],
                resources["gripper"],
                args.calibration,
            ]
        },
    )
    save(args.output / "summary.json", result)
    detail = html.escape(json.dumps(result, ensure_ascii=False, indent=2))
    page = (
        '<!doctype html><meta charset="utf-8"><title>视觉目标到传统运控</title><style>body{font:16px system-ui;margin:28px;background:#14202b;color:#dfebf5}img{max-width:960px;width:100%}pre{white-space:pre-wrap}a{color:#8cdae8}</style><h1>视觉抓取点 → 固定升降双臂运控</h1><p>真实RGB-D → NanoSAM分割 → FoundationPose估姿 → CAD箱沿点 → SDK左右臂分配 → 钩部到腕端转换 → 固定升降IK分段路径</p><p>图中左右臂按SDK坐标Y分配，不按图像A/B字母分配。未发任何运动指令。</p><p>碰撞检查使用保守机器人包围盒与箱体CAD材料体素。模型相交不等于已证实实物碰撞；结果也不覆盖未验收的周边场景。</p><p>按导轨下表面停止高度换算的夹持参考目标：</p><img src="assigned_grasp_targets.png"><p>以下仅为模型规划路径，未发送真机：</p><img src="trajectory_preview.png"><p><a href="target.json">视觉目标</a> · <a href="wrist_targets.json">SDK腕端目标</a> · <a href="plan.json">碰撞检查与执行拒绝原因</a> · <a href="kinematics_plan.json">未获执行许可的IK路径</a></p><pre>'
        + detail
        + "</pre>"
    )
    (args.output / "report.html").write_text(page)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
