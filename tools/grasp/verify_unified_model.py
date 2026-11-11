#!/usr/bin/env python3
"""Compare whole-robot MuJoCo FK with source URDF and replay real capture overlays."""

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import mujoco
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure

configure(ROOT)
from wheel_motion_core.gripper import finger_displacement
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics
from wheel_motion_core.unified_model import UnifiedModel, registered_camera
from wheel_perception_core.extrinsics import HeadExtrinsics
from wheel_perception_core.geometry import prepare_input
from wheel_perception_core.rgbd import load_observation
from wheel_perception_core.visualization import rasterize_triangles


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model", type=Path, required=True)
    p.add_argument("--capture", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--lift-offset-m", type=float, required=True)
    p.add_argument(
        "--calibration",
        type=Path,
        default=ROOT / "config/robots/calibration/head_camera_j4_rezero_20261104.json",
    )
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = json.loads((args.model.parent / "manifest.json").read_text())
    urdf = Path(manifest["urdf"])
    urdf_model = RobotModel(urdf)
    sdk_model = SDKKinematics(ROOT / "config/robots/calibration/sdk_arm_chains.json")
    calibration = HeadExtrinsics(args.calibration)
    unified_model = UnifiedModel(args.model, lift_direction=1, lift_offset_m=args.lift_offset_m)
    rng = np.random.default_rng(30)
    errors = []
    # Random input comparisons do not infer any physical encoder offset.
    for _ in range(30):
        joint_positions = np.zeros(16)
        for side, start in [("right", 0), ("left", 8)]:
            for i in range(7):
                joint_positions[start + i] = rng.uniform(
                    *urdf_model.joints[f"openarm_{side}_joint{i + 1}"]["bounds"]
                )
        lift = rng.uniform(0, 0.4)
        finger_displacements = {
            side: rng.uniform(*urdf_model.joints[f"openarm_{side}_finger_joint1"]["bounds"])
            for side in ["left", "right"]
        }
        joint_values = unified_model.update(
            joint_positions,
            (lift - args.lift_offset_m) * 1e6,
            finger_displacements=finger_displacements,
        )
        joint_values.update(
            {
                n: 0.0
                for n, j in urdf_model.joints.items()
                if j["kind"] != "fixed" and n not in joint_values
            }
        )
        for link in urdf_model.links:
            errors.append(
                float(
                    np.max(
                        np.abs(
                            urdf_model.transform(link, joint_values)
                            - unified_model.site("frame_" + link)
                        )
                    )
                )
            )
    if max(errors) > 1e-9:
        raise ValueError("MuJoCo/URDF FK mismatch")
    state = json.loads((args.capture / "paired_joint_state.json").read_text())
    lift_context = json.loads((args.capture / "lift_context.json").read_text())
    joint_positions = state["arms"]["position"]
    obs = prepare_input(load_observation(args.capture / "observation.npz"))
    finger_displacements = {side: 0.03 for side in ["left", "right"]}
    visible = []
    mapping_results = {}
    profile = json.loads((ROOT / "config/robots/calibration/gripper_v1.json").read_text())
    for side in ["left", "right"]:
        cfg = profile["sides"][side]
        approximation = finger_displacement(
            joint_positions[15 if side == "left" else 7],
            offset_rad=cfg["offset_rad"],
            opening_sign=cfg["opening_sign"],
            bounds=urdf_model.joints[f"openarm_{side}_finger_joint1"]["bounds"],
            closed_feedback_rad=cfg.get("closed_feedback_rad"),
            closed_displacement_m=cfg.get("closed_displacement_m"),
        )
        mapping_results[side] = approximation
        if approximation.get("within_urdf_limits"):
            finger_displacements[side] = approximation["estimated_finger_displacement_m"]
            visible.extend([f"openarm_{side}_ee_link1", f"openarm_{side}_ee_link2"])
    joint_values = unified_model.update(
        joint_positions, lift_context["height_um"], finger_displacements=finger_displacements
    )
    camera = registered_camera(
        urdf_model, sdk_model, calibration, lift_context["height_um"], joint_values["lift_joint"]
    )
    # A head fixed to the chassis must have constant base_T_camera across heights.
    cameras = [
        registered_camera(urdf_model, sdk_model, calibration, h, h * 1e-6 + args.lift_offset_m)
        for h in [-180000, 0, 180000]
    ]
    camera_error = max(float(np.max(np.abs(t - cameras[0]))) for t in cameras)
    if camera_error > 1e-9:
        raise ValueError("Lift compensation duplicated or inverted")
    image = cv2.cvtColor(obs.rgb, cv2.COLOR_RGB2BGR)
    xml = ET.parse(urdf).getroot()
    links = {
        f"openarm_{side}_{suffix}"
        for side in ["left", "right"]
        for suffix in ["link4", "link5", "link6", "ee_base_link"]
    } | set(visible)
    for link in links:
        t = np.linalg.inv(camera) @ unified_model.site("frame_" + link)
        geometry = "collision" if link.endswith(("link4", "link5", "link6")) else "visual"
        color = (0, 255, 0) if geometry == "collision" else (0, 180, 255)
        for v in xml.findall(f"link[@name='{link}']/{geometry}"):
            mesh = v.find("geometry/mesh")
            if mesh is None:
                continue
            asset = trimesh.load(urdf.parent / mesh.get("filename"), force="mesh")
            o = v.find("origin")
            local = np.eye(4)
            if o is not None:
                local[:3, 3] = np.fromstring(o.get("xyz", "0 0 0"), sep=" ")
                local[:3, :3] = Rotation.from_euler(
                    "xyz", np.fromstring(o.get("rpy", "0 0 0"), sep=" ")
                ).as_matrix()
            vertices = asset.vertices * np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
            points = (t @ local @ np.c_[vertices, np.ones(len(vertices))].T).T[:, :3]
            pixels = (obs.k @ points.T).T
            uv = pixels[:, :2] / pixels[:, 2, None]
            f = asset.faces
            valid = np.all(points[f, 2] > 0.01, axis=1) & np.all(np.abs(uv[f]) < 10000, axis=(1, 2))
            polys = list(np.rint(uv[f[valid]]).astype(np.int32))
            mask = rasterize_triangles(polys, image.shape)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image, contours, -1, color, 1)
    cv2.putText(
        image,
        "Whole URDF -> MuJoCo: measured arm q, NO J7 corrections",
        (5, 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.43,
        (255, 255, 255),
        1,
    )
    cv2.imwrite(str(args.output / "overlay.png"), image)
    report = dict(
        commands_sent=False,
        calibration_path=str(calibration.path),
        calibration_selection=calibration.config["selection"],
        deployable=False,
        model=str(args.model),
        mujoco_version=mujoco.__version__,
        opening_mapping=mapping_results,
        random_poses=30,
        link_comparisons=len(errors),
        max_fk_matrix_error=max(errors),
        camera_constancy_error=camera_error,
        lift_encoder_um=lift_context["height_um"],
        urdf_lift_m=joint_values["lift_joint"],
        lift_offset_m=args.lift_offset_m,
        lift_mapping_status="explicit candidate, not measured",
        camera_registration="left-base correspondence assumption; physical acceptance pending",
        base_footprint_T_camera=camera.tolist(),
        sdk_origin_T_camera=(np.linalg.inv(unified_model.site("arm_origin")) @ camera).tolist(),
        wrists={
            side: unified_model.site(side + "_ee_control_point").tolist()
            for side in ["left", "right"]
        },
        wheel_feedback_status="Not available; passive wheel rotation fixed to zero for base-attached FK checks",
        unknown_opening_fingers_omitted=[
            sdk_model
            for sdk_model in ["left", "right"]
            if f"openarm_{sdk_model}_ee_link1" not in visible
        ],
        physical_alignment_accepted=False,
    )
    (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(
        json.dumps(
            {
                k: v
                for k, v in report.items()
                if k not in ["wrists", "base_footprint_T_camera", "sdk_origin_T_camera"]
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
