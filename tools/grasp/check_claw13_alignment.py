#!/usr/bin/env python3
"""Offline full-geometry overlay of supplied claw1.3 on a paired real capture."""

import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure
from _robot_model import capture_model

configure(ROOT)
from wheel_motion_core.gripper import finger_displacement
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics
from wheel_perception_core.extrinsics import CalibratedHeadTransform, HeadExtrinsics
from wheel_perception_core.geometry import prepare_input
from wheel_perception_core.rgbd import load_observation
from wheel_perception_core.visualization import rasterize_triangles


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--capture", type=Path, required=True)
    p.add_argument("--calibration", type=Path, required=True, help="Match the capture's zero epoch")
    p.add_argument("--output", type=Path)
    p.add_argument("--body-only", action="store_true")
    args = p.parse_args()
    pose_path = args.capture / "pose_report.json"
    if pose_path.exists():
        capture_report = json.loads(pose_path.read_text())
    else:
        # Older 1.3 captures predate pose_report/model_resources metadata.
        source = json.loads(
            (ROOT / "config/robots/r680_lift_openarm_claw1.3.source.json").read_text()
        )
        capture_report = {"urdf_sha256": source["urdf_sha256"]}
    resources = capture_model(ROOT, capture_report)
    urdf = resources["urdf"]
    model_label = urdf.parent.name.removeprefix("r680_lift_openarm_")
    out = args.output or args.capture / model_label
    out.mkdir(parents=True, exist_ok=False)
    model = RobotModel(urdf)
    sdk = SDKKinematics(resources["sdk_chains"])
    xml = ET.parse(urdf).getroot()
    obs = load_observation(args.capture / "observation.npz")
    inp = prepare_input(obs)
    ctx = json.loads((args.capture / "lift_context.json").read_text())
    joint_state = json.loads((args.capture / "paired_joint_state.json").read_text())
    joint_values = sdk.from_sdk_vector(joint_state["arms"]["position"])
    oldcamera, _ = CalibratedHeadTransform(HeadExtrinsics(args.calibration)).resolve(obs, ctx)
    camera = oldcamera
    im = cv2.cvtColor(inp.rgb, cv2.COLOR_RGB2BGR)
    overlay = im.copy()
    report = {
        "executable": False,
        "commands_sent": False,
        "model": str(urdf),
        "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
        "capture": str(args.capture),
        "camera_registration": "Candidate camera extrinsics in arm_origin; each gripper uses its own SDK wrist and CAD wrist-to-gripper chain. Not hardware accepted.",
        "arm_origin_T_camera": camera.tolist(),
        "joint_convention": "direct measured angles, no sign changes or zero offsets",
        "fk_checks": {},
        "geometry": [],
    }
    mapping_config = json.loads(resources["gripper"].read_text())
    report["opening_mapping"] = {}
    rng = np.random.default_rng(13)
    for side, color in [("left", (255, 200, 0)), ("right", (0, 170, 255))]:
        base = model.transform(f"openarm_{side}_base_link", {}, base="lift_carriage_link")
        sdkbase = sdk.transform(f"openarm_{side}_base_link")
        poserr = []
        roterr = []
        for sample in range(100):
            values = {
                f"openarm_{side}_joint{k + 1}": float(
                    rng.uniform(*model.joints[f"openarm_{side}_joint{k + 1}"]["bounds"])
                )
                for k in range(7)
            }
            cad = np.linalg.inv(base) @ model.transform(
                f"openarm_{side}_link7", values, base="lift_carriage_link"
            )
            ref = np.linalg.inv(sdkbase) @ sdk.transform(side + "_ee_control_point", values)
            poserr.append(np.linalg.norm(cad[:3, 3] - ref[:3, 3]))
            roterr.append(Rotation.from_matrix(cad[:3, :3].T @ ref[:3, :3]).magnitude())
        report["fk_checks"][side] = {
            "poses": 100,
            "max_position_error_mm": float(max(poserr) * 1000),
            "max_angle_error_deg": float(np.rad2deg(max(roterr))),
            "scope": "Relative to each arm base, excluding bilateral installation and custom gripper",
        }
        cfg = mapping_config["sides"][side]
        mapping = finger_displacement(
            joint_state["arms"]["position"][15 if side == "left" else 7],
            offset_rad=cfg["offset_rad"],
            opening_sign=cfg["opening_sign"],
            closed_feedback_rad=cfg.get("closed_feedback_rad"),
            closed_displacement_m=cfg.get("closed_displacement_m"),
            bounds=model.joints[f"openarm_{side}_finger_joint1"]["bounds"],
        )
        report["opening_mapping"][side] = mapping
        names = [
            f"openarm_{side}_ee_base_link",
            f"openarm_{side}_ee_link1",
            f"openarm_{side}_ee_link2",
        ]
        if args.body_only:
            names = names[:1]
        for name in names:
            link = xml.find(f"link[@name='{name}']")
            opens = (
                [None]
                if name.endswith("base_link")
                else (
                    [mapping["estimated_finger_displacement_m"]]
                    if mapping["within_urdf_limits"]
                    else []
                )
            )
            for opening in opens:
                vals = joint_values.copy()
                if opening is not None:
                    vals[f"openarm_{side}_finger_joint1"] = opening
                link_transform = sdk.gripper_transform(model, side, name, vals)
                for visual in link.findall("visual"):
                    mesh = visual.find("geometry/mesh")
                    visual_origin = visual.find("origin")
                    visual_transform = np.eye(4)
                    if visual_origin is not None:
                        visual_transform[:3, :3] = Rotation.from_euler(
                            "xyz", np.fromstring(visual_origin.get("rpy", "0 0 0"), sep=" ")
                        ).as_matrix()
                        visual_transform[:3, 3] = np.fromstring(
                            visual_origin.get("xyz", "0 0 0"), sep=" "
                        )
                    path = urdf.parent / mesh.get("filename")
                    mesh_geometry = trimesh.load(path, force="mesh")
                    verts = mesh_geometry.vertices * np.fromstring(
                        mesh.get("scale", "1 1 1"), sep=" "
                    )
                    xyz = (
                        np.linalg.inv(camera)
                        @ link_transform
                        @ visual_transform
                        @ np.c_[verts, np.ones(len(verts))].T
                    ).T[:, :3]
                    uvh = (inp.k @ xyz.T).T
                    uv = uvh[:, :2] / uvh[:, 2, None]
                    face = mesh_geometry.faces
                    ok = np.all(xyz[face, 2] > 0.01, axis=1) & np.all(
                        np.abs(uv[face]) < 10000, axis=(1, 2)
                    )
                    mask = rasterize_triangles(np.rint(uv[face[ok]]).astype(np.int32), im.shape)
                    cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(overlay, cs, -1, color, 1)
                    report["geometry"].append(
                        {
                            "link": name,
                            "opening_m": opening,
                            "mesh_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                        }
                    )
    cv2.putText(
        overlay,
        model_label + ": fixed bodies only"
        if args.body_only
        else model_label + ": measured q / official approximate opening",
        (8, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
    )
    invalid = [
        side
        for side, mesh_geometry in report["opening_mapping"].items()
        if not mesh_geometry["within_urdf_limits"]
    ]
    cv2.putText(
        overlay,
        "Invalid opening, fingers omitted: " + ",".join(invalid),
        (8, 38),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (0, 220, 255),
        1,
    )
    cv2.imwrite(str(out / "overlay.png"), overlay)
    cv2.imwrite(str(out / "rgb.png"), im)
    report["limitations"] = [
        "Official approximate opening drawn only where within URDF limits; invalid fingers omitted, never clipped.",
        "Physical contact TCP must be extracted and verified for this gripper mesh.",
        "2D silhouette is not depth or full-workspace acceptance.",
    ]
    (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(out, report["fk_checks"])


if __name__ == "__main__":
    main()
