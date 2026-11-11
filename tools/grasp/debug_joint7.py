#!/usr/bin/env python3
"""Trace measured J7 through independent FK, camera projection and rasterization."""

import argparse
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
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics
from wheel_perception_core.geometry import prepare_input
from wheel_perception_core.rgbd import load_observation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--unified-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    state = json.loads((args.capture / "paired_joint_state.json").read_text())
    registration = json.loads(args.unified_report.read_text())
    observation = prepare_input(load_observation(args.capture / "observation.npz"))
    capture_report = json.loads((args.capture / "pose_report.json").read_text())
    urdf = capture_model(ROOT, capture_report)["urdf"]
    model = RobotModel(urdf)
    xml = ET.parse(urdf).getroot()
    values = SDKKinematics.from_sdk_vector(state["arms"]["position"])
    values["lift_joint"] = registration["urdf_lift_m"]
    camera_inverse = np.linalg.inv(registration["base_footprint_T_camera"])
    results = {}
    current_image = cv2.cvtColor(observation.rgb, cv2.COLOR_RGB2BGR)
    union_image = current_image.copy()
    for side, index in [("left", 14), ("right", 6)]:
        angle = state["arms"]["position"][index]
        cosine, sine = np.cos(angle), np.sin(angle)
        expected = np.array(
            [[1, 0, 0, 0], [0, cosine, -sine, 0], [0, sine, cosine, 0], [0, 0, 0, 1]]
        )
        link6 = model.transform(f"openarm_{side}_link6", values)
        wrist = model.transform(f"openarm_{side}_link7", values)
        relative = np.linalg.inv(link6) @ wrist
        frame = f"openarm_{side}_ee_base_link"
        body = model.transform(frame, values)
        visual = xml.find(f"link[@name='{frame}']/visual")
        mesh = visual.find("geometry/mesh")
        asset = trimesh.load(urdf.parent / mesh.get("filename"), force="mesh")
        local = np.eye(4)
        origin = visual.find("origin")
        if origin is not None:
            local[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
            local[:3, :3] = Rotation.from_euler(
                "xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
            ).as_matrix()
        vertices = asset.vertices * np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
        transform = camera_inverse @ body @ local
        points = (transform @ np.c_[vertices, np.ones(len(vertices))].T).T[:, :3]
        front = points[:, 2] > 0.01
        homogeneous = (observation.k @ points.T).T
        pixels = homogeneous[:, :2] / homogeneous[:, 2, None]
        projected, _ = cv2.projectPoints(
            vertices[front],
            cv2.Rodrigues(transform[:3, :3])[0],
            transform[:3, 3],
            observation.k,
            np.zeros(5),
        )
        faces = asset.faces
        valid = np.all(front[faces], axis=1) & np.all(np.abs(pixels[faces]) < 10000, axis=(1, 2))
        polygons = list(np.rint(pixels[faces[valid]]).astype(np.int32))
        current_mask = np.zeros(observation.rgb.shape[:2], np.uint8)
        union_mask = current_mask.copy()
        cv2.fillPoly(current_mask, polygons, 255)
        for polygon in polygons:
            cv2.fillConvexPoly(union_mask, polygon, 255)
        color = (255, 200, 0) if side == "left" else (0, 180, 255)
        silhouettes = []
        for mask, image, name in [
            (current_mask, current_image, "existing"),
            (union_mask, union_image, "triangle_union"),
        ]:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            silhouette = np.zeros_like(mask)
            cv2.drawContours(image, contours, -1, color, 1)
            cv2.drawContours(silhouette, contours, -1, 255, cv2.FILLED)
            silhouettes.append(silhouette > 0)
            cv2.imwrite(str(args.output / f"{side}_{name}_mask.png"), mask)
        changed_values = dict(values)
        changed_values[f"openarm_{side}_joint7"] += 0.01
        changed_wrist = model.transform(f"openarm_{side}_link7", changed_values)
        results[side] = dict(
            sdk_array_index=index,
            measured_rad=angle,
            measured_deg=float(np.rad2deg(angle)),
            declared_axis=model.joints[f"openarm_{side}_joint7"]["axis"].tolist(),
            analytic_rotation_max_error=float(np.max(np.abs(relative - expected))),
            urdf_vs_mujoco_wrist_max_error=float(
                np.max(np.abs(wrist - np.asarray(registration["wrists"][side])))
            ),
            manual_vs_opencv_pixel_error=float(np.max(np.abs(pixels[front] - projected[:, 0]))),
            origin_motion_for_j7_plus_001_rad_m=float(
                np.linalg.norm(changed_wrist[:3, 3] - wrist[:3, 3])
            ),
            base_T_link6=link6.tolist(),
            link6_T_link7=relative.tolist(),
            base_T_body=body.tolist(),
            camera_T_mesh=transform.tolist(),
            raster_mask_pixels_existing=int(np.count_nonzero(current_mask)),
            raster_mask_pixels_union=int(np.count_nonzero(union_mask)),
            external_silhouette_iou=float(
                np.count_nonzero(silhouettes[0] & silhouettes[1])
                / max(1, np.count_nonzero(silhouettes[0] | silhouettes[1]))
            ),
        )
    cv2.imwrite(str(args.output / "existing_overlay.png"), current_image)
    cv2.imwrite(str(args.output / "triangle_union_overlay.png"), union_image)
    report = dict(
        commands_sent=False,
        input_capture=str(args.capture),
        camera_K=observation.k.tolist(),
        camera_frame=observation.frame,
        input_reasons=observation.reasons,
        sdk_joint_order=state["arms"]["joint_order"],
        sdk_unit=state["arm_startup"]["joint_control"]["unit"],
        results=results,
        physical_parameter_correspondence_accepted=False,
    )
    (args.output / "trace.json").write_text(json.dumps(report, indent=2) + "\n")
    for side, result in results.items():
        print(
            side,
            {
                key: value
                for key, value in result.items()
                if key not in ["base_T_link6", "link6_T_link7", "base_T_body", "camera_T_mesh"]
            },
        )


if __name__ == "__main__":
    main()
