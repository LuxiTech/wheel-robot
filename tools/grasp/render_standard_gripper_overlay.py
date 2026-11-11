#!/usr/bin/env python3
"""Depth-tested solid mesh overlays from a saved observation; never controls hardware."""

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
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics
from wheel_motion_core.unified_model import registered_camera
from wheel_perception_core.extrinsics import HeadExtrinsics
from wheel_perception_core.geometry import prepare_input
from wheel_perception_core.rgbd import load_observation


class DepthRaster:
    """Pixel-center barycentrics with perspective-correct camera-Z interpolation."""

    def __init__(self, height, width):
        self.color = np.zeros((height, width, 3), np.uint8)
        self.depth = np.full((height, width), np.inf, np.float32)
        self.labels = np.full((height, width), -1, np.int16)

    def triangle(self, screen, camera_z, color, label):
        height, width = self.depth.shape
        low = np.maximum(np.ceil(np.min(screen, axis=0) - 0.5).astype(int), [0, 0])
        high = np.minimum(
            np.floor(np.max(screen, axis=0) - 0.5).astype(int), [width - 1, height - 1]
        )
        if np.any(low > high) or np.any(camera_z <= 0):
            return
        a, b, c = np.asarray(screen, float)
        determinant = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(determinant) < 1e-10:
            return
        xx, yy = np.meshgrid(
            np.arange(low[0], high[0] + 1) + 0.5, np.arange(low[1], high[1] + 1) + 0.5
        )
        first = ((b[1] - c[1]) * (xx - c[0]) + (c[0] - b[0]) * (yy - c[1])) / determinant
        second = ((c[1] - a[1]) * (xx - c[0]) + (a[0] - c[0]) * (yy - c[1])) / determinant
        third = 1 - first - second
        inside = (first >= -1e-8) & (second >= -1e-8) & (third >= -1e-8)
        inverse_z = first / camera_z[0] + second / camera_z[1] + third / camera_z[2]
        depth = np.full(inverse_z.shape, np.inf)
        np.divide(1.0, inverse_z, out=depth, where=inside & (inverse_z > 0))
        region = np.s_[low[1] : high[1] + 1, low[0] : high[0] + 1]
        update = inside & (depth < self.depth[region])
        self.depth[region][update] = depth[update]
        self.color[region][update] = color
        self.labels[region][update] = label

    def mesh(self, vertices, faces, intrinsics, color, label):
        homogeneous = vertices @ intrinsics.T
        projected = homogeneous[:, :2] / homogeneous[:, 2, None]
        triangles = vertices[faces]
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        norms = np.linalg.norm(normals, axis=1)
        light = np.array([-0.25, -0.4, -1.0])
        light /= np.linalg.norm(light)
        normals /= np.maximum(norms[:, None], 1e-12)
        shades = 0.45 + 0.55 * np.abs(normals @ light)
        for face, shade in zip(faces, shades):
            if np.any(vertices[face, 2] <= 0.01):
                continue
            self.triangle(projected[face], vertices[face, 2], np.asarray(color) * shade, label)

    def rgba(self, width, height):
        covered = self.labels >= 0
        high = np.dstack([self.color, covered.astype(np.uint8) * 255])
        result = cv2.resize(high, (width, height), interpolation=cv2.INTER_AREA)
        # Resize produces premultiplied colors at the transparent boundary.
        alpha = result[:, :, 3].astype(float) / 255
        result[:, :, :3] = np.clip(
            result[:, :, :3].astype(float) / np.maximum(alpha[:, :, None], 1e-12), 0, 255
        ).astype(np.uint8)
        return result


def blend(photo, layer, opacity):
    alpha = layer[:, :, 3:4].astype(float) / 255 * opacity
    return np.rint(photo * (1 - alpha) + layer[:, :, :3] * alpha).astype(np.uint8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    capture = args.capture.resolve()
    report = json.loads((capture / "pose_report.json").read_text())
    state = json.loads((capture / "paired_joint_state.json").read_text())
    resources = capture_model(ROOT, report)
    geometry = RobotModel(resources["urdf"])
    for joint in geometry.joints.values():
        joint["bounds"] = None
    sdk = SDKKinematics(resources["sdk_chains"])
    values = sdk.from_sdk_vector(state["arms"]["position"])
    values["lift_joint"] = report["height_um"] * 1e-6 + 0.2
    calibration = HeadExtrinsics(args.calibration or report["calibration_path"])
    camera = registered_camera(
        geometry, sdk, calibration, report["height_um"], values["lift_joint"]
    )
    prepared = prepare_input(load_observation(capture / "observation.npz"))
    photo = cv2.cvtColor(prepared.rgb, cv2.COLOR_RGB2BGR)
    height, width = photo.shape[:2]
    scale = 2
    intrinsics = prepared.k.copy()
    intrinsics[:2] *= scale
    full = DepthRaster(height * scale, width * scale)
    gripper = DepthRaster(height * scale, width * scale)
    xml = ET.parse(resources["urdf"]).getroot()
    parts = []
    for side, side_id in [("right", 0), ("left", 1)]:
        mapping = report["arms"][side]["official_approximation"]
        values[f"openarm_{side}_finger_joint1"] = mapping["estimated_finger_displacement_m"]
        for suffix in [*(f"link{i}" for i in range(1, 8)), "ee_base_link", "ee_link1", "ee_link2"]:
            name = f"openarm_{side}_{suffix}"
            transform = np.linalg.inv(camera) @ geometry.transform(name, values)
            is_gripper = suffix.startswith("ee_")
            color = (
                (35, 75, 245)
                if suffix.startswith("ee_link")
                else (0, 185, 255)
                if is_gripper
                else (225, 175, 35)
            )
            for visual in xml.findall(f"link[@name='{name}']/visual"):
                mesh_node = visual.find("geometry/mesh")
                if mesh_node is None:
                    continue
                mesh_path = resources["urdf"].parent / mesh_node.get("filename")
                mesh = trimesh.load(mesh_path, force="scene").to_geometry()
                local = np.eye(4)
                origin = visual.find("origin")
                if origin is not None:
                    local[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                    local[:3, :3] = Rotation.from_euler(
                        "xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
                    ).as_matrix()
                vertices = mesh.vertices * np.fromstring(mesh_node.get("scale", "1 1 1"), sep=" ")
                optical = (transform @ local @ np.c_[vertices, np.ones(len(vertices))].T).T[:, :3]
                full.mesh(optical, mesh.faces, intrinsics, color, side_id)
                if is_gripper:
                    gripper.mesh(optical, mesh.faces, intrinsics, color, side_id)
                parts.append(
                    dict(
                        link=name,
                        mesh=str(mesh_path),
                        mesh_sha256=hashlib.sha256(mesh_path.read_bytes()).hexdigest(),
                        camera_T_link=transform.tolist(),
                        triangles=len(mesh.faces),
                    )
                )
            print("RENDERED", name, flush=True)
    layers = dict(full=full.rgba(width, height), gripper=gripper.rgba(width, height))
    cv2.imwrite(str(output / "photo.png"), photo)
    for name, layer in layers.items():
        cv2.imwrite(str(output / ("model_" + name + ".png")), layer)
        cv2.imwrite(str(output / ("overlay_" + name + "_45.png")), blend(photo, layer, 0.45))
    crops = {}
    for side, side_id in [("right", 0), ("left", 1)]:
        yy, xx = np.where(gripper.labels == side_id)
        if len(xx) == 0:
            continue
        x1, x2 = max(0, int(xx.min() / scale) - 35), min(width, int(xx.max() / scale) + 36)
        y1, y2 = max(0, int(yy.min() / scale) - 55), min(height, int(yy.max() / scale) + 96)
        region = np.s_[y1:y2, x1:x2]
        cv2.imwrite(str(output / (side + "_photo.png")), photo[region])
        cv2.imwrite(str(output / (side + "_model.png")), layers["full"][region])
        cv2.imwrite(
            str(output / (side + "_overlay_45.png")),
            blend(photo[region], layers["full"][region], 0.45),
        )
        crops[side] = dict(
            pixel_box_xyxy=[x1, y1, x2, y2],
            image_side="left" if np.mean(xx) / scale < width / 2 else "right",
            independent_alignment_applied=False,
        )
    manifest = dict(
        schema="depth_tested_standard_gripper_overlay_v1",
        source_capture=str(capture),
        capture_time_utc=report["created_utc"],
        calibration=str(calibration.path),
        calibration_sha256=hashlib.sha256(calibration.path.read_bytes()).hexdigest(),
        urdf_sha256=hashlib.sha256(resources["urdf"].read_bytes()).hexdigest(),
        measured_joint_angles=state["arms"]["position"],
        angles_modified=False,
        image_size_wh=[width, height],
        rectified_intrinsics=prepared.k.tolist(),
        anti_aliasing_scale=scale,
        model_self_occlusion="camera-Z buffer; perspective-correct reciprocal depth",
        real_scene_depth_occlusion=False,
        display_opacity=0.45,
        visual_pose_adjustments=False,
        projection=parts,
        crops=crops,
        finger_mapping={
            side: report["arms"][side]["official_approximation"] for side in ("left", "right")
        },
        wrist_cameras_omitted=True,
        robot_commands_sent=False,
        height_extrapolation_diagnostic=calibration.config.get("height_extrapolation_diagnostic"),
    )
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    print("SAVED", output, flush=True)


if __name__ == "__main__":
    main()
