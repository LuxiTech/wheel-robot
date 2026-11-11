#!/usr/bin/env python3
"""Fit one camera-Z offset to red finger silhouettes; validate frozen captures offline."""

import argparse
import hashlib
import html
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
from wheel_perception_core.visualization import rasterize_triangles

SEGMENTATION_POLICIES = {
    "nominal": (12, 100, 50),
    "wide": (15, 80, 40),
    "strict": (9, 130, 65),
}


def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def project_mask(row, offset_mm):
    mask = np.zeros(row["photo"].shape[:2], np.uint8)
    for points, faces in row["parts"]:
        optical = points - row["camera_Z_direction"] * (offset_mm / 1000)
        homogeneous = optical @ row["k"].T
        pixels = homogeneous[:, :2] / homogeneous[:, 2, None]
        valid = np.all(optical[faces, 2] > 0.01, axis=1) & np.all(
            np.abs(pixels[faces]) < 10000, axis=(1, 2)
        )
        mask |= rasterize_triangles(np.rint(pixels[faces[valid]]).astype(np.int32), mask.shape)
    return mask


def segment_red(photo, roi, policy):
    hue_band, saturation, brightness = SEGMENTATION_POLICIES[policy]
    hsv = cv2.cvtColor(photo, cv2.COLOR_BGR2HSV)
    foreground = (
        ((hsv[:, :, 0] <= hue_band) | (hsv[:, :, 0] >= 180 - hue_band))
        & (hsv[:, :, 1] >= saturation)
        & (hsv[:, :, 2] >= brightness)
        & roi
    )
    count, labels, stats, _ = cv2.connectedComponentsWithStats(foreground.astype(np.uint8))
    keep = np.zeros(count, bool)
    if count > 1:
        largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        lx, ly, lw, lh, _ = stats[largest]
        for index in range(1, count):
            x, y, width, height, area = stats[index]
            horizontal_gap = max(lx - (x + width), x - (lx + lw), 0)
            vertical_gap = max(ly - (y + height), y - (ly + lh), 0)
            # Cluster fragmented finger surfaces around the dominant red
            # object. A separate red/skin object at the ROI edge is not a
            # finger merely because it passes the same hue threshold.
            keep[index] = area >= 20 and np.hypot(horizontal_gap, vertical_gap) <= 25
    result = keep[labels]
    if np.count_nonzero(result) < 200:
        raise ValueError("Insufficient visible red finger pixels")
    return result


def measure(predicted, observed):
    predicted = predicted > 0
    intersection = np.count_nonzero(predicted & observed)
    union = np.count_nonzero(predicted | observed)
    iou = intersection / union if union else 0.0
    edge_predicted = (
        cv2.morphologyEx(predicted.astype(np.uint8), cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8))
        > 0
    )
    edge_observed = (
        cv2.morphologyEx(observed.astype(np.uint8), cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8))
        > 0
    )
    to_observed = cv2.distanceTransform(
        (~edge_observed).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE
    )
    to_predicted = cv2.distanceTransform(
        (~edge_predicted).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE
    )
    distances = np.r_[to_observed[edge_predicted], to_predicted[edge_observed]]
    return dict(
        silhouette_IoU=float(iou),
        symmetric_boundary_mean_px=float(np.mean(distances)),
        symmetric_boundary_p95_px=float(np.percentile(distances, 95)),
        symmetric_boundary_max_px=float(np.max(distances)),
    )


def load_rows(capture, split, calibration, grid, output):
    report = json.loads((capture / "pose_report.json").read_text())
    state = json.loads((capture / "paired_joint_state.json").read_text())
    resources = capture_model(ROOT, report)
    model = RobotModel(resources["urdf"])
    for joint in model.joints.values():
        joint["bounds"] = None
    sdk = SDKKinematics(resources["sdk_chains"])
    values = sdk.from_sdk_vector(state["arms"]["position"])
    values["lift_joint"] = report["height_um"] * 1e-6 + 0.2
    camera = registered_camera(model, sdk, calibration, report["height_um"], values["lift_joint"])
    prepared = prepare_input(load_observation(capture / "observation.npz"))
    photo = cv2.cvtColor(prepared.rgb, cv2.COLOR_RGB2BGR)
    camera_inverse = np.linalg.inv(camera)
    xml = ET.parse(resources["urdf"]).getroot()
    rows = []
    for side in ("left", "right"):
        displacement = report["arms"][side]["official_approximation"][
            "estimated_finger_displacement_m"
        ]
        values[f"openarm_{side}_finger_joint1"] = displacement
        parts = []
        mesh_sources = []
        for number in (1, 2):
            link = f"openarm_{side}_ee_link{number}"
            transform = camera_inverse @ model.transform(link, values)
            for visual in xml.findall(f"link[@name='{link}']/visual"):
                mesh_node = visual.find("geometry/mesh")
                mesh_path = resources["urdf"].parent / mesh_node.get("filename")
                mesh_sources.append(
                    dict(
                        path=str(mesh_path),
                        sha256=hashlib.sha256(mesh_path.read_bytes()).hexdigest(),
                    )
                )
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
                parts.append((optical, mesh.faces))
        row = dict(
            name=capture.parent.name + "_" + side,
            capture=str(capture),
            split=split,
            side=side,
            height_um=report["height_um"],
            capture_time_utc=report["created_utc"],
            pairing=report["pairing"],
            measured_q16=state["arms"]["position"],
            urdf_sha256=hashlib.sha256(resources["urdf"].read_bytes()).hexdigest(),
            sdk_chains_sha256=hashlib.sha256(resources["sdk_chains"].read_bytes()).hexdigest(),
            finger_mesh_sources=mesh_sources,
            estimated_finger_displacement_m=displacement,
            photo=photo,
            parts=parts,
            k=prepared.k,
            camera_Z_direction=camera_inverse[:3, :3] @ np.array([0.0, 0.0, 1.0]),
        )
        # Freeze a broad ROI from the entire declared search range before
        # examining any candidate score; it is not shifted to follow a fit.
        projected = np.zeros(photo.shape[:2], np.uint8)
        for offset in (grid[0], 0.0, grid[-1]):
            projected |= project_mask(row, offset)
        yy, xx = np.where(projected > 0)
        if not len(xx):
            raise ValueError("Finger mesh outside image")
        x0, x1 = max(0, int(xx.min()) - 12), min(photo.shape[1], int(xx.max()) + 13)
        y0, y1 = max(0, int(yy.min()) - 12), min(photo.shape[0], int(yy.max()) + 13)
        roi = np.zeros(photo.shape[:2], bool)
        roi[y0:y1, x0:x1] = True
        row["roi_xyxy"] = [x0, y0, x1, y1]
        row["observed"] = {
            policy: segment_red(photo, roi, policy) for policy in SEGMENTATION_POLICIES
        }
        mask_image = photo.copy()
        mask_image[~roi] = mask_image[~roi] // 3
        contours, _ = cv2.findContours(
            row["observed"]["nominal"].astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(mask_image, contours, -1, (0, 255, 0), 1)
        cv2.rectangle(mask_image, (x0, y0), (x1 - 1, y1 - 1), (255, 200, 0), 1)
        cv2.imwrite(str(output / (row["name"] + "_segmentation.png")), mask_image)
        rows.append(row)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train", type=Path, action="append", required=True)
    parser.add_argument("--test", type=Path, action="append", required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--min-offset-mm", type=float, default=-100.0)
    parser.add_argument("--max-offset-mm", type=float, default=150.0)
    parser.add_argument("--step-mm", type=float, default=1.0)
    args = parser.parse_args()
    if {p.resolve() for p in args.train} & {p.resolve() for p in args.test}:
        raise ValueError("Training and test captures overlap")
    if not (args.min_offset_mm < args.max_offset_mm and args.step_mm > 0):
        raise ValueError("Invalid fixed search interval")
    args.output.mkdir(parents=True, exist_ok=False)
    calibration = HeadExtrinsics(args.calibration)
    source_hash = hashlib.sha256(calibration.path.read_bytes()).hexdigest()
    grid = np.arange(args.min_offset_mm, args.max_offset_mm + args.step_mm * 0.1, args.step_mm)
    rows = []
    for split, captures in (("train", args.train), ("test", args.test)):
        for capture in captures:
            rows.extend(load_rows(capture.resolve(), split, calibration, grid, args.output))
    scores = np.empty((len(rows), len(grid), len(SEGMENTATION_POLICIES)))
    for index, row in enumerate(rows):
        for step, offset in enumerate(grid):
            prediction = project_mask(row, offset) > 0
            for policy_index, policy in enumerate(SEGMENTATION_POLICIES):
                observed = row["observed"][policy]
                scores[index, step, policy_index] = np.count_nonzero(
                    prediction & observed
                ) / np.count_nonzero(prediction | observed)
        print("SCORED", row["name"], flush=True)
    train_indices = [i for i, row in enumerate(rows) if row["split"] == "train"]
    policy_fits = {}
    for index, policy in enumerate(SEGMENTATION_POLICIES):
        objective = np.mean(scores[train_indices, :, index], axis=0)
        best = int(np.argmax(objective))
        policy_fits[policy] = dict(
            offset_mm=float(grid[best]),
            mean_training_IoU=float(objective[best]),
            at_search_boundary=best in (0, len(grid) - 1),
        )
    selected = policy_fits["nominal"]["offset_mm"]
    result_rows = []
    for index, row in enumerate(rows):
        measured = row["observed"]["nominal"]
        result = {
            key: value
            for key, value in row.items()
            if key not in ("photo", "parts", "k", "camera_Z_direction", "observed")
        }
        result["segmented_pixel_count_by_policy"] = {
            key: int(np.count_nonzero(value)) for key, value in row["observed"].items()
        }
        result["metrics"] = {}
        for name, offset in (("baseline", 0.0), ("fixed_training_fit", selected)):
            predicted = project_mask(row, offset)
            result["metrics"][name] = measure(predicted, measured)
            image = row["photo"].copy()
            for mask, color in (
                (measured.astype(np.uint8), (0, 255, 0)),
                (predicted, (0, 170, 255)),
            ):
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(image, contours, -1, color, 1)
            cv2.imwrite(str(args.output / (row["name"] + "_" + name + ".png")), image)
        best = int(np.argmax(scores[index, :, 0]))
        result["per_image_oracle_for_diagnosis_only"] = dict(
            offset_mm=float(grid[best]),
            maximum_IoU=float(scores[index, best, 0]),
            applied_to_fixed_candidate=False,
        )
        result_rows.append(result)
    candidate = json.loads(calibration.path.read_text())
    candidate["T_origin_camera"][2][3] += selected / 1000
    candidate.update(
        diagnostic_only=True,
        deployable=False,
        selection="height_only_silhouette_training_fit_20261105",
    )
    candidate["height_only_fit"] = dict(
        source=str(calibration.path),
        source_sha256=source_hash,
        fixed_offset_mm=selected,
        training=[str(p.resolve()) for p in args.train],
        heldout=[str(p.resolve()) for p in args.test],
        scope="Red finger silhouette only; no physical TCP or complete arm acceptance",
    )
    save_json(args.output / "candidate.json", candidate)
    result = dict(
        schema="offline_one_camera_height_parameter_v1",
        calibration=str(calibration.path),
        calibration_sha256=source_hash,
        fixed_offset_from_calibration_mm=selected,
        segmentation_policy_fits=policy_fits,
        segmentation_component_policy="area >=20px; bounding-box gap <=25px from dominant red component; fixed before scoring",
        search_grid_mm=grid.tolist(),
        grid_spacing_mm=args.step_mm,
        heldout_used_for_fitting=False,
        height_grid_is_simulation_not_robot_motion=True,
        scores_IoU=scores.tolist(),
        rows=result_rows,
        camera_rotation_XY_joint_angles_and_opening_unchanged=True,
        commands_sent=False,
        formal_calibration_modified=False,
        limitations=[
            "Automatic red color segmentation is inspectable but not independent physical metrology",
            "Projected silhouette ignores external occlusion; inspect supplied segmentation images",
            "Geometry and approximate motor-to-finger mapping are fixed, not optimized",
            "Two captures are limited height/pose coverage; do not imply full workspace acceptance",
            "Different per-image optima are diagnostic only and never applied independently",
        ],
    )
    assert hashlib.sha256(calibration.path.read_bytes()).hexdigest() == source_hash
    save_json(args.output / "fit.json", result)
    sections = []
    for row in result_rows:
        name = html.escape(row["name"])
        before = row["metrics"]["baseline"]
        after = row["metrics"]["fixed_training_fit"]
        sections.append(
            f"<h2>{name} · {row['split']} · SDK {row['height_um'] / 1000:.3f}mm</h2><p>IoU {before['silhouette_IoU']:.3f} → {after['silhouette_IoU']:.3f}；边界P95 {before['symmetric_boundary_p95_px']:.2f} → {after['symmetric_boundary_p95_px']:.2f}px。</p><div class='grid'><img src='{name}_baseline.png'><img src='{name}_fixed_training_fit.png'></div><p><a href='{name}_segmentation.png'>检查原始红色分割与固定ROI</a></p>"
        )
    page = """<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>固定高度偏差泛化检验</title><style>body{max-width:1300px;margin:25px auto;padding:0 18px;font:18px/1.6 sans-serif;color:#16314a}.grid{display:flex;gap:15px}.grid img{width:49%}.note{padding:15px;background:#fff0d0}</style><h1>只拟合一个固定相机高度偏差</h1>"""
    page += f"<p>相对输入外参固定增加 {selected:.1f}mm。只用train图拟合；test图不参与选择。</p><p>每组左图是原高度，右图是固定修正后。绿色：照片中分割的红色夹指轮廓；橙色：计算模型轮廓。</p><p class='note'>IoU=1才表示二值轮廓完全相同。此指标仅检验红色夹指，不能代表腕部、整臂或真实钩尖三维精度。自动分割与遮挡会影响结果；分割原图保留供检查。</p>"
    page += "".join(sections)
    page += "<p><a href='fit.json'>完整拟合、留出验证及阈值敏感性</a> · <a href='candidate.json'>未部署的诊断候选</a></p></html>"
    (args.output / "report.html").write_text(page)
    print(
        json.dumps(dict(output=str(args.output), policy_fits=policy_fits), ensure_ascii=False),
        flush=True,
    )


if __name__ == "__main__":
    main()
