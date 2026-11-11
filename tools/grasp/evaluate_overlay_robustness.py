#!/usr/bin/env python3
"""Evaluate frozen and physical camera-error candidates on measured height/pose trials."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure

configure(ROOT)
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics
from wheel_motion_core.unified_model import registered_camera
from wheel_perception_core.extrinsics import HeadExtrinsics
from wheel_perception_core.geometry import prepare_input
from wheel_perception_core.rgbd import load_observation


def project(points, k):
    homogeneous = (k @ np.asarray(points).reshape(-1, 3).T).T
    return homogeneous[:, :2] / homogeneous[:, 2, None]


def detect_shaft(photo, predicted):
    """Circle proposals followed by consensus of fully enclosed dark shaft holes."""
    gray = cv2.cvtColor(photo, cv2.COLOR_BGR2GRAY)
    x, y = np.rint(predicted).astype(int)
    x0, y0 = max(0, x - 43), max(0, y - 43)
    x1, y1 = min(gray.shape[1], x + 44), min(gray.shape[0], y + 44)
    region = gray[y0:y1, x0:x1]
    circles = cv2.HoughCircles(
        cv2.GaussianBlur(region, (5, 5), 1),
        cv2.HOUGH_GRADIENT,
        1,
        12,
        param1=100,
        param2=18,
        minRadius=18,
        maxRadius=36,
    )
    if circles is None:
        raise ValueError("No motor-cap circle proposal")
    candidates = []
    for cx, cy, _ in circles[0] + [x0, y0, 0]:
        cx, cy = int(round(cx)), int(round(cy))
        cut = gray[cy - 14 : cy + 15, cx - 14 : cx + 15]
        if cut.shape != (29, 29):
            continue
        for threshold in [140, 150, 160, 170, 180]:
            count, _, stats, centres = cv2.connectedComponentsWithStats(
                (cut < threshold).astype(np.uint8)
            )
            for index in range(1, count):
                xx, yy, width, height, area = stats[index]
                if (
                    area < 30
                    or area > 180
                    or xx <= 0
                    or yy <= 0
                    or xx + width >= 29
                    or yy + height >= 29
                ):
                    continue
                candidates.append(centres[index] + [cx - 14, cy - 14])
    points = np.asarray(candidates)
    if not len(points):
        raise ValueError("No enclosed shaft-hole feature")
    counts = np.sum(np.linalg.norm(points[:, None] - points[None, :], axis=2) < 3, axis=1)
    cluster = points[np.linalg.norm(points - points[counts.argmax()], axis=1) < 3]
    if len(cluster) < 4 or max(np.ptp(cluster, axis=0)) > 3:
        raise ValueError("Shaft-hole detection not repeatable across thresholds")
    return np.median(cluster, axis=0), {
        "votes": len(cluster),
        "threshold_spread_px": np.ptp(cluster, axis=0).tolist(),
        "feature_uncertainty_px": 2.0,
    }


def prediction(row, parameters, rigid=True):
    points = np.asarray(row["points_camera"])
    if rigid:
        points = points @ Rotation.from_rotvec(parameters[:3]).as_matrix().T + parameters[3:]
    else:
        points = points + parameters
    return project(points, np.asarray(row["k"]))


def fit(rows, rigid):
    count = 6 if rigid else 3
    lower = [-0.15] * 3 + [-0.08] * 3 if rigid else [-0.08] * 3
    upper = [0.15] * 3 + [0.08] * 3 if rigid else [0.08] * 3
    result = least_squares(
        lambda value: np.concatenate(
            [(prediction(row, value, rigid) - row["observed"]).ravel() for row in rows]
        ),
        np.zeros(count),
        bounds=(lower, upper),
        x_scale="jac",
        loss="soft_l1",
        f_scale=1.5,
        max_nfev=1000,
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument(
        "--calibration", type=Path, required=True, help="Match the recorded zero epoch"
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    calibration = HeadExtrinsics(args.calibration)
    source_hash = hashlib.sha256(calibration.path.read_bytes()).hexdigest()
    model = RobotModel(
        ROOT / "assets/robots/r680_lift_openarm_claw1.3/r680_lift_openarm_robot_viewer.urdf"
    )
    sdk = SDKKinematics(ROOT / "config/robots/calibration/sdk_arm_chains.json")
    names = [
        "baseline",
        "height143",
        "height123",
        "height103",
        "height083",
        "height_return",
        "pose_j7",
        "pose_j4",
        "final_return",
        "extended_return",
    ]
    if (args.root / "latest/pose_report.json").is_file():
        names.append("latest")
    rows = []
    for name in names:
        capture = args.root / name
        record = json.loads((capture / "pose_report.json").read_text())
        if record["status"] != "nominal_gripper_fk_computed":
            raise ValueError("Invalid paired capture " + name)
        state = json.loads((capture / "paired_joint_state.json").read_text())
        values = sdk.from_sdk_vector(state["arms"]["position"])
        height = record["height_um"]
        values["lift_joint"] = height * 1e-6 + 0.2
        camera = registered_camera(model, sdk, calibration, height, values["lift_joint"])
        prepared = prepare_input(load_observation(capture / "observation.npz"))
        photo = cv2.cvtColor(prepared.rgb, cv2.COLOR_RGB2BGR)
        points, observed, qualities = [], [], []
        for side in ["left", "right"]:
            point = (
                np.linalg.inv(camera) @ model.transform(f"openarm_{side}_ee_base_link", values)
            )[:3, 3]
            predicted = project(point, prepared.k)[0]
            detected, quality = detect_shaft(photo, predicted)
            points.append(point.tolist())
            observed.append(detected.tolist())
            qualities.append(quality)
        rows.append(
            {
                "name": name,
                "height_um": height,
                "q": state["arms"]["position"],
                "capture_time_utc": record["created_utc"],
                "pairing": record["pairing"],
                "points_camera": points,
                "observed": observed,
                "k": prepared.k.tolist(),
                "detection": qualities,
                "capture": str(capture),
            }
        )
    training_names = ["baseline", "height123", "height083"]
    training = [row for row in rows if row["name"] in training_names]
    translation = fit(training, False)
    rigid = fit(training, True)
    frozen_record = json.loads(
        (ROOT / "outputs/grasp/left-error-localization-20261102/report.json").read_text()
    )
    frozen = np.asarray(
        frozen_record["shared_camera_translation_explanatory_fit"][
            "delta_camera_point_coordinates_m"
        ]
    )
    fixed_pixels = np.asarray(
        [
            frozen_record["measurements"][side]["observed_minus_predicted_px"]
            for side in ["left", "right"]
        ]
    )
    summaries = {
        key: {"training": [], "held_out": []}
        for key in ["original", "frozen_3d", "frozen_pixels", "translation_fit", "rigid_fit"]
    }
    for row in rows:
        photo = cv2.imread(str(Path(row["capture"]) / "rgb.png"))
        raw = project(row["points_camera"], np.asarray(row["k"]))
        predictions = {
            "original": raw,
            "frozen_3d": prediction(row, frozen, False),
            "frozen_pixels": raw + fixed_pixels,
            "translation_fit": prediction(row, translation.x, False),
            "rigid_fit": prediction(row, rigid.x, True),
        }
        row["training"] = row["name"] in training_names
        row["errors_px"] = {
            key: np.linalg.norm(value - row["observed"], axis=1).tolist()
            for key, value in predictions.items()
        }
        for key, errors in row["errors_px"].items():
            summaries[key]["training" if row["training"] else "held_out"].extend(errors)
        for index, side in enumerate(["left", "right"]):
            measured = tuple(np.rint(row["observed"][index]).astype(int))
            original = tuple(np.rint(raw[index]).astype(int))
            revised = tuple(np.rint(predictions["rigid_fit"][index]).astype(int))
            cv2.circle(photo, measured, 4, (50, 230, 50), 1)
            cv2.drawMarker(photo, original, (0, 150, 255), cv2.MARKER_CROSS, 13, 1)
            cv2.drawMarker(photo, revised, (255, 100, 20), cv2.MARKER_TILTED_CROSS, 12, 1)
            cv2.arrowedLine(photo, original, measured, (40, 40, 230), 1, tipLength=0.2)
        cv2.imwrite(str(args.output / (row["name"] + "_landmarks.png")), photo)
    for model_values in summaries.values():
        for split, values in model_values.items():
            model_values[split] = {
                "count": len(values),
                "rms_px": float(np.sqrt(np.mean(np.square(values)))),
                "median_px": float(np.median(values)),
                "max_px": float(max(values)),
            }
    error_transform = np.eye(4)
    error_transform[:3, :3] = Rotation.from_rotvec(rigid.x[:3]).as_matrix()
    error_transform[:3, 3] = rigid.x[3:]
    candidate = dict(calibration.config)
    candidate.update(
        selection="multiheight_shaft_candidate_20261103",
        deployable=False,
        status="diagnostic_candidate_needs_independent_pose_and_grasp_validation",
        T_origin_camera=(calibration.reference @ np.linalg.inv(error_transform)).tolist(),
    )
    candidate["source"] = {
        "path": str(args.output / "report.json"),
        "active_calibration_sha256": source_hash,
        "training_samples": training_names,
        "landmarks": "Two wrist output shaft centres, manually reviewable detections, about +/-2px",
        "scope": "Not a replacement for independent metrology or complete gripper contour verification",
    }
    candidate["prior_calibration_validation_summary"] = candidate.pop("validation_summary", {})
    candidate["validation_summary"] = {
        "wrist_shaft_projection": summaries,
        "feature_uncertainty_px": 2.0,
        "independent_full_gripper_or_tcp_validation": False,
    }
    candidate["measured_height_range_um"] = [
        min(row["height_um"] for row in rows),
        max(row["height_um"] for row in rows),
    ]
    candidate["T_ee_board_status"] = (
        "Inherited from prior calibration; not refitted for this candidate"
    )
    candidate["lift_compensation"] = dict(
        candidate["lift_compensation"],
        shaft_projection_cross_height_checked=True,
        cross_height_validated=False,
        reason="Only wrist shaft projections were checked over 80mm; full metric workspace/TCP accuracy not accepted",
    )
    candidate["limitations"] = [
        "Only two output-shaft centres per image, approximately +/-2px feature uncertainty",
        "Tested height span83.211..163.211mm and small J4/J7 changes; not a full workspace calibration",
        "Inherited board mount is not refitted; earlier board validation metrics do not describe this candidate",
        "No active config change and no deployment approval inferred from this diagnostic fit",
    ]
    (args.output / "candidate_camera.json").write_text(
        json.dumps(candidate, indent=2, ensure_ascii=False) + "\n"
    )
    singular = np.linalg.svd(rigid.jac, compute_uv=False)
    report = {
        "calibration_source": str(calibration.path),
        "active_calibration_sha256": source_hash,
        "active_calibration_modified": False,
        "candidate_applied_to_robot": False,
        "experiment_root": str(args.root),
        "training_samples": training_names,
        "held_out_scope": "Interleaved heights 143/103mm, small J7/J4 changes and baseline returns; J7 does not excite shaft centre position",
        "measured_height_span_um": [
            min(row["height_um"] for row in rows),
            max(row["height_um"] for row in rows),
        ],
        "rows": rows,
        "models": summaries,
        "translation_fit_camera_m": translation.x.tolist(),
        "rigid_fit_rotation_vector_rad": rigid.x[:3].tolist(),
        "rigid_fit_translation_camera_m": rigid.x[3:].tolist(),
        "jacobian_singular_values": singular.tolist(),
        "limitations": [
            "Shaft centres are only two landmarks; cap orientation and complete gripper contours are not independently measured",
            "Only an 80mm height span and small arm-pose changes are physically tested",
            "Do not infer global accuracy, hook TCP accuracy, or change joint zeros from this fit",
        ],
    }
    assert hashlib.sha256(calibration.path.read_bytes()).hexdigest() == source_hash
    (args.output / "report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    )
    print(
        json.dumps(
            {
                "models": summaries,
                "rotation_deg": np.degrees(rigid.x[:3]).tolist(),
                "translation_mm": (rigid.x[3:] * 1000).tolist(),
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
