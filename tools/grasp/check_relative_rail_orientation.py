#!/usr/bin/env python3
"""Compare inter-rail angles: invariant to a shared rigid camera extrinsic."""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
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


def angle(first, second):
    return float(np.rad2deg(np.arccos(np.clip(abs(np.dot(first, second)), 0, 1))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    observation = prepare_input(load_observation(args.capture / "observation.npz"))
    state = json.loads((args.capture / "paired_joint_state.json").read_text())
    values = SDKKinematics.from_sdk_vector(state["arms"]["position"])
    # Common prismatic height cannot affect either orientation or their angle.
    values["lift_joint"] = 0.2
    capture_report = json.loads((args.capture / "pose_report.json").read_text())
    model = RobotModel(capture_model(ROOT, capture_report)["urdf"])

    def model_directions(joints):
        result = {}
        for side in ["left", "right"]:
            body = model.transform(f"openarm_{side}_ee_base_link", joints)
            finger = model.joints[f"openarm_{side}_finger_joint1"]
            result[side] = body[:3, :3] @ finger["t"][:3, :3] @ finger["axis"]
        return result

    predicted = model_directions(values)
    predicted_angle = angle(predicted["left"], predicted["right"])
    hsv = cv2.cvtColor(observation.rgb, cv2.COLOR_RGB2HSV)
    rois = {"left": [82, 208, 175, 239], "right": [489, 205, 590, 240]}
    trials = []
    image = cv2.cvtColor(observation.rgb, cv2.COLOR_RGB2BGR)
    defaults = {}
    for saturation in [50, 65, 80]:
        for brightness in [140, 160, 180]:
            for depth_gate in [0.015, 0.025, 0.04]:
                directions = {}
                samples = {}
                for side, (x0, y0, x1, y1) in rois.items():
                    mask = np.zeros(observation.depth.shape, np.uint8)
                    region = hsv[y0:y1, x0:x1]
                    mask[y0:y1, x0:x1] = (
                        (region[:, :, 1] < saturation) & (region[:, :, 2] > brightness)
                    ).astype(np.uint8)
                    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
                    if count < 2:
                        raise ValueError("Missing visible rail feature")
                    selected = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
                    yy, xx = np.where(
                        (labels == selected)
                        & np.isfinite(observation.depth)
                        & (observation.depth > 0)
                    )
                    depth = observation.depth[yy, xx]
                    keep = np.abs(depth - np.median(depth)) < depth_gate
                    xx, yy, depth = xx[keep], yy[keep], depth[keep]
                    if len(depth) < 100:
                        raise ValueError("Insufficient feature depth")
                    points = (
                        np.linalg.inv(observation.k) @ np.array([xx, yy, np.ones(len(xx))])
                    ).T * depth[:, None]
                    centre = points.mean(axis=0)
                    _, singular, vectors = np.linalg.svd(points - centre, full_matrices=False)
                    directions[side] = vectors[0]
                    samples[side] = dict(
                        point_count=len(points),
                        direction_camera=vectors[0].tolist(),
                        principal_axis_ratio=float(singular[0] / singular[1]),
                    )
                    if saturation == 65 and brightness == 160 and depth_gate == 0.025:
                        cv2.rectangle(image, (x0, y0), (x1, y1), (0, 255, 0), 1)
                        endpoints = np.array(
                            [centre - 0.065 * vectors[0], centre + 0.065 * vectors[0]]
                        )
                        homogeneous = (observation.k @ endpoints.T).T
                        pixels = np.rint(homogeneous[:, :2] / homogeneous[:, 2, None]).astype(int)
                        cv2.line(image, tuple(pixels[0]), tuple(pixels[1]), (0, 0, 255), 2)
                        defaults[side] = samples[side]
                trials.append(
                    dict(
                        max_saturation=saturation,
                        min_brightness=brightness,
                        depth_gate_m=depth_gate,
                        angle_deg=angle(directions["left"], directions["right"]),
                        samples=samples,
                    )
                )
    observed = [trial["angle_deg"] for trial in trials]
    # Prove that arbitrary rigid camera rotations cannot repair inter-rail angle.
    invariant_errors = []
    for _ in range(10):
        rotation = Rotation.random(random_state=np.random.default_rng(_)).as_matrix()
        invariant_errors.append(
            abs(
                predicted_angle - angle(rotation @ predicted["left"], rotation @ predicted["right"])
            )
        )
    report = dict(
        commands_sent=False,
        uses_camera_extrinsics=False,
        source_capture=str(args.capture),
        feature_definition="Visible elongated white guide/rail features; manually selected ROIs, 3D principal directions from RGB-D, not surveyed fiducials",
        model_definition="URDF prismatic finger axes in whole-robot frame",
        rois_xyxy=rois,
        model_inter_rail_angle_deg=predicted_angle,
        observed_angle_range_deg=[min(observed), max(observed)],
        observed_angle_median_deg=float(np.median(observed)),
        trial_count=len(trials),
        default_samples=defaults,
        extrinsic_angle_invariance_max_error_deg=max(invariant_errors),
        trials=trials,
        conclusion="Large relative-orientation discrepancy persists without any camera extrinsic. Prioritize mapping of measured arm angles to the installed gripper/rail frame; a shared rigid camera extrinsic cannot alter this angle.",
        limitations=[
            "Feature identity and parallelism to physical slide axes require confirmation.",
            "RGB-D depth metric semantics remain unaccepted; range measures threshold robustness, not absolute metrology uncertainty.",
            "This does not distinguish J7 angular reference from same-axis fixed mounting/CAD-frame discrepancy, nor fully exclude upstream orientation errors.",
        ],
    )
    (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    cv2.putText(
        image,
        f"RGB-D rails {min(observed):.1f}..{max(observed):.1f} deg; model {predicted_angle:.1f} deg",
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 220, 255),
        1,
    )
    cv2.imwrite(str(args.output / "selected_rails.png"), image)
    print(
        {
            key: report[key]
            for key in [
                "uses_camera_extrinsics",
                "model_inter_rail_angle_deg",
                "observed_angle_range_deg",
                "observed_angle_median_deg",
                "trial_count",
            ]
        }
    )


if __name__ == "__main__":
    main()
