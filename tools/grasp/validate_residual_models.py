#!/usr/bin/env python3
"""Offline height holdouts and independent gripper-silhouette validation.

Fits from checkerboard data are applied to the corresponding camera and arm
transforms only. A fitted board mount never modifies a gripper mesh. Additional
per-arm contour fits are diagnostic hypotheses, not hardware-zero estimates.
"""

import argparse
import copy
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.ndimage import map_coordinates
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure
from _robot_model import capture_model, selected_model

configure(ROOT)
from grasp import compare_residual_models as comparison
from grasp import fit_overlay_height as silhouettes
from grasp.render_standard_gripper_overlay import DepthRaster
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics
from wheel_motion_core.unified_model import registered_camera
from wheel_perception_core.extrinsics import HeadExtrinsics


def matrix(rotation, translation):
    value = np.eye(4)
    value[:3, :3], value[:3, 3] = rotation, translation
    return value


class GripperMountModel(comparison.Model):
    """One constant six-dimensional installation hypothesis in each wrist frame."""

    def __init__(self, context, spec):
        super().__init__(context, spec)
        self.add("gripper_rotation", 3, np.deg2rad(8), "rad in local wrist frame")
        self.add("gripper_translation", 3, 0.06, "m in local wrist frame")

    def export(self):
        value = super().export()
        value["model_kind"] = "constant_local_wrist_to_gripper_installation_correction"
        return value


def transform(points, value):
    return points @ value[:3, :3].T + value[:3, 3]


def project(points, k):
    pixels = points @ k.T
    return pixels[:, :2] / pixels[:, 2:]


def boundary(mask):
    return (
        cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)) > 0
    )


def evenly(points, count=280):
    return points[np.linspace(0, len(points) - 1, min(len(points), count)).round().astype(int)]


def load_capture(path, split, calibration, output):
    record = json.loads((path / "pose_report.json").read_text())
    resources = capture_model(ROOT, record)
    sdk = SDKKinematics(resources["sdk_chains"])
    geometry = RobotModel(resources["urdf"])
    for joint in geometry.joints.values():
        joint["bounds"] = None
    output.mkdir(parents=True, exist_ok=True)
    rows = silhouettes.load_rows(path, split, calibration, [-100, 0, 150], output)
    cv2.imwrite(str(output / "photo.png"), rows[0]["photo"])
    for row in rows:
        row["name"] = output.name + "_" + row["side"]
        values = sdk.from_sdk_vector(row["measured_q16"])
        values["lift_joint"] = row["height_um"] * 1e-6 + 0.2
        camera_world = registered_camera(
            geometry, sdk, calibration, row["height_um"], values["lift_joint"]
        )
        camera_origin = calibration.camera_to_origin(row["height_um"])
        wrist = sdk.transform(row["side"] + "_ee_control_point", values)
        reference = np.linalg.inv(camera_world) @ geometry.transform(
            "openarm_" + row["side"] + "_link7", values
        )
        error = float(np.max(np.abs(reference - np.linalg.inv(camera_origin) @ wrist)))
        if error > 1e-9:
            raise ValueError("URDF/SDK registration differs for " + row["side"])
        row.update(camera_origin=camera_origin, wrist=wrist, registration_error=error)
        height, width = row["photo"].shape[:2]
        raster = DepthRaster(height, width)
        for points, faces in row["parts"]:
            raster.mesh(points, faces, row["k"], [255, 100, 0], 1)
        covered = raster.labels >= 0
        visible_edge = covered & ~cv2.erode(
            covered.astype(np.uint8), np.ones((3, 3), np.uint8)
        ).astype(bool)
        yy, xx = np.where(visible_edge)
        camera_points = np.c_[xx + 0.5, yy + 0.5, np.ones(len(xx))] @ np.linalg.inv(row["k"]).T
        camera_points *= raster.depth[yy, xx, None]
        row["contour_points"] = evenly(camera_points)
        observed = boundary(row["observed"]["nominal"])
        yy, xx = np.where(observed)
        row["observed_points"] = evenly(np.c_[xx, yy])
        row["distance_to_observed"] = cv2.distanceTransform(
            (~observed).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE
        )
    return rows


def prediction(row, model, *, correction_side="left", parameters=None):
    # Empirical board fits are left-arm-only. Camera terms are common to both
    # arms; the right arm never receives a copied left joint correction.
    offset = 8 if correction_side == "left" else 0
    q = np.array(row["measured_q16"])[offset : offset + 7]
    data = dict(
        q=q[None, :],
        h=np.array([row["height_um"] * 1e-6]),
        rays=np.zeros((1, 88, 2)),
        focal=np.ones((1, 2)),
    )
    result = model.predict(data, parameters)
    camera = matrix(result["camera_R"], result["camera_t"][0])
    wrist = (
        matrix(result["arm_R"][0], result["arm_t"][0])
        if row["side"] == correction_side
        else row["wrist"]
    )
    if "gripper_rotation" in model.slices and row["side"] == correction_side:
        parameters = model.value if parameters is None else parameters
        correction = matrix(
            Rotation.from_rotvec(model.at(parameters, "gripper_rotation")).as_matrix(),
            model.at(parameters, "gripper_translation"),
        )
        wrist = wrist @ correction
    delta = np.linalg.inv(camera) @ wrist @ np.linalg.inv(row["wrist"]) @ row["camera_origin"]
    shift = (
        model.at(model.value if parameters is None else parameters, "image_shift")
        if model.spec.pixels
        else np.zeros(2)
    )
    return delta, shift


def full_mask(row, model, correction_side="left"):
    delta, shift = prediction(row, model, correction_side=correction_side)
    mask = np.zeros(row["photo"].shape[:2], np.uint8)
    for vertices, faces in row["parts"]:
        points = transform(vertices, delta)
        pixels = project(points, row["k"]) + shift
        valid = np.all(points[faces, 2] > 0.01, axis=1)
        mask |= silhouettes.rasterize_triangles(
            np.rint(pixels[faces[valid]]).astype(np.int32), mask.shape
        )
    return mask


def silhouette_metrics(predicted, observed):
    if not np.any(predicted):
        return dict(
            silhouette_IoU=0.0,
            symmetric_boundary_mean_px=None,
            symmetric_boundary_p95_px=None,
            symmetric_boundary_max_px=None,
            projection_status="outside_image",
        )
    return dict(silhouettes.measure(predicted, observed), projection_status="visible")


def evaluate_silhouettes(rows, model, output, correction_side="left"):
    output.mkdir(parents=True, exist_ok=True)
    results = []
    for row in rows:
        predicted = full_mask(row, model, correction_side)
        image = row["photo"].copy()
        for mask, color in (
            (row["observed"]["nominal"].astype(np.uint8), (0, 255, 0)),
            (predicted, (0, 150, 255)),
        ):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image, contours, -1, color, 1)
        filename = row["name"] + ".png"
        cv2.imwrite(str(output / filename), image)
        result = {
            key: row[key]
            for key in (
                "name",
                "side",
                "capture",
                "split",
                "height_um",
                "capture_time_utc",
                "measured_q16",
                "estimated_finger_displacement_m",
                "registration_error",
                "roi_xyxy",
                "urdf_sha256",
                "sdk_chains_sha256",
                "finger_mesh_sources",
            )
        }
        result["image"] = str(output / filename)
        result["metrics"] = {
            policy: silhouette_metrics(predicted, observed)
            for policy, observed in row["observed"].items()
        }
        results.append(result)
    return results


def fit_contour(rows, context, spec):
    model = (
        GripperMountModel(context, spec)
        if spec.name.endswith("gripper_mount")
        else comparison.Model(context, spec)
    )
    side = "right" if context.joint_names[0].startswith("openarm_right") else "left"
    count = len(model.names)
    prior_scale = np.array(
        [
            20.0 if "image_shift" in name else np.deg2rad(5) if "rad" in name else 0.05
            for name in model.names
        ]
    )

    def residual(parameters):
        values = []
        for row in rows:
            delta, shift = prediction(row, model, correction_side=side, parameters=parameters)
            pixels = project(transform(row["contour_points"], delta), row["k"]) + shift
            forward = map_coordinates(
                row["distance_to_observed"],
                [pixels[:, 1], pixels[:, 0]],
                order=1,
                mode="constant",
                cval=100,
            )
            backward = cKDTree(pixels).query(row["observed_points"])[0]
            values.extend((forward / np.sqrt(len(forward)), backward / np.sqrt(len(backward))))
        # Explicit weak parameter prior stabilizes correlated joint fits. Its
        # rows are excluded when reporting rank from actual contour data.
        return np.r_[np.concatenate(values), 0.2 * parameters / prior_scale]

    result = least_squares(
        residual,
        np.zeros(count),
        bounds=(model.lower, model.upper),
        diff_step=1e-3,
        max_nfev=100,
        loss="soft_l1",
        f_scale=1.0,
        x_scale="jac",
    )
    model.value = result.x
    singular = np.linalg.svd(result.jac[:-count], compute_uv=False)

    def wrist_motion(parameters):
        motions = []
        for row in rows:
            delta, shift = prediction(row, model, correction_side=side, parameters=parameters)
            motions.extend(
                np.r_[
                    Rotation.from_matrix(delta[:3, :3]).as_rotvec(), 10 * delta[:3, 3], shift / 100
                ]
            )
        return np.array(motions)

    columns = []
    for index in range(count):
        step = np.zeros(count)
        step[index] = 1e-5
        columns.append((wrist_motion(result.x + step) - wrist_motion(result.x - step)) / 2e-5)
    geometric_singular = np.linalg.svd(np.array(columns).T, compute_uv=False)
    rank = int(np.count_nonzero(geometric_singular > max(geometric_singular[0] * 1e-6, 1e-8)))
    return model, dict(
        success=bool(result.success),
        nfev=result.nfev,
        message=result.message,
        data_rank=rank,
        parameter_count=count,
        singular_values=singular.tolist(),
        rank_definition="Relative camera/wrist motion Jacobian before contour noise or parameter prior; image shifts add two display-only dimensions",
        wrist_motion_singular_values=geometric_singular.tolist(),
        wrist_motion_condition_number=float(
            geometric_singular[0] / max(geometric_singular[-1], 1e-15)
        ),
        at_bounds=[
            name
            for name, value, lo, hi in zip(model.names, result.x, model.lower, model.upper)
            if min(value - lo, hi - value) < 1e-5
        ],
        prior="0.2 * parameter / (5deg or 0.05 gain) in residual units",
        caveat="Fixed visible model boundary samples approximate contour motion; full meshes are rasterized independently for reported metrics",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fits", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--captures", type=Path, required=True, help="JSON list of name, path, split"
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    provenance = json.loads((args.fits / "provenance.json").read_text())
    calibration = json.loads(Path(provenance["source_calibration"]).read_text())
    resources = selected_model(ROOT)
    sdk = SDKKinematics(resources["sdk_chains"])
    context = comparison.Context(calibration, sdk)
    pairs = json.loads(Path(provenance["pairs"]).read_text())
    training = [p for p in pairs if p["group"] in provenance["training_groups"]]
    specs = {spec.name: spec for spec in comparison.catalog()}
    height_rows = []
    # Fit the added correction using only two heights; the existing base
    # calibration was not independent of all these heights.
    for name in (
        "baseline_original",
        "height_constant",
        "height_linear",
        "height_quadratic",
        "global_rigid",
        "camera_and_board_mount",
        "left_joint_offsets",
        "left_joint_affine",
        "left_joint_nonlinear_sine",
        "camera_board_and_joint_offsets",
        "pose_residual_RBF",
        "pose_residual_quadratic_ridge",
    ):
        for held in sorted({p["height_group"] for p in training}):
            if name == "height_quadratic":
                height_rows.append(
                    dict(
                        name=name,
                        held_height_group=held,
                        skipped="Only two training heights cannot identify three quadratic coefficients",
                    )
                )
                continue
            train = [p for p in training if p["height_group"] != held]
            test = [p for p in training if p["height_group"] == held]
            model = comparison.Model(context, specs[name])
            fitted = model.fit(context.data(comparison.representatives(train, 3)), 100)
            measured = comparison.evaluate(model, context.data(test))
            height_rows.append(
                dict(
                    name=name,
                    held_height_group=held,
                    held_height_mm=test[0]["height_m"] * 1000,
                    fit=fitted,
                    metrics=measured["metrics"],
                    samples=measured["samples"],
                )
            )
        print("HEIGHT", name, flush=True)
    comparison.save(args.output / "height_holdouts.json", height_rows)
    extrinsics = HeadExtrinsics(Path(provenance["source_calibration"]))
    # The pre-existing 180mm capture is explicitly an out-of-range diagnostic;
    # do not change the formal file or production range for this replay.
    extrinsics.height_range = np.array([-39990, 180000])
    rows = []
    for entry in json.loads(args.captures.read_text()):
        path = (ROOT / entry["path"]).resolve()
        rows.extend(
            load_capture(
                path, entry["split"], extrinsics, args.output / "segmentation" / entry["name"]
            )
        )
        print("LOADED", entry["name"], flush=True)
    validation = []
    for name in (
        "baseline_original",
        "baseline_URDF_height",
        "height_constant",
        "height_linear",
        "height_quadratic",
        "image_translation_constant",
        "global_translation",
        "global_rigid",
        "camera_and_board_mount",
        "left_joint_offsets",
        "left_joint_affine",
        "left_joint_nonlinear_sine",
        "camera_and_joint_offsets",
        "camera_board_and_joint_offsets",
        "pose_residual_linear_ridge",
        "pose_residual_quadratic_ridge",
        "pose_residual_RBF",
        "pose_residual_RBF_smoothed",
    ):
        saved = json.loads((args.fits / (name + ".json")).read_text())
        model = comparison.Model.restore(context, saved["model"])
        validation.append(
            dict(
                name=name,
                source="LEFT checkerboard only",
                rows=evaluate_silhouettes(rows, model, args.output / "board-fit-overlays" / name),
            )
        )
        print("VALIDATED", name, flush=True)
    comparison.save(args.output / "board_fits_gripper_validation.json", validation)
    global_results = []
    for name in (
        "image_translation_constant",
        "height_constant",
        "height_linear",
        "height_quadratic",
        "global_translation",
        "global_rigid",
    ):
        model, fit = fit_contour(
            [row for row in rows if row["split"] == "train"], context, specs[name]
        )
        global_results.append(
            dict(
                name="contour_global_" + name,
                fit=fit,
                model=model.export(),
                rows=evaluate_silhouettes(
                    rows, model, args.output / "global-contour-overlays" / name
                ),
            )
        )
        comparison.save(args.output / "global_contour_fits.json", global_results)
        print("GLOBAL_CONTOUR", name, fit["data_rank"], "/", fit["parameter_count"], flush=True)
    for side in ("left", "right"):
        side_context = copy.copy(context)
        side_context.chain = sdk.chains[side]
        side_context.joint_names = [f"openarm_{side}_joint{i + 1}" for i in range(7)]
        train = [row for row in rows if row["side"] == side and row["split"] == "train"]
        tested = [row for row in rows if row["side"] == side]
        side_results = []
        for index in range(11):
            spec = (
                comparison.Spec(
                    f"{side}_J{index + 1}_contour_offset", joint="individual", individual=index
                )
                if index < 7
                else comparison.Spec(f"{side}_contour_gripper_mount")
                if index == 10
                else comparison.Spec(
                    f"{side}_contour_{('offset', 'affine', 'sine')[index - 7]}",
                    joint=("offset", "affine", "sine")[index - 7],
                )
            )
            model, fit = fit_contour(train, side_context, spec)
            result = dict(
                name=spec.name,
                fit=fit,
                model=model.export(),
                rows=evaluate_silhouettes(
                    tested,
                    model,
                    args.output / "contour-fit-overlays" / spec.name,
                    correction_side=side,
                ),
            )
            side_results.append(result)
            comparison.save(args.output / (side + "_contour_fits.json"), side_results)
            print("CONTOUR", spec.name, fit["data_rank"], "/", fit["parameter_count"], flush=True)
    q = np.array([p["q"] for p in training])
    comparison.save(
        args.output / "coverage.json",
        dict(
            training_joint_ranges_rad=np.c_[q.min(axis=0), q.max(axis=0)].tolist(),
            capture_joint_angles=[
                dict(name=row["name"], split=row["split"], q16=row["measured_q16"]) for row in rows
            ],
            no_hardware_zero_estimate=True,
            active_calibration_unchanged=comparison.sha(Path(provenance["source_calibration"]))
            == provenance["source_calibration_sha256"],
            rectifier_sha256=comparison.sha(
                ROOT / "packages/wheel_perception_core/wheel_perception_core/geometry.py"
            ),
            code_sha256=comparison.sha(Path(__file__)),
        ),
    )


if __name__ == "__main__":
    main()
