#!/usr/bin/env python3
"""Refit the selected frozen-split camera calibration after verified J4 zero changes."""

import argparse
import copy
import datetime
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure
from _robot_model import selected_model

configure(ROOT)
from grasp import recalibrate_recorded_head as common
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def zero_changes(record):
    if record.get("selected_zero_writes") != 2 or record.get("other_joint_zero_writes") != 0:
        raise ValueError("Expected exactly two isolated J4 hardware-zero writes")
    changes = {}
    for item in record["joints"]:
        side = item["side"]
        index = {"right": 3, "left": 11}.get(side)
        if (
            index is None
            or item["sdk_index"] != index
            or item["motor_id"] != 4
            or item["bus"] != "luxi_" + side
            or item["joint"] != "openarm_" + side + "_joint4"
            or index in changes
        ):
            raise ValueError("Unexpected hardware-zero joint mapping")
        delta = item["observed_feedback_coordinate_shift_rad"]
        observed = item["new_sdk_rad_after_stamp"] - item["old_sdk_rad_at_hardware_stamp"]
        if not np.isfinite(delta) or not np.isclose(delta, observed, rtol=0, atol=1e-12):
            raise ValueError("Zero shift differs from before/after measured feedback")
        changes[index] = delta
    if set(changes) != {3, 11}:
        raise ValueError("Both J4 zero events are required")
    return changes


def rebase_pairs(pairs, record, original_sdk, current_sdk, model):
    """Recompute cached FK; preserve images, PnP, grouping and all unselected angles."""
    changes = zero_changes(record)
    converted = []
    errors = dict(recorded_cached_fk=0.0, unchanged_left_frame=0.0, current_urdf_sdk_fk=0.0)
    geometry = copy.deepcopy(model)
    bound_exceedances = Counter()
    for joint in geometry.joints.values():
        joint["bounds"] = None
    for original in pairs:
        if "q_before_j4_rezero" in original:
            raise ValueError("Source already uses the new J4 epoch; refusing a double rebase")
        raw = np.asarray(original["q"], dtype=float)
        if raw.shape != (16,) or not np.isfinite(raw).all():
            raise ValueError("Expected finite recorded measured q[16]")
        old_values = original_sdk.from_sdk_vector(raw)
        cached = np.asarray(original["A"])
        old_fk = original_sdk.transform("left_ee_control_point", old_values)
        same_angles = current_sdk.transform("left_ee_control_point", old_values)
        errors["recorded_cached_fk"] = max(
            errors["recorded_cached_fk"], float(np.max(np.abs(cached - old_fk)))
        )
        errors["unchanged_left_frame"] = max(
            errors["unchanged_left_frame"], float(np.max(np.abs(old_fk - same_angles)))
        )
        shifted = raw.copy()
        for index, delta in changes.items():
            shifted[index] += delta
        values = current_sdk.from_sdk_vector(shifted)
        for name, angle in values.items():
            bounds = model.joints[name]["bounds"]
            if bounds and not bounds[0] <= angle <= bounds[1]:
                bound_exceedances[name] += 1
        current_fk = current_sdk.transform("left_ee_control_point", values)
        base = "openarm_left_base_link"
        urdf_fk = current_sdk.transform(base) @ geometry.transform(
            "openarm_left_link7", values, base=base
        )
        errors["current_urdf_sdk_fk"] = max(
            errors["current_urdf_sdk_fk"], float(np.max(np.abs(current_fk - urdf_fk)))
        )
        pair = copy.deepcopy(original)
        pair.update(q_before_j4_rezero=raw.tolist(), q=shifted.tolist(), A=current_fk.tolist())
        converted.append(pair)
    if max(errors.values()) > 1e-10:
        raise ValueError("Recorded/current coordinate chains disagree: " + str(errors))
    errors["recorded_angles_outside_urdf_limits"] = dict(bound_exceedances)
    errors["measured_angles_clipped"] = False
    return converted, errors


def refit(training, height, initial):
    """Keep the original equal-pose-height weights and robust pixel objective."""
    counts = Counter((pair["height_group"], pair["group"]) for pair in training)
    weights = np.repeat([1 / counts[(p["height_group"], p["group"])] for p in training], 176)

    def loss(squared):
        root = np.sqrt(1 + squared)
        return np.vstack([2 * (root - 1) * weights, weights / root, -0.5 * weights / root**3])

    result = least_squares(
        lambda parameters: common.reprojection_residual(
            parameters, training, height, np.array([0, 0, 1])
        ),
        initial,
        loss=loss,
        f_scale=0.5,
        x_scale="jac",
        max_nfev=200,
        ftol=1e-10,
        xtol=1e-10,
        gtol=1e-10,
    )
    if not result.success or not np.isfinite(result.x).all():
        raise RuntimeError("Calibration refit failed: " + result.message)
    return result.x, dict(
        success=True,
        nfev=result.nfev,
        cost=float(result.cost),
        weighting="equal pose-height cells",
        jacobian_singular_values=np.linalg.svd(result.jac, compute_uv=False).tolist(),
    )


def evaluate(parameters, pairs, height):
    result = common.evaluate_pairs(parameters, pairs, height, [0, 0, 1])
    result["by_height"] = {
        str(group): common.evaluate_pairs(
            parameters, [p for p in pairs if p["height_group"] == group], height, [0, 0, 1]
        )["metrics"]
        for group in sorted({p["height_group"] for p in pairs})
    }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--zero-record", type=Path, required=True)
    parser.add_argument("--split", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    original = json.loads(args.calibration.read_text())
    record = json.loads(args.zero_record.read_text())
    resources = selected_model(ROOT)
    if sha256(resources["urdf"]) != record["urdf_sha256"]:
        raise ValueError("Selected URDF differs from the hardware-zero reference")
    source = Path(original["source"]["pairs"])
    if sha256(source) != original["source"]["pairs_sha256"]:
        raise ValueError("Original calibration pairs changed")
    if sha256(args.split) != original["validation_summary"]["split_manifest_sha256"]:
        raise ValueError("Frozen training/test split changed")
    split = json.loads(args.split.read_text())
    pairs = json.loads(source.read_text())
    old_sdk = SDKKinematics(ROOT / "config/robots/calibration/sdk_arm_chains.json")
    sdk = SDKKinematics(resources["sdk_chains"])
    model = RobotModel(resources["urdf"])
    converted, fk_audit = rebase_pairs(pairs, record, old_sdk, sdk, model)
    train_ids = original["source"]["training_frame_indices"]
    test_ids = split["test_frame_indices"]
    by_id = {pair["frame_index"]: pair for pair in converted}
    old_by_id = {pair["frame_index"]: pair for pair in pairs}
    training = [by_id[index] for index in train_ids]
    test = [by_id[index] for index in test_ids]
    if set(train_ids) & set(test_ids) or {p["group"] for p in training} & {
        p["group"] for p in test
    }:
        raise ValueError("Frozen split leaks test poses into training")
    if {p["height_group"] for p in training} != {1}:
        raise ValueError("Expected the selected fixed-height-1 training policy")
    common.save_json(output / "rebased_pairs.json", converted)
    common.save_json(output / "test_pairs.json", test)
    height = original["reference_height_um"] * 1e-6
    initial = np.r_[
        common.pack(np.array(original["T_origin_camera"])),
        common.pack(np.array(original["T_ee_board"])),
    ]
    baseline = evaluate(initial, [old_by_id[index] for index in test_ids], height)
    expected_rms = original["validation_summary"]["metrics"]["translation_mm"]["rms"]
    if not np.isclose(
        baseline["metrics"]["translation_mm"]["rms"], expected_rms, atol=1e-8, rtol=0
    ):
        raise ValueError("Original test evaluation is not reproducible")
    stale = evaluate(initial, test, height)
    print("REFITTING", len(training), "train", len(test), "heldout", flush=True)
    parameters, fit = refit(training, height, initial)
    validation = evaluate(parameters, test, height)
    provenance = dict(
        schema="recorded_two_J4_zero_rebase_v1",
        source_pairs=str(source),
        source_pairs_sha256=sha256(source),
        zero_record=str(args.zero_record.resolve()),
        zero_record_sha256=sha256(args.zero_record),
        changes_rad={str(index): delta for index, delta in zero_changes(record).items()},
        formula="q_new[3,11] = q_recorded[3,11] + measured(q_after_stamp - q_before_stamp); recompute left wrist FK",
        raw_recording_modified=False,
        image_corners_and_PnP_modified=False,
        current_feedback_offset_applied=False,
        fk_audit=fk_audit,
        split_manifest=str(args.split.resolve()),
        split_manifest_sha256=sha256(args.split),
        training_frame_ids_unchanged=True,
        heldout_frame_ids_unchanged=True,
        training_pose_groups=sorted({p["group"] for p in training}),
        heldout_pose_groups=sorted({p["group"] for p in test}),
        solver_objective="Same equal pose-height weighted robust pixel objective; warm start from previous accepted candidate",
    )
    candidate = copy.deepcopy(original)
    candidate.update(
        selection="fixed_height_1_after_J4_rezero_20261104",
        status="candidate_rebased_J4_common_test_evaluated",
        deployable=False,
        T_origin_camera=common.unpack(parameters[:6]).tolist(),
        T_ee_board=common.unpack(parameters[6:12]).tolist(),
        fit=fit,
        coordinate_rebase=provenance,
        limitations=[
            "Reused pre-J4-zero recording with measured coordinate changes; not a new recording",
            "Mechanical-stop zero reference and fixed board mount remain assumptions",
            "Same-session pose-group heldout closure; not independent physical TCP accuracy",
            "Nominal lift axis, scale and validated height range unchanged",
            "New live overlay is a separate diagnostic, not physical acceptance",
        ],
    )
    candidate["validation_summary"].update(
        metrics=validation["metrics"],
        metrics_by_height=validation["by_height"],
        test_pairs_sha256=sha256(output / "test_pairs.json"),
    )
    candidate["source"].update(
        previous_calibration=str(args.calibration.resolve()),
        previous_calibration_sha256=sha256(args.calibration),
        recorded_sdk_model_sha256=original["source"]["sdk_model_sha256"],
        sdk_model_sha256=sdk.config["source"]["sha256"],
        pairs=str(output / "rebased_pairs.json"),
        pairs_sha256=sha256(output / "rebased_pairs.json"),
        joint_coordinate_policy=provenance["formula"],
    )
    filename = output / "head_camera_j4_rezero_20261104.json"
    common.save_json(filename, candidate)
    old_matrix = np.array(original["T_origin_camera"])
    new_matrix = common.unpack(parameters[:6])
    report = dict(
        created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        candidate=str(filename),
        coordinate_rebase=provenance,
        baseline_old_epoch=baseline,
        old_calibration_with_new_epoch=stale,
        updated_new_epoch=validation,
        camera_translation_change_mm=((new_matrix[:3, 3] - old_matrix[:3, 3]) * 1000).tolist(),
        camera_rotation_change_deg=float(
            np.rad2deg(Rotation.from_matrix(new_matrix[:3, :3] @ old_matrix[:3, :3].T).magnitude())
        ),
        physical_accuracy_accepted=False,
        robot_commands_sent=False,
    )
    common.save_json(output / "refit_report.json", report)
    print(
        json.dumps(
            {
                "candidate": str(filename),
                "new_metrics": validation["metrics"],
                "camera_translation_change_mm": report["camera_translation_change_mm"],
                "camera_rotation_change_deg": report["camera_rotation_change_deg"],
            }
        ),
        flush=True,
    )


if __name__ == "__main__":
    main()
