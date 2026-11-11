#!/usr/bin/env python3
"""Offline constant-vs-linear relative-height fits with pose and height holdouts."""

import argparse
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure

configure(ROOT)
from grasp import recalibrate_recorded_head as common


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parameterize(base, values, linear):
    """b is metres at the original reference; a is dimensionless mm/mm."""
    parameters = base.copy()
    parameters[5] += values[0]
    slope = float(values[1]) if linear else 0.0
    return parameters, np.array([0.0, 0.0, 1.0 - slope])


def fit(base, pairs, reference, linear):
    counts = Counter((pair["height_group"], pair["group"]) for pair in pairs)
    weights = np.repeat(
        [1 / np.sqrt(counts[(pair["height_group"], pair["group"])]) for pair in pairs], 176
    )
    if linear and len({pair["height_group"] for pair in pairs}) < 2:
        raise ValueError("At least two measured height groups identify slope and intercept")

    def residual(values):
        parameters, axis = parameterize(base, values, linear)
        return common.reprojection_residual(parameters, pairs, reference, axis) * weights

    result = least_squares(
        residual,
        [0.0, 0.0] if linear else [0.0],
        bounds=([-0.15, -0.75], [0.15, 0.75]) if linear else (-0.15, 0.15),
        x_scale=[0.01, 0.1] if linear else [0.01],
        ftol=1e-11,
        xtol=1e-11,
        gtol=1e-11,
    )
    if not result.success or not np.isfinite(result.x).all():
        raise ValueError("Height fit failed: " + result.message)
    singular = np.linalg.svd(result.jac, compute_uv=False)
    return dict(
        intercept_at_reference_mm=float(result.x[0] * 1000),
        slope_mm_per_mm=float(result.x[1]) if linear else 0.0,
        effective_relative_lift_scale=1.0 - float(result.x[1]) if linear else 1.0,
        optimizer_success=bool(result.success),
        cost=float(result.cost),
        nfev=int(result.nfev),
        jacobian_singular_values=singular.tolist(),
        fitted_parameters=result.x.tolist(),
        linear=linear,
    )


def evaluate(base, fitted, pairs, reference):
    parameters, axis = parameterize(base, fitted["fitted_parameters"], fitted["linear"])
    result = common.evaluate_pairs(parameters, pairs, reference, axis)
    result["by_height"] = {
        str(height): common.evaluate_pairs(
            parameters,
            [pair for pair in pairs if pair["height_group"] == height],
            reference,
            axis,
        )["metrics"]
        for height in sorted({pair["height_group"] for pair in pairs})
    }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--calibration",
        type=Path,
        default=ROOT / "config/robots/calibration/head_camera_j4_rezero_20261104.json",
    )
    parser.add_argument("--test-pairs", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    source_hash = digest(args.calibration)
    calibration = json.loads(args.calibration.read_text())
    pairs_path = Path(calibration["source"]["pairs"])
    if digest(pairs_path) != calibration["source"]["pairs_sha256"]:
        raise ValueError("Recorded, zero-rebased pairs changed")
    test_path = args.test_pairs or pairs_path.parent / "test_pairs.json"
    if digest(test_path) != calibration["validation_summary"]["test_pairs_sha256"]:
        raise ValueError("Frozen held-out frames changed")
    pairs = json.loads(pairs_path.read_text())
    test = json.loads(test_path.read_text())
    test_groups = {pair["group"] for pair in test}
    training = [pair for pair in pairs if pair["group"] not in test_groups]
    if len(training) + len(test) != len(pairs):
        raise ValueError("Incomplete frozen pose split")
    if {pair["frame_index"] for pair in training} & {pair["frame_index"] for pair in test}:
        raise ValueError("Training/test frames overlap")
    base = np.r_[
        common.pack(np.array(calibration["T_origin_camera"])),
        common.pack(np.array(calibration["T_ee_board"])),
    ]
    reference = calibration["reference_height_um"] * 1e-6
    baseline = dict(fitted_parameters=[0.0], linear=False)
    variants = {"original": baseline}
    variants["constant"] = fit(base, training, reference, False)
    variants["linear"] = fit(base, training, reference, True)
    metrics = {}
    for name, fitted in variants.items():
        metrics[name] = {
            split: evaluate(base, fitted, data, reference)
            for split, data in (("training", training), ("heldout", test))
        }
        print(
            "FIT",
            name,
            "heldout RMS px",
            metrics[name]["heldout"]["metrics"]["reprojection_rms_px"]["rms"],
            flush=True,
        )
    expected = calibration["validation_summary"]["metrics"]["reprojection_rms_px"]["rms"]
    if not np.isclose(
        metrics["original"]["heldout"]["metrics"]["reprojection_rms_px"]["rms"],
        expected,
        rtol=0,
        atol=1e-10,
    ):
        raise ValueError("Original heldout baseline is not reproducible")
    height_tests = []
    for height in sorted({pair["height_group"] for pair in pairs}):
        train_height = [pair for pair in training if pair["height_group"] != height]
        test_height = [pair for pair in test if pair["height_group"] == height]
        item = dict(
            heldout_height_group=height,
            heldout_height_mm=sorted({pair["height_m"] * 1000 for pair in test_height}),
            training_groups=sorted({pair["group"] for pair in train_height}),
            test_groups=sorted({pair["group"] for pair in test_height}),
            training_height_groups=sorted({pair["height_group"] for pair in train_height}),
            heldout_is_interpolation=bool(
                min(pair["height_m"] for pair in train_height)
                < test_height[0]["height_m"]
                < max(pair["height_m"] for pair in train_height)
            ),
            models={},
        )
        for name, linear in (("constant", False), ("linear", True)):
            fitted = fit(base, train_height, reference, linear)
            item["models"][name] = dict(
                fit=fitted, evaluation=evaluate(base, fitted, test_height, reference)
            )
        height_tests.append(item)
    report = dict(
        schema="relative_camera_height_linear_fit_v1",
        reference_height_mm=reference * 1000,
        source_calibration=str(args.calibration.resolve()),
        source_calibration_sha256=source_hash,
        recorded_pairs=str(pairs_path),
        recorded_pairs_sha256=digest(pairs_path),
        frozen_test_pairs=str(test_path),
        frozen_test_pairs_sha256=digest(test_path),
        coordinate_rebase=calibration["coordinate_rebase"],
        formula="delta_Z_mm = a*(SDK_height_mm-reference_height_mm)+b_mm; relative_height_mm = original_reference_Z_mm-(SDK_height_mm-reference_height_mm)+delta_Z_mm",
        training_frames=len(training),
        training_pose_groups=sorted({pair["group"] for pair in training}),
        heldout_frames=len(test),
        heldout_pose_groups=sorted(test_groups),
        heldout_used_for_fitting=False,
        other_calibration_parameters_fixed=True,
        fits=variants,
        evaluations=metrics,
        leave_one_height_out=height_tests,
        commands_sent=False,
        active_calibration_modified=False,
        limitations=[
            "Board-camera-wrist closure, not independent hook or complete gripper metrology",
            "Only the additional height-model parameters are held out; existing R/XY and board mount are frozen from previous calibration",
            "No real motion or new physical multiheight capture is implied by offline replay",
            "An empirical slope is not proof of actual lift encoder scale error",
            "A fixed physical head camera requires any accepted scale correction to be integrated consistently into whole-robot lift registration",
        ],
    )
    common.save_json(args.output / "evaluation.json", report)
    candidate = dict(
        schema="offline_relative_height_linear_candidate_v1",
        diagnostic_only=True,
        deployable=False,
        reference_height_mm=reference * 1000,
        delta_Z_intercept_at_reference_mm=variants["linear"]["intercept_at_reference_mm"],
        delta_Z_slope_mm_per_mm=variants["linear"]["slope_mm_per_mm"],
        original_reference_Z_mm=calibration["T_origin_camera"][2][3] * 1000,
        source_calibration=str(args.calibration.resolve()),
        source_calibration_sha256=source_hash,
        evaluation=str((args.output / "evaluation.json").resolve()),
        recorded_height_range_um=calibration["height_range_um"],
    )
    common.save_json(args.output / "linear_model.json", candidate)
    if digest(args.calibration) != source_hash:
        raise ValueError("Source calibration was modified")
    print(json.dumps(variants["linear"], ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
