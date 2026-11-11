#!/usr/bin/env python3
"""Replay the recorded multi-height session through the installed conversion code."""

import argparse
import hashlib
import json
import sys
from copy import deepcopy
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure

configure(ROOT)
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics
from wheel_perception_core.extrinsics import HeadExtrinsics


def metrics(errors, angles):
    return dict(
        count=len(errors),
        rms_mm=float(np.sqrt(np.mean(np.square(errors)))),
        p95_mm=float(np.percentile(errors, 95)),
        max_mm=float(max(errors)),
        rotation_rms_deg=float(np.sqrt(np.mean(np.square(angles)))),
    )


def urdf_replay(model, sdk, frames):
    """Recompute recorded left wrist poses using current URDF arm geometry."""
    base = "openarm_left_base_link"
    origin_T_base = sdk.transform(base)
    # Recorded measurements can exceed current command limits. Evaluate the
    # exact geometry without clipping in an offline-only copy, and report it.
    geometry = deepcopy(model)
    outside = {}
    for spec in geometry.joints.values():
        spec["bounds"] = None
    errors = []
    poses = {}
    for index, frame in enumerate(frames):
        if frame.get("A") is None:
            continue
        values = sdk.from_sdk_vector(frame["q"])
        for name, value in values.items():
            if not name.startswith("openarm_left_"):
                continue
            bounds = model.joints[name]["bounds"]
            if bounds and not bounds[0] - 1e-7 <= value <= bounds[1] + 1e-7:
                outside.setdefault(name, dict(bounds_rad=list(bounds), samples=[]))[
                    "samples"
                ].append(dict(frame_index=index, measured_rad=value))
        current = origin_T_base @ geometry.transform("openarm_left_link7", values, base=base)
        errors.append(float(np.max(np.abs(current - np.asarray(frame["A"])))))
        poses[index] = current
    if not errors or max(errors) > 1e-10:
        raise ValueError("Current URDF does not reproduce recorded SDK wrist FK")
    return poses, dict(
        count=len(errors),
        max_matrix_abs_error=max(errors),
        outside_current_command_limits=outside,
        offline_geometry_only=True,
        convention="SDK arm_origin_T_left_base @ current URDF left_base_T_link7; unchanged measured q",
    )


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--session", type=Path, required=True, help="multiheight_restart_session directory"
    )
    p.add_argument(
        "--calibration",
        type=Path,
        required=True,
        help="Explicit historical calibration matching the supplied archived session; no active calibration is selected implicitly",
    )
    p.add_argument(
        "--sdk-model",
        type=Path,
        help="Explicit SDK XML file to hash-check against the frozen chain; no local SDK checkout is loaded automatically",
    )
    p.add_argument(
        "--original-session",
        type=Path,
        help="Original continuous_session for source hashes and original fit closure replay",
    )
    p.add_argument(
        "--output", type=Path, default=ROOT / "outputs/coordinate_chain/replay_validation.json"
    )
    a = p.parse_args(argv)
    calibration = HeadExtrinsics(a.calibration)
    sdk = SDKKinematics(ROOT / "config/robots/calibration/sdk_arm_chains.json")
    reference = json.loads((a.session / "frozen_reference.json").read_text())
    if not np.array_equal(calibration.reference, np.asarray(reference["X_fixed"])):
        raise ValueError("Selected calibration changed from frozen first result")
    if calibration.config["reference_height_um"] != reference["reference_height_um"]:
        raise ValueError("Reference height changed")
    if (
        a.sdk_model is not None
        and hashlib.sha256(a.sdk_model.read_bytes()).hexdigest() != sdk.config["source"]["sha256"]
    ):
        raise ValueError("SDK model changed since chain snapshot")
    frames = json.loads((a.session / "offline/frames.json").read_text())
    fk_errors = []
    for f in frames:
        if f.get("A") is None:
            continue
        t = sdk.transform("left_ee_control_point", sdk.from_sdk_vector(f["q"]))
        fk_errors.append(float(np.max(np.abs(t - np.asarray(f["A"])))))
    if not fk_errors or max(fk_errors) > 1e-10:
        raise ValueError("SDK chain does not reproduce recorded native FK")
    recorded = json.loads((a.session / "offline/multiheight_validation.json").read_text())
    y = np.asarray(reference["T_ee_board_new"])
    rows = []
    for sample in recorded["selected_poses"]:
        fk = np.asarray(sample["A"])
        b = np.asarray(sample["B"])
        computed = calibration.object_pose(b, sample["height_m"] * 1e6)
        error = np.linalg.inv(fk @ y) @ computed
        rows.append(
            dict(
                height_m=sample["height_m"],
                frame_index=sample["frame_index"],
                translation_mm=float(np.linalg.norm(error[:3, 3]) * 1000),
                rotation_deg=float(np.rad2deg(Rotation.from_matrix(error[:3, :3]).magnitude())),
            )
        )
    measured = metrics([r["translation_mm"] for r in rows], [r["rotation_deg"] for r in rows])
    baseline = recorded["models"]["nominal_positive_sdk_z"]["all_selected"]
    for key in ["rms_mm", "p95_mm", "max_mm", "rotation_rms_deg"]:
        if not np.isclose(measured[key], baseline[key], atol=1e-8):
            raise ValueError("Replay differs: " + key)
    # Compare the current model shoulder origins without assuming frame equivalence.
    model_path = (
        ROOT
        / json.loads((ROOT / "config/robots/r680_lift_openarm_claw1.3.source.json").read_text())[
            "urdf"
        ]
    )
    model = RobotModel(model_path)
    current_poses, current_fk = urdf_replay(model, sdk, frames)
    current_errors = []
    current_angles = []
    for sample in recorded["selected_poses"]:
        error = np.linalg.inv(current_poses[sample["frame_index"]] @ y) @ calibration.object_pose(
            sample["B"], sample["height_m"] * 1e6
        )
        current_errors.append(float(np.linalg.norm(error[:3, 3]) * 1000))
        current_angles.append(float(np.rad2deg(Rotation.from_matrix(error[:3, :3]).magnitude())))
    current_metrics = metrics(current_errors, current_angles)
    for key in ["rms_mm", "p95_mm", "max_mm", "rotation_rms_deg"]:
        if not np.isclose(current_metrics[key], measured[key], atol=1e-8):
            raise ValueError("Current URDF changes calibration closure: " + key)
    original = None
    if a.original_session:
        source = a.original_session / "extrinsics_candidate_NOT_ACCEPTED.json"
        if (
            hashlib.sha256(source.read_bytes()).hexdigest()
            != calibration.config["source"]["sha256"]
        ):
            raise ValueError("Original calibration source hash mismatch")
        candidate = json.loads(source.read_text())
        if not np.array_equal(np.asarray(candidate["T_origin_camera"]), calibration.reference):
            raise ValueError("Current extrinsic differs from original candidate")
        for name, digest in candidate["input_sha256"].items():
            if hashlib.sha256((a.original_session / name).read_bytes()).hexdigest() != digest:
                raise ValueError("Original input hash mismatch: " + name)
        original_frames = json.loads((a.original_session / "offline/frames.json").read_text())
        original_poses, original_fk = urdf_replay(model, sdk, original_frames)
        errors = []
        angles = []
        with np.load(a.original_session / "offline/diagnostic_pairs.npz") as data:
            for index, b in zip(data["indices"], data["B"]):
                error = (
                    np.linalg.inv(original_poses[int(index)] @ np.asarray(candidate["T_ee_board"]))
                    @ calibration.reference
                    @ b
                )
                errors.append(float(np.linalg.norm(error[:3, 3]) * 1000))
                angles.append(float(np.rad2deg(Rotation.from_matrix(error[:3, :3]).magnitude())))
        original_metrics = metrics(errors, angles)
        if not np.isclose(
            original_metrics["rms_mm"], candidate["fit_metrics"]["translation_mm"]["rms"], atol=1e-8
        ):
            raise ValueError("Original fit closure not reproduced with current URDF")
        original = dict(
            source_and_input_hashes_verified=True,
            current_urdf_fk=original_fk,
            current_urdf_closure=original_metrics,
            scope="Recorded PnP poses replayed; no image redetection or new physical acceptance",
        )
    urdf_bases = [
        model.transform(f"openarm_{s}_base_link", {"lift_joint": 0})[:3, 3]
        for s in ["left", "right"]
    ]
    sdk_bases = [sdk.transform(f"openarm_{s}_base_link")[:3, 3] for s in ["left", "right"]]
    report = dict(
        status="offline_conversion_verified_not_hardware_acceptance",
        executable=False,
        sdk_source_hash_check=dict(
            checked=a.sdk_model is not None,
            path=str(a.sdk_model) if a.sdk_model is not None else None,
        ),
        first_extrinsic_unchanged=True,
        sdk_native_fk_comparison=dict(count=len(fk_errors), max_matrix_abs_error=max(fk_errors)),
        current_urdf=dict(
            path=str(model_path),
            sha256=hashlib.sha256(model_path.read_bytes()).hexdigest(),
            recorded_fk=current_fk,
            multiheight=current_metrics,
        ),
        original_session=original,
        multiheight=measured,
        by_height=[
            dict(
                height_m=h,
                **metrics(
                    [r["translation_mm"] for r in rows if r["height_m"] == h],
                    [r["rotation_deg"] for r in rows if r["height_m"] == h],
                ),
            )
            for h in sorted({r["height_m"] for r in rows})
        ],
        source_model_comparison=dict(
            urdf_base_origin_separation_m=float(np.linalg.norm(urdf_bases[0] - urdf_bases[1])),
            sdk_base_origin_separation_m=float(np.linalg.norm(sdk_bases[0] - sdk_bases[1])),
            note="Origins may have different physical definitions; cannot equate these frames or infer one common mounting transform from their names.",
        ),
        blockers=[
            "Head live aligned RGB-D must be available for object depth",
            "SDK-to-base_link/base_footprint registration unresolved",
            "Physical EE-to-grasp-TCP transforms unresolved",
        ],
        samples=rows,
    )
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    print(
        json.dumps(
            {k: v for k, v in report.items() if k not in ["samples", "by_height"]},
            ensure_ascii=False,
            indent=2,
        )
    )
    print("Report:", a.output)


if __name__ == "__main__":
    main()
