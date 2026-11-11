#!/usr/bin/env python3
"""Offline head eye-to-hand calibration from a post-zero ROS bag; never controls hardware."""

import argparse
import concurrent.futures
import hashlib
import html
import json
import shutil
import sys
from collections import Counter
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

CUSTOM_MESSAGES = {
    "luxi_interfaces/msg/CameraFrame": """std_msgs/Header header
string camera_id
uint64 sequence
int64 device_timestamp
int64 receive_mono_ns
string timestamp_quality
uint32 width
uint32 height
string encoding
uint8[] data
""",
    "luxi_interfaces/msg/ArmState": """std_msgs/Header header
int64 receive_mono_ns
float64[16] position
float64[16] velocity
float64[16] effort
int32[16] mos_temperature
int32[16] rotor_temperature
int64[16] age_ns
bool[16] valid
bool enabled
string phase
""",
}
PATTERN = (11, 8)
SQUARE_M = 0.010
OBJECT_POINTS = np.column_stack([np.mgrid[0:11, 0:8].T.reshape(-1, 2) * SQUARE_M, np.zeros(88)])


def save_json(path, value):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def rebase_recorded_output(previous, output, zero_record):
    """Convert archived feedback to the new encoder epoch; never alter source data."""
    previous, output, zero_record = map(Path, (previous, output, zero_record))
    if (previous / "coordinate_rebase.json").exists():
        raise ValueError("Source is already rebased; refusing to apply a zero change twice")
    record = json.loads(zero_record.read_text())
    if (
        record.get("joint") != "openarm_left_joint3"
        or record.get("sdk_index") != 10
        or record.get("motor_id") != 3
        or record.get("bus") != "luxi_left"
        or record.get("selected_zero_writes") != 1
        or record.get("other_joint_zero_writes") != 0
        or record.get("sdk_configs_changed") is not False
    ):
        raise ValueError("Expected a verified, isolated left J3 hardware-zero event")
    delta = float(record["observed_feedback_coordinate_shift_rad"])
    observed = record["new_sdk_rad_after_stamp"] - record["old_sdk_rad_at_hardware_stamp"]
    if not np.isfinite(delta) or not np.isclose(delta, observed, rtol=0, atol=1e-12):
        raise ValueError("Zero change disagrees with before/after encoder feedback")
    urdf = ROOT / record["urdf_path"]
    if hashlib.sha256(urdf.read_bytes()).hexdigest() != record["urdf_sha256"]:
        raise ValueError("URDF changed since the measured zero event")
    sdk = SDKKinematics(ROOT / "config/robots/calibration/sdk_arm_chains.json")
    summary = json.loads((previous / "extraction_summary.json").read_text())
    if summary["source_model_sha256"] != sdk.config["source"]["sha256"]:
        raise ValueError("Recorded SDK chain does not match current calibration chain")
    frames = json.loads((previous / "frames.json").read_text())
    for frame in frames:
        raw = np.asarray(frame["q"], dtype=float)
        if raw.shape != (16,) or not np.isfinite(raw).all():
            raise ValueError("Invalid recorded measured joint vector")
        frame["q_before_rezero"] = raw.tolist()
        converted = raw.copy()
        converted[10] += delta
        frame["q"] = converted.tolist()
        # Cached FK belongs to the old epoch and MUST be recomputed.
        frame["A"] = sdk.transform(
            "left_ee_control_point", sdk.from_sdk_vector(frame["q"])
        ).tolist()
    provenance = dict(
        schema="recorded_left_j3_zero_rebase_v1",
        previous_output=str(previous.resolve()),
        zero_record=str(zero_record.resolve()),
        zero_record_sha256=hashlib.sha256(zero_record.read_bytes()).hexdigest(),
        previous_frames_sha256=hashlib.sha256((previous / "frames.json").read_bytes()).hexdigest(),
        sdk_index=10,
        delta_rad=delta,
        delta_deg=float(np.rad2deg(delta)),
        formula="q_new[10] = q_recorded[10] + (q_after_stamp - q_before_stamp)",
        other_joints="unchanged; no correction applied to live feedback",
        stationary_quality_gates="unchanged; constant shift preserves ranges and pose groups",
        raw_recording_modified=False,
        detection="Reuse original image corners, PnP, CameraInfo, timestamps and lift readings",
        limitations=[
            "Before/after zero readings are quantized; observed shift has encoder quantization uncertainty",
            "Mechanical-reference residual is recorded separately; it is not added as another offset",
        ],
    )
    output.mkdir(parents=True, exist_ok=False)
    (output / "images").symlink_to((previous / "images").resolve(), target_is_directory=True)
    shutil.copytree(previous / "pnp", output / "pnp")
    for name in ("lifts.json", "detection_summary.json"):
        shutil.copy2(previous / name, output / name)
    with np.load(previous / "recorded_states.npz") as original:
        arrays = {name: original[name].copy() for name in original.files}
    arrays["q_before_rezero"] = arrays["q"].copy()
    arrays["q"][:, 10] += delta
    np.savez_compressed(output / "recorded_states.npz", **arrays)
    summary["joint_convention"] = provenance["formula"] + "; all other joints unchanged"
    summary["coordinate_rebase"] = provenance
    save_json(output / "frames.json", frames)
    save_json(output / "extraction_summary.json", summary)
    save_json(output / "coordinate_rebase.json", provenance)
    print("REBASED", len(frames), "frames; left J3 delta deg", provenance["delta_deg"], flush=True)


def stamp_ns(message):
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def inverse(transform):
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def pack(transform):
    return np.r_[Rotation.from_matrix(transform[:3, :3]).as_rotvec(), transform[:3, 3]]


def unpack(parameters):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(parameters[:3]).as_matrix()
    result[:3, 3] = parameters[3:6]
    return result


def extract(source, output, image_period_s=0.5):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    output.mkdir(parents=True, exist_ok=True)
    (output / "images").mkdir(exist_ok=True)
    sdk = SDKKinematics(ROOT / "config/robots/calibration/sdk_arm_chains.json")
    model_path = source / "provenance/model_xml/openarm_bimanual.xml"
    model_hash = hashlib.sha256(model_path.read_bytes()).hexdigest()
    assert model_hash == sdk.config["source"]["sha256"], "Recorded SDK model changed"
    store = get_typestore(Stores.ROS2_HUMBLE)
    for name, text in CUSTOM_MESSAGES.items():
        store.register(get_types_from_msg(text, name))
    arm_rows = []
    lifts = []
    camera_infos = []
    frames = []
    counts = Counter()
    last_image_ns = 0
    with AnyReader([source / "raw_bag"], default_typestore=store) as reader:
        for connection, bag_stamp, data in reader.messages():
            topic = connection.topic
            counts[topic] += 1
            message = reader.deserialize(data, connection.msgtype)
            if topic == "/luxi/arms/state":
                arm_rows.append(
                    (int(message.receive_mono_ns), message.position, message.valid, message.age_ns)
                )
            elif topic == "/luxi/lift/state":
                row = json.loads(message.data)
                row["bag_stamp"] = int(bag_stamp)
                lifts.append(row)
            elif topic == "/camera/head_rgb/color/camera_info":
                camera_infos.append(
                    dict(
                        stamp_ns=stamp_ns(message),
                        width=int(message.width),
                        height=int(message.height),
                        K=message.k.tolist(),
                        D=message.d.tolist(),
                        distortion_model=message.distortion_model,
                        frame=message.header.frame_id,
                    )
                )
            elif topic == "/luxi/cameras/head_rgb":
                receive_ns = int(message.receive_mono_ns)
                if receive_ns - last_image_ns < image_period_s * 1e9:
                    continue
                last_image_ns = receive_ns
                image = f"images/{len(frames):05d}.jpg"
                (output / image).write_bytes(bytes(message.data))
                frames.append(
                    dict(
                        image=image,
                        t=receive_ns,
                        header_stamp_ns=stamp_ns(message),
                        bag_stamp_ns=int(bag_stamp),
                        sequence=int(message.sequence),
                        frame=message.header.frame_id,
                        width=int(message.width),
                        height=int(message.height),
                        timestamp_quality=message.timestamp_quality,
                    )
                )
    arm_rows.sort(key=lambda row: row[0])
    arm_times = np.array([row[0] for row in arm_rows], dtype=np.int64)
    joint_positions = np.array([row[1] for row in arm_rows])
    joint_valid = np.array([row[2] for row in arm_rows])
    joint_age = np.array([row[3] for row in arm_rows])
    valid_lifts = sorted(
        [
            row
            for row in lifts
            if row.get("valid") and not row.get("mock", True) and not row.get("fault")
        ],
        key=lambda row: int(row["receive_mono_ns"]),
    )
    lift_times = np.array([int(row["receive_mono_ns"]) for row in valid_lifts], dtype=np.int64)
    heights_um = np.array([row["current_um"] for row in valid_lifts])
    assert len(lift_times), "No valid recorded lift feedback"
    camera_infos.sort(key=lambda row: row["stamp_ns"])
    info_times = np.array([row["stamp_ns"] for row in camera_infos], dtype=np.int64)
    stage_counts = Counter()
    for frame in frames:
        time_ns = frame["t"]
        nearest = int(np.argmin(abs(arm_times - time_ns)))
        lower, upper = np.searchsorted(arm_times, [time_ns - 600_000_000, time_ns + 600_000_000])
        window = joint_positions[lower:upper]
        frame["q"] = joint_positions[nearest].tolist()
        frame["A"] = sdk.transform(
            "left_ee_control_point", sdk.from_sdk_vector(frame["q"])
        ).tolist()
        frame["joint_delta_ms"] = float((arm_times[nearest] - time_ns) / 1e6)
        frame["left_range_rad"] = (
            float(np.ptp(window[:, 8:15], axis=0).max()) if len(window) else None
        )
        frame["gripper_range_rad"] = float(np.ptp(window[:, 15])) if len(window) else None
        arm_ok = bool(
            len(window) > 20
            and arm_times[lower] < time_ns - 400_000_000
            and arm_times[upper - 1] > time_ns + 400_000_000
            and joint_valid[lower:upper].all()
            and ((joint_age[lower:upper] >= 0) & (joint_age[lower:upper] < 150_000_000)).all()
            and abs(frame["joint_delta_ms"]) < 30
        )
        stationary = bool(
            arm_ok and frame["left_range_rad"] < 0.003 and frame["gripper_range_rad"] < 0.005
        )
        stage_counts["arm_fresh"] += arm_ok
        stage_counts["arm_stationary"] += stationary
        lift_window = abs(lift_times - time_ns) <= 600_000_000
        lift_before = lift_times[lift_window & (lift_times <= time_ns)]
        lift_after = lift_times[lift_window & (lift_times >= time_ns)]
        lift_heights = heights_um[lift_window]
        frame["height_m"] = float(np.median(lift_heights) * 1e-6) if len(lift_heights) else None
        frame["lift_range_um"] = float(np.ptp(lift_heights)) if len(lift_heights) else None
        frame["lift_before_ms"] = (
            float((time_ns - lift_before.max()) / 1e6) if len(lift_before) else None
        )
        frame["lift_after_ms"] = (
            float((lift_after.min() - time_ns) / 1e6) if len(lift_after) else None
        )
        lift_ok = bool(
            len(lift_heights) >= 3
            and len(lift_before)
            and len(lift_after)
            and frame["lift_before_ms"] < 150
            and frame["lift_after_ms"] < 150
            and frame["lift_range_um"] < 500
        )
        info_index = int(np.argmin(abs(info_times - frame["header_stamp_ns"])))
        info = camera_infos[info_index]
        frame["camera_info_delta_ms"] = (info["stamp_ns"] - frame["header_stamp_ns"]) / 1e6
        frame["intrinsics"] = info
        info_ok = bool(
            abs(frame["camera_info_delta_ms"]) < 20
            and info["frame"] == frame["frame"]
            and (info["height"], info["width"]) == (frame["height"], frame["width"])
        )
        frame["stationary_valid"] = stationary and lift_ok and info_ok
        stage_counts["lift_valid"] += lift_ok
        stage_counts["intrinsics_matched"] += info_ok
        stage_counts["accepted_stationary"] += frame["stationary_valid"]
    accepted = [frame for frame in frames if frame["stationary_valid"]]
    intrinsics_versions = Counter(json.dumps(row["K"]) for row in camera_infos)
    summary = dict(
        source=str(source.resolve()),
        source_model_sha256=model_hash,
        message_counts=dict(counts),
        candidate_frames=len(frames),
        stages=dict(stage_counts),
        valid_lift_range_um=[int(heights_um.min()), int(heights_um.max())],
        stationary_lift_range_m=[
            min(f["height_m"] for f in accepted),
            max(f["height_m"] for f in accepted),
        ]
        if accepted
        else None,
        left_joint_span_rad=np.ptp(joint_positions[:, 8:15], axis=0).tolist(),
        camera_intrinsics_versions=[
            dict(K=json.loads(key), count=value) for key, value in intrinsics_versions.items()
        ],
        image_selection=f"Minimum interval {image_period_s}s; timestamp selection precedes stationary and image-quality gates",
        joint_convention="Unmodified measured q[8:15] for left J1-J7, right first in q[16]",
    )
    np.savez_compressed(
        output / "recorded_states.npz",
        arm_times=arm_times,
        q=joint_positions,
        valid=joint_valid,
        age_ns=joint_age,
        lift_times=lift_times,
        heights_um=heights_um,
    )
    save_json(output / "lifts.json", lifts)
    save_json(output / "frames.json", frames)
    save_json(output / "extraction_summary.json", summary)
    print(json.dumps(summary), flush=True)


def real_sense_intrinsics(info):
    import pyrealsense2 as rs

    intrinsics = rs.intrinsics()
    intrinsics.width = info["width"]
    intrinsics.height = info["height"]
    matrix = np.array(info["K"]).reshape(3, 3)
    intrinsics.fx = matrix[0, 0]
    intrinsics.fy = matrix[1, 1]
    intrinsics.ppx = matrix[0, 2]
    intrinsics.ppy = matrix[1, 2]
    assert info["distortion_model"] == "realsense_2", info["distortion_model"]
    intrinsics.model = rs.distortion.inverse_brown_conrady
    intrinsics.coeffs = info["D"]
    return intrinsics


def detect_frame(task):
    import pyrealsense2 as rs

    output, frame = task
    cv2.setNumThreads(1)
    image = cv2.imread(str(output / frame["image"]))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners = None
    for scale in (1, 2, 3):
        scaled = (
            gray
            if scale == 1
            else cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        )
        found, detected = cv2.findChessboardCorners(
            scaled, PATTERN, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        if found:
            corners = (
                cv2.cornerSubPix(
                    scaled,
                    detected,
                    (3, 3),
                    (-1, -1),
                    (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001),
                ).reshape(-1, 2)
                / scale
            )
            break
    result = dict(image=frame["image"], found=corners is not None)
    if corners is not None:
        intrinsics = real_sense_intrinsics(frame["intrinsics"])
        rays = np.array(
            [rs.rs2_deproject_pixel_to_point(intrinsics, p.tolist(), 1)[:2] for p in corners]
        )
        _, rotations, translations, _ = cv2.solvePnPGeneric(
            OBJECT_POINTS, rays, np.eye(3), None, flags=cv2.SOLVEPNP_IPPE
        )
        candidates = []
        for rotation, translation in zip(rotations, translations):
            transform = np.eye(4)
            transform[:3, :3] = cv2.Rodrigues(rotation)[0]
            transform[:3, 3] = translation.ravel()
            points = OBJECT_POINTS @ transform[:3, :3].T + transform[:3, 3]
            if np.any(points[:, 2] <= 0):
                continue
            projected = np.array(
                [rs.rs2_project_point_to_pixel(intrinsics, point.tolist()) for point in points]
            )
            error = np.linalg.norm(projected - corners, axis=1)
            candidates.append(dict(B=transform.tolist(), rms_px=float(np.sqrt(np.mean(error**2)))))
        candidates.sort(key=lambda candidate: candidate["rms_px"])
        grid = corners.reshape(8, 11, 2)
        spacing = np.r_[
            np.linalg.norm(np.diff(grid, axis=0), axis=2).ravel(),
            np.linalg.norm(np.diff(grid, axis=1), axis=2).ravel(),
        ]
        result.update(
            corners=corners.tolist(),
            rays=rays.tolist(),
            candidates=candidates,
            scale=scale,
            spacing_px=float(np.median(spacing)),
        )
        cv2.drawChessboardCorners(image, PATTERN, corners.reshape(-1, 1, 2), True)
        cv2.imwrite(str(output / "pnp" / (Path(frame["image"]).stem + ".jpg")), image)
    save_json(output / "pnp" / (Path(frame["image"]).stem + ".json"), result)
    return result


def detect(output, workers):
    (output / "pnp").mkdir(exist_ok=True)
    frames = json.loads((output / "frames.json").read_text())
    selected = [frame for frame in frames if frame["stationary_valid"]]
    results = []
    with concurrent.futures.ProcessPoolExecutor(max_workers=workers) as pool:
        for index, result in enumerate(
            pool.map(detect_frame, [(output, frame) for frame in selected])
        ):
            results.append(result)
            if index % 50 == 0:
                print("Detected", index, "of", len(selected), flush=True)
    summary = dict(
        stationary_frames=len(selected),
        detected=sum(row["found"] for row in results),
        passing_0_3px=sum(
            bool(row.get("candidates")) and row["candidates"][0]["rms_px"] <= 0.3 for row in results
        ),
    )
    save_json(output / "detection_summary.json", summary)
    print(json.dumps(summary), flush=True)


def load_pairs(output):
    frames = json.loads((output / "frames.json").read_text())
    pairs = []
    anchors = []
    for frame_index, frame in enumerate(frames):
        if not frame["stationary_valid"]:
            continue
        path = output / "pnp" / (Path(frame["image"]).stem + ".json")
        detection = json.loads(path.read_text())
        if not detection.get("candidates") or detection["candidates"][0]["rms_px"] > 0.3:
            continue
        joints = np.array(frame["q"])[8:15]
        group = next(
            (i for i, anchor in enumerate(anchors) if np.max(abs(joints - anchor)) < 0.08),
            len(anchors),
        )
        if group == len(anchors):
            anchors.append(joints)
        pairs.append(
            dict(
                frame_index=frame_index,
                image=frame["image"],
                t=frame["t"],
                A=frame["A"],
                B=detection["candidates"][0]["B"],
                height_m=frame["height_m"],
                group=group,
                corners=detection["corners"],
                rays=detection["rays"],
                intrinsics=frame["intrinsics"],
                pnp_rms_px=detection["candidates"][0]["rms_px"],
                q=frame["q"],
            )
        )
    return pairs


def camera_at_height(reference, height_m, reference_height_m, axis):
    result = reference.copy()
    result[:3, 3] -= np.asarray(axis) * (height_m - reference_height_m)
    return result


def reprojection_residual(parameters, pairs, reference_height_m, axis):
    reference = unpack(parameters[:6])
    board_mount = unpack(parameters[6:12])
    wrist_points = OBJECT_POINTS @ board_mount[:3, :3].T + board_mount[:3, 3]
    arm_poses = np.array([pair["A"] for pair in pairs])
    origin_points = np.einsum("nij,kj->nki", arm_poses[:, :3, :3], wrist_points)
    origin_points += arm_poses[:, None, :3, 3]
    height_deltas = np.array([pair["height_m"] - reference_height_m for pair in pairs])
    camera_points = (
        origin_points - reference[:3, 3] + height_deltas[:, None, None] * axis
    ) @ reference[:3, :3]
    normalized = camera_points[:, :, :2] / camera_points[:, :, 2:]
    observed_rays = np.array([pair["rays"] for pair in pairs])
    focal = np.array([[pair["intrinsics"]["K"][0], pair["intrinsics"]["K"][4]] for pair in pairs])
    return ((normalized - observed_rays) * focal[:, None, :]).ravel()


def fit_pairs(pairs, reference_height_m, axis, initial=None):
    assert len(pairs) >= 10, "At least ten frame pairs are required"
    arm = np.array([pair["A"] for pair in pairs])
    board = np.array([pair["B"] for pair in pairs])
    counts = Counter(pair["group"] for pair in pairs)
    representatives = [next(pair for pair in pairs if pair["group"] == group) for group in counts]
    assert len(representatives) >= 3, "Insufficient rotational excitation"
    relative_rotations = np.array(
        [
            Rotation.from_matrix(
                (inverse(np.array(representatives[0]["A"])) @ np.array(pair["A"]))[:3, :3]
            ).as_rotvec()
            for pair in representatives[1:]
        ]
    )
    singular = np.linalg.svd(relative_rotations, compute_uv=False)
    assert len(singular) >= 2 and singular[1] > 0.1, "Insufficient rotational excitation"
    starts = []
    if not hasattr(cv2, "calibrateHandEye"):
        raise RuntimeError(
            "Use the calibration environment with OpenCV 4.x (calibrateHandEye required)"
        )
    if initial is not None:
        starts.append(initial)
    gripper_inverse = np.array([inverse(transform) for transform in arm])
    for method in (cv2.CALIB_HAND_EYE_PARK, cv2.CALIB_HAND_EYE_HORAUD, cv2.CALIB_HAND_EYE_TSAI):
        try:
            rotation, translation = cv2.calibrateHandEye(
                list(gripper_inverse[:, :3, :3]),
                list(gripper_inverse[:, :3, 3]),
                list(board[:, :3, :3]),
                list(board[:, :3, 3]),
                method=method,
            )
            if not np.isfinite(rotation).all() or np.linalg.det(rotation) < 0.5:
                continue
            reference = np.eye(4)
            reference[:3, :3] = rotation
            reference[:3, 3] = translation.ravel()
            mounts = np.array([inverse(a) @ reference @ b for a, b in zip(arm, board)])
            mount = np.eye(4)
            mount[:3, :3] = Rotation.from_matrix(mounts[:, :3, :3]).mean().as_matrix()
            mount[:3, 3] = np.median(mounts[:, :3, 3], axis=0)
            starts.append(np.r_[pack(reference), pack(mount)])
        except (cv2.error, ValueError):
            continue
    assert starts, "No valid hand-eye initialization"
    # Each distinct pose contributes equal total weight, irrespective of hold duration.
    weights = np.repeat([1 / counts[pair["group"]] for pair in pairs], len(OBJECT_POINTS) * 2)

    def loss(squared):
        root = np.sqrt(1 + squared)
        return np.vstack([2 * (root - 1) * weights, weights / root, -0.5 * weights / root**3])

    fits = []
    for start in starts:
        result = least_squares(
            reprojection_residual,
            start,
            args=(pairs, reference_height_m, np.asarray(axis)),
            loss=loss,
            f_scale=0.5,
            max_nfev=300,
            ftol=1e-10,
            xtol=1e-10,
            gtol=1e-10,
        )
        fits.append(result)
    successful = [result for result in fits if result.success and np.isfinite(result.x).all()]
    assert successful, "All refinements failed"
    best = min(successful, key=lambda result: result.cost)
    return best.x, dict(
        costs=[float(result.cost) for result in fits],
        cost=float(best.cost),
        nfev=best.nfev,
        success=bool(best.success),
        distinct_pose_groups=len(counts),
        excitation_singular_values=singular.tolist(),
        jacobian_singular_values=np.linalg.svd(best.jac, compute_uv=False).tolist(),
    )


def summarize(values):
    values = np.asarray(values, float)
    return dict(
        count=len(values),
        rms=float(np.sqrt(np.mean(values**2))),
        median=float(np.median(values)),
        p95=float(np.percentile(values, 95)),
        max=float(values.max()),
    )


def evaluate_pairs(parameters, pairs, reference_height_m, axis):
    import pyrealsense2 as rs

    reference = unpack(parameters[:6])
    mount = unpack(parameters[6:12])
    pixel_residual = reprojection_residual(
        parameters, pairs, reference_height_m, np.asarray(axis)
    ).reshape(-1, 88, 2)
    rows = []
    for index, pair in enumerate(pairs):
        expected = np.array(pair["A"]) @ mount
        observed = camera_at_height(
            reference, pair["height_m"], reference_height_m, axis
        ) @ np.array(pair["B"])
        error = inverse(expected) @ observed
        expected_points = OBJECT_POINTS @ expected[:3, :3].T + expected[:3, 3]
        observed_points = OBJECT_POINTS @ observed[:3, :3].T + observed[:3, 3]
        point_error_mm = np.linalg.norm(expected_points - observed_points, axis=1) * 1000
        optical = inverse(camera_at_height(reference, pair["height_m"], reference_height_m, axis))
        expected_camera_points = expected_points @ optical[:3, :3].T + optical[:3, 3]
        intrinsics = real_sense_intrinsics(pair["intrinsics"])
        projected = np.array(
            [
                rs.rs2_project_point_to_pixel(intrinsics, point.tolist())
                for point in expected_camera_points
            ]
        )
        exact_pixel_error = np.linalg.norm(projected - np.array(pair["corners"]), axis=1)
        rows.append(
            dict(
                frame_index=pair["frame_index"],
                image=pair["image"],
                group=pair["group"],
                time_ns=pair["t"],
                height_m=pair["height_m"],
                translation_mm=float(np.linalg.norm(error[:3, 3]) * 1000),
                rotation_deg=float(np.rad2deg(Rotation.from_matrix(error[:3, :3]).magnitude())),
                board_points_rms_mm=float(np.sqrt(np.mean(point_error_mm**2))),
                undistorted_reprojection_rms_px=float(
                    np.sqrt(np.mean(np.sum(pixel_residual[index] ** 2, axis=1)))
                ),
                reprojection_rms_px=float(np.sqrt(np.mean(exact_pixel_error**2))),
                pnp_rms_px=pair["pnp_rms_px"],
            )
        )
    metrics = {
        key: summarize([row[key] for row in rows])
        for key in ("translation_mm", "rotation_deg", "board_points_rms_mm", "reprojection_rms_px")
    }
    metrics["translation_below_10mm"] = sum(row["translation_mm"] < 10 for row in rows)
    metrics["pose_group_count"] = len({pair["group"] for pair in pairs})
    return dict(metrics=metrics, samples=rows)


def verify_urdf(pairs):
    sdk = SDKKinematics(ROOT / "config/robots/calibration/sdk_arm_chains.json")
    model = RobotModel(
        ROOT / "assets/robots/r680_lift_openarm_claw1.3/r680_lift_openarm_robot_viewer.urdf"
    )
    for joint in model.joints.values():
        joint["bounds"] = None
    base = "openarm_left_base_link"
    differences = []
    for pair in pairs:
        joints = sdk.from_sdk_vector(pair["q"])
        urdf = sdk.transform(base) @ model.transform("openarm_left_link7", joints, base=base)
        differences.append(float(np.max(abs(urdf - np.array(pair["A"])))))
    assert max(differences) < 1e-10
    return dict(
        count=len(differences),
        max_matrix_abs_error=max(differences),
        convention="Unmodified measured q; independent URDF geometry, no clipping",
    )


def solve(output):
    rebase_path = output / "coordinate_rebase.json"
    rebase = json.loads(rebase_path.read_text()) if rebase_path.exists() else None
    pairs = load_pairs(output)
    assert len(pairs) >= 20, "Insufficient qualifying observations"
    save_json(output / "pairs.json", pairs)
    groups = sorted({pair["group"] for pair in pairs})
    reference_height_m = float(np.median([pair["height_m"] for pair in pairs]))
    height_span_m = float(np.ptp([pair["height_m"] for pair in pairs]))
    axis = np.array([0.0, 0.0, 1.0])
    assert height_span_m < 0.0005, (
        "This single-height fit must not silently ignore height variation"
    )
    folds = []
    heldout_rows = []
    for fold in range(4):
        train = [pair for pair in pairs if pair["group"] % 4 != fold]
        test = [pair for pair in pairs if pair["group"] % 4 == fold]
        parameters, fit = fit_pairs(train, reference_height_m, axis)
        evaluation = evaluate_pairs(parameters, test, reference_height_m, axis)
        heldout_rows.extend(evaluation["samples"])
        folds.append(
            dict(
                fold=fold,
                train_groups=sorted({pair["group"] for pair in train}),
                test_groups=sorted({pair["group"] for pair in test}),
                fit=fit,
                **evaluation,
            )
        )
        print("FOLD", fold, json.dumps(evaluation["metrics"]), flush=True)
    parameters, full_fit = fit_pairs(pairs, reference_height_m, axis)
    training = evaluate_pairs(parameters, pairs, reference_height_m, axis)
    cross_validation_metrics = {
        key: summarize([row[key] for row in heldout_rows])
        for key in ("translation_mm", "rotation_deg", "board_points_rms_mm", "reprojection_rms_px")
    }
    pose_rows = []
    for group in groups:
        rows = [row for row in heldout_rows if row["group"] == group]
        pose_rows.append(
            {
                key: float(np.median([row[key] for row in rows]))
                for key in (
                    "translation_mm",
                    "rotation_deg",
                    "board_points_rms_mm",
                    "reprojection_rms_px",
                )
            }
        )
    group_metrics = {key: summarize([row[key] for row in pose_rows]) for key in pose_rows[0]}
    old_path = (
        Path(rebase["previous_output"]) / "extrinsics_candidate.json"
        if rebase
        else ROOT / "config/robots/calibration/head_camera_first.json"
    )
    old = json.loads(old_path.read_text())
    old_reference = np.array(old["T_origin_camera"])
    old_at_height = camera_at_height(
        old_reference,
        reference_height_m,
        old["reference_height_um"] * 1e-6,
        old["lift_axis_in_origin"],
    )
    candidate = dict(
        schema="wheel_head_extrinsics_v1",
        selection="post_zero_20261102_candidate",
        status="candidate_group_cross_validated_at_one_height",
        deployable=False,
        parent_frame="arm_origin",
        child_frame="head_rgb_color_optical",
        units="metres",
        T_origin_camera=unpack(parameters[:6]).tolist(),
        T_ee_board=unpack(parameters[6:12]).tolist(),
        reference_height_um=int(round(reference_height_m * 1e6)),
        lift_axis_in_origin=axis.tolist(),
        height_range_um=[-180000, 180000],
        measured_height_range_um=[int(round(reference_height_m * 1e6))] * 2,
        lift_compensation=dict(
            formula="R(h)=R0; t(h)=t0-[0,0,1]*(h_um-h0_um)*1e-6",
            source="Recorded SDK cell.xml: lifter axis +Z, arms attached without rotation",
            fitted_from_this_bag=False,
            cross_height_validated=False,
            reason="All valid lift feedback in this bag equals 179494 um; direction/scale unobservable",
            height_range_policy="Configured hardware envelope only; accuracy evaluated at measured height only",
        ),
        camera=dict(
            id="head_rgb",
            serials=["261043062235"],
            optical_frames=["head_rgb_color_optical"],
            intrinsics=pairs[0]["intrinsics"],
        ),
        base_T_origin_at_reference=None,
        ee_T_tcp=dict(left=None, right=None),
        limitations=[
            "Single session, pose-group held-out validation, not independent physical metrology",
            "Lift compensation is nominal and not evaluated across heights by this bag",
            "Board-to-wrist mounting is estimated; board pose does not establish grasp TCP",
            "Hardware zero was changed before this recording; old recordings are not pooled",
        ],
        source=dict(
            path=str((output / "pairs.json").resolve()),
            sha256=hashlib.sha256((output / "pairs.json").read_bytes()).hexdigest(),
            bag=json.loads((output / "extraction_summary.json").read_text())["source"],
            board_pattern_inner_corners=list(PATTERN),
            square_size_m=SQUARE_M,
            board_reference="Existing operator-confirmed pattern and square size",
        ),
    )

    if rebase:
        candidate["selection"] = "left_j3_rezero_20261103_candidate"
        candidate["source"]["coordinate_rebase"] = rebase
        candidate["limitations"][-1] = (
            "Recording predates left J3 zero change; archived q[10] converted using measured "
            "before/after encoder change; all other recorded angles unchanged"
        )
    candidate["validation_summary"] = dict(
        accepted_stationary_frames=len(pairs),
        pose_groups=len(groups),
        in_sample=training["metrics"],
        pose_group_cross_validation=cross_validation_metrics,
        reference=str((output / "calibration_result.json").resolve()),
    )

    report = dict(
        status="offline_recalculated",
        reference_height_m=reference_height_m,
        height_span_m=height_span_m,
        accepted_frames=len(pairs),
        distinct_pose_groups=len(groups),
        frame_quality_gates="Arm +/-0.6s range<0.003rad, gripper<0.005rad; valid fresh lift brackets<150ms, range<500um; same-time CameraInfo; PnP<=0.3px",
        grouping="Same group if all left J1-J7 within 0.08rad of an existing pose anchor, including later repeated poses",
        validation="Four-fold pose-group holdout; no frames from held-out groups enter that fold's X/Y fit",
        full_fit=full_fit,
        in_sample=training,
        grouped_cross_validation=dict(
            metrics=cross_validation_metrics, equal_pose_weight_metrics=group_metrics, folds=folds
        ),
        urdf_sdk_fk=verify_urdf(pairs),
        old_extrinsic_at_new_height=old_at_height.tolist(),
        candidate=candidate,
    )
    if rebase:
        report["coordinate_rebase"] = rebase
        report["urdf_sdk_fk"]["convention"] = (
            "Archived measured q converted to the new left J3 encoder epoch; no clipping"
        )
    save_json(output / "extrinsics_candidate.json", candidate)
    save_json(output / "calibration_result.json", report)
    print(
        "FINAL",
        json.dumps(
            dict(
                train=training["metrics"],
                heldout=cross_validation_metrics,
                pose_balanced_heldout=group_metrics,
            )
        ),
        flush=True,
    )


def build_report(output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["font.family"] = "Noto Sans CJK SC"
    result = json.loads((output / "calibration_result.json").read_text())
    extraction = json.loads((output / "extraction_summary.json").read_text())
    rebase = result.get("coordinate_rebase")
    coordinate_note = (
        f"本次复用2026-11-02录包；已将其中左J3的实测角统一换算到2026-11-03新硬件零点："
        f"q_new[10] = q_recorded[10] + {rebase['delta_rad']:.12f} rad"
        f"（+{rebase['delta_deg']:.6f}°）。其余15个角度、图像、内参、升降与筛选分组保持原值。"
        "该换算只用于历史数据，当前SDK反馈不再叠加此角度。"
        if rebase
        else "录包角度属于2026-11-02硬件标零之后，未做额外关节角换算。"
    )
    fk_note = (
        "历史实测q已按写零前后反馈换算，再输入SDK和claw1.3 URDF"
        if rebase
        else "实测q直接输入SDK和claw1.3 URDF"
    )
    candidate = result["candidate"]
    training = result["in_sample"]["metrics"]
    validation = result["grouped_cross_validation"]["metrics"]
    group_metrics = result["grouped_cross_validation"]["equal_pose_weight_metrics"]
    heldout = sorted(
        [row for fold in result["grouped_cross_validation"]["folds"] for row in fold["samples"]],
        key=lambda row: row["time_ns"],
    )
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), layout="constrained")
    times = (np.array([row["time_ns"] for row in heldout]) - heldout[0]["time_ns"]) / 1e9
    groups = np.array([row["group"] for row in heldout])
    axes[0, 0].scatter(
        times, [row["translation_mm"] for row in heldout], c=groups, cmap="tab20", s=14
    )
    axes[0, 0].axhline(10, color="red", linestyle="--", label="10 mm参考线")
    axes[0, 0].set(
        xlabel="合格停稳帧相对时间 / s",
        ylabel="平移闭环误差 / mm",
        title="按姿态分组留出：每张图均未进入对应折拟合",
    )
    axes[0, 0].legend()
    axes[0, 1].scatter(
        times, [row["rotation_deg"] for row in heldout], c=groups, cmap="tab20", s=14
    )
    axes[0, 1].set(
        xlabel="合格停稳帧相对时间 / s",
        ylabel="旋转闭环误差 / °",
        title="同一棋盘格坐标系的旋转闭环",
    )
    unique_groups = sorted(set(groups))
    per_group = [
        np.median([row["translation_mm"] for row in heldout if row["group"] == group])
        for group in unique_groups
    ]
    axes[1, 0].bar(unique_groups, per_group, color="#247a97")
    axes[1, 0].axhline(10, color="red", linestyle="--")
    axes[1, 0].set(
        xlabel="姿态组（重复停留归入同组）", ylabel="组内误差中位数 / mm", title="各姿态组等权比较"
    )
    axes[1, 1].plot([0, 524.231], [179.494, 179.494], color="#247a97")
    axes[1, 1].set(
        xlabel="录包相对时间 / s",
        ylabel="有效升降反馈 / mm",
        ylim=(178.5, 180.5),
        title="本包只有一个高度，无法验证跨高度补偿",
    )
    fig.savefig(output / "accuracy.png", dpi=170)
    plt.close(fig)
    rows = []
    for name, metric in [
        ("全部合格帧拟合内闭环", training),
        ("姿态组留出交叉验证", validation),
        ("留出结果按姿态等权", group_metrics),
    ]:
        t = metric["translation_mm"]
        r = metric["rotation_deg"]
        rows.append(
            f"<tr><td>{name}</td><td>{t['rms']:.2f}</td><td>{t['p95']:.2f}</td><td>{t['max']:.2f}</td><td>{r['rms']:.2f}</td></tr>"
        )
    overlays = []
    pairs = {pair["frame_index"]: pair for pair in json.loads((output / "pairs.json").read_text())}
    parameters = np.r_[
        pack(np.array(candidate["T_origin_camera"])), pack(np.array(candidate["T_ee_board"]))
    ]
    # Representative best/median/worst images use the final all-data model and are labelled in-sample.
    sample_rows = sorted(result["in_sample"]["samples"], key=lambda row: row["translation_mm"])
    for label, row in [
        ("best", sample_rows[0]),
        ("median", sample_rows[len(sample_rows) // 2]),
        ("worst", sample_rows[-1]),
    ]:
        import pyrealsense2 as rs

        pair = pairs[row["frame_index"]]
        frame = cv2.imread(str(output / pair["image"]))
        board_pose = np.array(pair["A"]) @ unpack(parameters[6:])
        camera_pose = inverse(unpack(parameters[:6])) @ board_pose
        points = OBJECT_POINTS @ camera_pose[:3, :3].T + camera_pose[:3, 3]
        intrinsics = real_sense_intrinsics(pair["intrinsics"])
        pixels = [rs.rs2_project_point_to_pixel(intrinsics, point.tolist()) for point in points]
        for observed, predicted in zip(pair["corners"], pixels):
            a = tuple(np.rint(observed).astype(int))
            b = tuple(np.rint(predicted).astype(int))
            cv2.circle(frame, a, 2, (0, 220, 0), -1)
            cv2.circle(frame, b, 2, (0, 0, 255), 1)
            cv2.line(frame, a, b, (0, 160, 255), 1)
        filename = f"overlay_{label}.jpg"
        cv2.imwrite(str(output / filename), frame)
        overlays.append(
            f'<figure><img src="{filename}"><figcaption>最终模型拟合内回放：{label}；平移 {row["translation_mm"]:.2f} mm；重投影 {row["reprojection_rms_px"]:.2f} px。绿点=检测，红圈=运动学预测。</figcaption></figure>'
        )
    text = f"""<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>标零后头部外参重算与精度检查</title>
<style>body{{font:17px/1.65 sans-serif;max-width:1180px;margin:40px auto;padding:0 24px;color:#182d39}}h1,h2{{line-height:1.3}}table{{border-collapse:collapse;width:100%}}td,th{{padding:10px;border-bottom:1px solid #ccd9df;text-align:left}}img{{max-width:100%}}pre{{background:#f0f5f7;padding:16px;overflow:auto}}.note{{background:#fff2d8;padding:15px;border-left:5px solid #c78c13}}figure{{margin:24px 0}}</style>
<h1>标零后头部相机外参重算与精度检查</h1>
<p>数据：2026-11-02 11:15:48—11:24:33 的录包。{extraction["candidate_frames"]} 张原始RGB全部检查；{extraction["stages"]["accepted_stationary"]} 张通过停稳、反馈、内参配对检查，{result["accepted_frames"]} 张通过棋盘格与PnP≤0.3px检查，合为 {result["distinct_pose_groups"]} 组姿态。棋盘格11×8内角点，格长10mm，沿用用户既有尺寸确认。</p>
<p>{html.escape(coordinate_note)}</p>
<p class="note">这是棋盘格视觉位姿与关节运动学之间的闭环一致性，不是独立计量或抓取TCP精度。全部有效升降反馈均为179.494mm；本包不能估计或验证跨高度升降方向、比例与精度。</p>
<h2>精度</h2><table><tr><th>统计口径</th><th>平移RMS / mm</th><th>平移P95 / mm</th><th>平移最大 / mm</th><th>旋转RMS / °</th></tr>{"".join(rows)}</table>
<p>“拟合内”使用全部合格数据估计X/Y后回放；“姿态组留出”按整组做四折训练/验证，重复姿态不会逐帧随机分到两侧。“姿态等权”先取每组中位误差，避免长时间停留支配统计。当前误差不支持全工作空间稳定10mm以内的结论。</p>
<img src="accuracy.png" alt="闭环精度与单高度观测图">
<h2>外参与升降补偿</h2><p>列向量，单位米，X=arm_origin_T_head_rgb_color_optical，h0=179494μm。</p>
<pre>{html.escape(json.dumps(candidate["T_origin_camera"], indent=2))}</pre>
<pre>R(h) = R0\nt(h) = t0 − [0, 0, 1] × (h_um − 179494) × 10⁻⁶</pre>
<p>补偿轴来自录包保存的SDK cell.xml中+Z升降轴，机械臂原点随升降上移时，固定头相机在臂原点坐标中的Z减小。SDK腕端FK不再额外叠加一次升降量。[-180000,180000]μm是配置允许的模型计算范围，实测精度范围仅179494μm；跨高度结果为名义预测。</p>
<p>同期CameraInfo的fx/fy=387.0193787/386.5122070px，640×480，使用RealSense inverse Brown模型去畸变，不使用旧的独立内参文件。{html.escape(fk_note)}；两套腕端FK最大矩阵差 {result["urdf_sdk_fk"]["max_matrix_abs_error"]:.3g}。</p>
<h2>叠图回放</h2>{"".join(overlays)}
<h2>可复核文件</h2><p><a href="calibration_result.json">全部拟合与分组验证结果</a> · <a href="extrinsics_candidate.json">带升降补偿的外参候选</a> · <a href="pairs.json">输入配对与分组</a> · <a href="extraction_summary.json">原始数据核查</a></p>
<p>本次未修改关节零点、URDF、运动参数或真机部署；旧已选外参文件保留，新结果单独保存为候选。</p></html>"""
    (output / "report.html").write_text(text)
    print("REPORT", output / "report.html", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("extract", "detect", "rebase", "solve", "report"))
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--image-period-s", type=float, default=0.5)
    parser.add_argument("--zero-record", type=Path)
    args = parser.parse_args()
    if args.phase == "rebase":
        if args.zero_record is None:
            parser.error(
                "rebase requires --zero-record; --source is the previous analysis directory"
            )
        rebase_recorded_output(args.source, args.output, args.zero_record)
    elif args.phase == "extract":
        extract(args.source, args.output, args.image_period_s)
    elif args.phase == "detect":
        detect(args.output, args.workers)
    elif args.phase == "solve":
        solve(args.output)
    else:
        build_report(args.output)


if __name__ == "__main__":
    main()
