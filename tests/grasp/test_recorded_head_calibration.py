"""Synthetic end-to-end checks for the recorded camera/arm/lift conventions."""

import json
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/grasp"))
import recalibrate_recorded_head as calibration


@pytest.mark.skipif(
    not hasattr(calibration.cv2, "calibrateHandEye"),
    reason="Requires OpenCV 4.x calibration environment",
)
def test_recovers_camera_and_board_with_relative_lift_motion():
    rng = np.random.default_rng(20261102)
    camera = np.eye(4)
    camera[:3, :3] = Rotation.from_rotvec([0.1, -0.2, 0.05]).as_matrix()
    camera[:3, 3] = [-0.1, 0.02, -0.6]
    board_mount = np.eye(4)
    board_mount[:3, :3] = Rotation.from_rotvec([-0.1, 0.2, 0.3]).as_matrix()
    board_mount[:3, 3] = [0.04, -0.02, 0.15]
    reference_height = 0.179494
    pairs = []
    for index, height in enumerate(np.linspace(-0.1, reference_height, 24)):
        arm = np.eye(4)
        arm[:3, :3] = Rotation.from_rotvec(rng.normal(0, 0.45, 3)).as_matrix()
        arm[:3, 3] = rng.normal(0, 0.06, 3)
        camera_at_height = camera.copy()
        camera_at_height[2, 3] -= height - reference_height
        observed_board = np.linalg.inv(camera_at_height) @ arm @ board_mount
        observed_points = (
            calibration.OBJECT_POINTS @ observed_board[:3, :3].T + observed_board[:3, 3]
        )
        assert (observed_points[:, 2] > 0).all()
        pairs.append(
            dict(
                group=index,
                A=arm.tolist(),
                B=observed_board.tolist(),
                rays=(observed_points[:, :2] / observed_points[:, 2:]).tolist(),
                height_m=float(height),
                intrinsics=dict(K=[400, 0, 320, 0, 400, 240, 0, 0, 1]),
            )
        )
    parameters, status = calibration.fit_pairs(pairs, reference_height, np.array([0, 0, 1]))
    assert status["success"]
    np.testing.assert_allclose(calibration.unpack(parameters[:6]), camera, atol=1e-7)
    np.testing.assert_allclose(calibration.unpack(parameters[6:]), board_mount, atol=1e-7)
    # A 100 mm rise moves the fixed camera 100 mm down in the moving arm frame.
    shifted = calibration.camera_at_height(
        camera, reference_height + 0.1, reference_height, [0, 0, 1]
    )
    np.testing.assert_allclose(shifted[:3, 3] - camera[:3, 3], [0, 0, -0.1], atol=1e-15)
    np.testing.assert_array_equal(shifted[:3, :3], camera[:3, :3])


def test_repeated_stationary_pose_cannot_identify_hand_eye():
    pair = dict(group=0, A=np.eye(4).tolist(), B=np.eye(4).tolist())
    with pytest.raises(AssertionError, match="rotational excitation"):
        calibration.fit_pairs([pair] * 20, 0.179494, np.array([0, 0, 1]))


def test_rebase_changes_only_archived_left_j3_and_recomputes_cached_fk(tmp_path):
    previous = tmp_path / "previous"
    previous.mkdir()
    (previous / "images").mkdir()
    (previous / "pnp").mkdir()
    calibration.save_json(previous / "pnp/frame.json", {"corners": [[12, 34]]})
    sdk = calibration.SDKKinematics(
        calibration.ROOT / "config/robots/calibration/sdk_arm_chains.json"
    )
    q = np.zeros((2, 16))
    q[1, 8:15] = [0.2, -0.1, 0.3, 0.7, 0.1, 0.2, -0.4]
    frames = [
        {
            "q": row.tolist(),
            "A": sdk.transform("left_ee_control_point", sdk.from_sdk_vector(row)).tolist(),
            "stationary_valid": True,
        }
        for row in q
    ]
    calibration.save_json(previous / "frames.json", frames)
    original_bytes = (previous / "frames.json").read_bytes()
    calibration.save_json(
        previous / "extraction_summary.json",
        {"source_model_sha256": sdk.config["source"]["sha256"]},
    )
    for name in ("lifts.json", "detection_summary.json"):
        calibration.save_json(previous / name, {})
    np.savez_compressed(previous / "recorded_states.npz", q=q, arm_times=[10, 20])
    record = calibration.ROOT / "config/robots/calibration/left_j3_rezero_20261103.json"
    delta = json.loads(record.read_text())["observed_feedback_coordinate_shift_rad"]
    output = tmp_path / "converted"
    calibration.rebase_recorded_output(previous, output, record)
    converted = json.loads((output / "frames.json").read_text())
    for old, new in zip(frames, converted):
        expected = np.array(old["q"])
        expected[10] += delta
        np.testing.assert_array_equal(new["q_before_rezero"], old["q"])
        np.testing.assert_array_equal(new["q"], expected)
        np.testing.assert_allclose(
            new["A"], sdk.transform("left_ee_control_point", sdk.from_sdk_vector(expected))
        )
        assert not np.allclose(new["A"], old["A"])
        assert new["stationary_valid"] is True
    assert (previous / "frames.json").read_bytes() == original_bytes
    with np.load(output / "recorded_states.npz") as data:
        np.testing.assert_array_equal(data["q_before_rezero"], q)
        expected = q.copy()
        expected[:, 10] += delta
        np.testing.assert_array_equal(data["q"], expected)
        np.testing.assert_array_equal(data["arm_times"], [10, 20])
    with pytest.raises(ValueError, match="already rebased"):
        calibration.rebase_recorded_output(output, tmp_path / "twice", record)


def test_rebase_rejects_angle_not_supported_by_encoder_event(tmp_path):
    source_record = calibration.ROOT / "config/robots/calibration/left_j3_rezero_20261103.json"
    record = json.loads(source_record.read_text())
    # The inferred mechanical reference is NOT the actual encoder coordinate change.
    record["observed_feedback_coordinate_shift_rad"] = -record["design_zero_in_old_sdk_rad"]
    changed = tmp_path / "wrong_record.json"
    calibration.save_json(changed, record)
    output = tmp_path / "converted"
    with pytest.raises(ValueError, match="before/after encoder feedback"):
        calibration.rebase_recorded_output(tmp_path / "unused_source", output, changed)
    assert not output.exists()
