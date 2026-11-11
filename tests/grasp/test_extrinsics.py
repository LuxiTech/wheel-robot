from pathlib import Path

import numpy as np
import pytest
from wheel_motion_core.sdk_kinematics import SDKKinematics
from wheel_perception_core.extrinsics import CalibratedHeadTransform, HeadExtrinsics
from wheel_perception_core.rgbd import Observation
from wheel_sdk_bridge.calibrated_capture import LiftCaptureHistory, ReadOnlyHeadCapture

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def calibration():
    return HeadExtrinsics(ROOT / "config/robots/calibration/head_camera_first.json")


@pytest.fixture
def sdk():
    return SDKKinematics(ROOT / "config/robots/calibration/sdk_arm_chains.json")


def state(t, height=0, speed=0):
    return dict(
        system=dict(mock=False),
        server_mono_ns=str(t + 1_000_000),
        lift=dict(
            valid=True,
            mock=False,
            receive_mono_ns=str(t),
            effective_age_ms=5,
            current_um=height,
            speed_rpm=speed,
            fault=0,
        ),
    )


def context():
    history = LiftCaptureHistory()
    for t in range(800_000_000, 1_250_000_000, 50_000_000):
        history.append_state(state(t, 170000))
    meta = dict(
        pair_id="123:7",
        receive_mono_ns="1000000000",
        camera_id="head_rgb",
        frame_id="head_rgb_color_optical",
    )
    return meta, history.context(meta, "261043062235")


def test_height_compensation_direction_and_reference(calibration):
    np.testing.assert_array_equal(calibration.camera_to_origin(170000), calibration.reference)
    a = calibration.camera_to_origin(-180000)
    b = calibration.camera_to_origin(180000)
    np.testing.assert_allclose(b[:3, 3] - a[:3, 3], [0, 0, -0.36], atol=1e-15)
    np.testing.assert_array_equal(a[:3, :3], b[:3, :3])
    for h in [float("nan"), float("inf"), 180001, -180001]:
        with pytest.raises(ValueError):
            calibration.camera_to_origin(h)


def test_post_zero_reference_height_and_selection():
    selected = HeadExtrinsics(ROOT / "config/robots/calibration/head_camera_rezero_20261102.json")
    reference_height = selected.config["reference_height_um"]
    assert reference_height == 179494
    np.testing.assert_array_equal(selected.camera_to_origin(reference_height), selected.reference)
    shifted = selected.camera_to_origin(reference_height - 10000)
    np.testing.assert_allclose(shifted[:3, 3] - selected.reference[:3, 3], [0, 0, 0.01], atol=1e-15)
    np.testing.assert_array_equal(shifted[:3, :3], selected.reference[:3, :3])
    meta, capture_context = context()
    capture_context["height_um"] = reference_height
    transform, reasons = CalibratedHeadTransform(selected).resolve(
        Observation(meta, None, None, 0), capture_context
    )
    np.testing.assert_array_equal(transform, selected.reference)
    assert "selected_calibration:post_zero_20261102_candidate" in reasons
    with pytest.raises(ValueError, match="Camera/frame"):
        selected.check_camera("head_rgb", "head_rgb_optical", "261043062235")


def test_conversion_cli_defaults_to_selected_color_frame(capsys):
    import json

    from grasp.camera_to_sdk import main

    code = main(
        [
            "--height-um",
            "81814",
            "--camera-serial",
            "261043062235",
            "--point",
            "0",
            "0",
            "0.5",
        ]
    )
    result = json.loads(capsys.readouterr().out)
    assert code == 0
    assert result["calibration_selection"] == "fixed_height_1_after_J4_rezero_20261104"
    assert result["reference_height_um"] == 81814
    selected = HeadExtrinsics(result["calibration_path"])
    np.testing.assert_allclose(
        result["frames"]["arm_origin"]["point_m"],
        (selected.reference @ [0, 0, 0.5, 1])[:3],
    )


def test_frame_identity_and_no_guessed_base(calibration):
    calibration.check_camera("head_rgb", "head_rgb_color_optical", "261043062235")
    calibration.check_camera("head_rgb", "head_rgb_optical", "260922304036")
    for camera, frame, serial in [
        ("right_wrist_rgb", "head_rgb_optical", "261043062235"),
        ("head_rgb", "head_d455_color_optical_frame", "261043062235"),
        ("head_rgb", "head_rgb_optical", "wrong"),
    ]:
        with pytest.raises(ValueError):
            calibration.check_camera(camera, frame, serial)
    with pytest.raises(ValueError, match="Unregistered"):
        calibration.transform(0, target_frame="base_link")
    with pytest.raises(ValueError, match="TCP"):
        calibration.sdk_ee_target(np.eye(4), "left")


def test_sdk_both_arms_pose_roundtrip(calibration, sdk):
    q = np.zeros(16)
    q[3] = 0.8
    q[11] = 1.1
    q[13] = -0.2
    q[6] = 0.3
    joints = sdk.from_sdk_vector(q)
    for side in ["left", "right"]:
        for frame in [side + "_ee_control_point", "openarm_" + side + "_base_link"]:
            t = calibration.transform(-90000, target_frame=frame, sdk_fk=sdk, joints=joints)
            np.testing.assert_allclose(
                sdk.transform(frame, joints) @ t, calibration.camera_to_origin(-90000), atol=1e-14
            )
            point = np.array([0.1, -0.2, 0.8])
            p = calibration.point(point, -90000, target_frame=frame, sdk_fk=sdk, joints=joints)
            np.testing.assert_allclose(p, (t @ np.r_[point, 1])[:3])
    for side, sign in [("left", 1), ("right", -1)]:
        t = sdk.transform(side + "_ee_control_point", sdk.from_sdk_vector(np.zeros(16)))
        np.testing.assert_allclose(t[:3, 3], [0, sign * 0.1535, -0.436])
    with pytest.raises(ValueError, match="Missing"):
        sdk.transform("left_ee_control_point", {})
    with pytest.raises(ValueError):
        sdk.from_sdk_vector([0] * 14)


def test_context_pairing_and_stationarity(calibration):
    meta, c = context()
    obs = Observation(meta, None, None, 0)
    adapter = CalibratedHeadTransform(calibration)
    t, reasons = adapter.resolve(obs, c)
    np.testing.assert_array_equal(t, calibration.reference)
    assert reasons
    for override in [
        dict(pair_id="other"),
        dict(image_receive_mono_ns="1"),
        dict(stationary=False),
        dict(time_basis="workstation_monotonic"),
        dict(bracket_mono_ns=["1", "2"]),
        dict(camera_serial="wrong"),
    ]:
        with pytest.raises(ValueError):
            adapter.resolve(obs, dict(c, **override))


def test_diagnostic_candidates_preserve_gate_and_reject_wrong_capture(calibration):
    meta, c = context()
    obs = Observation(meta, None, None, 0)
    adapter = CalibratedHeadTransform(calibration)
    target = np.eye(4)
    target[:3, 3] = [0.1, 0.2, 0.8]
    result = adapter.diagnostic_candidates(obs, c, {"A": target}, {"A": np.eye(4)})
    np.testing.assert_allclose(
        np.array(result["grasp"]["A"])[:3, 3], calibration.point([0.1, 0.2, 0.8], 170000)
    )
    assert result["pair_id"] == meta["pair_id"]
    assert result["executable"] is False and result["diagnostic_only"] is True
    assert result["arm_assignment_validated"] is False
    with pytest.raises(ValueError, match="another image"):
        adapter.diagnostic_candidates(obs, dict(c, pair_id="wrong"), {"A": target}, {})
    bad = target.copy()
    bad[0, 0] = float("nan")
    with pytest.raises(ValueError):
        adapter.diagnostic_candidates(obs, c, {"A": bad}, {})


def test_lift_history_rejects_motion_staleness_and_reboots():
    history = LiftCaptureHistory()
    for t in range(800_000_000, 1_250_000_000, 50_000_000):
        history.append_state(state(t, t / 1e6 * 2))
    with pytest.raises(ValueError, match="moving"):
        history.context(dict(pair_id="a", receive_mono_ns="1000000000"), "s")
    bad = state(1_300_000_000)
    bad["lift"]["effective_age_ms"] = 500
    with pytest.raises(ValueError, match="Stale"):
        history.append_state(bad)
    with pytest.raises(ValueError, match="reset"):
        history.append_state(state(1))
    assert not history.samples
    bad = state(2)
    bad["system"]["mock"] = True
    with pytest.raises(ValueError, match="Real"):
        history.append_state(bad)


def test_rgb_only_driver_does_not_pass_live_capture(calibration):
    source = ReadOnlyHeadCapture("http://unused", calibration)
    s = dict(
        cameras={
            "head_rgb": dict(
                driver=dict(fresh=True, mock=False, phase="streaming", serial="261043062235"),
                rgbd=dict(fresh=False),
            )
        }
    )
    with pytest.raises(ValueError, match="RGB-D unavailable"):
        source.camera_serial(s)


def test_calibrated_pipeline_uses_sdk_frame_not_urdf(monkeypatch, calibration):
    from types import SimpleNamespace

    from wheel_perception_core.pipeline import GraspPipeline

    meta, c = context()
    obs = Observation(meta, None, None, 0)
    inp = SimpleNamespace(
        rgb=np.zeros((20, 20, 3), np.uint8),
        depth=np.ones((20, 20)),
        frame=meta["frame_id"],
        diagnostic_only=False,
        reasons=[],
    )
    monkeypatch.setattr("wheel_perception_core.pipeline.prepare_input", lambda _: inp)
    monkeypatch.setattr(
        "wheel_perception_core.pipeline.mask_quality", lambda *_: dict(accepted=True)
    )
    monkeypatch.setattr(
        "wheel_perception_core.pipeline.surface_quality", lambda *_: dict(accepted=True)
    )

    class Robot:
        def transform(self, *_):
            raise AssertionError("CAD fallback must not be used")

    class Segmenter:
        def segment(self, *_):
            return np.ones((20, 20), bool)

    class Estimator:
        def register(self, *_):
            return np.eye(4)

    p = GraspPipeline(
        Robot(),
        Segmenter(),
        Estimator(),
        dict(width=0.4, height=0.22, inset=0.007, below_top=0.006),
        [],
        base="arm_origin",
        camera_transform=CalibratedHeadTransform(calibration),
    )
    out = p.process(obs, {}, target_id="box", box=[0, 0, 20, 20], transform_context=c)
    np.testing.assert_allclose(out["pose"], calibration.reference)
    assert out["frame_id"] == "arm_origin" and out["diagnostic_only"] and not out["executable"]
    with pytest.raises(ValueError, match="frame"):
        GraspPipeline(
            None, None, None, {}, [], camera_transform=CalibratedHeadTransform(calibration)
        )
