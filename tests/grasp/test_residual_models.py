"""Protect coordinate meaning and uncertainty handling in offline residual fits."""

import copy
import json
from pathlib import Path

import numpy as np
import pytest
from grasp import compare_residual_models as models
from grasp import validate_residual_models as validation
from grasp.collect_overlay_residuals import submit_when_stationary
from wheel_motion_core.sdk_kinematics import SDKKinematics

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def context():
    calibration = json.loads(
        (ROOT / "config/robots/calibration/head_camera_j4_rezero_20261104.json").read_text()
    )
    sdk = SDKKinematics(ROOT / "config/robots/calibration/sdk_arm_chains_claw14.json")
    return models.Context(calibration, sdk), sdk


def test_vectorized_joint_order_and_outside_nominal_angles_match_sdk(context):
    ctx, sdk = context
    rng = np.random.default_rng(20261105)
    q = rng.uniform(-1.6, 1.6, size=(12, 7))
    rotation, translation = ctx.fk(q)
    for index, angles in enumerate(q):
        full = np.r_[np.zeros(8), angles, 0]
        expected = sdk.transform("left_ee_control_point", sdk.from_sdk_vector(full))
        np.testing.assert_allclose(rotation[index], expected[:3, :3], atol=1e-14)
        np.testing.assert_allclose(translation[index], expected[:3, 3], atol=1e-14)


def test_board_mount_fit_cannot_move_gripper_or_copy_left_offsets_to_right(context):
    ctx, sdk = context
    q = [0.0] * 16
    q[3], q[11] = 1.5, 1.4
    height = 167206
    camera = ctx.camera.copy()
    camera[2, 3] -= height * 1e-6 - ctx.reference
    values = sdk.from_sdk_vector(q)
    row = dict(
        measured_q16=q,
        height_um=height,
        side="right",
        wrist=sdk.transform("right_ee_control_point", values),
        camera_origin=camera,
    )
    spec = models.Spec("diagnostic", camera="rigid", joint="offset", mount=True)
    model = models.Model(ctx, spec)
    model.value = np.zeros(len(model.names))
    baseline, _ = validation.prediction(row, model)
    model.value[model.slices["joint_offsets"]] = 0.04
    model.value[model.slices["board_translation"]] = 0.02
    model.value[model.slices["board_rotation"]] = 0.03
    unaffected, _ = validation.prediction(row, model)
    np.testing.assert_allclose(unaffected, baseline, atol=1e-14)
    row["side"] = "left"
    row["wrist"] = sdk.transform("left_ee_control_point", values)
    with_mount, _ = validation.prediction(row, model)
    model.value[model.slices["board_translation"]] = 0
    model.value[model.slices["board_rotation"]] = 0
    without_mount, _ = validation.prediction(row, model)
    np.testing.assert_allclose(with_mount, without_mount, atol=1e-14)
    assert not np.allclose(with_mount, np.eye(4))


def test_known_joint_offset_is_recovered_without_modifying_measured_q(context):
    ctx, _ = context
    rng = np.random.default_rng(1)
    q = rng.uniform(-0.3, 0.3, size=(12, 7))
    q[:, 3] += 1.4
    pairs = [dict(group=i, height_group=i % 3) for i in range(len(q))]
    data = dict(
        q=q,
        h=np.linspace(-0.03, 0.16, len(q)),
        rays=np.zeros((len(q), 88, 2)),
        focal=np.full((len(q), 2), 388.0),
        pairs=pairs,
    )
    reference = copy.deepcopy(data)
    model = models.Model(ctx, models.Spec("synthetic", joint="individual", individual=4))
    prediction = model.predict(data, np.array([0.023]))
    points = prediction["camera_points"]
    data["rays"] = points[:, :, :2] / points[:, :, 2:]
    fitted = model.fit(data, 100)
    assert fitted["success"]
    np.testing.assert_allclose(model.value, [0.023], atol=1e-7)
    np.testing.assert_array_equal(data["q"], reference["q"])
    restored = models.Model.restore(ctx, model.export())
    np.testing.assert_allclose(
        restored.predict(data)["camera_points"], model.predict(data)["camera_points"]
    )


def test_outside_image_projection_has_no_invented_pixel_distance():
    mask = np.zeros((32, 32), np.uint8)
    observed = mask.copy()
    observed[10:20, 10:20] = 1
    result = validation.silhouette_metrics(mask, observed)
    assert result["projection_status"] == "outside_image"
    assert result["silhouette_IoU"] == 0
    assert result["symmetric_boundary_p95_px"] is None


def test_local_gripper_installation_hypothesis_does_not_rotate_other_arm(context):
    ctx, sdk = context
    q = [0.0] * 16
    q[3], q[11] = 1.4, 1.3
    height = 81814
    model = validation.GripperMountModel(ctx, models.Spec("left_contour_gripper_mount"))
    model.value = np.array([0.02, 0, 0, 0.01, 0, 0])
    values = sdk.from_sdk_vector(q)
    for side in ("left", "right"):
        row = dict(
            measured_q16=q,
            height_um=height,
            side=side,
            wrist=sdk.transform(side + "_ee_control_point", values),
            camera_origin=ctx.camera,
        )
        delta, _ = validation.prediction(row, model)
        if side == "left":
            assert not np.allclose(delta, np.eye(4))
        else:
            np.testing.assert_allclose(delta, np.eye(4), atol=1e-14)


def test_uncertain_submitted_motion_is_never_retried(monkeypatch):
    import grasp.collect_overlay_residuals as collector

    monkeypatch.setattr(collector, "wait_stationary", lambda client: None)

    class Client:
        pending_id = None
        calls = 0

        def move(self, target, speed):
            self.calls += 1
            self.pending_id = "submitted"
            raise RuntimeError("Arms are not stationary")

    client = Client()
    with pytest.raises(RuntimeError, match="not stationary"):
        submit_when_stationary(client, [0] * 16, 0.03)
    assert client.calls == 1


def test_only_known_local_unsubmitted_rejection_can_retry(monkeypatch):
    import grasp.collect_overlay_residuals as collector

    monkeypatch.setattr(collector, "wait_stationary", lambda client: None)

    class Client:
        pending_id = None
        calls = 0

        def move(self, target, speed):
            self.calls += 1
            if self.calls == 1:
                raise RuntimeError("Arms are not stationary")
            return {"accepted": True}

    client = Client()
    assert submit_when_stationary(client, [0] * 16, 0.03)["accepted"]
    assert client.calls == 2
