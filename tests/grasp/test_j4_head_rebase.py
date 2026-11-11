"""Historical J4 conversion preserves raw data and cannot be silently applied twice."""

import copy
import json
from pathlib import Path

import numpy as np
import pytest
from grasp.recalibrate_j4_head import rebase_pairs, zero_changes
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics

ROOT = Path(__file__).resolve().parents[2]


def inputs():
    record = json.loads((ROOT / "config/robots/calibration/j4_rezero_20261104.json").read_text())
    old = SDKKinematics(ROOT / "config/robots/calibration/sdk_arm_chains.json")
    current = SDKKinematics(ROOT / "config/robots/calibration/sdk_arm_chains_claw14.json")
    model = RobotModel(ROOT / record["urdf_path"])
    q = np.zeros(16)
    q[3], q[11] = 0.7, 1.1
    q[12] = -1.6  # Math must retain recorded feedback even beyond a nominal CAD limit.
    pair = dict(
        q=q.tolist(),
        A=old.transform("left_ee_control_point", old.from_sdk_vector(q)).tolist(),
        B=np.eye(4).tolist(),
        height_m=0.081814,
        group=2,
        corners=[[12, 34]],
    )
    return record, old, current, model, pair


def test_rebase_uses_actual_encoder_change_recomputes_fk_and_keeps_raw_observation():
    record, old, current, model, pair = inputs()
    before = copy.deepcopy(pair)
    converted, audit = rebase_pairs([pair], record, old, current, model)
    result = converted[0]
    expected = np.array(before["q"])
    for item in record["joints"]:
        expected[item["sdk_index"]] += (
            item["new_sdk_rad_after_stamp"] - item["old_sdk_rad_at_hardware_stamp"]
        )
    np.testing.assert_array_equal(result["q"], expected)
    np.testing.assert_array_equal(
        result["A"], current.transform("left_ee_control_point", current.from_sdk_vector(expected))
    )
    assert not np.allclose(result["A"], before["A"])
    assert pair == before
    assert result["q_before_j4_rezero"] == before["q"]
    for key in ("B", "corners", "group", "height_m"):
        assert result[key] == before[key]
    assert audit["measured_angles_clipped"] is False
    assert audit["recorded_angles_outside_urdf_limits"]["openarm_left_joint5"] == 1
    assert model.joints["openarm_left_joint5"]["bounds"] is not None


def test_double_conversion_and_stale_cached_fk_are_rejected():
    record, old, current, model, pair = inputs()
    converted, _ = rebase_pairs([pair], record, old, current, model)
    with pytest.raises(ValueError, match="double rebase"):
        rebase_pairs(converted, record, old, current, model)
    pair["A"][0][3] += 0.001
    with pytest.raises(ValueError, match="coordinate chains disagree"):
        rebase_pairs([pair], record, old, current, model)


def test_design_stop_estimate_cannot_replace_observed_coordinate_change():
    record, *_ = inputs()
    record["joints"][0]["observed_feedback_coordinate_shift_rad"] = -record["joints"][0][
        "design_zero_in_old_sdk_rad"
    ]
    with pytest.raises(ValueError, match="before/after measured"):
        zero_changes(record)
