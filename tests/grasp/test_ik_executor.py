"""Live-coordinate IK and staged execution against in-memory robot doubles."""

from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
from wheel_execution.ik_executor import SDKIKExecutor
from wheel_motion_core.live_kinematics import SDKAlignedModel
from wheel_sdk_bridge.http_client import SDKError

ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / "assets/robots/r680_lift_openarm_claw1.3/r680_lift_openarm_robot_viewer.urdf"
CHAINS = ROOT / "config/robots/calibration/sdk_arm_chains.json"
LOW = [
    -1.3,
    -0.174533,
    -1.570796,
    0,
    -1.570796,
    -0.785398,
    -1.570796,
    -1.05,
    -2,
    -2,
    -1.570796,
    0,
    -1.570796,
    -0.785398,
    -1.570796,
    -0.68,
]
HIGH = [
    2,
    2,
    1.570796,
    2.443461,
    1.570796,
    0.785398,
    1.570796,
    1.42,
    1.396263,
    0.174533,
    1.570796,
    2.443461,
    1.570796,
    0.785398,
    1.570796,
    1.89,
]
Q = [
    -0.0528,
    0.0299,
    0.0044,
    0.2539,
    0.078,
    0.1265,
    -0.5488,
    0.01,
    -0.3359,
    -0.0277,
    0.0826,
    0.1833,
    -0.1123,
    -0.0483,
    0.5461,
    0.02,
]


class Arms:
    def __init__(self, events):
        self.events = events
        self.pending_id = None
        self.fail = False
        self.target = None
        self.s = dict(
            arms=dict(position=Q.copy(), velocity=[0] * 16, enabled=True),
            arm_startup=dict(
                control_generation="1",
                nominal_target=Q.copy(),
                joint_control=dict(lower=LOW, upper=HIGH, phase="ready"),
            ),
        )

    def snapshot(self):
        return deepcopy(self.s)

    def prepare(self, q, speed):
        assert all(lo <= v <= hi for lo, v, hi in zip(LOW, q, HIGH))

    def move(self, q, speed):
        self.events.append("arms_move")
        self.target = q
        self.pending_id = "arm-1"
        return dict(request_id=self.pending_id, accepted=True, completed=False)

    def result(self, rid):
        if self.fail:
            raise SDKError("feedback lost")
        self.s["arms"]["position"] = self.target.copy()
        self.s["arm_startup"]["nominal_target"] = self.target.copy()
        self.pending_id = None
        return dict(completed=True, phase="ready")

    def stop(self, rid):
        self.events.append("arms_stop")
        self.pending_id = None
        return dict(success=True)


class Lift:
    def __init__(self, events):
        self.events = events
        self.owner = None
        self.target = None
        self.fail = False
        self.on_result = None
        self.s = dict(
            lift=dict(current_um=105738, process_started_mono_ns="1"),
            lift_status=dict(stationary=True),
        )

    def snapshot(self):
        return deepcopy(self.s)

    def open(self):
        self.events.append("lift_open")
        self.owner = "mine"

    def move(self, h):
        self.events.append("lift_move")
        self.target = h
        return dict(task_id=1)

    def result(self, tid):
        if self.on_result:
            self.on_result()
        if self.fail:
            raise SDKError("lift fault")
        self.s["lift"]["current_um"] = round(self.target * 1e6)
        return dict(completed=True)

    def _active(self):
        assert self.owner

    def _owned(self, s):
        assert self.owner

    def stop(self):
        self.events.append("lift_stop")
        self.owner = None
        return dict(stopped=True)


def executor(tmp_path):
    events = []
    a = Arms(events)
    lift_client = Lift(events)
    return SDKIKExecutor(a, lift_client, URDF, CHAINS, output=tmp_path), a, lift_client, events


def plan(e):
    return e.plan_relative([0, 0.05, 0.06], [0, 0, 0.03], lift_delta_bounds=(0.025, 0.035))


def test_urdf_and_sdk_fk_match_with_encoder_lift():
    m = SDKAlignedModel(URDF, CHAINS, LOW, HIGH)
    for h in [-0.18, 0, 0.18]:
        values = m.values(Q, h)
        actual = m.ik().fk(values)
        ref = m.sdk.fk(values)
        for side in ["left", "right"]:
            ref[side][2, 3] += h
            assert np.allclose(actual[side], ref[side], atol=1e-10)
    with pytest.raises(ValueError):
        m.transform("openarm_left_ee_base_link", values, base="base_footprint")


def test_full_gripper_uses_each_sdk_base_and_measured_joint7():
    m = SDKAlignedModel(URDF, CHAINS, LOW, HIGH)
    values = m.values(Q, 0)
    for side in ("left", "right"):
        frame = f"openarm_{side}_ee_link1"
        finger = f"openarm_{side}_finger_joint1"
        with pytest.raises(ValueError, match="Missing live joint"):
            m.sdk.gripper_transform(m.urdf, side, frame, values)
        values[finger] = 0.03
        actual = m.sdk.gripper_transform(m.urdf, side, frame, values)
        expected = m.sdk.transform(f"openarm_{side}_base_link") @ m.urdf.transform(
            frame, values, base=f"openarm_{side}_base_link"
        )
        np.testing.assert_allclose(actual, expected, atol=1e-10)
        local = np.linalg.inv(m.sdk.fk(values)[side]) @ actual
        changed = dict(values)
        changed[f"openarm_{side}_joint7"] += 0.1
        moved = m.sdk.gripper_transform(m.urdf, side, frame, changed)
        assert not np.allclose(moved, actual)
        np.testing.assert_allclose(
            np.linalg.inv(m.sdk.fk(changed)[side]) @ moved, local, atol=1e-10
        )
    with pytest.raises(ValueError):
        m.sdk.gripper_transform(m.urdf, "left", "openarm_right_ee_link1", values)
    # A single left-derived chassis registration cannot place the right gripper.
    transfer = m.sdk.transform("openarm_left_base_link") @ np.linalg.inv(
        m.urdf.transform("openarm_left_base_link", {}, base="lift_carriage_link")
    )
    wrong = transfer @ m.urdf.transform("openarm_right_ee_link1", values, base="lift_carriage_link")
    right = m.sdk.gripper_transform(m.urdf, "right", "openarm_right_ee_link1", values)
    assert np.linalg.norm(wrong[:3, 3] - right[:3, 3]) == pytest.approx(0.12, abs=0.001)


def test_joint_ik_solves_lift_and_arms_then_executes(tmp_path):
    e, a, lift_client, events = executor(tmp_path)
    p = plan(e)
    assert not events
    assert 0.025 <= p["lift_delta_m"] <= 0.035
    assert max(abs(v) for v in p["arm_delta_deg"].values()) > 10
    assert max(p["position_errors_m"].values()) < 0.002
    assert p["arm_target"][7] == Q[7] and p["arm_target"][15] == Q[15]
    r = e.execute(p["plan_id"], scene_clear=True)
    assert r["completed"] and max(r["position_errors_m"].values()) < 1e-4
    assert events == ["lift_open", "lift_move", "arms_move", "lift_stop"]
    assert not lift_client.owner
    assert (tmp_path / "execution.json").exists()


def test_no_writes_without_scene_clear_or_for_unreachable_target(tmp_path):
    e, a, lift_client, events = executor(tmp_path)
    p = plan(e)
    with pytest.raises(ValueError, match="clearance"):
        e.execute(p["plan_id"])
    with pytest.raises(ValueError):
        e.plan_relative([5, 0, 0], [0, 0, 0])
    assert not events


def test_plan_is_immutable_at_execution_boundary(tmp_path):
    e, a, lift_client, events = executor(tmp_path)
    p = plan(e)
    p["arm_target"][8] = 100
    assert e.execute(p["plan_id"], scene_clear=True)["completed"]
    assert abs(a.target[8]) < 2


@pytest.mark.parametrize("change", ["arm", "lift", "owner", "expired"])
def test_changed_state_or_expired_plan_does_not_write(tmp_path, change):
    e, a, lift_client, events = executor(tmp_path)
    p = plan(e)
    if change == "arm":
        a.s["arms"]["position"][8] += 0.02
    if change == "lift":
        lift_client.s["lift"]["current_um"] += 1000
    if change == "owner":
        a.s["arm_startup"]["control_generation"] = "other"
    if change == "expired":
        old = e._plans[p["plan_id"]]
        e._plans[p["plan_id"]] = (old[0] - 31, *old[1:])
    with pytest.raises(SDKError):
        e.execute(p["plan_id"], scene_clear=True)
    assert not events


@pytest.mark.parametrize("stage", ["lift", "arms", "cancel"])
def test_failure_and_cancellation_stop_owned_channels(tmp_path, stage):
    e, a, lift_client, events = executor(tmp_path)
    p = plan(e)
    if stage == "lift":
        lift_client.fail = True
    if stage == "arms":
        a.fail = True
    if stage == "cancel":
        lift_client.on_result = e.cancel
    with pytest.raises(SDKError):
        e.execute(p["plan_id"], scene_clear=True)
    assert events[-1] == "lift_stop" and not lift_client.owner
    if stage == "arms":
        assert "arms_stop" in events
    else:
        assert "arms_move" not in events


def test_sdk_limits_intersect_urdf_limits():
    m = SDKAlignedModel(URDF, CHAINS, LOW, HIGH)
    assert m.joints["openarm_left_joint1"]["bounds"][0] == -2
    bad = LOW.copy()
    bad[8] = 10
    with pytest.raises(ValueError):
        SDKAlignedModel(URDF, CHAINS, bad, HIGH)


def test_existing_control_request_is_not_stopped_by_rejected_ik(tmp_path):
    e, a, lift_client, events = executor(tmp_path)
    p = plan(e)
    a.pending_id = "previous"
    lift_client.owner = "previous"
    with pytest.raises(SDKError, match="Existing control"):
        e.execute(p["plan_id"], scene_clear=True)
    assert not events and a.pending_id == "previous" and lift_client.owner == "previous"
