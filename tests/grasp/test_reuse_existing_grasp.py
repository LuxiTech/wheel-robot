"""Reuse existing resources without hiding stale vision or inventing force acceptance."""

import copy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from test_visual_grasp_flow import JointFeedback
from wheel_execution.grasp_feedback import HeadModelClearanceSource
from wheel_execution.reuse_existing import (
    REUSE_EXISTING,
    ObservedBoxFollowVerifier,
    SDKConfiguredGrippers,
    runtime_blockers,
)
from wheel_execution.stationary_pick import create_stationary_pick
from wheel_execution.visual_servo import visual_packet

ROOT = Path(__file__).resolve().parents[2]


def test_existing_head_and_measured_rail_geometry_keeps_one_lift_shift_and_image_identity():
    measured = list(range(16))
    heights = [100000]
    used_angles = []

    def values(angles, height):
        used_angles.append(angles)
        return height

    def fk(height):
        pose = np.eye(4)
        pose[2, 3] = height + 0.8
        return {side: pose.copy() for side in ("left", "right")}

    geometry = SimpleNamespace(
        model=SimpleNamespace(values=values),
        ik=SimpleNamespace(fk=fk),
        mounts={side: np.eye(4) for side in ("left", "right")},
        rail_points_tool={
            side: np.array([[0, 0, -0.1], [0.1, 0, -0.2]]) for side in ("left", "right")
        },
    )
    driver = SimpleNamespace(
        planner=geometry,
        trace=None,
        measured=lambda: (
            dict(arms=dict(position=measured, receive_mono_ns="123")),
            dict(current_um=heights[0]),
        ),
    )
    source = HeadModelClearanceSource(driver, clearance_m=0.01, uncertainty_m=0.005)
    target = annotated_target()
    target["rim_top"] = {side: [0, 0, 0.5] for side in ("left", "right")}
    first = source(target)
    heights[0] = 150000
    second = source(target)
    for result in (first, second):
        assert result["accepted"] is True
        assert result["image_receive_mono_ns"] == target["image_receive_mono_ns"]
        assert result["observed_at_monotonic"] == 10.0
        assert result["pair_ids"]["left"] == "live"
        assert result["remaining_downward_m"]["left"] == pytest.approx(0.085)
    assert used_angles == [measured, measured]


def annotated_target():
    return dict(
        target_id="crate",
        pair_id="live",
        image_receive_mono_ns=123456,
        frame_id="arm_origin",
        tool_frame="cad_contact_midpoint_pinch_y",
        observed_at_monotonic=10.0,
        object_pose=np.eye(4).tolist(),
        diagnostic_only=True,
        execution_blockers=[
            "selected_calibration:existing",
            "stationary_lift_host_receive_alignment",
            "sdk_to_urdf_and_physical_tcp_not_registered",
            "metric_depth_semantics_unvalidated",
        ],
    )


def test_user_confirmed_resource_annotations_do_not_mutate_acceptance_or_timestamp():
    target = annotated_target()
    original = copy.deepcopy(target)
    packet = visual_packet(target, {}, resource_policy=REUSE_EXISTING)
    assert packet["image_receive_mono_ns"] == 123456
    assert target == original
    with pytest.raises(ValueError, match="unaccepted"):
        visual_packet(target, {})


@pytest.mark.parametrize(
    "reason",
    ["visual_surface_quality_rejected", "saved_target_is_replay_not_live", "new_unknown_fault"],
)
def test_real_vision_failures_are_never_waived(reason):
    target = annotated_target()
    target["execution_blockers"].append(reason)
    with pytest.raises(ValueError, match="unaccepted"):
        visual_packet(target, {}, resource_policy=REUSE_EXISTING)


def test_runtime_wrist_spread_is_preserved_while_old_acceptance_notes_are_informational():
    reasons = [
        "local_top_rim_correspondence_unconfirmed",
        "optical_to_rail_correspondence_unconfirmed",
        "aligned_depth_metric_semantics_unconfirmed",
        "rim_height_spread_too_large",
    ]
    assert runtime_blockers(reasons, REUSE_EXISTING) == ["rim_height_spread_too_large"]


@pytest.mark.parametrize(
    "config_name", ["grasp_execution.json", "grasp_execution.laptop-legacy.json"]
)
def test_live_factory_reuses_original_false_flags_without_contact_policy(
    tmp_path, config_name, monkeypatch
):
    joints = JointFeedback()
    joints.http = SimpleNamespace(base_url="http://unused")
    model = json.loads((ROOT / "config/robots/robot_model.json").read_text())
    contacts = json.loads((ROOT / model["contacts"]).read_text())
    mapping = json.loads((ROOT / model["gripper"]).read_text())["sides"]
    original = copy.deepcopy(contacts)
    configuration = json.loads((ROOT / "config/robots" / config_name).read_text())
    if config_name == "grasp_execution.json":
        import trimesh

        configuration["moveit_crate_mesh"] = "fixture"
        load_mesh = trimesh.load
        monkeypatch.setattr(
            trimesh,
            "load",
            lambda path, **kwargs: (
                SimpleNamespace(vertices=[], faces=[])
                if path == "fixture"
                else load_mesh(path, **kwargs)
            ),
        )
    configuration["resource_policy"] = REUSE_EXISTING
    executor = create_stationary_pick(
        target_source=lambda identity: annotated_target(),
        joints=joints,
        lift_client=None,
        urdf=ROOT / model["urdf"],
        sdk_chains=ROOT / model["sdk_chains"],
        contacts=contacts,
        collision_check=None,
        contact_policy=None,
        wrist_geometry_validated=False,
        metric_depth_validated=False,
        clearance_m=0.01,
        uncertainty_m=0.005,
        output=tmp_path,
        gripper_config=mapping,
        execution_config=configuration,
        max_insertion_m=0.2,
    )
    try:
        assert executor.resource_policy == REUSE_EXISTING
        assert executor.wrist_stop.resource_policy == REUSE_EXISTING
        assert contacts == original and contacts["hardware_validated"] is False
        assert isinstance(executor.close_verify.__self__, SDKConfiguredGrippers)
        assert executor.driver.trace is not None
        if config_name == "grasp_execution.json":
            from wheel_execution.moveit_driver import MoveItTrajectoryDriver

            assert isinstance(executor.driver, MoveItTrajectoryDriver)
        assert joints.moves == []
    finally:
        executor.driver.finish()


@pytest.mark.parametrize("delta_z,expected", [(0, False), (0.01, True), (0.03, False)])
def test_demo_success_uses_actual_box_following_not_closed_motor_or_fake_contact(delta_z, expected):
    before = annotated_target()
    after = copy.deepcopy(before)
    after["pair_id"] = "new"
    after["object_pose"][2][3] = delta_z
    verifier = ObservedBoxFollowVerifier(
        lambda identity: after, SimpleNamespace(clock=lambda: 10.5, trace=None)
    )
    result = verifier(
        before, dict(bilateral_closure_observed=True, bilateral_contact_verified=False)
    )
    assert result["box_followed_both_hands"] is expected
    assert result["force_contact_calibrated"] is False


class ClosingDriver:
    def __init__(self, *, empty=False):
        self.now = 0.0
        self.trace = None
        self.empty = empty
        self.targets = []
        self.state = dict(
            arms=dict(
                position=[0.0] * 16, velocity=[0.0] * 16, effort=[0.0] * 16, receive_mono_ns="100"
            ),
            arm_startup=dict(nominal_target=[0.0] * 16),
        )
        for index in (7, 15):
            self.state["arms"]["position"][index] = -0.45
            self.state["arm_startup"]["nominal_target"][index] = -0.42

    def clock(self):
        return self.now

    def measured(self):
        return copy.deepcopy(self.state), {}

    def move_until(self, target, callback):
        self.targets.append(target)
        if self.empty:
            for index in (7, 15):
                self.state["arms"]["position"][index] = target[index]
            return dict(contact_stop=False, state=copy.deepcopy(self.state))
        for tick in range(8):
            self.now += 0.1
            self.state["arms"]["receive_mono_ns"] = str(int(self.now * 1e9))
            # First the right jaw stops, then the left. Unchanged arm references stay held.
            self.state["arms"]["velocity"][15] = 0.2 if len(self.targets) == 1 else 0.0
            if callback(self.state):
                return dict(contact_stop=True, state=copy.deepcopy(self.state))
        raise AssertionError("Expected measured jaw stop")


@pytest.mark.parametrize("empty", [False, True])
def test_configured_closing_preserves_first_jaw_hold_and_does_not_claim_force_contact(empty):
    driver = ClosingDriver(empty=empty)
    grippers = SDKConfiguredGrippers(
        driver, {s: dict(configured_closed_rad=0.0) for s in ("left", "right")}
    )
    result = grippers.close({})
    assert result["bilateral_closure_observed"] is True
    assert result["bilateral_contact_verified"] is False
    if not empty:
        assert len(driver.targets) == 2
        assert driver.targets[1][7] == -0.42
        assert driver.targets[1][15] == 0
    assert all(target[i] == 0 for target in driver.targets for i in range(16) if i not in (7, 15))
