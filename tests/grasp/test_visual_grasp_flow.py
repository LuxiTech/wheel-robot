"""Software integration on synthetic feedback; never hardware acceptance."""

import copy
import json
from pathlib import Path

import numpy as np
import pytest
from test_ik_executor import HIGH, LOW, Q
from wheel_execution.grasp_executor import (
    SDKContactCloser,
    SDKGripperOpener,
    SDKWaypointDriver,
    StationaryGraspExecutor,
)
from wheel_motion_core.visual_grasp import VisualGraspPlanner
from wheel_perception_core.targets import visual_target
from wheel_task_core.grasp import assign_candidates

ROOT = Path(__file__).resolve().parents[2]


def planner(check=lambda values: True):
    return VisualGraspPlanner(
        ROOT / "assets/robots/r680_lift_openarm_claw1.4/r680_lift_openarm_robot_viewer.urdf",
        ROOT / "config/robots/calibration/sdk_arm_chains_claw14.json",
        json.loads((ROOT / "config/robots/calibration/claw14_contacts.json").read_text()),
        LOW,
        HIGH,
        collision_check=check,
    )


def nearby_target(p, pair="first"):
    current = p.ik.fk(p.model.values(Q, 0.1))
    target = dict(
        target_id="crate",
        pair_id=pair,
        frame_id="arm_origin",
        tool_frame="cad_contact_midpoint_pinch_y",
        observed_at_monotonic=0.0,
        diagnostic_only=False,
        execution_blockers=[],
        pregrasp={},
        grasp={},
    )
    for side in current:
        tool = current[side] @ p.mounts[side]
        tool[2, 3] -= 0.1
        target["pregrasp"][side] = tool.copy()
        target["grasp"][side] = tool.copy()
        target["grasp"][side][2, 3] -= 0.002
    return target


class JointFeedback:
    def __init__(self):
        self.pending_id = None
        self.moves = []
        self.stops = []
        self.state = dict(
            arms=dict(position=list(Q), velocity=[0.0] * 16, effort=[0.0] * 16, enabled=True),
            arm_startup=dict(
                control_generation="same",
                nominal_target=list(Q),
                joint_control=dict(lower=LOW, upper=HIGH),
            ),
        )

    def snapshot(self):
        return copy.deepcopy(self.state)

    def move(self, target, speed):
        assert self.pending_id is None
        assert speed == 0.03
        self.moves.append(list(target))
        self.pending_id = "owned-" + str(len(self.moves))
        return dict(request_id=self.pending_id)

    def result(self, request_id):
        assert request_id == self.pending_id
        self.state["arms"]["position"] = self.moves[-1].copy()
        self.state["arm_startup"]["nominal_target"] = self.moves[-1].copy()
        self.pending_id = None
        return dict(phase="ready", completed=True)

    def stop(self, request_id):
        self.stops.append(request_id)
        return dict(success=True)


def setup_runner(tmp_path, *, bad_wrist=False, bad_lift=False):
    p = planner()
    joints = JointFeedback()
    height = dict(current_um=100000, stationary=True)
    driver = SDKWaypointDriver(
        joints, lambda: height, p, clock=lambda: 0.0, sleep=lambda value: None
    )
    targets = iter([nearby_target(p), nearby_target(p, "second")])
    observed = []
    counter = [0]
    calls = []

    def observe(target_id):
        observed.append(target_id)
        return next(targets)

    def wrist(target):
        counter[0] += 1
        return dict(
            accepted=not bad_wrist,
            blockers=["bad geometry"] if bad_wrist else [],
            pair_ids={side: side + str(counter[0]) for side in ("left", "right")},
            observed_at_monotonic=0.0,
            remaining_downward_m={
                side: 0.002 if counter[0] == 1 else 0 for side in ("left", "right")
            },
        )

    def opened(target):
        calls.append("open")
        return dict(bilateral_open_verified=True)

    def close(target):
        calls.append("close")
        return dict(bilateral_contact_verified=True)

    def lift(target, contact):
        calls.append("verify_lift")
        return dict(box_followed_both_hands=not bad_lift)

    runner = StationaryGraspExecutor(
        p,
        driver,
        observe,
        wrist,
        close,
        lift,
        open_verify=opened,
        output=tmp_path,
        clock=lambda: 0.0,
    )
    return runner, joints, observed, calls


def test_assignment_uses_sdk_y_not_camera_candidate_names():
    a, b = np.eye(4), np.eye(4)
    a[1, 3], b[1, 3] = -0.2, 0.2
    result = assign_candidates({"A": a, "B": b}, {"A": a, "B": b})
    assert result["left"]["candidate"] == "B"
    with pytest.raises(ValueError, match="distinct"):
        assign_candidates({"A": a, "B": a}, {"A": a, "B": a})


def test_tool_to_wrist_conversion_preserves_requested_tool_pose():
    p = planner()
    target = nearby_target(p)
    actual = p.wrist_targets(target, "grasp", 0.1)
    shift = np.eye(4)
    shift[2, 3] = 0.1
    for side in actual:
        np.testing.assert_allclose(
            actual[side] @ p.mounts[side], shift @ target["grasp"][side], atol=1e-12
        )
        assert np.linalg.norm(p.mounts[side][:3, 3]) > 0.15


def test_fixed_lift_plan_and_collision_rejection():
    p = planner()
    result = p.plan(nearby_target(p), Q, 0.1, steps=2)
    assert result["execution_ready"]
    assert set(result["stages"]) == {"pregrasp", "grasp"}
    for stage in result["stages"].values():
        assert all(point["joints"]["lift_joint"] == 0.1 for point in stage["points"])
        assert all(q[7] == Q[7] and q[15] == Q[15] for q in stage["arm_targets"])
    p = planner(lambda values: False)
    rejected = p.plan(nearby_target(p), Q, 0.1, steps=2)
    assert not rejected["execution_ready"]
    assert "collision" in rejected["errors"]["pregrasp"]


def test_full_pick_sequence_uses_real_ik_and_sdk_driver_with_synthetic_feedback(tmp_path):
    runner, joints, observed, calls = setup_runner(tmp_path)
    result = runner.run("crate")
    assert result["completed"]
    phases = [event["phase"] for event in runner.events]
    assert phases == [
        "PREGRASP",
        "REOBSERVE",
        "ALIGNED",
        "OPEN",
        "OPEN_CONFIRMED",
        "APPROACH",
        "RAIL_STOP",
        "CLOSE",
        "TEST_LIFT",
        "COMPLETE",
    ]
    assert observed == ["crate", "crate"]
    assert calls == ["open", "close", "verify_lift"]
    assert len(joints.moves) > 20
    assert all(q[7] == Q[7] and q[15] == Q[15] for q in joints.moves)


def test_wrist_rejection_prevents_closure_and_trial_lift(tmp_path):
    runner, joints, _, calls = setup_runner(tmp_path, bad_wrist=True)
    with pytest.raises(ValueError, match="Wrist insertion"):
        runner.run("crate")
    assert calls == ["open"]
    assert not runner.may_be_holding
    assert runner.events[-1]["phase"] == "BLOCKED"
    assert not joints.stops  # No global stop while already holding a completed request.


def test_failed_trial_lift_retains_possible_load(tmp_path):
    runner, _, _, calls = setup_runner(tmp_path, bad_lift=True)
    with pytest.raises(ValueError, match="test-lift"):
        runner.run("crate")
    assert runner.may_be_holding
    assert runner.events[-1]["phase"] == "HOLD"
    assert calls == ["open", "close", "verify_lift"]


def test_diagnostic_target_never_submits_motor_commands(tmp_path):
    runner, joints, _, _ = setup_runner(tmp_path)
    packet = nearby_target(runner.planner)
    packet.update(diagnostic_only=True, execution_blockers=["unaccepted geometry"])
    runner.observe = lambda target_id: packet
    with pytest.raises(ValueError, match="diagnostic"):
        runner.run("crate")
    assert joints.moves == []


@pytest.mark.parametrize("failure", ["duplicate_pair", "stale_wrist", "overshoot"])
def test_bad_wrist_stream_cannot_reach_close(tmp_path, failure):
    runner, _, _, calls = setup_runner(tmp_path)
    counter = [0]

    def wrist(target):
        counter[0] += 1
        return dict(
            accepted=True,
            blockers=[],
            pair_ids={
                s: s + ("same" if failure == "duplicate_pair" else str(counter[0]))
                for s in ("left", "right")
            },
            observed_at_monotonic=-1.0 if failure == "stale_wrist" else 0.0,
            remaining_downward_m={
                s: -0.02 if failure == "overshoot" else 0.0 for s in ("left", "right")
            },
        )

    runner.wrist_stop = wrist
    with pytest.raises(ValueError):
        runner.run("crate")
    assert calls == ["open"]


def test_gripper_policy_is_required_and_empty_closed_is_not_capture(tmp_path):
    runner, _, _, _ = setup_runner(tmp_path)
    with pytest.raises(ValueError, match="policy"):
        SDKContactCloser(runner.driver, dict(hardware_validated=False))
    policy = dict(hardware_validated=True)
    for side in ("left", "right"):
        policy[side] = dict(
            contact_effort_min_nm=0.2,
            hard_effort_max_nm=1.0,
            contact_angle_min_rad=-0.5,
            contact_angle_max_rad=-0.01,
            step_rad=0.01,
        )
    closer = SDKContactCloser(runner.driver, policy)
    runner.driver.begin()
    # Synthetic fixture Q has a positive empty-closed gripper angle.
    with pytest.raises(ValueError, match="without rim contact"):
        closer({})
    assert runner.driver.feedback_guard is None


def test_visual_target_keeps_quality_and_replay_failures():
    box = np.eye(4)
    # Opposite CAD X walls span SDK Y, so hand assignment is unambiguous.
    box[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    target = visual_target(
        target_id="crate",
        pair_id="camera:1",
        object_pose_origin=box,
        rules=dict(width=0.445, height=0.18, inset=0.007, below_top=0.001),
        height_um=-119999,
        quality=dict(
            silhouette_iou=0.9,
            median_depth_error_m=0.009,
            p95_depth_error_m=0.4,
            metric_depth_semantics_validated=False,
        ),
        source_blockers=["geometry_not_accepted"],
    )
    assert target["diagnostic_only"]
    assert set(target["execution_blockers"]) == {
        "geometry_not_accepted",
        "visual_surface_quality_rejected",
        "metric_depth_semantics_unvalidated",
        "saved_target_is_replay_not_live",
    }
    assert target["grasp"]["left"][1][3] > target["grasp"]["right"][1][3]


def contact_policy():
    result = dict(hardware_validated=True)
    for side in ("left", "right"):
        result[side] = dict(
            contact_effort_min_nm=0.2,
            hard_effort_max_nm=1.0,
            contact_angle_min_rad=-0.5,
            contact_angle_max_rad=-0.01,
            step_rad=0.01,
        )
    return result


def test_contact_guard_remains_active_during_trial_lift(tmp_path):
    runner, joints, _, _ = setup_runner(tmp_path)
    for index in (7, 15):
        joints.state["arms"]["position"][index] = -0.2
        joints.state["arms"]["effort"][index] = 0.3
    runner.driver.begin()
    closer = SDKContactCloser(runner.driver, contact_policy())
    assert closer({})["bilateral_contact_verified"]
    assert not joints.moves
    joints.state["arms"]["effort"][7] = 1.1
    with pytest.raises(RuntimeError, match="hard effort"):
        runner.driver.measured()


@pytest.mark.parametrize("delta_z,expected", [(0.0, False), (0.01, True), (0.03, False)])
def test_lift_requires_observed_box_motion_and_bilateral_contact(tmp_path, delta_z, expected):
    from wheel_execution.grasp_feedback import VisualTestLiftVerifier

    runner, joints, _, _ = setup_runner(tmp_path)
    for index in (7, 15):
        joints.state["arms"]["position"][index] = -0.2
        joints.state["arms"]["effort"][index] = 0.3
    runner.driver.begin()
    target = dict(target_id="crate", pair_id="old", object_pose=np.eye(4).tolist())
    fresh = copy.deepcopy(target)
    fresh.update(
        pair_id="new",
        observed_at_monotonic=0.0,
        frame_id="arm_origin",
        diagnostic_only=False,
        execution_blockers=[],
    )
    fresh["object_pose"][2][3] = delta_z
    verifier = VisualTestLiftVerifier(
        lambda identity: fresh, runner.driver, contact_policy(), clock=lambda: 0.0
    )
    result = verifier(target, dict(bilateral_contact_verified=True))
    assert result["box_followed_both_hands"] is expected


def test_live_composition_rejects_known_geometry_failure_before_IO():
    from wheel_execution.stationary_pick import create_stationary_pick

    with pytest.raises(ValueError, match="wrist geometry"):
        create_stationary_pick(
            target_source=None,
            joints=None,
            lift_client=None,
            urdf=None,
            sdk_chains=None,
            contacts=None,
            collision_check=lambda values: True,
            contact_policy=None,
            wrist_geometry_validated=False,
            metric_depth_validated=False,
            clearance_m=0.01,
            uncertainty_m=0.005,
            execution_config=dict(
                schema="wheel_grasp_execution_v1",
                execution_mode="laptop_visual_servo",
                collision_policy="required",
                resource_policy="strict",
            ),
        )


def test_nan_effort_never_causes_more_closing_commands(tmp_path):
    runner, joints, _, _ = setup_runner(tmp_path)
    runner.driver.begin()
    closer = SDKContactCloser(runner.driver, contact_policy())
    joints.state["arms"]["effort"][7] = float("nan")
    with pytest.raises(RuntimeError, match="Invalid gripper effort"):
        closer({})
    assert joints.moves == []


def test_rail_stop_target_places_actual_underside_above_rim():
    p = planner()
    target = nearby_target(p)
    target.update(
        insertion_reference="sliding_rail_underside",
        rail_stop_clearance_m=0.01,
        pregrasp_vertical_clearance_m=0.03,
        rim_top={s: [0.2, 0.2 if s == "left" else -0.2, -0.1] for s in ("left", "right")},
    )
    resolved = p.resolve_target(target)
    for side in ("left", "right"):
        grasp = np.asarray(resolved["grasp"][side])
        rails = p.rail_points_tool[side] @ grasp[:3, :3].T + grasp[:3, 3]
        assert rails[:, 2].min() - target["rim_top"][side][2] == pytest.approx(0.01)
        alignment = np.asarray(resolved["pregrasp"][side])
        wrist = alignment @ np.linalg.inv(p.mounts[side])
        points = p.gripper_envelope(side) @ wrist[:3, :3].T + wrist[:3, 3]
        assert points[:, 2].min() - max(v[2] for v in target["rim_top"].values()) == pytest.approx(
            0.03
        )
    repeated = p.resolve_target(resolved)
    for stage in ("grasp", "pregrasp"):
        for side in ("left", "right"):
            np.testing.assert_array_equal(repeated[stage][side], resolved[stage][side])


def test_live_vision_adapter_requires_same_pair_lift_context():
    from wheel_perception.service import PlanningTargetSource
    from wheel_task_core.contracts import GraspObservation

    box = np.eye(4)
    box[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    data = dict(
        pose=box,
        quality=dict(accepted=True, surface_rms_m=0.002, surface_p95_m=0.004),
        capture_context=dict(schema="wheel_lift_capture_v1", pair_id="pair:1", height_um=-119999),
    )

    class Vision:
        async def observe(self, target_id):
            return GraspObservation(target_id, "pair:1", "arm_origin", 12.0, True, False, data)

    source = PlanningTargetSource(
        Vision(),
        dict(width=0.445, height=0.18, inset=0.007, below_top=0.001),
        metric_depth_validated=True,
    )
    target = source("crate")
    assert target["height_um"] == -119999
    assert target["diagnostic_only"] is False
    data["capture_context"]["pair_id"] = "other-pair"
    with pytest.raises(ValueError, match="matching visual capture"):
        source("crate")


@pytest.mark.parametrize("old_image", [False, True])
def test_wrist_source_rejects_unaccepted_geometry_or_prearrival_images(tmp_path, old_image):
    import hashlib

    from wheel_execution.grasp_feedback import WristClearanceSource
    from wheel_perception_core.rgbd import Observation

    runner, joints, _, _ = setup_runner(tmp_path)
    joints.state["arms"]["receive_mono_ns"] = "100000000"
    runner.driver.begin()
    runner.driver.last_arrival_robot_ns = 100_000_000
    urdf = ROOT / "assets/robots/r680_lift_openarm_claw1.4/r680_lift_openarm_robot_viewer.urdf"
    source = WristClearanceSource(
        runner.driver,
        urdf,
        hashlib.sha256(urdf.read_bytes()).hexdigest(),
        "http://127.0.0.1:1",
        clearance_m=0.01,
        uncertainty_m=0.005,
        clock=lambda: 0.0,
    )
    target = dict(rim_top={})
    values = runner.planner.model.values(Q, 0.1)
    values["lift_joint"] += 0.2
    for side in ("left", "right"):
        camera = runner.planner.model.sdk.gripper_transform(
            source.robot, side, side + "_wrist_eb435i_color_optical_frame", values
        )
        target["rim_top"][side] = (camera @ [0, 0, 0.4, 1])[:3].tolist()

    def receive(camera_id):
        metadata = dict(
            camera_id=camera_id,
            pair_id=camera_id + ":1",
            frame_id=camera_id + "_color_optical",
            receive_mono_ns="90000000" if old_image else "100000000",
            intrinsics=dict(
                K=[600, 0, 320, 0, 600, 240, 0, 0, 1], D=[0] * 5, distortion_model="plumb_bob"
            ),
            depth=dict(value_semantics="color_optical_z"),
        )
        rgb = np.zeros((480, 640, 3), np.uint8)
        rgb[:] = [0, 60, 200]
        return Observation(metadata, rgb, np.full((480, 640), 0.4, np.float32), 0.0)

    source.client.receive = receive
    if old_image:
        with pytest.raises(ValueError, match="predates"):
            source(target)
    else:
        feedback = source(target)
        assert feedback["accepted"] is False
        assert any("optical_to_rail" in reason for reason in feedback["blockers"])
    assert not joints.moves


def test_scene_change_after_planning_rejects_waypoint_before_motor_POST(tmp_path):
    runner, joints, _, _ = setup_runner(tmp_path)
    plan = runner.planner.plan(nearby_target(runner.planner), Q, 0.1, steps=2)
    assert plan["stages"]["pregrasp"]["collision_checked"]
    runner.driver.begin()
    runner.planner.collision_check = lambda values: False
    with pytest.raises(ValueError, match="collision changed"):
        runner.driver.execute(plan["stages"]["pregrasp"])
    assert not joints.moves


def opener_mapping():
    return {
        side: dict(configured_open_rad=-0.3, configured_closed_rad=0.0, opening_sign=-1)
        for side in ("left", "right")
    }


def test_gripper_opening_keeps_arm_targets_and_confirms_both_feedbacks(tmp_path):
    runner, joints, _, _ = setup_runner(tmp_path)
    runner.driver.begin()
    opener = SDKGripperOpener(
        runner.driver, opener_mapping(), effort_limits=dict(left=1.0, right=1.0)
    )
    result = opener({})
    assert result["bilateral_open_verified"]
    for target in joints.moves:
        for index in list(range(7)) + list(range(8, 15)):
            assert target[index] == Q[index]
    assert abs(joints.state["arms"]["position"][7] + 0.3) <= 0.01
    assert abs(joints.state["arms"]["position"][15] + 0.3) <= 0.01


def test_opening_failure_or_position_change_prevents_downward_insertion(tmp_path):
    runner, _, _, calls = setup_runner(tmp_path)
    runner.open_verify = lambda target: dict(bilateral_open_verified=False)
    runner.wrist_stop = lambda target: pytest.fail(
        "Insertion must not begin before opening confirmation"
    )
    with pytest.raises(ValueError, match="opening not confirmed"):
        runner.run("crate")
    assert calls == []
    assert "APPROACH" not in [event["phase"] for event in runner.events]


def test_moved_target_prevents_opening_after_alignment(tmp_path):
    runner, _, _, calls = setup_runner(tmp_path)
    first = nearby_target(runner.planner)
    moved = nearby_target(runner.planner, "moved")
    moved["pregrasp"]["left"][0, 3] += 0.02
    sequence = iter([first, moved])
    runner.observe = lambda identity: next(sequence)
    with pytest.raises(ValueError, match="no longer aligned"):
        runner.run("crate")
    assert calls == []


def test_opening_obstruction_is_rejected_before_finger_command(tmp_path):
    runner, joints, _, _ = setup_runner(tmp_path)
    runner.driver.begin()
    opener = SDKGripperOpener(
        runner.driver, opener_mapping(), effort_limits=dict(left=1.0, right=1.0)
    )
    runner.planner.collision_check = lambda values: False
    with pytest.raises(ValueError, match="opening sweep collides"):
        opener({})
    assert not joints.moves


def test_clearance_alignment_has_no_diagonal_descent():
    from scipy.spatial.transform import Rotation, Slerp

    p = planner()
    target = nearby_target(p)
    current = p.ik.fk(p.model.values(Q, 0.1))
    lows = []
    for side in ("left", "right"):
        points = p.gripper_envelope(side)
        lows.append(
            float((points @ current[side][:3, :3].T + current[side][:3, 3])[:, 2].min()) - 0.1
        )
        goal = np.asarray(target["pregrasp"][side]).copy()
        goal[0, 3] += 0.02
        goal[:3, :3] = Rotation.from_euler("z", 10, degrees=True).as_matrix() @ goal[:3, :3]
        target["pregrasp"][side] = goal
    rim_height = min(lows) - 0.03
    target["rim_top"] = {side: [0, 0, rim_height] for side in ("left", "right")}
    target["alignment_clearance_m"] = 0.03
    goals = p.alignment_targets(target, Q, 0.1)
    for side in ("left", "right"):
        raised = goals["clearance"][side]
        aligned = goals["align"][side]
        destination = goals["pregrasp"][side]
        np.testing.assert_allclose(raised[:2, 3], current[side][:2, 3])
        np.testing.assert_allclose(raised[:3, :3], current[side][:3, :3])
        assert raised[2, 3] >= current[side][2, 3]
        assert aligned[2, 3] == raised[2, 3]
        np.testing.assert_allclose(aligned[:2, 3], destination[:2, 3])
        rotations = Slerp([0, 1], Rotation.from_matrix([raised[:3, :3], aligned[:3, :3]]))
        for u in np.linspace(0, 1, 11):
            position = (1 - u) * raised[:3, 3] + u * aligned[:3, 3]
            points = p.gripper_envelope(side) @ rotations([u]).as_matrix()[0].T + position
            assert points[:, 2].min() >= rim_height + 0.1 + 0.03 - 1e-10


def test_opening_does_not_move_nominal_grasp_midpoint_between_jaws():
    p = planner()
    contacts = json.loads((ROOT / "config/robots/calibration/claw14_contacts.json").read_text())[
        "sides"
    ]
    for side in ("left", "right"):
        midpoints = []
        for opening in (0.012860000208020216, 0.044, 0.048):
            points = []
            for item in contacts[side]["contacts"]:
                transform = p.gripper_geometry.transform(
                    item["link"],
                    {f"openarm_{side}_finger_joint1": opening},
                    base=f"openarm_{side}_link7",
                )
                points.append((transform @ np.r_[item["point_in_finger_m"], 1])[:3])
            midpoints.append(np.mean(points, axis=0))
        np.testing.assert_allclose(midpoints[0], midpoints[1], atol=1e-12)
        np.testing.assert_allclose(midpoints[0], midpoints[2], atol=1e-12)


def test_insertion_trial_limit_rejects_before_alignment_or_opening(tmp_path):
    runner, joints, _, calls = setup_runner(tmp_path)
    first = nearby_target(runner.planner)
    first["grasp"]["left"][2, 3] -= 0.08
    first["grasp"]["right"][2, 3] -= 0.08
    runner.observe = lambda identity: first
    with pytest.raises(ValueError, match="trial bound before alignment"):
        runner.run("crate")
    assert not joints.moves
    assert calls == []


def test_rotation_clearance_includes_lowest_point_between_endpoints():
    from scipy.spatial.transform import Rotation
    from wheel_motion_core.visual_grasp import lowest_rotation_sweep

    points = np.array([[1.0, 0, 0]])
    end = Rotation.from_euler("y", 120, degrees=True).as_matrix()
    assert lowest_rotation_sweep(points, np.eye(3), end) == pytest.approx(-1.0)
    assert (points @ end.T)[0, 2] > -0.9
