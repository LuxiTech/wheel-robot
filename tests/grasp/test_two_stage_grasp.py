import copy

import pytest
from test_visual_grasp_flow import JointFeedback, nearby_target, planner
from wheel_execution.contact_closer import SDKContinuousContactCloser
from wheel_execution.grasp_executor import SDKGripperOpener, SDKWaypointDriver
from wheel_execution.phase_driver import SDKTrajectoryDriver
from wheel_execution.two_stage_grasp import TwoStageGraspExecutor


class PhaseJoints(JointFeedback):
    def __init__(self):
        super().__init__()
        self.counter = 0
        self.activate_after = 0
        self.activation = None
        self.state["arms"]["receive_mono_ns"] = "100000000"
        self.state["arm_startup"]["joint_control"].update(
            request_id="initial",
            phase="ready",
            reason="initial",
            target=self.state["arms"]["position"].copy(),
            trajectory_protocol=1,
        )

    def snapshot(self):
        self.counter += 1
        if self.activation is not None:
            self.activate_after -= 1
            if self.activate_after <= 0:
                self.activation()
                self.activation = None
        self.state["arms"]["receive_mono_ns"] = str(500_000_000 + self.counter * 1_000_000)
        return copy.deepcopy(self.state)


class PhaseTransport:
    def __init__(self, joints, residuals=None, *, delay=0, terminal="ready"):
        self.joints = joints
        self.submitted = []
        self.stops = []
        self.residuals = list(residuals or [0.0])
        self.delay = delay
        self.terminal = terminal

    def require_supported(self, state=None):
        return state or self.joints.snapshot()

    def submit(self, points, **kwargs):
        assert self.joints.pending_id is None
        self.submitted.append(copy.deepcopy(points))
        request = "phase-" + str(len(self.submitted))
        self.joints.pending_id = request
        residual = self.residuals[min(len(self.submitted) - 1, len(self.residuals) - 1)]

        def activate():
            actual = list(points[-1])
            actual[0] += residual
            self.joints.state["arms"]["position"] = actual
            self.joints.state["arm_startup"]["nominal_target"] = list(points[-1])
            self.joints.state["arm_startup"]["control_generation"] = request
            phase = "needs_correction" if residual > 0.02 else self.terminal
            self.joints.state["arm_startup"]["joint_control"].update(
                request_id=request, phase=phase, reason="test", target=list(points[-1])
            )

        self.joints.activation = activate
        self.joints.activate_after = self.delay
        return dict(request_id=request)

    def result(self, request_id, *, state):
        joint = state["arm_startup"]["joint_control"]
        assert request_id == joint["request_id"]
        completed = joint["phase"] == "ready"
        if joint["phase"] in ("ready", "needs_correction", "fault"):
            self.joints.pending_id = None
        error = max(abs(a - b) for a, b in zip(state["arms"]["position"], joint["target"]))
        return dict(
            request_id=request_id,
            phase=joint["phase"],
            completed=completed,
            max_error_rad=error,
            duration_s=1.0,
            fraction=1.0,
        )

    def stop(self, request_id):
        assert request_id == self.joints.pending_id
        self.stops.append(request_id)
        self.joints.state["arm_startup"]["joint_control"]["phase"] = "stopped"
        return dict(success=True)


def driver_fixture(*, residuals=None, delay=0, terminal="ready", policy="onsite_supervised_skip"):
    model = planner(check=None)
    joints = PhaseJoints()
    transport = PhaseTransport(joints, residuals, delay=delay, terminal=terminal)
    driver = SDKTrajectoryDriver(
        joints,
        lambda: dict(current_um=100000, stationary=True),
        model,
        trajectory_client=transport,
        collision_policy=policy,
        clock=lambda: 0.0,
        sleep=lambda value: None,
    )
    return model, joints, transport, driver


def alignment_stage(model, joints):
    target = nearby_target(model)
    goal = model.wrist_targets(target, "pregrasp", 0.1)
    return model.stage(goal, joints.state["arms"]["position"], 0.1, steps=40)


def test_forty_internal_nodes_use_one_submission_and_only_endpoint_wait():
    model, joints, transport, driver = driver_fixture(delay=3)
    stage = alignment_stage(model, joints)
    driver.begin()
    result = driver.execute_phase([stage], name="align")
    assert len(transport.submitted) == 1
    assert len(transport.submitted[0]) == 41
    assert not joints.moves
    assert result["result"]["completed"]
    assert result["collision_checked"] is False
    assert driver.generation == "phase-1"


def test_endpoint_correction_uses_actual_feedback_and_does_not_repeat_whole_route():
    model, joints, transport, driver = driver_fixture(residuals=[0.03, 0.0])
    stage = alignment_stage(model, joints)
    driver.begin()
    result = driver.execute_phase([stage], name="align")
    assert len(transport.submitted) == 2
    assert result["correction"] == 1
    assert transport.submitted[1][0][0] == pytest.approx(transport.submitted[0][-1][0] + 0.03)
    assert len(transport.submitted[1]) == 21


def test_fresh_visual_correction_does_not_replay_clearance_route(monkeypatch):
    model, joints, transport, driver = driver_fixture()
    initial = nearby_target(model)
    changed = nearby_target(model, "second")
    for stage in ("pregrasp", "grasp"):
        for goal in changed[stage].values():
            goal[0, 3] += 0.01
    final = copy.deepcopy(changed)
    final["pair_id"] = "third"
    observations = iter([changed, final])
    calls = []
    original_plan = model.plan

    def plan(*args, **kwargs):
        calls.append(True)
        return original_plan(*args, **kwargs)

    monkeypatch.setattr(model, "plan", plan)
    runner = TwoStageGraspExecutor(
        model,
        driver,
        lambda identity: next(observations),
        None,
        None,
        None,
        open_verify=None,
        clock=lambda: 0.0,
    )
    driver.begin()
    result = runner._alignment(initial)
    assert result["pair_id"] == "third"
    assert len(calls) == 1 and len(transport.submitted) == 2
    assert transport.submitted[1][0] == transport.submitted[0][-1]


def test_saturated_or_unimproving_error_stops_instead_of_retrying_forever():
    model, joints, transport, driver = driver_fixture(residuals=[0.03])
    stage = alignment_stage(model, joints)
    driver.begin()
    with pytest.raises(RuntimeError, match="no measurable progress"):
        driver.execute_phase([stage], name="align")
    assert len(transport.submitted) == 2
    assert joints.state["arms"]["enabled"] is True


def test_hard_sdk_fault_never_triggers_endpoint_correction():
    model, joints, transport, driver = driver_fixture(terminal="fault")
    stage = alignment_stage(model, joints)
    driver.begin()
    with pytest.raises(RuntimeError, match="SDK continuous phase failed"):
        driver.execute_phase([stage], name="align")
    driver.hold()
    assert len(transport.submitted) == 1
    assert not transport.stops


def test_wrist_stop_interrupts_owned_phase_without_waiting_for_next_waypoint():
    model, joints, transport, driver = driver_fixture(terminal="moving")
    stage = alignment_stage(model, joints)
    driver.begin()
    result = driver.execute_phase([stage], name="insert", monitor=lambda state, lift: True)
    assert result["result"]["monitored_stop"]
    assert transport.stops == ["phase-1"]
    assert driver.request_id is None
    assert joints.pending_id is None


def test_contact_stop_precedes_result_poll_that_could_clear_request_ownership(monkeypatch):
    _, joints, transport, driver = driver_fixture()
    driver.begin()

    def move(target, speed):
        assert speed == 0.2
        joints.pending_id = "full-close"
        joints.state["arm_startup"]["control_generation"] = "full-close"
        joints.state["arm_startup"]["joint_control"].update(request_id="full-close", phase="ready")
        return dict(request_id="full-close")

    def result(request_id):
        pytest.fail("Contact must stop the owned request before result clears pending_id")

    monkeypatch.setattr(joints, "move", move)
    monkeypatch.setattr(joints, "result", result)
    result = driver.move_until(joints.state["arms"]["position"], lambda state: True)
    assert result["contact_stop"]
    assert transport.stops == ["full-close"]
    assert joints.pending_id is None and driver.request_id is None
    assert driver.last_motion_end_robot_ns is not None
    assert driver.last_arrival_robot_ns is None  # A contact stop is not endpoint arrival.


def test_collision_flag_from_old_linear_path_is_not_used_for_new_cubic():
    model, joints, transport, driver = driver_fixture(policy="required")
    stage = alignment_stage(model, joints)
    stage["collision_checked"] = True
    driver.begin()
    with pytest.raises(ValueError, match="curve collision validator"):
        driver.execute_phase([stage], name="align")
    assert not transport.submitted


def test_history_pairs_moving_image_without_zeroing_or_reversing_joint_angles():
    _, joints, _, driver = driver_fixture()
    driver.begin()
    initial = copy.deepcopy(joints.state)
    later = copy.deepcopy(initial)
    initial["arms"]["position"][6] = -0.2
    later["arms"]["position"][6] = -0.1
    lift = dict(current_um=100000, stationary=True)
    driver.history.clear()
    driver.history.append((100_000_000, initial, lift))
    driver.history.append((140_000_000, later, lift))
    state, _ = driver.state_at(120_000_000)
    assert state["arms"]["position"][6] == pytest.approx(-0.15)
    later["arm_startup"]["control_generation"] = "other"
    with pytest.raises(ValueError, match="generation changed"):
        driver.state_at(120_000_000)


def test_complete_pick_has_two_main_phases_and_one_separate_trial_lift(tmp_path):
    model, joints, transport, driver = driver_fixture()
    targets = iter([nearby_target(model), nearby_target(model, "second")])
    observations = []
    calls = []
    pair = [0]

    def observe(identity):
        observations.append(identity)
        return next(targets)

    def wrist(target):
        pair[0] += 1
        return dict(
            accepted=True,
            blockers=[],
            pair_ids={side: side + str(pair[0]) for side in ("left", "right")},
            observed_at_monotonic=0.0,
            remaining_downward_m={
                side: 0.002 if pair[0] == 1 else 0.0 for side in ("left", "right")
            },
        )

    wrist.supports_motion = True

    def opened(target):
        calls.append("open")
        return dict(bilateral_open_verified=True)

    def closed(target):
        calls.append("close")
        return dict(bilateral_contact_verified=True)

    def lifted(target, contact):
        calls.append("lift")
        return dict(box_followed_both_hands=True)

    runner = TwoStageGraspExecutor(
        model,
        driver,
        observe,
        wrist,
        closed,
        lifted,
        open_verify=opened,
        output=tmp_path,
        clock=lambda: 0.0,
    )
    result = runner.run("crate")
    assert result["completed"] and result["motion_phases"] == 2
    assert len(transport.submitted) == 3
    assert [row["phase"] for row in driver.phase_events] == ["align", "insert", "test_lift"]
    assert calls == ["open", "close", "lift"]
    assert observations == ["crate", "crate"]
    assert not joints.moves


def test_full_open_target_does_not_wait_at_every_point():
    model = planner()
    joints = JointFeedback()
    driver = SDKWaypointDriver(
        joints,
        lambda: dict(current_um=100000, stationary=True),
        model,
        clock=lambda: 0.0,
        sleep=lambda seconds: None,
    )
    driver.begin()
    mapping = {
        side: dict(configured_open_rad=-0.4, configured_closed_rad=0.0, opening_sign=-1)
        for side in ("left", "right")
    }
    opener = SDKGripperOpener(
        driver, mapping, effort_limits=dict(left=1.0, right=1.0), step_rad=None
    )
    result = opener({})
    assert result["bilateral_open_verified"]
    assert len(joints.moves) == 1


class ContactDriver:
    def __init__(self, *, empty=False, overload=False):
        self.state = PhaseJoints().state
        self.state["arms"]["position"][7] = -0.8
        self.state["arms"]["position"][15] = -0.8
        self.state["arm_startup"]["nominal_target"] = self.state["arms"]["position"].copy()
        self.feedback_guard = None
        self.moves = []
        self.empty = empty
        self.overload = overload

    def measured(self):
        if self.feedback_guard is not None:
            self.feedback_guard(self.state)
        return copy.deepcopy(self.state), dict(current_um=100000, stationary=True)

    def sleep(self, seconds):
        pass

    def move_until(self, target, stop_when):
        self.moves.append(list(target))
        if self.empty:
            self.state["arms"]["position"][7] = 0.0
            self.state["arms"]["position"][15] = 0.0
            return dict(contact_stop=False)
        index = 7 if len(self.moves) == 1 else 15
        self.state["arms"]["position"][index] = -0.3
        self.state["arms"]["effort"][index] = 10.0 if self.overload else 0.5
        self.state["arm_startup"]["nominal_target"] = self.state["arms"]["position"].copy()
        state, _ = self.measured()
        assert stop_when(state)
        return dict(contact_stop=True)


def contact_closer(driver, *, accepted=True):
    side_policy = dict(
        contact_effort_min_nm=0.2,
        hard_effort_max_nm=1.0,
        contact_angle_min_rad=-0.5,
        contact_angle_max_rad=-0.1,
        step_rad=0.01,
    )
    policy = dict(hardware_validated=accepted, left=side_policy, right=side_policy)
    mapping = {side: dict(configured_closed_rad=0.0) for side in ("left", "right")}
    return SDKContinuousContactCloser(driver, policy, mapping)


def test_continuous_close_handles_separate_jaw_contacts_with_two_full_goals():
    driver = ContactDriver()
    result = contact_closer(driver)({})
    assert result["bilateral_contact_verified"]
    assert len(driver.moves) == 2
    assert driver.moves[0][7] == driver.moves[0][15] == 0.0
    assert driver.moves[1][7] == -0.3
    assert driver.moves[1][15] == 0.0
    assert driver.feedback_guard is not None  # Retain hard-load guard during trial lift.
    driver.state["arms"]["effort"][7] = 2.0
    with pytest.raises(RuntimeError, match="hard effort limit"):
        driver.measured()


def test_empty_or_overloaded_continuous_close_is_never_reported_as_capture():
    empty = ContactDriver(empty=True)
    with pytest.raises(ValueError, match="without verified rim contact"):
        contact_closer(empty)({})
    assert len(empty.moves) == 1 and empty.feedback_guard is None
    overloaded = ContactDriver(overload=True)
    with pytest.raises(RuntimeError, match="hard effort limit"):
        contact_closer(overloaded)({})
    assert len(overloaded.moves) == 1 and overloaded.feedback_guard is None


def test_continuous_close_requires_existing_measured_contact_policy():
    driver = ContactDriver()
    with pytest.raises(ValueError, match="Measured bilateral"):
        contact_closer(driver, accepted=False)
    assert not driver.moves
