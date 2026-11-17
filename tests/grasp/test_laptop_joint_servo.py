"""Real claw1.4 geometry, Python/C++ wire agreement and isolated UDP IO."""

import copy
import hashlib
import json
import socket
import struct
import subprocess
import threading
import time
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from test_ik_executor import HIGH, LOW, Q
from test_joint_client import Transport, client
from wheel_execution.laptop_servo import LaptopReferenceLoop, LaptopVisualServoDriver
from wheel_execution.visual_servo import LatestObservation
from wheel_motion_core.joint_servo import ARM_AXES, JointPostureResolver, JointReferenceGenerator
from wheel_motion_core.live_kinematics import SDKAlignedModel
from wheel_sdk_bridge.http_client import SDKError
from wheel_sdk_bridge.joint_stream_client import MAGIC, PACKET, JointStreamClient

ROOT = Path(__file__).resolve().parents[2]


def test_discarded_image_keeps_only_original_unexpired_target():
    clock = [100.0]
    second_poll = threading.Event()
    release = threading.Event()
    calls = [0]

    def observe():
        calls[0] += 1
        if calls[0] == 2:
            second_poll.set()
            release.wait(1)
        return None

    source = LatestObservation(observe, clock=lambda: clock[0])
    source.value = dict(pair_id="original", observed_at_monotonic=100.0)
    source.start()
    try:
        assert second_poll.wait(1)
        clock[0] = 102.0
        assert source.latest()["pair_id"] == "original"
        clock[0] = 102.001
        assert source.latest() is None
        assert source.value["observed_at_monotonic"] == 100.0
    finally:
        release.set()
        source.close()


@pytest.fixture(scope="module")
def model():
    return SDKAlignedModel(
        ROOT / "assets/robots/r680_lift_openarm_claw1.4/r680_lift_openarm_robot_viewer.urdf",
        ROOT / "config/robots/calibration/sdk_arm_chains_claw14.json",
        LOW,
        HIGH,
    )


def test_analytic_jacobian_matches_original_chain_at_nonzero_angles(model):
    generator = JointReferenceGenerator(model, Q)
    for side, start in (("right", 0), ("left", 8)):
        actual, jacobian = generator.geometry(side, Q, -0.05713)
        original = model.transform(f"openarm_{side}_ee_base_link", model.values(Q, -0.05713))
        np.testing.assert_allclose(actual, original, atol=1e-12)
        for axis in range(7):
            shifted = list(Q)
            shifted[start + axis] += 1e-7
            changed, _ = generator.geometry(side, shifted, -0.05713)
            numerical = np.r_[
                (changed[:3, 3] - actual[:3, 3]) / 1e-7,
                Rotation.from_matrix(changed[:3, :3] @ actual[:3, :3].T).as_rotvec() / 1e-7,
            ]
            np.testing.assert_allclose(jacobian[:, axis], numerical, atol=1e-7)


def test_fixed_sdk_J7_limits_reach_ik_and_references_without_changing_fk(model):
    lower, upper = list(LOW), list(HIGH)
    bound = float(np.deg2rad(52))
    for index in (6, 14):
        lower[index], upper[index] = -bound, bound
    limited = SDKAlignedModel(
        ROOT / "assets/robots/r680_lift_openarm_claw1.4/r680_lift_openarm_robot_viewer.urdf",
        ROOT / "config/robots/calibration/sdk_arm_chains_claw14.json",
        lower,
        upper,
    )
    resolver = JointPostureResolver(limited)
    generator = JointReferenceGenerator(limited, Q)
    for side, index in (("right", 6), ("left", 14)):
        name = f"openarm_{side}_joint7"
        assert limited.joints[name]["bounds"] == (-bound, bound)
        ik_index = resolver.ik.names.index(name)
        assert resolver.ik.lower[ik_index] == pytest.approx(-bound + 0.02)
        assert resolver.ik.upper[ik_index] == pytest.approx(bound - 0.02)
        assert generator.lower[index] == -bound
        assert generator.upper[index] == bound
        np.testing.assert_allclose(
            limited.transform(f"openarm_{side}_ee_base_link", limited.values(Q, 0.1)),
            model.transform(f"openarm_{side}_ee_base_link", model.values(Q, 0.1)),
        )
        for sign in (-1, 1):
            measured = list(Q)
            measured[index] = sign * (bound + 1e-5)
            with pytest.raises(ValueError, match="outside original limits"):
                generator.step({}, measured, 0.1, 0.01)


def test_continuous_retarget_converges_without_velocity_reset_or_gripper_motion(model):
    generator = JointReferenceGenerator(model, Q)
    goal_q = list(Q)
    goal_q[1] += 0.015
    goal_q[9] -= 0.015
    targets = {side: generator.geometry(side, goal_q, 0.1)[0] for side in ("left", "right")}
    previous_v = np.zeros(16)
    velocities, errors = [], []
    for tick in range(700):
        if tick == 150:
            goal_q[1] += 0.01
            goal_q[9] -= 0.01
            targets = {side: generator.geometry(side, goal_q, 0.1)[0] for side in ("left", "right")}
        positions, velocity = generator.step(targets, generator.position, 0.1, 0.01)
        velocity = np.asarray(velocity)
        assert np.max(np.abs(velocity)) <= 0.2 + 1e-12
        assert np.max(np.abs(velocity - previous_v)) <= 0.005 + 1e-12
        assert positions[7] == Q[7] and positions[15] == Q[15]
        velocities.append(velocity.copy())
        errors.append(max(e["position_m"] for e in generator.measured_errors.values()))
        previous_v = velocity
    assert errors[-1] < 0.002
    assert np.linalg.norm(velocities[150][ARM_AXES]) > 0
    assert errors[-1] < errors[0]


def test_saturated_axis_redistributes_cartesian_velocity_to_other_joints():
    jacobian = np.c_[np.eye(6), [1, 0, 0, 0, 0, 0]]
    low, high = np.full(7, -0.2), np.full(7, 0.2)
    low[0] = high[0] = 0
    velocity = JointReferenceGenerator.bounded_velocity(
        jacobian, np.array([0.1, 0, 0, 0, 0, 0]), low, high, np.zeros(7)
    )
    assert velocity[0] == 0
    assert abs(velocity[6] - 0.1) < 0.0001


def test_reference_brakes_inside_original_wrist_limit_without_changing_feedback(model):
    initial = list(Q)
    initial[5] = 0.6
    generator = JointReferenceGenerator(model, initial)
    desired = list(initial)
    desired[5] = 0.775
    goals = {side: generator.geometry(side, desired, 0.1)[0] for side in ("left", "right")}
    old_velocity = np.zeros(16)
    for _ in range(500):
        positions, velocities = generator.step(goals, generator.position, 0.1, 0.01)
        assert positions[5] <= generator.reference_upper[5] + 1e-8
        assert abs(velocities[5]) <= 0.2 + 1e-10
        assert np.max(np.abs(np.asarray(velocities) - old_velocity)) <= 0.005 + 1e-10
        old_velocity = np.asarray(velocities)
    assert generator.reference_upper[5] <= generator.upper[5] - 0.019
    assert initial[5] == 0.6


def test_joint_goal_tracking_uses_measured_error_under_static_wrist_deflection(model):
    initial = list(Q)
    initial[5] = 0.55
    goal = list(initial)
    goal[5] = 0.75
    generator = JointReferenceGenerator(model, initial)
    targets = {side: generator.geometry(side, goal, 0.1)[0] for side in ("left", "right")}
    previous_actual = np.asarray(initial)
    previous_actual[5] += 0.06
    for _ in range(1800):
        measured = generator.position.copy()
        measured[5] += 0.06
        velocity = (measured - previous_actual) / 0.01
        generator.step(targets, measured, 0.1, 0.01, joint_goal=goal, measured_velocity=velocity)
        assert measured[5] < generator.upper[5]
        previous_actual = measured
    assert abs(generator.position[5] + 0.06 - goal[5]) < 0.002


def test_stalled_measured_joint_does_not_allow_unbounded_reference_windup(model):
    generator = JointReferenceGenerator(model, Q)
    goal = list(Q)
    goal[8] += 0.8
    targets = {side: generator.geometry(side, goal, 0.1)[0] for side in ("left", "right")}
    bias = np.zeros(16)
    for tick in range(1200):
        bias[8] = min(0.1, tick * 0.00005)
        generator.step(targets, Q, 0.1, 0.01, joint_goal=goal, command_bias=bias)
        lead = generator.position[8] + bias[8] - Q[8]
        assert lead < 0.18
    assert abs(generator.velocity[8]) < 0.01


def test_low_rate_posture_ik_retains_measurement_and_original_joint_limits(model):
    measured = list(Q)
    goal = list(Q)
    goal[1] += 0.01
    generator = JointReferenceGenerator(model, measured)
    targets = {side: generator.geometry(side, goal, 0.1)[0] for side in ("left", "right")}
    resolver = JointPostureResolver(model)
    solved, report = resolver.solve(targets, measured, 0.1)
    assert report["success"] is True
    assert max(report["position_errors"].values()) <= 0.005
    assert max(report["rotation_errors"].values()) <= 0.02
    assert measured == list(Q)
    assert solved[7] == measured[7] and solved[15] == measured[15]
    assert np.all(np.asarray(solved)[ARM_AXES] >= generator.lower[ARM_AXES])
    assert np.all(np.asarray(solved)[ARM_AXES] <= generator.upper[ARM_AXES])


def test_spawned_ik_process_accepts_real_model_and_keeps_motion_io_in_parent(model):
    driver = LaptopVisualServoDriver(
        SimpleNamespace(),
        lambda: {},
        SimpleNamespace(model=model),
        servo_client=SimpleNamespace(close=lambda: None),
        isolate_posture=True,
    )
    generator = JointReferenceGenerator(model, Q)
    goals = {side: generator.geometry(side, Q, 0.1)[0] for side in ("left", "right")}
    try:
        packet = dict(
            targets=goals, alignment_phase="align", pair_id="real", image_receive_mono_ns=10
        )
        resolved = driver.resolve_packet(
            packet, dict(arms=dict(position=list(Q))), dict(current_um=100000)
        )
        assert len(resolved["joint_goal"]) == 16
        assert resolved["image_receive_mono_ns"] == 10
    finally:
        driver.finish()


def test_pending_ik_never_blocks_or_refreshes_old_target_timestamp(model):
    driver = LaptopVisualServoDriver(
        SimpleNamespace(),
        lambda: {},
        SimpleNamespace(model=model),
        servo_client=SimpleNamespace(close=lambda: None),
        isolate_posture=False,
    )
    generator = JointReferenceGenerator(model, Q)
    goals = {side: generator.geometry(side, Q, 0.1)[0] for side in ("left", "right")}
    state, lift = dict(arms=dict(position=list(Q))), dict(current_um=100000)
    first = dict(
        targets=goals, alignment_phase="clearance", pair_id="old", image_receive_mono_ns=10
    )
    driver.resolve_packet(first, state, lift)
    future = Future()
    driver.posture_process = SimpleNamespace(
        submit=lambda *args: future, shutdown=lambda **kwargs: None
    )
    second = dict(first, alignment_phase="align", pair_id="new", image_receive_mono_ns=20)
    try:
        pending = driver.resolve_packet(second, state, lift)
        assert pending["pair_id"] == "old" and pending["image_receive_mono_ns"] == 10
        assert pending["alignment_phase"] == "clearance"
        future.set_result((list(Q), dict(success=True)))
        resolved = driver.resolve_packet(second, state, lift)
        assert resolved["pair_id"] == "new" and resolved["image_receive_mono_ns"] == 20
        assert resolved["alignment_phase"] == "align"
    finally:
        driver.finish()


@pytest.mark.parametrize("failure", ["late_tick", "nan", "following", "limit"])
def test_invalid_input_never_produces_a_reference(model, failure):
    generator = JointReferenceGenerator(model, Q)
    original = generator.position.copy()
    targets = {s: generator.geometry(s, Q, 0.1)[0] for s in ("left", "right")}
    measured = list(Q)
    dt = 0.01
    if failure == "late_tick":
        dt = 0.05
    elif failure == "nan":
        targets["left"][0, 3] = float("nan")
    elif failure == "following":
        measured[0] += 0.26
    else:
        measured[0] = 30
    with pytest.raises(ValueError):
        generator.step(targets, measured, 0.1, dt)
    np.testing.assert_array_equal(generator.position, original)


def test_sdk_axis_speed_is_respected_and_gravity_deflection_is_not_integrated(model):
    generator = JointReferenceGenerator(model, Q, speed_limits=[0.01] * 16)
    targets = {side: generator.geometry(side, Q, 0.1)[0] for side in ("left", "right")}
    measured = list(Q)
    measured[1] += 0.03
    for _ in range(10):
        generator.step(targets, measured, 0.1, 0.01)
    np.testing.assert_allclose(generator.position, Q, atol=1e-12)
    assert generator.measured_errors["right"]["position_m"] > 0.001
    changed = list(Q)
    changed[1] += 0.1
    targets["right"] = generator.geometry("right", changed, 0.1)[0]
    for _ in range(20):
        _, velocity = generator.step(targets, generator.position, 0.1, 0.01)
        assert max(abs(v) for v in velocity) <= 0.01 + 1e-12


def test_reference_near_original_joint_bound_brakes_without_clipping_feedback(model):
    seed = list(Q)
    seed[0] = model.joints["openarm_right_joint1"]["bounds"][1] - 0.001
    generator = JointReferenceGenerator(model, seed)
    goals = {side: generator.geometry(side, seed, 0.1)[0] for side in ("left", "right")}
    goals["right"][1, 3] += 0.01
    for _ in range(150):
        previous = generator.velocity.copy()
        positions, velocity = generator.step(goals, generator.position, 0.1, 0.01)
        assert positions[0] <= generator.upper[0]
        assert np.max(np.abs(np.asarray(velocity) - previous)) <= 0.005 + 1e-12


def test_packet_send_failure_is_not_retried():
    class FailingSocket:
        def setblocking(self, value):
            pass

        def connect(self, address):
            pass

        def send(self, packet):
            calls.append(packet)
            raise BlockingIOError("send would block")

        def close(self):
            pass

    calls = []
    _, stream = connect_stream(18034, socket_factory=lambda *args: FailingSocket())
    identity = stream.start()["request_id"]
    with pytest.raises(BlockingIOError):
        stream.send(
            identity,
            [0.0] * 16,
            [0.0] * 16,
            feedback_stamp_ns=990_000_000,
            image_stamp_ns=980_000_000,
            pair_id="frame",
        )
    assert len(calls) == 1 and stream.sequence == 0
    stream.close()


def connect_stream(port, **kwargs):
    transport = Transport()
    transport.base_url = "http://127.0.0.1:18030"
    transport.s["arm_startup"]["nominal_target"] = [0.0] * 16
    transport.s["arm_startup"]["joint_control"].update(
        joint_stream_protocol=1,
        joint_stream_port=port,
        joint_stream_speed_limits=[0.2] * 16,
        joint_stream_image_max_age_ms=2000,
    )
    return transport, JointStreamClient(client(transport), **kwargs)


def test_real_udp_stream_uses_http_only_for_start_and_stop():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(1)
        transport, stream = connect_stream(receiver.getsockname()[1])
        result = stream.start()
        identity = result["request_id"]
        for tick in range(3):
            stream.send(
                identity,
                [0.001 * tick] + [0.0] * 15,
                [0.0] * 16,
                feedback_stamp_ns=990_000_000,
                image_stamp_ns=980_000_000,
                pair_id="same-frame",
            )
            packet, _ = receiver.recvfrom(1024)
            fields = PACKET.unpack(packet)
            assert fields[0] == MAGIC and fields[2] == tick + 1
            assert fields[3:5] == (990_000_000, 980_000_000)
            assert fields[5] == hashlib.sha256(b"same-frame").digest()[:16]
        assert len(transport.posts) == 1
        assert json.loads(transport.posts[0][1]["detail"])["stream_key"] == stream.key
        transport.s["arm_startup"]["joint_control"]["request_id"] = identity
        stream.stop(identity)
        assert [row[1]["command"] for row in transport.posts] == [
            "joint_stream_start",
            "joint_stop",
        ]
        assert stream.socket is None


def test_unsupported_sdk_rejected_before_socket_or_command():
    transport, stream = connect_stream(18034)
    del transport.s["arm_startup"]["joint_control"]["joint_stream_protocol"]
    with pytest.raises(SDKError, match="not installed"):
        stream.start()
    assert not transport.posts and stream.socket is None


def test_uncertain_start_does_not_retry_or_forget_ownership():
    transport, stream = connect_stream(18034)
    transport.failure = SDKError("lost reply", uncertain=True)
    with pytest.raises(SDKError):
        stream.start()
    assert stream.joints.pending_id and stream.socket is None
    assert len(transport.posts) == 1
    with pytest.raises(SDKError, match="reconciled"):
        stream.start()
    assert len(transport.posts) == 1


@pytest.fixture(scope="module")
def guard_binary(tmp_path_factory):
    binary = tmp_path_factory.mktemp("joint-stream-native") / "guard"
    subprocess.run(
        [
            "g++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-O2",
            str(ROOT / "deployment/robot/joint_stream/test_guard.cpp"),
            "-o",
            str(binary),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return binary


def wire(
    *,
    sequence=1,
    feedback=990_000_000,
    image=980_000_000,
    pair="frame",
    key="a" * 32,
    positions=None,
    velocities=None,
):
    return PACKET.pack(
        MAGIC,
        key.encode(),
        sequence,
        feedback,
        image,
        hashlib.sha256(pair.encode()).digest()[:16],
        *(positions or [0.0] * 16),
        *(velocities or [0.0] * 16),
    )


def native(binary, records):
    data = b"".join(struct.pack("!Q", now) + packet for now, packet in records)
    result = subprocess.run([str(binary)], input=data, capture_output=True, check=True)
    return [row.split() for row in result.stdout.decode().splitlines()]


def test_native_wire_latest_sequence_gripper_and_independent_watchdogs(guard_binary):
    records = [
        (1_000_000_000, wire(sequence=1, positions=[0.001] + [0.0] * 15)),
        (1_010_000_000, wire(sequence=3, positions=[0.003] + [0.0] * 15)),
        (1_020_000_000, wire(sequence=2)),  # Out-of-order command cannot replace sequence 3.
        (1_030_000_000, wire(sequence=4, key="b" * 32)),
        (1_040_000_000, wire(sequence=5, positions=[0.0] * 7 + [0.01] + [0.0] * 8)),
        (1_200_000_000, wire(sequence=6)),  # New sequence cannot refresh old feedback.
    ]
    rows = native(guard_binary, records)
    assert [row[:2] for row in rows] == [
        ["1", "1"],
        ["1", "3"],
        ["0", "3"],
        ["0", "3"],
        ["0", "3"],
        ["0", "3"],
    ]
    assert rows[-1][-1] == "joint_stream_command_timeout"
    rows = native(
        guard_binary,
        [
            (1_000_000_000, wire()),
            (1_490_000_000, wire(sequence=2, feedback=1_480_000_000)),
            (1_510_000_000, wire(sequence=3, feedback=1_500_000_000)),
        ],
    )
    assert (
        rows[1][0] == "0"
    )  # The 150ms command watchdog remains independent of the 2s image window.
    assert rows[2][0] == "0"


@pytest.mark.parametrize("change", ["future", "jump", "nan", "speed", "same_pair_new_stamp"])
def test_native_invalid_reference_cannot_mutate_accepted_goal(guard_binary, change):
    options = dict(sequence=2)
    if change == "future":
        options["feedback"] = 2_000_000_000
    elif change == "jump":
        options["positions"] = [0.5] + [0.0] * 15
    elif change == "nan":
        options["positions"] = [float("nan")] + [0.0] * 15
    elif change == "speed":
        options["velocities"] = [0.3] + [0.0] * 15
    else:
        options["image"] = 990_000_000
    rows = native(guard_binary, [(1_000_000_000, wire()), (1_010_000_000, wire(**options))])
    assert rows[1][:3] == ["0", "1", "0"]


def test_native_board_limits_reference_velocity_and_acceleration_under_packet_jitter(guard_binary):
    records = []
    for tick in range(60):
        now = 1_000_000_000 + tick * 10_000_000
        position = 0.003 if tick % 7 < 4 else -0.003
        records.append(
            (
                now,
                wire(
                    sequence=tick + 1,
                    feedback=now - 1_000_000,
                    image=now - 5_000_000,
                    pair=str(tick),
                    positions=[position] + [0.0] * 15,
                ),
            )
        )
    rows = native(guard_binary, records)
    assert all(row[0] == "1" for row in rows)
    positions = [0.0] + [float(row[3]) for row in rows]
    velocity = np.diff(positions) / 0.01
    acceleration = np.diff(np.r_[0.0, velocity]) / 0.01
    assert np.max(np.abs(velocity)) <= 0.2 + 1e-10
    assert np.max(np.abs(acceleration)) <= 0.5 + 1e-10


def test_native_continuous_reference_survives_board_acceleration_lag(guard_binary):
    records = []
    for tick in range(80):
        now = 1_000_000_000 + tick * 10_000_000
        records.append(
            (
                now,
                wire(
                    sequence=tick + 1,
                    feedback=now - 1_000_000,
                    image=980_000_000,
                    positions=[0.002 * tick] + [0.0] * 15,
                    velocities=[0.2] + [0.0] * 15,
                ),
            )
        )
    rows = native(guard_binary, records)
    # A legal stream may lead the board while its acceleration guard ramps up.
    # Dropping these packets turns healthy feedback into a misleading timeout.
    assert all(row[0] == "1" and row[-1] == "active" for row in rows)
    assert rows[-1][1] == "80"
    output = np.array([0.0] + [float(row[3]) for row in rows])
    assert max(float(row[2]) - float(row[3]) for row in rows) > 0.032
    velocity = np.diff(output) / 0.01
    assert np.max(np.abs(velocity)) <= 0.2 + 1e-10
    assert np.max(np.abs(np.diff(np.r_[0.0, velocity]) / 0.01)) <= 0.5 + 1e-10


def test_native_image_expiry_cannot_be_hidden_by_new_sequences_or_revived_by_late_image(
    guard_binary,
):
    records = []
    for tick in range(202):
        now = 1_000_000_000 + tick * 10_000_000
        records.append((now, wire(sequence=tick + 1, feedback=now - 1_000_000)))
    records.append(
        (
            3_020_000_000,
            wire(sequence=1000, feedback=3_019_000_000, image=3_015_000_000, pair="late-new-image"),
        )
    )
    rows = native(guard_binary, records)
    assert rows[198][0] == "1"  # Exactly 2s remains valid.
    assert rows[199][-1] == "joint_stream_image_timeout"
    assert rows[-1][0] == "0" and rows[-1][-1] == "joint_stream_image_timeout"


def loop_fixture(model):
    generator = JointReferenceGenerator(model, Q)
    targets = {s: generator.geometry(s, Q, 0.1)[0] for s in ("left", "right")}
    state = dict(
        server_mono_ns="1000000000",
        arms=dict(position=list(Q), fresh=True, enabled=True, receive_mono_ns="990000000"),
        arm_startup=dict(
            control_generation="ours", joint_control=dict(request_id="ours", phase="streaming")
        ),
    )
    sends = []
    driver = SimpleNamespace(
        request_id="ours",
        generation="ours",
        lift_um=100000,
        cached_feedback=lambda: (
            copy.deepcopy(state),
            dict(current_um=100000, stationary=True),
            10.0,
        ),
        servo=SimpleNamespace(
            send=lambda *args, **kwargs: sends.append((args, kwargs)), close=lambda: None
        ),
    )
    loop = LaptopReferenceLoop(driver, generator, clock=lambda: 10.0)
    loop.update(dict(targets=targets, pair_id="original", image_receive_mono_ns=980_000_000))
    return loop, state, sends


def test_one_visual_frame_generates_multiple_references_without_retimestamping(model):
    loop, _, sends = loop_fixture(model)
    for _ in range(5):
        loop.tick(0.01)
    assert len(sends) == 5
    assert all(row[1]["image_stamp_ns"] == 980_000_000 for row in sends)
    assert all(row[1]["pair_id"] == "original" for row in sends)


@pytest.mark.parametrize(
    "age_ns,allowed", [(1_999_000_000, True), (2_000_000_000, True), (2_001_000_000, False)]
)
def test_laptop_image_window_is_two_seconds_with_fresh_feedback(model, age_ns, allowed):
    loop, state, sends = loop_fixture(model)
    state["server_mono_ns"] = "3000000000"
    state["arms"]["receive_mono_ns"] = "2990000000"
    loop.packet["image_receive_mono_ns"] = 3_000_000_000 - age_ns
    if allowed:
        loop.tick(0.01)
        assert len(sends) == 1
    else:
        with pytest.raises(RuntimeError, match="Visual image source expired"):
            loop.tick(0.01)
        assert not sends


def test_sdk_stop_still_sent_if_execution_logging_fails():
    class BrokenTrace:
        def record(self, *args, **kwargs):
            raise OSError("disk unavailable")

    transport, stream = connect_stream(18034)
    identity = stream.start()["request_id"]
    transport.s["arm_startup"]["joint_control"]["request_id"] = identity
    stream.trace = BrokenTrace()
    with pytest.raises(SDKError, match="receipt logging failed") as failure:
        stream.stop(identity)
    assert failure.value.uncertain
    assert transport.posts[-1][1]["command"] == "joint_stop"
    stream.close()


def test_logging_failure_stops_reference_loop_before_more_udp(model):
    loop, _, sends = loop_fixture(model)
    closed = threading.Event()

    class BrokenTrace:
        def check(self):
            raise RuntimeError("execution log failed")

        def record(self, *args, **kwargs):
            raise RuntimeError("execution log failed")

    loop.driver.trace = BrokenTrace()
    loop.driver.servo.close = closed.set
    loop.start()
    try:
        assert closed.wait(timeout=1)
        with pytest.raises(RuntimeError, match="execution log failed"):
            loop.check()
        assert not sends
    finally:
        loop.close()


@pytest.mark.parametrize("failure", ["image", "feedback", "owner", "generation", "lift"])
def test_control_loop_stops_before_sending_when_sources_expire_or_ownership_changes(model, failure):
    loop, state, sends = loop_fixture(model)
    if failure == "image":
        loop.packet["image_receive_mono_ns"] = 1
        state["server_mono_ns"] = "3000000000"
        state["arms"]["receive_mono_ns"] = "2990000000"
    elif failure == "feedback":
        state["arms"]["receive_mono_ns"] = "1"
    elif failure == "owner":
        state["arm_startup"]["joint_control"]["request_id"] = "other"
    elif failure == "generation":
        state["arm_startup"]["control_generation"] = "other"
    else:
        loop.driver.lift_um = 50000
    with pytest.raises(RuntimeError):
        loop.tick(0.01)
    assert not sends


def test_worker_expires_even_when_perception_or_supervisor_is_blocked(model):
    loop, _, sends = loop_fixture(model)
    closed = threading.Event()
    loop.clock = time.monotonic
    loop.driver.cached_feedback = lambda: (_ for _ in ()).throw(RuntimeError("expired"))
    loop.driver.servo.close = closed.set
    loop.start()
    assert closed.wait(1)
    loop.close()
    with pytest.raises(RuntimeError, match="expired"):
        loop.check()
    assert not sends


@pytest.mark.parametrize("failure", ["deadline", "feedback_cache"])
def test_failed_tick_inputs_are_logged_even_before_a_reference_is_generated(
    model, tmp_path, failure
):
    from wheel_execution.execution_trace import ExecutionTrace

    loop, state, sends = loop_fixture(model)
    clock_values = iter([10.0, 10.0, 10.04])
    loop.clock = lambda: next(clock_values, 10.04)
    if failure == "feedback_cache":
        loop.driver.cached_feedback = lambda: (_ for _ in ()).throw(RuntimeError("cache expired"))
    closed = threading.Event()
    loop.driver.servo.close = closed.set
    trace = loop.driver.trace = ExecutionTrace(tmp_path / "trace")
    try:
        loop.start()
        assert closed.wait(1)
        loop.close()
    finally:
        loop.close()
        trace.close()
    events = [json.loads(line) for line in (trace.directory / "events.jsonl").open()]
    failure_record = next(row["detail"] for row in events if row["kind"] == "reference_loop_failed")
    context = failure_record["failed_tick"]
    assert context["dt_s"] == pytest.approx(0.04)
    assert context["packet"]["image_receive_mono_ns"] == 980_000_000
    assert context["request_id"] == "ours"
    if failure == "deadline":
        assert context["stage"] == "joint_reference_generation"
        assert context["state"]["arms"]["position"] == state["arms"]["position"]
        assert "deadline missed" in failure_record["error"]
    else:
        assert context["stage"] == "feedback_cache"
        assert "state" not in context
    assert failure_record["metrics"]["generated"] == 0
    assert not sends
    assert not any(row["kind"] == "reference_send_attempt" for row in events)


def test_full_phase_with_slow_observer_keeps_independent_udp_reference_loop(model):
    """Synthetic motor feedback through real UDP; no robot endpoint is contacted."""
    from wheel_execution.visual_servo import visual_packet

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(0.05)
        transport, _ = connect_stream(receiver.getsockname()[1])
        state = transport.s
        state["arms"]["position"] = list(Q)
        state["arm_startup"].update(control_generation="initial", nominal_target=list(Q))
        state["arm_startup"]["joint_control"].update(
            lower=LOW, upper=HIGH, joint_stream_fixed_target_protocol=1
        )
        lock = threading.Lock()
        received = []
        receiving = threading.Event()
        snapshots = []

        def snapshot():
            snapshots.append(threading.current_thread().name)
            assert threading.current_thread().name != "laptop-joint-100hz"
            with lock:
                now = time.monotonic_ns()
                state["server_mono_ns"] = str(now)
                state["arms"]["receive_mono_ns"] = str(now)
                return copy.deepcopy(state)

        def post(method, path, payload):
            assert method == "POST"
            detail = json.loads(payload["detail"])
            with lock:
                if payload["command"] == "joint_stream_start":
                    state["arm_startup"]["joint_control"].update(
                        request_id=detail["request_id"], phase="streaming"
                    )
                    state["arm_startup"]["control_generation"] = detail["request_id"]
                else:
                    assert (
                        detail["request_id"] == state["arm_startup"]["joint_control"]["request_id"]
                    )
                    state["arm_startup"]["joint_control"]["phase"] = "stopped"
                    state["arms"]["velocity"] = [0.0] * 16
            return dict(success=True)

        joints = SimpleNamespace(
            snapshot=snapshot,
            pending_id=None,
            lock=threading.Lock(),
            http=SimpleNamespace(base_url=transport.base_url, request=post),
            _check_compensation_handover=lambda value: None,
        )

        def receive():
            while not receiving.is_set():
                try:
                    data, _ = receiver.recvfrom(1024)
                except socket.timeout:
                    continue
                fields = PACKET.unpack(data)
                received.append(fields)
                with lock:
                    state["arms"]["position"] = list(fields[6:22])
                    state["arms"]["velocity"] = list(fields[22:38])
                    state["arm_startup"]["nominal_target"] = list(fields[6:22])

        worker = threading.Thread(target=receive, daemon=True)
        worker.start()
        driver = LaptopVisualServoDriver(
            joints,
            lambda: dict(current_um=100000, stationary=True),
            SimpleNamespace(model=model, ik=model.ik()),
            collision_policy="onsite_supervised_skip",
        )
        generator = JointReferenceGenerator(model, Q)
        goal_q = list(Q)
        goal_q[1] += 0.02
        goal_q[9] -= 0.02
        goals = {s: generator.geometry(s, goal_q, 0.1)[0] for s in ("left", "right")}
        observations = []

        def observe():
            # Deliberately much slower than the 10ms joint reference period.
            time.sleep(0.04)
            stamp = time.monotonic_ns()
            observations.append(stamp)
            return dict(
                pair_id=str(stamp),
                image_receive_mono_ns=stamp,
                diagnostic_only=False,
                execution_blockers=[],
                frame_id="arm_origin",
                tool_frame="cad_contact_midpoint_pinch_y",
            )

        try:
            driver.begin()
            _, result = driver.follow(
                SimpleNamespace(latest=observe),
                lambda target, state, lift: visual_packet(target, goals),
                name="align",
                timeout_s=5,
                stable_s=2.2,
            )
            assert result["completed"] and result["execution_mode"] == "laptop_visual_servo"
            assert result["reference_timing"]["generated"] > len(observations) * 2
            assert len(received) > len(observations) * 2
            assert len(observations) == 1
            assert {packet[0] for packet in received} == {b"WJFIXD1\0"}
            assert len({packet[4] for packet in received}) == 1
            assert joints.pending_id is None and state["arms"]["enabled"]
            assert driver.reference_loop.thread.is_alive() is False
            assert max(abs(received[-1][index + 6] - Q[index]) for index in ARM_AXES) > 0.001
        finally:
            driver.finish()
            receiving.set()
            worker.join(timeout=1)


def test_laptop_hold_uses_atomic_scoped_stop_instead_of_legacy_global_stop(model):
    stream = SimpleNamespace(
        require_supported=lambda: None,
        close=lambda: None,
        stop=lambda identity: dict(stopped=identity),
    )
    joints = SimpleNamespace(
        pending_id="ours", stop=lambda *args: pytest.fail("Global stop forbidden")
    )
    driver = LaptopVisualServoDriver(
        joints,
        lambda: None,
        SimpleNamespace(model=model),
        servo_client=stream,
        collision_policy="onsite_supervised_skip",
    )
    assert driver.hold() == dict(stopped="ours")
    assert driver.cancelled.is_set()


def test_fixed_target_native_keeps_source_identity_after_two_seconds(guard_binary):
    def fixed(**kwargs):
        return b"WJFIXD1\0" + wire(**kwargs)[8:]

    records = [
        (
            1_000_000_000 + i * 10_000_000,
            fixed(sequence=i + 1, feedback=1_000_000_000 + i * 10_000_000),
        )
        for i in range(300)
    ]
    assert all(row[0] == "1" for row in native(guard_binary, records))
    first = (1_000_000_000, fixed())
    for changed in (
        fixed(sequence=2, feedback=1_010_000_000, image=1_000_000_000),
        fixed(sequence=2, feedback=1_010_000_000, pair="different"),
        wire(sequence=2, feedback=1_010_000_000),
    ):
        assert native(guard_binary, [first, (1_010_000_000, changed)])[-1][0] == "0"
    # A future image cannot be admitted even in fixed-target mode.
    assert native(guard_binary, [(1_010_000_000, fixed(image=2_000_000_000))])[0][0] == "0"
    timeout = native(
        guard_binary, [first, (1_200_000_000, fixed(sequence=2, feedback=1_200_000_000))]
    )
    assert timeout[-1][-1] == "joint_stream_command_timeout"


def test_fixed_target_requires_explicit_sdk_capability():
    _, stream = connect_stream(18034)
    with pytest.raises(SDKError, match="fixed-target protocol"):
        stream.start(fixed_target=True)


def test_fixed_target_still_requires_fresh_joint_feedback(model):
    loop, state, sends = loop_fixture(model)
    loop.packet["fixed_target"] = True
    state["server_mono_ns"] = "5000000000"
    state["arms"]["receive_mono_ns"] = "4990000000"
    loop.tick(0.01)
    assert len(sends) == 1
    assert sends[0][1]["image_stamp_ns"] == 980_000_000
    state["arms"]["receive_mono_ns"] = "4800000000"
    with pytest.raises(RuntimeError, match="Measured joint source expired"):
        loop.tick(0.01)
    assert len(sends) == 1


def test_planned_session_keeps_admission_identity_and_feedback_watchdog(guard_binary):
    def planned(**kwargs):
        return b"WJPLAN1\0" + wire(image=1_000_000_000, **kwargs)[8:]

    records = [
        (
            1_000_000_000 + i * 10_000_000,
            planned(sequence=i + 1, feedback=1_000_000_000 + i * 10_000_000),
        )
        for i in range(301)
    ]
    assert all(row[0] == "1" for row in native(guard_binary, records))
    first = records[0]
    changed = planned(sequence=2, feedback=1_010_000_000, pair="other-session")
    assert native(guard_binary, [first, (1_010_000_000, changed)])[-1][0] == "0"
    old_session = b"WJPLAN1\0" + wire()[8:]
    assert native(guard_binary, [(1_000_000_000, old_session)])[0][0] == "0"
    switched = b"WJFIXD1\0" + planned(sequence=2, feedback=1_010_000_000)[8:]
    assert native(guard_binary, [first, (1_010_000_000, switched)])[-1][0] == "0"
    late = planned(sequence=2, feedback=1_200_000_000)
    assert (
        native(guard_binary, [first, (1_200_000_000, late)])[-1][-1]
        == "joint_stream_command_timeout"
    )


def test_stream_stop_survives_out_of_limits_feedback_and_remains_scoped():
    transport, stream = connect_stream(18034)
    identity = stream.start()["request_id"]
    transport.s["arms"]["position"][13] = -2.01
    with pytest.raises(SDKError, match="outside SDK limits"):
        stream.joints.snapshot()
    stream.stop(identity)
    assert stream.socket is None
    assert json.loads(transport.posts[-1][1]["detail"]) == {"request_id": identity}
    count = len(transport.posts)
    for unowned in ("", "other-session"):
        with pytest.raises(SDKError, match="unowned"):
            stream.stop(unowned)
    assert len(transport.posts) == count


def test_stream_stop_propagates_sdk_supersession_rejection():
    transport, stream = connect_stream(18034)
    identity = stream.start()["request_id"]
    transport.failure = SDKError("SDK request_id no longer owned", 409)
    with pytest.raises(SDKError, match="no longer owned"):
        stream.stop(identity)
    assert stream.socket is None
    assert json.loads(transport.posts[-1][1]["detail"]) == {"request_id": identity}
