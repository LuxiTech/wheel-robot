import copy
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import pytest
from wheel_sdk_bridge.http_client import SDKError
from wheel_sdk_bridge.joint_client import JointClient


def state():
    return dict(
        system=dict(mock=False),
        server_mono_ns="1000000000",
        arms=dict(
            fresh=True,
            receive_mono_ns="999000000",
            joint_order="right_then_left",
            phase="manual_pose_hold",
            enabled=True,
            position=[0.0] * 16,
            velocity=[0.0] * 16,
            valid=[True] * 16,
            age_ns=[10000] * 16,
        ),
        arm_startup=dict(
            fresh=True,
            receive_mono_ns="999000000",
            static_compensation_rad=[0.0] * 16,
            joint_control=dict(
                phase="idle",
                reason="test",
                request_id="",
                unit="rad",
                joint_order="right_then_left",
                lower=[-2.0] * 16,
                upper=[2.0] * 16,
                target=[0.0] * 16,
                min_speed_rad_s=0.02,
                max_speed_rad_s=0.2,
            ),
        ),
    )


class Transport:
    def __init__(self):
        self.s = state()
        self.posts = []
        self.failure = None

    def request(self, method, path, payload=None):
        if method == "GET":
            assert path == "/api/state"
            return copy.deepcopy(self.s)
        self.posts.append((path, payload))
        if self.failure:
            raise self.failure
        return dict(success=True, message="accepted")


def client(transport):
    return JointClient("http://unused", {}, transport=transport)


def test_shipped_policy_sends_move_and_owned_stop():
    root = Path(__file__).resolve().parents[2]
    policy = json.loads((root / "config/robots/joint_control.json").read_text())
    t = Transport()
    t.s["feedback_source"] = "ros_joint_feedback_v1"
    c = JointClient(policy["base_url"], policy, transport=t, feedback_transport=t)
    assert c.preflight()["motion_allowed"]
    result = c.move([0] * 16)
    assert result["accepted"]
    t.s["arm_startup"]["joint_control"]["request_id"] = result["request_id"]
    c.stop(result["request_id"])
    assert [payload["command"] for _, payload in t.posts] == ["joint_move", "joint_stop"]


def test_full_vector_rad_and_no_automatic_enable_or_compensation_drop():
    t = Transport()
    c = client(t)
    q = [0.0] * 16
    q[8] = 0.01
    t.s["arms"]["enabled"] = False
    with pytest.raises(SDKError, match="implicit"):
        c.move(q)
    t.s["arms"]["enabled"] = True
    t.s["arm_startup"]["static_compensation_rad"][3] = 0.1
    with pytest.raises(SDKError, match="compensation"):
        c.move(q)
    assert not t.posts
    t.s["arm_startup"]["static_compensation_rad"] = [0] * 16
    result = c.move(q)
    path, payload = t.posts[0]
    detail = json.loads(payload["detail"])
    assert path == "/api/arm_startup" and payload["command"] == "joint_move"
    assert detail["positions"] == q and detail["unit"] == "rad" and detail["speed_rad_s"] == 0.02
    assert result["accepted"] and not result["completed"]
    with pytest.raises(SDKError, match="reconciled"):
        c.move(q)
    assert len(t.posts) == 1


def test_invalid_target_and_stale_feedback_no_write():
    t = Transport()
    c = client(t)
    for q in [[0] * 15, [3] * 16, [float("nan")] * 16, [True] * 16]:
        with pytest.raises(ValueError):
            c.prepare(q)
    for speed in [0.01, 0.3, float("nan"), True]:
        with pytest.raises(ValueError):
            c.prepare([0] * 16, speed)
    t.s["arms"]["age_ns"][9] = 500000000
    with pytest.raises(SDKError, match="stale"):
        c.move([0] * 16)
    assert not t.posts


def test_lost_ack_is_uncertain_and_never_retried_or_disabled():
    t = Transport()
    c = client(t)
    t.failure = SDKError("timeout", 503, uncertain=True)
    with pytest.raises(SDKError):
        c.move([0] * 16)
    request = c.pending_id
    assert request
    with pytest.raises(SDKError, match="reconciled"):
        c.move([0] * 16)
    assert len(t.posts) == 1
    assert c.result(request)["phase"] == "unknown_or_superseded"
    assert c.pending_id == request


def test_arrival_requires_matching_id_and_measured_feedback():
    t = Transport()
    c = client(t)
    q = [0.1] * 16
    accepted = c.move(q)
    request = accepted["request_id"]
    j = t.s["arm_startup"]["joint_control"]
    j.update(request_id=request, target=q, phase="ready")
    assert not c.result(request)["completed"]
    t.s["arms"]["position"] = q
    assert c.result(request)["completed"] and c.pending_id is None
    with pytest.raises(SDKError, match="unowned"):
        c.stop("another-client")
    assert len(t.posts) == 1


def test_protocol_against_local_http_stub():
    calls = []

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            assert self.path == "/api/state"
            self.send_response(200)
            self.end_headers()
            self.wfile.write(json.dumps(state()).encode())

        def do_POST(self):
            calls.append(
                (self.path, json.loads(self.rfile.read(int(self.headers["Content-Length"]))))
            )
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'{"success":true,"message":"stub only"}')

        def log_message(self, *args):
            pass

    server = HTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        c = JointClient("http://127.0.0.1:" + str(server.server_port), {})
        assert c.move([0] * 16)["accepted"]
        assert len(calls) == 1 and json.loads(calls[0][1]["detail"])["unit"] == "rad"
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def test_separate_feedback_channel_never_falls_back_and_commands_stay_on_sdk():
    command, feedback = Transport(), Transport()
    feedback.s["feedback_source"] = "ros_joint_feedback_v1"
    c = JointClient(
        "http://unused",
        dict(feedback_url="http://feedback"),
        transport=command,
        feedback_transport=feedback,
    )
    command.s["system"]["mock"] = True  # Old GET channel must not be used.
    assert c.move([0] * 16)["accepted"]
    assert len(command.posts) == 1 and not feedback.posts
    feedback.s["system"]["mock"] = None
    with pytest.raises(SDKError, match="Real robot"):
        c.snapshot()
    assert len(command.posts) == 1


@pytest.mark.parametrize("age_ms,accepted", [(90, True), (149, True), (151, False)])
def test_stream_feedback_retains_independent_timestamps_and_150ms_status_lease(age_ms, accepted):
    from types import SimpleNamespace

    sample = state()
    sample["feedback_source"] = "ros_joint_feedback_v1"
    sample["topic_pairing"] = "independent_receive_monotonic"
    sample["arm_startup"]["receive_mono_ns"] = str(1_000_000_000 - age_ms * 1_000_000)

    def receive(method, path):
        assert method == "GET" and path == "/api/stream_state"
        return copy.deepcopy(sample)

    reader = JointClient(
        "http://unused",
        dict(
            feedback_url="http://feedback",
            feedback_path="/api/stream_state",
            feedback_max_age_s=0.15,
        ),
        feedback_transport=SimpleNamespace(request=receive),
    )
    if accepted:
        assert reader.snapshot()["arms"]["receive_mono_ns"] == "999000000"
    else:
        with pytest.raises(SDKError, match="stale"):
            reader.snapshot()


def test_rejected_raw_feedback_and_timing_are_saved(tmp_path):
    t = Transport()
    t.s["arm_startup"]["receive_mono_ns"] = "500000000"
    path = tmp_path / "feedback.jsonl"
    c = JointClient("http://unused", dict(feedback_diagnostics=str(path)), transport=t)
    with pytest.raises(SDKError, match="arm_startup"):
        c.snapshot()
    record = json.loads(path.read_text())
    assert not record["accepted"]
    assert record["raw"] == t.s
    assert record["timing"]["arm_startup_effective_ms"] >= 500
    assert record["timing"]["request_ms"] >= 0
    assert not t.posts


def test_fast_feedback_rejects_mismatched_timestamps():
    t = Transport()
    t.s["feedback_source"] = "ros_joint_feedback_v1"
    t.s["arm_startup"]["receive_mono_ns"] = "960000000"
    c = JointClient("http://unused", dict(feedback_url="http://feedback"), feedback_transport=t)
    with pytest.raises(SDKError, match="timestamp mismatch"):
        c.snapshot()
    assert not t.posts


def test_network_failure_is_recorded_without_fallback(tmp_path):
    class Failed:
        def request(self, *args):
            raise SDKError("test connection failed")

    t = Transport()
    path = tmp_path / "failed.jsonl"
    c = JointClient(
        "http://unused",
        dict(feedback_diagnostics=str(path)),
        transport=t,
        feedback_transport=Failed(),
    )
    with pytest.raises(SDKError, match="connection failed"):
        c.snapshot()
    record = json.loads(path.read_text())
    assert record["raw"] is None and not record["accepted"]
    assert not t.posts


def test_slow_request_rejected_at_400ms_limit(monkeypatch):
    import wheel_sdk_bridge.joint_client as module

    ticks = iter([10.0, 10.410, 10.411])
    monkeypatch.setattr(module.time, "monotonic", lambda: next(ticks))
    t = Transport()
    c = client(t)
    with pytest.raises(SDKError, match="Joint feedback stale: arms"):
        c.snapshot()
    assert c.last_diagnostic["timing"]["request_ms"] > 400
    assert not t.posts


@pytest.mark.parametrize(
    "elapsed,axis_age_ns,accepted",
    [(0.380, 10000, True), (0.410, 10000, False), (0.380, 30000000, False)],
)
def test_400ms_limit_includes_request_and_per_joint_age(
    monkeypatch, elapsed, axis_age_ns, accepted
):
    import wheel_sdk_bridge.joint_client as module

    ticks = iter([10.0, 10.0 + elapsed, 10.0 + elapsed])
    monkeypatch.setattr(module.time, "monotonic", lambda: next(ticks))
    t = Transport()
    t.s["arms"]["age_ns"][0] = axis_age_ns
    c = client(t)
    if accepted:
        assert c.snapshot()["arms"]["fresh"]
    else:
        with pytest.raises(SDKError, match="stale"):
            c.snapshot()
    assert not t.posts


def compensated_client(scope="all_control_modes"):
    t = Transport()
    b = t.s["arm_startup"]
    b.update(
        control_generation="g1",
        nominal_target=[0.0] * 16,
        support_target=[0.05] * 7 + [0.0] + [0.05] * 7 + [0.0],
        static_compensation_rad=[0.05] * 7 + [0.0] + [0.05] * 7 + [0.0],
        static_compensation=dict(scope=scope, arm_joints_enabled=True, grippers_enabled=False),
        act=dict(raw_post_compensation=False),
        manual_pose=dict(fault_latched=False),
    )
    c = JointClient(
        "http://unused",
        dict(hold_takeover_enabled=True, compensated_handover_protocol="nominal_target_v1"),
        transport=t,
    )
    return c, t


@pytest.mark.parametrize("scope", ["all_control_modes", "non_act_control_modes"])
def test_takeover_preserves_nominal_and_requires_result_before_next_move(scope):
    c, t = compensated_client(scope)
    result = c.takeover_hold()
    assert result["accepted"]
    detail = json.loads(t.posts[0][1]["detail"])
    assert detail["positions"] == [0.0] * 16 and detail["speed_rad_s"] == 0.02
    assert t.posts[0][1]["command"] == "joint_move"
    with pytest.raises(SDKError, match="reconciled"):
        c.move([0.1] * 16)
    assert len(t.posts) == 1


@pytest.mark.parametrize(
    "change", ["support", "gripper", "disabled", "fault", "unsettled", "missing_protocol"]
)
@pytest.mark.parametrize("scope", ["all_control_modes", "non_act_control_modes"])
def test_takeover_refuses_unsafe_or_incompatible_states(change, scope):
    c, t = compensated_client(scope)
    b = t.s["arm_startup"]
    if change == "support":
        b["support_target"][0] = 0.2
    if change == "gripper":
        b["static_compensation_rad"][7] = 0.01
    if change == "disabled":
        t.s["arms"]["enabled"] = False
    if change == "fault":
        b["manual_pose"]["fault_latched"] = True
    if change == "unsettled":
        t.s["arms"]["position"][0] = 0.03
    if change == "missing_protocol":
        c.policy.pop("compensated_handover_protocol")
    with pytest.raises(SDKError):
        c.takeover_hold()
    assert not t.posts


@pytest.mark.parametrize("scope", ["all_control_modes", "non_act_control_modes"])
def test_compensated_move_supported_without_motion_policy(scope):
    c, t = compensated_client(scope)
    c.move([0.01] * 16)
    assert len(t.posts) == 1


@pytest.mark.parametrize(
    "scope,act",
    [
        ("unknown", {}),
        (None, {}),
        ("non_act_control_modes", {}),
        ("non_act_control_modes", {"raw_post_compensation": True}),
    ],
)
def test_compensated_move_rejects_unknown_scope_or_raw_act(scope, act):
    c, t = compensated_client(scope)
    t.s["arm_startup"]["act"] = act
    with pytest.raises(SDKError, match="handover is not validated"):
        c.move([0.01] * 16)
    assert not t.posts


def test_takeover_changed_nominal_does_not_send():
    c, t = compensated_client()
    original = t.request
    count = 0

    def request(method, path, payload=None):
        nonlocal count
        if method == "GET":
            count += 1
            if count == 3:
                t.s["arm_startup"]["control_generation"] = "g2"
        return original(method, path, payload)

    t.request = request
    with pytest.raises(SDKError, match="changed"):
        c.takeover_hold()
    assert not t.posts


def test_explicit_sdk_enable_sends_only_requested_joint_target():
    t = Transport()
    t.s["arms"]["enabled"] = False
    # Disabled SDK reanchors from measured feedback, not old compensation.
    t.s["arm_startup"]["static_compensation_rad"][3] = 0.1
    c = client(t)
    q = [0.0] * 16
    q[8] = -0.1
    assert c.move(q, enable_if_needed=True)["accepted"]
    assert len(t.posts) == 1
    assert t.posts[0][1]["command"] == "joint_move"
    assert json.loads(t.posts[0][1]["detail"])["positions"] == q


def test_enable_option_is_explicit_boolean():
    t = Transport()
    c = client(t)
    with pytest.raises(ValueError):
        c.move([0] * 16, enable_if_needed="false")
    assert not t.posts
