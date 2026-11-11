"""Fake SDK tests only: no real network addresses or hardware motion."""

import asyncio
import json
import threading
import time
from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest
from wheel_execution.lift_executor import SDKLiftExecutor
from wheel_sdk_bridge.http_client import SDKError
from wheel_sdk_bridge.lift_motion_client import LiftMotionClient, metres_to_um
from wheel_sdk_bridge.sdk import LiftMapping


class SDK:
    def __init__(self):
        self.posts = []
        self.lock = threading.RLock()
        self.fail = None
        self.reach = True
        self.ignore_stop = False
        self.age = 10
        self.s = dict(
            system=dict(mock=False, services=dict(lift=True)),
            lift=dict(
                mock=False,
                valid=True,
                current_um=100000,
                fault=0,
                process_started_mono_ns="1",
                run_segment=0,
            ),
            lift_status=dict(
                fresh=True,
                feedback_healthy=True,
                fault=0,
                enabled=False,
                phase="disabled",
                stationary=True,
                session_present=False,
                session=False,
                dashboard_position_available=True,
                dashboard_position=False,
                allow_motion=True,
                external_control=False,
                external_owner="",
                external_lease_ms=1000,
                external_task_id=0,
                external_target_um=0,
                external_result="idle",
                ready_for_command=False,
                host_min_um=-180000,
                host_max_um=180000,
                min_um=-180000,
                max_um=180000,
            ),
        )

    def request(self, method, path, payload=None):
        with self.lock:
            if method == "GET":
                assert path == "/api/state"
                s = deepcopy(self.s)
                now = time.monotonic_ns()
                s["server_mono_ns"] = str(now)
                for key in ("lift", "lift_status"):
                    s[key]["receive_mono_ns"] = str(now - int(self.age * 1e6))
                s["lift"]["effective_age_ms"] = self.age
                return s
            assert path == "/api/lift"
            op = payload["command"]
            d = json.loads(payload["detail"])
            self.posts.append((op, d))
            ls = self.s["lift_status"]
            if op == "position_arm":
                self.s["lift"]["run_segment"] += 1
                ls.update(
                    enabled=True,
                    phase="idle",
                    dashboard_position=True,
                    external_control=True,
                    external_owner=d["owner"],
                    ready_for_command=True,
                )
            else:
                assert d["owner"] == ls["external_owner"]
            if op == "position_move":
                self.s["lift"]["run_segment"] += 1
                ls.update(
                    external_task_id=d["task_id"],
                    external_target_um=d["target_um"],
                    external_result="reached" if self.reach else "accepted",
                )
                if self.reach:
                    self.s["lift"]["current_um"] = d["target_um"]
            if op == "position_stop" and not self.ignore_stop:
                ls.update(
                    enabled=False,
                    dashboard_position=False,
                    external_control=False,
                    phase="disabled",
                    external_result="stopped",
                )
            if op == self.fail:
                raise SDKError("lost acknowledgement", uncertain=True)
            return dict(success=True)


def client(sdk):
    return LiftMotionClient("http://unused", transport=sdk, feedback_transport=sdk)


def test_full_lease_sequence_arrival_and_stop():
    sdk = SDK()
    c = client(sdk)
    assert c.preflight()["position_m"] == 0.1 and not sdk.posts
    with c:
        first = c.move(0.102)
        assert first["target_um"] == 102000 and not first["completed"]
        assert c.wait(first["task_id"])["completed"]
        time.sleep(0.18)
        second = c.move(0.1)
        assert second["task_id"] > first["task_id"]
        assert c.wait(second["task_id"])["completed"]
    assert not c.owner
    ops = [op for op, _ in sdk.posts]
    assert ops[0] == "position_arm" and ops[-1] == "position_stop"
    assert "position_heartbeat" in ops
    assert [d["sequence"] for _, d in sdk.posts] == list(range(1, len(sdk.posts) + 1))
    count = len(sdk.posts)
    time.sleep(0.1)
    assert len(sdk.posts) == count


@pytest.mark.parametrize("target", [True, float("nan"), float("inf"), 0.180001, -0.180001, "0.1"])
def test_invalid_targets_never_write(target):
    sdk = SDK()
    c = client(sdk)
    with pytest.raises(ValueError):
        c.move(target)
    assert not sdk.posts


def test_unit_conversion():
    assert metres_to_um(-0.105738) == -105738
    assert metres_to_um(0.18) == 180000


def test_stale_or_foreign_session_cannot_arm():
    sdk = SDK()
    sdk.age = 450
    with pytest.raises(SDKError, match="stale"):
        client(sdk).open()
    sdk.age = 10
    sdk.s["lift_status"]["external_control"] = True
    with pytest.raises(SDKError, match="unowned"):
        client(sdk).open()
    assert not sdk.posts


@pytest.mark.parametrize("op", ["position_arm", "position_move"])
def test_lost_write_ack_stops_without_replay(op):
    sdk = SDK()
    sdk.fail = op
    c = client(sdk)
    try:
        if op == "position_arm":
            with pytest.raises(SDKError, match="lost"):
                c.open()
        else:
            c.open()
            with pytest.raises(SDKError, match="lost"):
                c.move(0.101)
        assert c.error and c._halt.is_set()
        assert sum(x == op for x, _ in sdk.posts) == 1
        assert sdk.posts[-1][0] == "position_stop"
        with pytest.raises(SDKError):
            c.move(0.102)
    finally:
        c.close()


def test_heartbeat_failure_revokes_and_stops():
    sdk = SDK()
    c = client(sdk)
    c.open()
    sdk.fail = "position_heartbeat"
    deadline = time.monotonic() + 1
    while not c.error and time.monotonic() < deadline:
        time.sleep(0.01)
    assert c.error
    c.close()
    assert [x for x, _ in sdk.posts].count("position_heartbeat") == 1
    assert [x for x, _ in sdk.posts].count("position_stop") == 1


@pytest.mark.parametrize(
    "change", ["wrong_id", "wrong_target", "session_open", "wrong_position", "driver_restart"]
)
def test_arrival_requires_correlated_measured_locked_feedback(change):
    sdk = SDK()
    c = client(sdk)
    c.open()
    t = c.move(0.11)
    ls = sdk.s["lift_status"]
    if change == "wrong_id":
        ls["external_task_id"] += 1
    if change == "wrong_target":
        ls["external_target_um"] += 1000
    if change == "session_open":
        ls["session_present"] = True
    if change == "wrong_position":
        sdk.s["lift"]["current_um"] -= 1000
    if change == "driver_restart":
        sdk.s["lift"]["process_started_mono_ns"] = "2"
    try:
        if change in ("wrong_id", "wrong_target", "driver_restart"):
            with pytest.raises(SDKError):
                c.result(t["task_id"])
        else:
            assert not c.result(t["task_id"])["completed"]
    finally:
        ls["session_present"] = False
        c.close()


def test_pending_move_and_timeout():
    sdk = SDK()
    sdk.reach = False
    c = client(sdk)
    with c:
        t = c.move(0.12)
        with pytest.raises(SDKError, match="Reconcile"):
            c.move(0.13)
        with pytest.raises(SDKError, match="timed out"):
            c.wait(t["task_id"], 0.02)
    assert sum(op == "position_move" for op, _ in sdk.posts) == 1


def test_stop_ack_is_not_stop_confirmation():
    sdk = SDK()
    sdk.ignore_stop = True
    c = client(sdk)
    c.open()
    with pytest.raises(SDKError, match="unconfirmed"):
        c.stop(timeout=0.02)
    assert c.owner
    sdk.s["lift_status"]["enabled"] = False
    assert c.stop()["stopped"]  # Lease expiry feedback, no duplicate stop write.
    assert sum(op == "position_stop" for op, _ in sdk.posts) == 1


def test_read_does_not_block_heartbeat():
    sdk = SDK()

    class SlowFeedback:
        def request(self, *args):
            time.sleep(0.2)
            return sdk.request(*args)

    c = client(sdk)
    c.open()
    c.feedback = SlowFeedback()
    before = len(sdk.posts)
    c.snapshot()
    assert any(op == "position_heartbeat" for op, _ in sdk.posts[before:])
    c.close()


def test_loopback_http_contract():
    sdk = SDK()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            self.reply(sdk.request("GET", self.path))

        def do_POST(self):
            payload = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            assert isinstance(payload["detail"], str)
            self.reply(sdk.request("POST", self.path, payload))

        def reply(self, result):
            self.send_response(200)
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with LiftMotionClient("http://127.0.0.1:" + str(server.server_port)) as c:
            assert c.wait(c.move(0.101)["task_id"])["completed"]
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def test_async_ik_mapping_is_explicit():
    async def run():
        sdk = SDK()
        e = SDKLiftExecutor(client(sdk))
        with pytest.raises(ValueError, match="LiftMapping"):
            await e.move_joint_m(0.2)
        assert not sdk.posts
        e.mapping = LiftMapping(-1, 0.2, -180000, 180000)
        await e.open()
        try:
            t = await e.move_joint_m(0.25)
            assert t["target_um"] == -50000
            assert (await e.wait(t["task_id"]))["completed"]
        finally:
            await e.close()

    asyncio.run(run())


def test_temporary_invalid_feedback_waits_for_new_valid_sample():
    sdk = SDK()
    original = sdk.request
    reads = []

    def request(method, *args):
        s = original(method, *args)
        if method == "GET":
            reads.append(1)
            if len(reads) < 3:
                s["lift"]["valid"] = False
        return s

    sdk.request = request
    assert client(sdk).snapshot()["lift"]["valid"]
    assert len(reads) == 3 and not sdk.posts


def test_arm_waits_for_gateway_to_publish_new_owner():
    sdk = SDK()
    original = sdk.request
    cached = deepcopy(sdk.s)
    old_reads = []

    def request(method, *args):
        s = original(method, *args)
        if method == "GET" and sdk.posts and not old_reads:
            old_reads.append(1)
            s["lift_status"].update(cached["lift_status"])
        return s

    sdk.request = request
    with client(sdk) as c:
        assert c.owner
    assert old_reads


def test_driver_limits_reject_without_move_write():
    sdk = SDK()
    sdk.s["lift_status"]["max_um"] = 110000
    c = client(sdk)
    with c:
        with pytest.raises(ValueError, match="limits"):
            c.move(0.12)
    assert not any(op == "position_move" for op, _ in sdk.posts)


def test_async_cancel_wait_stops_and_releases_session():
    async def run():
        sdk = SDK()
        sdk.reach = False
        c = client(sdk)
        e = SDKLiftExecutor(c)
        await e.open()
        t = await e.move_lift_m(0.11)
        task = asyncio.create_task(e.wait(t["task_id"]))
        await asyncio.sleep(0.02)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert not c.owner
        assert sdk.posts[-1][0] == "position_stop"

    asyncio.run(run())


def test_result_feedback_failure_stops_without_more_motion():
    sdk = SDK()
    c = client(sdk)
    c.open()
    t = c.move(0.11)
    sdk.age = 450
    with pytest.raises(SDKError, match="stale"):
        c.result(t["task_id"])
    sdk.age = 10
    c.close()
    assert sum(op == "position_move" for op, _ in sdk.posts) == 1
    assert sum(op == "position_stop" for op, _ in sdk.posts) == 1


def test_inhibited_firmware_is_reported_without_motion_write():
    sdk = SDK()
    sdk.s["lift"].update(flags=128, result=11, target_status=230, valid=False)
    sdk.s["lift_status"].update(health="motion_inhibited", feedback_healthy=False)
    with pytest.raises(SDKError, match="motion inhibited: result=11 target_status=230"):
        client(sdk).open()
    assert not sdk.posts


def test_startup_grace_is_progress_only_never_arrival_or_planning():
    sdk = SDK()
    c = client(sdk)
    c.open()
    t = c.move(0.11)
    sdk.s["lift"].update(valid=False, flags=1, speed_rpm=0)
    sdk.s["lift_status"].update(
        phase="move_ack",
        startup_feedback_grace=True,
        motion_started_mono_ns=str(time.monotonic_ns()),
        startup_origin_um=110000,
    )
    original = sdk.request

    def request(method, *args):
        s = original(method, *args)
        if method == "GET":
            s["lift"]["effective_age_ms"] = 900
        return s

    sdk.request = request
    r = c.result(t["task_id"])
    assert not r["completed"] and r["position_m"] is None
    with pytest.raises(SDKError):
        c.snapshot()
    sdk.request = original
    sdk.s["lift"]["valid"] = True
    sdk.s["lift_status"]["phase"] = "idle"
    assert c.result(t["task_id"])["completed"]
    c.close()
