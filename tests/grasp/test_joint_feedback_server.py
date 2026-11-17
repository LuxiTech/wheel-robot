import importlib.util
import json
import threading
import time
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

spec = importlib.util.spec_from_file_location(
    "joint_feedback_server",
    Path(__file__).resolve().parents[2] / "deployment/robot/joint_feedback_server.py",
)
server_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server_module)


def test_cache_recomputes_age_and_does_not_relabel_old_data():
    c = server_module.StateCache()
    assert c.snapshot()["system"]["mock"] is None
    now = time.monotonic_ns()
    c.arms = dict(receive_mono_ns=str(now - 401_000_000))
    c.startup = dict(receive_mono_ns=str(now))
    c.mock, c.checked_ns = False, now
    s = c.snapshot()
    assert not s["arms"]["fresh"] and s["arm_startup"]["fresh"]
    assert s["arms"]["receive_mono_ns"] == c.arms["receive_mono_ns"]
    assert "fresh" not in c.arms
    assert s["system"]["mock"] is False
    c.checked_ns = now - 3_000_000_000
    assert c.snapshot()["system"]["mock"] is None


def test_service_is_get_only():
    server = server_module.make_server(server_module.StateCache(), "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = "http://127.0.0.1:" + str(server.server_port)
    try:
        with urlopen(url + "/api/state") as response:
            assert json.load(response)["feedback_source"] == "ros_joint_feedback_v1"
        with pytest.raises(HTTPError) as error:
            urlopen(Request(url + "/api/arm_startup", data=b"{}", method="POST"))
        assert error.value.code == 501
        with pytest.raises(HTTPError) as error:
            urlopen(url + "/api/other")
        assert error.value.code == 404
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def test_server_400ms_boundary(monkeypatch):
    monkeypatch.setattr(server_module.time, "monotonic_ns", lambda: 1_000_000_000)
    c = server_module.StateCache()
    c.arms = dict(receive_mono_ns="620000000")
    assert c.snapshot()["arms"]["fresh"]
    c.arms["receive_mono_ns"] = "600000000"
    assert c.snapshot()["arms"]["fresh"]
    c.arms["receive_mono_ns"] = "599999999"
    assert not c.snapshot()["arms"]["fresh"]


def test_cache_waits_for_live_topic_pair_without_rewriting_timestamps(monkeypatch):
    cache = server_module.StateCache()
    now = time.monotonic_ns()
    cache.arms = dict(receive_mono_ns=str(now), position=[0.2] * 16)
    cache.startup = dict(receive_mono_ns=str(now - 50_000_000), phase="idle")
    waiting = threading.Event()
    original_wait = cache.updated.wait

    def wait_for_update(timeout):
        waiting.set()
        return original_wait(timeout)

    monkeypatch.setattr(cache.updated, "wait", wait_for_update)
    result = []
    reader = threading.Thread(target=lambda: result.append(cache.snapshot()))
    reader.start()
    try:
        assert waiting.wait(1)
        with cache.updated:
            cache.startup = dict(receive_mono_ns=str(now), phase="ready")
            cache.updated.notify_all()
        reader.join(1)
        assert not reader.is_alive()
        assert result[0]["arm_startup"]["phase"] == "ready"
        assert result[0]["arm_startup"]["receive_mono_ns"] == str(now)
        assert result[0]["arms"]["receive_mono_ns"] == str(now)
        assert result[0]["arms"]["position"] == [0.2] * 16
    finally:
        reader.join(1)


def test_missing_topic_update_returns_mismatch_for_client_to_reject():
    cache = server_module.StateCache()
    now = time.monotonic_ns()
    cache.arms = dict(receive_mono_ns=str(now))
    cache.startup = dict(receive_mono_ns=str(now - 50_000_000))
    started = time.monotonic()
    snapshot = cache.snapshot()
    elapsed = time.monotonic() - started
    assert 0.06 <= elapsed < 1
    assert snapshot["arms"]["receive_mono_ns"] == str(now)
    assert snapshot["arm_startup"]["receive_mono_ns"] == str(now - 50_000_000)


def test_stream_route_returns_new_arm_sample_without_waiting_for_slow_status(monkeypatch):
    cache = server_module.StateCache()
    now = time.monotonic_ns()
    cache.arms = dict(receive_mono_ns=str(now), position=[0.3] * 16)
    cache.startup = dict(receive_mono_ns=str(now - 90_000_000), phase="streaming")
    monkeypatch.setattr(
        cache.updated, "wait", lambda *args: pytest.fail("Fast feedback must not wait")
    )
    snapshot = cache.snapshot(pair_topics=False)
    assert snapshot["arms"]["receive_mono_ns"] == str(now)
    assert snapshot["arm_startup"]["receive_mono_ns"] == str(now - 90_000_000)
    assert snapshot["topic_pairing"] == "independent_receive_monotonic"
