"""Execution evidence must correlate frames and commands, and fail closed on loss."""

import hashlib
import json
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
from wheel_execution.execution_trace import ExecutionTrace
from wheel_execution.grasp_executor import StationaryGraspExecutor


def test_trace_preserves_original_frame_and_correlates_command(tmp_path):
    trace = ExecutionTrace(tmp_path / "run")
    body = b"original decoded multipart body\0depth floats"
    trace.rgbd_response({"X-RGBD-Pair-Id": "pair-7", "Content-Encoding": "x-lz4-block"}, body)
    q = np.zeros(16)
    trace.record(
        "reference_send_attempt",
        request_id="request-2",
        sequence=11,
        pair_id="pair-7",
        image_receive_mono_ns=12345,
        reference_positions=q,
    )
    q[0] = 99  # Queued evidence must not change with a later control tick.
    trace.record("reference_sent", request_id="request-2", sequence=11)
    trace.close()
    rows = [
        json.loads(line) for line in (trace.directory / "events.jsonl").read_text().splitlines()
    ]
    assert [row["event_sequence"] for row in rows] == [1, 2, 3, 4]
    assert len({row["run_id"] for row in rows}) == 1
    frame, attempt, sent = rows[1:]
    assert (trace.directory / frame["artifact"]["file"]).read_bytes() == body
    assert frame["artifact"]["sha256"] == hashlib.sha256(body).hexdigest()
    assert frame["detail"]["stored_body_encoding"] == "decoded_multipart"
    assert attempt["detail"]["pair_id"] == frame["detail"]["headers"]["X-RGBD-Pair-Id"]
    assert attempt["detail"]["reference_positions"][0] == 0
    assert attempt["detail"]["sequence"] == sent["detail"]["sequence"]
    with pytest.raises(FileExistsError):
        ExecutionTrace(trace.directory)


def test_recording_capacity_loss_is_explicit_and_never_blocks_control(tmp_path, monkeypatch):
    release = threading.Event()
    writer = ExecutionTrace._write

    def delayed(self):
        release.wait(1)
        writer(self)

    monkeypatch.setattr(ExecutionTrace, "_write", delayed)
    trace = ExecutionTrace(tmp_path / "run", capacity=1)
    try:
        with pytest.raises(RuntimeError, match="queue full"):
            trace.record("must_not_drop_silently")
        with pytest.raises(RuntimeError, match="queue full"):
            trace.check()
    finally:
        release.set()
        with pytest.raises(RuntimeError, match="queue full"):
            trace.close()


def test_writer_failure_reaches_producer_and_close(tmp_path):
    trace = ExecutionTrace(tmp_path / "run")
    trace.record("bad_input", invalid=float("nan"))
    deadline = time.monotonic() + 2
    while trace.error is None and time.monotonic() < deadline:
        time.sleep(0.005)
    with pytest.raises(RuntimeError, match="logging failed"):
        trace.check()
    with pytest.raises(RuntimeError, match="logging failed"):
        trace.close()


def test_stop_result_preserves_original_error_when_trace_is_already_broken(tmp_path):
    class BrokenTrace:
        def record(self, *args, **kwargs):
            raise OSError("trace unavailable")

    executor = StationaryGraspExecutor.__new__(StationaryGraspExecutor)
    executor.driver = SimpleNamespace(trace=BrokenTrace())
    executor.events = []
    executor.clock = lambda: 100.0
    executor.output = tmp_path
    executor.may_be_holding = False
    executor.event("BLOCKED", error="original following error")
    result = json.loads((tmp_path / "execution.json").read_text())
    assert result["events"][0]["error"] == "original following error"


def test_grasp_visualization_preserves_frame_time_and_projection(tmp_path):
    pytest.importorskip("cv2")
    trace = ExecutionTrace(tmp_path / "run")
    point = np.eye(4)
    point[:3, 3] = [0, 0, 2]
    target = dict(pair_id="frame-original", grasp={"left": point}, pregrasp={"left": point})
    rgb = np.zeros((40, 60, 3), dtype=np.uint8)
    metadata = dict(stamp=dict(sec=123, nanosec=456), receive_mono_ns="999")
    trace.grasp_points(
        rgb,
        np.array([[20, 0, 30], [0, 20, 20], [0, 0, 1]]),
        np.eye(4),
        target,
        metadata,
        dict(height_um=100000),
    )
    metadata["stamp"]["sec"] = 9999
    trace.close()
    rows = [
        json.loads(line) for line in (trace.directory / "events.jsonl").read_text().splitlines()
    ]
    row = rows[-1]
    evidence = json.loads((trace.directory / row["artifact"]["metadata"]).read_text())
    assert evidence["detail"]["source_metadata"]["stamp"]["sec"] == 123
    assert evidence["detail"]["projections"]["left_grasp"]["pixel"] == [30, 20]
    svg = (trace.directory / row["artifact"]["file"]).read_text()
    assert 'cx="30.00" cy="20.00"' in svg
    assert "frame-original" in svg
    assert trace.pending_bytes == 0
