"""Read-only stationary freshness policy and bounded retry behavior."""

from pathlib import Path

import pytest
from wheel_perception_core.extrinsics import HeadExtrinsics
from wheel_sdk_bridge import calibrated_capture as capture


@pytest.fixture
def calibration():
    return HeadExtrinsics(
        Path(__file__).resolve().parents[2] / "config/robots/calibration/head_camera_first.json"
    )


def test_retry_discards_attempt_and_has_deadline(monkeypatch, calibration):
    clock = [0.0]
    monkeypatch.setattr(capture.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(capture.time, "sleep", lambda dt: clock.__setitem__(0, clock[0] + dt))
    source = capture.ReadOnlyHeadCapture("http://unused", calibration, capture_timeout=0.1)
    calls = []

    def attempt():
        calls.append(1)
        if len(calls) == 1:
            raise capture.CaptureNotReady("Stale lift feedback")
        return "new image", "new context"

    monkeypatch.setattr(source, "_capture_once", attempt)
    assert source.capture_sync() == ("new image", "new context")
    assert len(source.attempts) == 1

    def stale():
        raise capture.CaptureNotReady("Stale lift feedback")

    monkeypatch.setattr(source, "_capture_once", stale)
    with pytest.raises(capture.CaptureNotReady, match="timed out"):
        source.capture_sync()
    assert clock[0] <= 0.15

    def fault():
        raise ValueError("Invalid/faulted lift feedback")

    monkeypatch.setattr(source, "_capture_once", fault)
    with pytest.raises(ValueError, match="faulted"):
        source.capture_sync()
    assert source.attempts == []


def lift_state(age=300, valid=False, **overrides):
    lift = dict(
        schema="lift_state_v1",
        transport="uart",
        mock=False,
        valid=valid,
        fault=0,
        flags=49,
        current_um=0,
        speed_rpm=0,
        receive_first_mono_ns="990000000",
        receive_mono_ns="1000000000",
        feedback_age_ms=age - 20,
        effective_age_ms=age,
    )
    lift.update(overrides)
    return dict(system={"mock": False}, server_mono_ns="1020000000", lift=lift)


def test_diagnostic_accepts_age_only_without_relabeling():
    import copy

    s = lift_state()
    original = copy.deepcopy(s)
    with pytest.raises(capture.CaptureNotReady):
        capture.LiftCaptureHistory().append_state(s)
    h = capture.LiftCaptureHistory(stationary_diagnostic=True)
    h.append_state(s)
    assert s == original and h.samples[0]["sdk_valid"] is False
    assert h.samples[0]["total_age_ms"] == 320
    # Total includes server cache age plus the entire HTTP round trip.
    capture.LiftCaptureHistory(stationary_diagnostic=True).append_state(lift_state(470), 0.01)
    with pytest.raises(capture.CaptureNotReady, match="Stale"):
        capture.LiftCaptureHistory(stationary_diagnostic=True).append_state(lift_state(470), 0.011)


@pytest.mark.parametrize(
    "overrides",
    [
        dict(fault=1),
        dict(flags=48),
        dict(flags=49 | 128),
        dict(flags=49 | 8),
        dict(mock=True),
        dict(transport="unknown"),
        dict(receive_first_mono_ns="1"),
        dict(effective_age_ms=float("nan")),
    ],
)
def test_diagnostic_rejects_fault_invalid_source_and_disconnect(overrides):
    with pytest.raises(ValueError):
        capture.LiftCaptureHistory(stationary_diagnostic=True).append_state(lift_state(**overrides))


def test_unknown_invalid_reason_and_motion_not_accepted():
    with pytest.raises(ValueError, match="other than age"):
        capture.LiftCaptureHistory(stationary_diagnostic=True).append_state(lift_state(100))
    h = capture.LiftCaptureHistory(stationary_diagnostic=True)
    for t in range(800_000_000, 1_150_000_000, 50_000_000):
        s = lift_state(
            100,
            True,
            receive_first_mono_ns=str(t - 10_000_000),
            receive_mono_ns=str(t),
            speed_rpm=1,
        )
        s["server_mono_ns"] = str(t + 20_000_000)
        h.append_state(s)
    with pytest.raises(ValueError, match="moving"):
        h.context(dict(pair_id="p", receive_mono_ns="1000000000"), "serial")


def test_diagnostic_retries_wrapped_timeout_and_waits_for_stationarity(monkeypatch, calibration):
    from urllib.error import URLError

    source = capture.ReadOnlyHeadCapture("http://unused", calibration, stationary_diagnostic=True)
    assert source.capture_timeout == 8
    assert capture.ReadOnlyHeadCapture("http://unused", calibration).capture_timeout == 3
    calls = []

    def attempt():
        calls.append(1)
        if len(calls) == 1:
            raise URLError(TimeoutError("timed out"))
        if len(calls) == 2:
            capture.LiftCaptureHistory(stationary_diagnostic=True).append_state(
                lift_state(flags=49 | 8)
            )
        return "new stationary image", "new paired height"

    monkeypatch.setattr(source, "_capture_once", attempt)
    assert source.capture_sync() == ("new stationary image", "new paired height")
    assert len(source.attempts) == 2 and len(calls) == 3

    def refused():
        raise URLError(ConnectionRefusedError("refused"))

    monkeypatch.setattr(source, "_capture_once", refused)
    with pytest.raises(URLError):
        source.capture_sync()
    assert source.attempts == []


def test_never_stationary_still_times_out(monkeypatch, calibration):
    clock = [0.0]
    monkeypatch.setattr(capture.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(capture.time, "sleep", lambda dt: clock.__setitem__(0, clock[0] + dt))
    source = capture.ReadOnlyHeadCapture(
        "http://unused", calibration, stationary_diagnostic=True, capture_timeout=0.1
    )

    def moving():
        capture.LiftCaptureHistory(stationary_diagnostic=True).append_state(
            lift_state(flags=49 | 8)
        )

    monkeypatch.setattr(source, "_capture_once", moving)
    with pytest.raises(capture.CaptureNotReady, match="timed out"):
        source.capture_sync()
    assert clock[0] <= 0.125
