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


def test_explicit_sdk_lift_age_still_requires_native_validity():
    state = lift_state(220, valid=True)
    with pytest.raises(capture.CaptureNotReady):
        capture.LiftCaptureHistory().append_state(state)
    history = capture.LiftCaptureHistory(strict_max_age_ms=250)
    history.append_state(state)
    assert history.samples[0]["sdk_valid"] is True
    assert history.samples[0]["total_age_ms"] == 240
    with pytest.raises(capture.CaptureNotReady):
        capture.LiftCaptureHistory(strict_max_age_ms=250).append_state(lift_state(220, valid=False))
    with pytest.raises(capture.CaptureNotReady):
        capture.LiftCaptureHistory(strict_max_age_ms=250).append_state(lift_state(231, valid=True))
    with pytest.raises(ValueError, match="250"):
        capture.LiftCaptureHistory(strict_max_age_ms=251)


def test_fixed_lift_stream_brackets_image_with_established_native_valid_history(calibration):
    from types import SimpleNamespace

    from wheel_perception_core.extrinsics import CalibratedHeadTransform

    history = capture.LiftCaptureHistory(strict_max_age_ms=250, max_sample_gap_ms=210)
    height = calibration.config["reference_height_um"]
    for stamp in range(1_000_000_000, 1_500_000_000, 100_000_000):
        state = lift_state(
            100,
            valid=True,
            current_um=height,
            receive_mono_ns=str(stamp),
            receive_first_mono_ns=str(stamp - 10_000_000),
        )
        state["server_mono_ns"] = str(stamp + 20_000_000)
        history.append_state(state)
    metadata = dict(
        pair_id="live",
        receive_mono_ns="1399000000",
        camera_id="head_rgb",
        frame_id=calibration.config["camera"]["optical_frames"][0],
    )
    serial = calibration.config["camera"]["serials"][0]
    with pytest.raises(capture.CaptureNotReady):
        history.context(metadata, serial)
    context = history.context(metadata, serial, fixed_lift_stream=True)
    assert context["schema"] == "wheel_lift_stream_capture_v1"
    assert context["bracket_mono_ns"] == ["1300000000", "1400000000"]
    assert context["stationary_window_mono_ns"] == [1_000_000_000, 1_400_000_000]
    assert all(context["sdk_valid"]) and context["exposure_synchronized"] is False
    transform = CalibratedHeadTransform(calibration)
    matrix, _ = transform.resolve(SimpleNamespace(metadata=metadata), context)
    assert matrix.shape == (4, 4)
    context["sdk_valid"][0] = False
    with pytest.raises(ValueError, match="Native-valid"):
        transform.resolve(SimpleNamespace(metadata=metadata), context)
    history.samples[0]["height_um"] += 500
    with pytest.raises(ValueError, match="moving"):
        history.context(metadata, serial, fixed_lift_stream=True)


def test_stream_pair_distinguishes_missing_future_sample_from_unrepairable_past_gap():
    history = capture.LiftCaptureHistory()

    def sample(stamp):
        return dict(
            stamp=stamp, height_um=0, stationary=True, sdk_valid=True, total_age_ms=100, speed_rpm=0
        )

    history.samples.extend(
        sample(s) for s in (800_000_000, 900_000_000, 1_000_000_000, 1_100_000_000)
    )
    metadata = dict(pair_id="real", receive_mono_ns="1250000000")
    with pytest.raises(capture.CaptureNotReady) as future:
        history.context(metadata, "camera", fixed_lift_stream=True)
    assert not isinstance(future.value, capture.CapturePairRejected)
    history.samples.append(sample(1_400_000_000))
    with pytest.raises(capture.CapturePairRejected, match="feedback gap"):
        history.context(metadata, "camera", fixed_lift_stream=True)
    history.samples.append(sample(1_600_000_000))
    with pytest.raises(capture.CapturePairRejected, match="feedback gap"):
        history.context(metadata, "camera", fixed_lift_stream=True)
    history.samples.append(sample(1_700_000_000))
    next_image = dict(pair_id="new", receive_mono_ns="1650000000")
    repaired = history.context(next_image, "camera", fixed_lift_stream=True)
    assert repaired["bracket_mono_ns"] == ["1600000000", "1700000000"]


@pytest.mark.parametrize(
    "overrides",
    [
        dict(fault=1),
        dict(flags=48),
        dict(flags=49 | 128),
        dict(flags=49 | 8),
        dict(flags=49 | 4),
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
            speed_rpm=3,
        )
        s["server_mono_ns"] = str(t + 20_000_000)
        h.append_state(s)
    with pytest.raises(ValueError, match="moving"):
        h.context(dict(pair_id="p", receive_mono_ns="1000000000"), "serial")


@pytest.mark.parametrize("speed_rpm, accepted", [(-2, True), (-1, True), (2, True), (3, False)])
def test_stationary_capture_uses_verified_sdk_rpm_semantics(speed_rpm, accepted):
    history = capture.LiftCaptureHistory(stationary_diagnostic=True)
    for stamp in range(800_000_000, 1_250_000_000, 50_000_000):
        state = lift_state(
            receive_first_mono_ns=str(stamp - 10_000_000),
            receive_mono_ns=str(stamp),
            speed_rpm=speed_rpm,
        )
        state["server_mono_ns"] = str(stamp + 20_000_000)
        history.append_state(state)
    metadata = dict(pair_id="p", receive_mono_ns="1000000000")
    if accepted:
        assert history.context(metadata, "serial")["height_span_um"] == 0
    else:
        with pytest.raises(capture.CaptureNotReady, match="Lift moving"):
            history.context(metadata, "serial")


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
