"""GET-only RGB-D/lift acquisition. Host-receive alignment requires stationary lift.

All association uses robot monotonic timestamps. Workstation monotonic is used
only for request duration and local deadlines; clocks are never subtracted.
"""

import asyncio
import json
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from urllib.error import URLError
from urllib.request import urlopen

import numpy as np
from wheel_perception_core.rgbd import RGBDClient


class CaptureNotReady(ValueError):
    """Transient freshness failure; discard the whole capture before retrying."""


class LiftCaptureHistory:
    def __init__(self, capacity=256, *, stationary_diagnostic=False):
        self.stationary_diagnostic = stationary_diagnostic
        self.max_age_ms = 500 if stationary_diagnostic else 200
        self.samples = deque(maxlen=capacity)
        self.last_server = None

    def append_state(self, state, request_seconds=0.0):
        if state["system"].get("mock") is not False:
            raise ValueError("Real SDK feedback required")
        server = int(state["server_mono_ns"])
        if self.last_server is not None and server < self.last_server:
            self.samples.clear()
            self.last_server = server
            raise ValueError("Robot clock reset; discard capture")
        self.last_server = server
        lift = state.get("lift")
        if not isinstance(lift, dict) or lift.get("mock") is not False:
            raise ValueError("Valid nonmock lift feedback required")
        if lift.get("fault", 0) != 0:
            raise ValueError("Invalid/faulted lift feedback")
        stamp = int(lift["receive_mono_ns"])
        age_ms = (server - stamp) / 1e6 + request_seconds * 1000
        effective_age = float(lift["effective_age_ms"])
        if (
            not np.isfinite(request_seconds)
            or request_seconds < 0
            or not 0 <= age_ms <= 150
            or not np.isfinite(effective_age)
            or not 0 <= effective_age + age_ms <= self.max_age_ms
        ):
            raise CaptureNotReady("Stale lift feedback")
        if self.stationary_diagnostic:
            # SDK valid also includes a 250 ms age limit. Accept age-only
            # rejection for this read-only stationary diagnostic, never relabel
            # the SDK flag or admit missing/faulted/unknown-source measurements.
            flags = lift.get("flags")
            first = int(lift.get("receive_first_mono_ns", 0))
            first_age = (server - first) / 1e6 + request_seconds * 1000
            device_age = float(lift.get("feedback_age_ms", float("nan")))
            if (
                lift.get("schema") != "lift_state_v1"
                or lift.get("transport") != "uart"
                or type(flags) is not int
                or not flags & 1
                or flags & 128
                or type(lift.get("valid")) is not bool
                or "fault" not in lift
                or not 0 < first <= stamp <= server
                or not 0 <= first_age <= 300
                or not np.isfinite(device_age)
                or not 0 <= device_age <= effective_age
            ):
                raise ValueError("Invalid stationary diagnostic lift feedback")
            server_age = effective_age + (server - stamp) / 1e6
            if lift["valid"] is False and server_age <= 250:
                raise ValueError("SDK rejected lift feedback for a reason other than age")
            if flags & 8:
                raise CaptureNotReady("Lift moving; wait for a stationary capture")
        elif lift.get("valid") is not True:
            raise CaptureNotReady("Valid lift feedback not yet available")
        height = float(lift["current_um"])
        speed = float(lift["speed_rpm"])
        if not np.isfinite([height, speed]).all() or lift.get("fault", 0) != 0:
            raise ValueError("Invalid/faulted lift feedback")
        if self.samples and stamp < self.samples[-1]["stamp"]:
            raise ValueError("Out-of-order lift feedback")
        if self.samples and stamp == self.samples[-1]["stamp"]:
            return
        self.samples.append(
            dict(
                stamp=stamp,
                height_um=height,
                speed_rpm=speed,
                sdk_valid=lift["valid"],
                total_age_ms=effective_age + age_ms,
            )
        )

    def context(self, metadata, camera_serial):
        stamp = int(metadata["receive_mono_ns"])
        # Cover at least 100 ms on both sides of host image reception. It is
        # NOT an exposure timestamp or support for compensating moving lift.
        before = [s for s in self.samples if s["stamp"] <= stamp - 100_000_000]
        after = [s for s in self.samples if s["stamp"] >= stamp + 100_000_000]
        if not before or not after:
            raise CaptureNotReady("No stationary lift window bracketing image")
        a, b = before[-1], after[0]
        if b["stamp"] - a["stamp"] > 500_000_000:
            raise CaptureNotReady("Excessive lift history gap")
        samples = [s for s in self.samples if a["stamp"] <= s["stamp"] <= b["stamp"]]
        if any(y["stamp"] - x["stamp"] > 200_000_000 for x, y in zip(samples, samples[1:])):
            raise CaptureNotReady("Lift feedback gap")
        heights = [s["height_um"] for s in samples]
        if max(heights) - min(heights) > 200 or any(abs(s["speed_rpm"]) > 0.01 for s in samples):
            error = CaptureNotReady if self.stationary_diagnostic else ValueError
            raise error("Lift moving; wait for a stationary capture")
        return dict(
            schema="wheel_lift_capture_v1",
            pair_id=metadata["pair_id"],
            image_receive_mono_ns=str(stamp),
            camera_serial=str(camera_serial),
            time_basis="robot_receive_monotonic",
            stationary=True,
            height_um=float(np.median(heights)),
            height_span_um=max(heights) - min(heights),
            bracket_mono_ns=[str(a["stamp"]), str(b["stamp"])],
            sample_count=len(samples),
            diagnostic_only=True,
            exposure_synchronized=False,
            lift_max_age_ms=self.max_age_ms,
            lift_max_observed_age_ms=max(s["total_age_ms"] for s in samples),
            sdk_valid=[s["sdk_valid"] for s in samples],
            freshness_policy="stationary_diagnostic" if self.stationary_diagnostic else "strict",
        )


class ReadOnlyHeadCapture:
    """Head only; static camera-to-arm-origin needs lift, not moving-arm FK."""

    ready = True  # Operational source; hardware readiness is checked per capture.

    def __init__(
        self,
        url,
        calibration,
        *,
        max_age_ms=250,
        on_response=None,
        capture_timeout=None,
        stationary_diagnostic=False,
    ):
        self.stationary_diagnostic = stationary_diagnostic
        self.url = url.rstrip("/")
        self.calibration = calibration
        self.client = RGBDClient(url, max_age_ms=max_age_ms, on_response=on_response)
        self.max_age_ms = max_age_ms
        if capture_timeout is None:
            capture_timeout = 8.0 if stationary_diagnostic else 3.0
        if not np.isfinite(capture_timeout) or capture_timeout <= 0:
            raise ValueError("capture_timeout must be positive and finite")
        self.capture_timeout = capture_timeout
        self.attempts = []
        self._deadline = None

    def remaining(self):
        remaining = self._deadline - time.monotonic() if self._deadline is not None else 3.0
        if remaining <= 0:
            raise CaptureNotReady("Capture deadline expired")
        return remaining

    def state(self):
        start = time.monotonic()
        with urlopen(self.url + "/api/state", timeout=min(3.0, self.remaining())) as response:
            state = json.load(response)
        return state, time.monotonic() - start

    def camera_serial(self, state):
        camera = state["cameras"]["head_rgb"]
        driver = camera.get("driver", {})
        if (
            driver.get("fresh") is not True
            or driver.get("mock") is not False
            or driver.get("phase") != "streaming"
            or driver.get("aligned_to_color") is not True
            or camera.get("rgbd", {}).get("fresh") is not True
        ):
            raise ValueError("Head aligned RGB-D unavailable; RGB-only cannot supply object depth")
        serial = str(driver["serial"])
        self.calibration.check_camera("head_rgb", "head_rgb_color_optical", serial)
        return serial

    def capture_sync(self):
        self._deadline = time.monotonic() + self.capture_timeout
        self.attempts = []
        try:
            while time.monotonic() < self._deadline:
                try:
                    try:
                        result = self._capture_once()
                    except URLError as exc:
                        # urllib wraps connection/read timeouts. Retry the full
                        # diagnostic capture, never reuse an earlier image/height.
                        if not self.stationary_diagnostic or not isinstance(
                            exc.reason, TimeoutError
                        ):
                            raise
                        raise CaptureNotReady("Network timeout: " + str(exc.reason)) from exc
                    self.remaining()
                    return result
                except (CaptureNotReady, TimeoutError) as exc:
                    self.attempts.append(str(exc))
                    time.sleep(min(0.025, max(0.0, self._deadline - time.monotonic())))
            failures = [reason for reason in self.attempts if reason != "Capture deadline expired"]
            raise CaptureNotReady(
                "Capture timed out after %d attempts; last failure: %s"
                % (len(self.attempts), failures[-1] if failures else "deadline")
            )
        finally:
            self._deadline = None

    def _capture_once(self):
        history = LiftCaptureHistory(stationary_diagnostic=self.stationary_diagnostic)
        # Explicitly allowed older diagnostic frames still require >=100 ms
        # lift history before their receive timestamp. Collect that history
        # before requesting the image; retain the default 350 ms warmup.
        deadline = time.monotonic() + max(0.35, self.max_age_ms / 1000 + 0.1)
        serial = None
        while True:
            state, duration = self.state()
            identity = self.camera_serial(state)
            if serial is not None and serial != identity:
                raise ValueError("Camera source changed during capture")
            serial = identity
            history.append_state(state, duration)
            if time.monotonic() >= deadline:
                break
            time.sleep(0.025)
        self.client.timeout = min(3.0, self.remaining())
        try:
            # Image transfer can exceed the allowed gap between lift samples.
            # Continue polling lift feedback while the GET downloads, instead
            # of loosening stationarity or timestamp pairing requirements.
            with ThreadPoolExecutor(max_workers=1) as executor:
                pending = executor.submit(self.client.receive, "head_rgb")
                while not pending.done():
                    state, duration = self.state()
                    if self.camera_serial(state) != serial:
                        raise ValueError("Camera source changed during capture")
                    history.append_state(state, duration)
                    time.sleep(0.025)
                observation = pending.result()
        except ValueError as exc:
            if str(exc).startswith(("RGBD observation is stale:", "Duplicate RGBD pair")):
                raise CaptureNotReady(str(exc)) from exc
            raise
        deadline = time.monotonic() + 0.6
        while True:
            state, duration = self.state()
            if self.camera_serial(state) != serial:
                raise ValueError("Camera source changed during capture")
            history.append_state(state, duration)
            if (
                history.samples[-1]["stamp"]
                >= int(observation.metadata["receive_mono_ns"]) + 100_000_000
            ):
                break
            if time.monotonic() >= deadline:
                raise CaptureNotReady("Timed out waiting for lift feedback after image")
            time.sleep(0.025)
        context = history.context(observation.metadata, serial)
        age = (
            float(observation.metadata["transport_age_ms"]) / 1000
            + time.monotonic()
            - observation.received_at
        )
        if age * 1000 > self.max_age_ms:
            raise CaptureNotReady("Image became stale while pairing lift")
        return observation, context

    async def capture(self, target_id):
        from wheel_perception.service import SynchronizedCapture

        obs, context = await asyncio.get_running_loop().run_in_executor(None, self.capture_sync)
        captured_at = obs.received_at - float(obs.metadata["transport_age_ms"]) / 1000
        return SynchronizedCapture(obs, {}, captured_at, transform_context=context)
