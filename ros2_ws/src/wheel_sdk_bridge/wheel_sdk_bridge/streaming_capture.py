"""Continuous read-only head capture reusing measured lift history across frames."""

import json
import threading
import time
from urllib.request import urlopen

from wheel_sdk_bridge.calibrated_capture import (
    CaptureNotReady,
    CapturePairRejected,
    LiftCaptureHistory,
    ReadOnlyHeadCapture,
)


class StreamingHeadCapture(ReadOnlyHeadCapture):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.history = LiftCaptureHistory(
            stationary_diagnostic=self.stationary_diagnostic,
            strict_max_age_ms=self.strict_lift_max_age_ms,
            max_sample_gap_ms=self.lift_max_sample_gap_ms,
        )
        self.history_lock = threading.Lock()
        self.stop_poll = threading.Event()
        self.poll_error = None
        self.serial = None
        self.worker = None

    def start(self):
        if self.worker is not None:
            return

        def poll():
            while not self.stop_poll.is_set():
                started = time.monotonic()
                try:
                    with urlopen(self.url + "/api/state", timeout=0.4) as response:
                        state = json.load(response)
                    duration = time.monotonic() - started
                    serial = self.camera_serial(state)
                    with self.history_lock:
                        if self.serial is not None and self.serial != serial:
                            raise ValueError("Streaming camera identity changed")
                        self.serial = serial
                        self.history.append_state(state, duration)
                except CaptureNotReady:
                    # Missing samples never extend a previous frame's lease.
                    pass
                except Exception as exc:
                    self.poll_error = exc
                    return
                self.stop_poll.wait(0.025)

        self.worker = threading.Thread(target=poll, daemon=True)
        self.worker.start()

    def close(self):
        self.stop_poll.set()
        if self.worker is not None:
            self.worker.join(timeout=0.5)

    def _capture_once(self):
        self.start()
        while self.remaining() > 0:
            if self.poll_error is not None:
                raise RuntimeError("Streaming lift history failed: " + str(self.poll_error))
            with self.history_lock:
                warm = (
                    len(self.history.samples) >= 3
                    and self.history.samples[-1]["stamp"] - self.history.samples[0]["stamp"]
                    >= 300_000_000
                )
            if warm:
                break
            time.sleep(0.01)
        self.client.timeout = min(0.4, self.remaining())
        observation = self.client.receive("head_rgb")
        while self.remaining() > 0:
            if self.poll_error is not None:
                raise RuntimeError("Streaming lift history failed: " + str(self.poll_error))
            with self.history_lock:
                try:
                    context = self.history.context(
                        observation.metadata, self.serial, fixed_lift_stream=True
                    )
                    break
                except CapturePairRejected:
                    raise
                except CaptureNotReady:
                    context = None
            time.sleep(0.005)
        if context is None:
            raise CaptureNotReady("No valid lift bracket for streaming image")
        age = (
            float(observation.metadata["transport_age_ms"]) / 1000
            + time.monotonic()
            - observation.received_at
        )
        if age * 1000 > self.max_age_ms:
            raise CaptureNotReady("Streaming image became stale while pairing lift")
        return observation, context
