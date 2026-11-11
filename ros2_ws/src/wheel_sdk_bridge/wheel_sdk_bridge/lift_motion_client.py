"""Leased lift position control through the SDK's public HTTP service.

Positions are absolute encoder metres, NOT URDF prismatic-joint coordinates.
Writes are never retried. A lease failure stops renewal and requests scoped stop.
Use a context manager or close() to release the session; no destructor motion.
"""

import json
import math
import threading
import time
import uuid

from wheel_sdk_bridge.http_client import JSONClient, SDKError


def metres_to_um(position_m):
    if type(position_m) not in (int, float) or not math.isfinite(position_m):
        raise ValueError("Lift position must be finite metres")
    if not -0.18 <= position_m <= 0.18:
        raise ValueError("Lift position outside SDK travel [-0.18, 0.18] m")
    return round(position_m * 1_000_000)


class LiftMotionClient:
    def __init__(
        self, base_url, *, transport=None, feedback_transport=None, feedback_max_age_s=0.4
    ):
        if not 0 < feedback_max_age_s <= 0.5:
            raise ValueError("Invalid lift feedback age limit")
        self.http = transport or JSONClient(base_url, timeout=0.35)
        self.feedback = feedback_transport or JSONClient(base_url, timeout=0.5)
        self.max_age = feedback_max_age_s
        self.owner = None
        self.sequence = 0
        self.task_id = 0
        self.pending = None
        self.incarnation = None
        self.error = None
        self.stop_error = None
        self._wire = threading.RLock()
        self._operation = threading.RLock()
        self._halt = threading.Event()
        self._thread = None
        self._stop_sent = False
        self._interval = 0.075

    def snapshot(self, *, monitor=False):
        # SDK locked/idle samples can alternate valid/invalid. Never use the
        # invalid sample; wait a bounded time for a new valid one. Heartbeats
        # use an independent transport and continue during these GETs.
        deadline = time.monotonic() + 0.5
        while True:
            try:
                return self._snapshot_once(monitor=monitor)
            except SDKError as exc:
                if (
                    str(exc) not in ("Fresh healthy lift feedback required", "Lift feedback stale")
                    or time.monotonic() >= deadline
                ):
                    raise
                time.sleep(0.025)

    def _snapshot_once(self, *, monitor=False):
        start = time.monotonic()
        s = self.feedback.request("GET", "/api/state")
        elapsed = time.monotonic() - start
        ls, lift = s.get("lift_status") or {}, s.get("lift") or {}
        system = s.get("system") or {}
        if (
            system.get("mock") is not False
            or lift.get("mock") is not False
            or (system.get("services") or {}).get("lift") is not True
        ):
            raise SDKError("Real lift SDK service unavailable")
        if lift.get("fault", 0) != 0 or ls.get("fault", 0) != 0:
            raise SDKError("Lift reports a hardware fault")
        if int(lift.get("flags", 0)) & 128 or ls.get("health") == "motion_inhibited":
            raise SDKError(
                "Lift motion inhibited: result=%s target_status=%s"
                % (lift.get("result"), lift.get("target_status"))
            )
        if (
            ls.get("fresh") is not True
            or (not monitor and lift.get("valid") is not True)
            or ls.get("feedback_healthy") is not True
        ):
            raise SDKError("Fresh healthy lift feedback required")
        try:
            now = int(s["server_mono_ns"])
            status_age = (now - int(ls["receive_mono_ns"])) / 1e9
            sample_age = (now - int(lift["receive_mono_ns"])) / 1e9
            effective_age = float(lift["effective_age_ms"]) / 1000
            if any(
                not math.isfinite(a) or a < 0 or a + elapsed > self.max_age
                for a in (status_age, sample_age)
            ):
                raise SDKError("Lift feedback stale")
            age_limit = self.max_age
            if monitor and lift.get("valid") is not True:
                # Status-only monitoring mirrors the driver's existing watchdog.
                # Never use these samples to plan a target or confirm arrival.
                age_limit = 0.5
                started = int(ls.get("motion_started_mono_ns", 0))
                if (
                    ls.get("startup_feedback_grace") is True
                    and started > 0
                    and 0 <= (now - started) / 1e9 + elapsed < 2
                    and ls.get("phase") in ("move_ack", "moving")
                    and not int(lift.get("flags", 0)) & (4 | 128)
                    and abs(lift.get("speed_rpm", 999)) <= 2
                    and abs(lift["current_um"] - ls.get("startup_origin_um", 1e9)) <= 10
                ):
                    age_limit = 1.5
            if not math.isfinite(effective_age) or not 0 <= effective_age + elapsed <= age_limit:
                raise SDKError("Lift feedback stale")
            if type(lift["current_um"]) is not int or not -180000 <= lift["current_um"] <= 180000:
                raise SDKError("Invalid lift encoder position")
            if ls["fault"] != 0 or lift["fault"] != 0:
                raise SDKError("Lift reports a hardware fault")
        except (KeyError, TypeError, ValueError) as exc:
            raise SDKError("Incompatible lift feedback") from exc
        return s

    @staticmethod
    def _incarnation(s):
        # run_segment also increments on normal configuration/session opening.
        return s["lift"]["process_started_mono_ns"]

    def preflight(self):
        s = self.snapshot()
        ls = s["lift_status"]
        return dict(
            connected=True,
            position_m=s["lift"]["current_um"] / 1e6,
            enabled=ls["enabled"],
            phase=ls["phase"],
            position_control_available=ls.get("dashboard_position_available") is True,
            owner=ls.get("external_owner"),
            lease_ms=ls["external_lease_ms"],
            commands_sent=False,
        )

    def _post(self, op, **extra):
        with self._wire:
            if not self.owner:
                raise SDKError("No lift control session")
            if op != "stop" and (self.error or self._halt.is_set()):
                raise SDKError(self.error or "Lift session closing")
            self.sequence += 1
            detail = dict(owner=self.owner, sequence=self.sequence, **extra)
            result = self.http.request(
                "POST",
                "/api/lift",
                dict(command="position_" + op, detail=json.dumps(detail, allow_nan=False)),
            )
            if result.get("success") is not True:
                raise SDKError(str(result.get("message", "Lift command rejected")), 409)
            return result

    def _stop_once(self):
        with self._wire:
            if self.owner and not self._stop_sent:
                self._stop_sent = True
                return self._post("stop")

    def _abort(self, exc):
        self.error = str(exc)
        self._halt.set()
        try:
            self._stop_once()
        except Exception as stop_exc:
            self.stop_error = str(stop_exc)

    def _heartbeat(self):
        while not self._halt.wait(self._interval):
            try:
                self._post("heartbeat")
            except Exception as exc:
                if not self._halt.is_set():
                    self._abort(exc)
                return

    def _owned(self, s):
        ls = s["lift_status"]
        if (
            ls.get("external_owner") != self.owner
            or not ls.get("dashboard_position")
            or not ls.get("external_control")
            or ls.get("enabled") is not True
            or self._incarnation(s) != self.incarnation
        ):
            raise SDKError("Lift session lost or driver restarted")
        return ls

    def _active(self):
        if not self.owner or self.error or self._halt.is_set():
            raise SDKError(self.error or "Open a new lift session first")

    def open(self, timeout=8.0):
        with self._operation:
            if self.owner:
                raise SDKError("Close the previous lift session first")
            s = self.snapshot()
            ls = s["lift_status"]
            if (
                ls.get("dashboard_position_available") is not True
                or ls.get("allow_motion") is not True
                or ls.get("enabled") is not False
                or ls.get("external_control") is not False
                or ls.get("stationary") is not True
                or ls.get("session_present") is not False
            ):
                raise SDKError("Lift must be available, disabled, stopped and unowned")
            lease = ls.get("external_lease_ms")
            if type(lease) is not int or not 200 <= lease <= 2000:
                raise SDKError("Unsupported lift lease duration")
            self._interval = min(0.075, lease / 10000)
            if isinstance(self.http, JSONClient):
                self.http.timeout = min(0.35, lease / 4000)
            self.owner = "wheel-lift-" + uuid.uuid4().hex
            self.sequence = self.task_id = 0
            self.pending = self.error = self.stop_error = None
            self.incarnation = self._incarnation(s)
            self._stop_sent = False
            self._halt.clear()
            try:
                self._post("arm")
                self._thread = threading.Thread(
                    target=self._heartbeat, daemon=True, name="lift-lease"
                )
                self._thread.start()
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    self._active()
                    current = self.snapshot()
                    if (
                        current["lift_status"].get("external_owner") == ls.get("external_owner")
                        and current["lift_status"].get("enabled") is False
                        and self._incarnation(current) == self.incarnation
                    ):
                        time.sleep(0.05)
                        continue  # Gateway has not published the arm acknowledgement yet.
                    s = current
                    ls = self._owned(s)
                    if (
                        ls["phase"] == "idle"
                        and ls.get("ready_for_command") is True
                        and ls.get("stationary") is True
                        and ls.get("session_present") is False
                    ):
                        return dict(owner=self.owner, position_m=s["lift"]["current_um"] / 1e6)
                    time.sleep(0.05)
                raise SDKError("Lift arm/configuration timed out")
            except Exception as exc:
                self._abort(exc)
                raise

    def move(self, position_m):
        target = metres_to_um(position_m)
        with self._operation:
            self._active()
            if self.pending is not None:
                raise SDKError("Reconcile the previous lift target before another move")
            try:
                s = self.snapshot()
                ls = self._owned(s)
                if (
                    ls["phase"] != "idle"
                    or ls.get("ready_for_command") is not True
                    or ls.get("stationary") is not True
                    or ls.get("session_present") is not False
                ):
                    raise SDKError("Lift is not ready for a target")
                lower = max(ls["host_min_um"], ls["min_um"])
                upper = min(ls["host_max_um"], ls["max_um"])
                if not lower <= target <= upper:
                    raise ValueError("Lift target outside configured device limits")
                self.task_id += 1
                self.pending = dict(task_id=self.task_id, target_um=target)
                self._post("move", **self.pending)
                return dict(**self.pending, target_m=target / 1e6, accepted=True, completed=False)
            except Exception as exc:
                self._abort(exc)
                raise

    def result(self, task_id):
        with self._operation:
            self._active()
            if self.pending is None or self.pending["task_id"] != task_id:
                raise SDKError("Unknown lift task")
            try:
                s = self.snapshot(monitor=True)
                ls = self._owned(s)
                target = self.pending["target_um"]
                if ls["external_task_id"] != task_id:
                    if ls["external_task_id"] < task_id:
                        return dict(task_id=task_id, completed=False, phase="awaiting_feedback")
                    raise SDKError("Lift task superseded")
                if ls["external_target_um"] != target:
                    raise SDKError("Lift target mismatch")
                phase = ls["external_result"]
                if phase not in ("running", "accepted", "settling", "reached"):
                    raise SDKError("Lift task ended without arrival: " + str(phase))
                if s["lift"].get("valid") is not True:
                    return dict(
                        task_id=task_id,
                        completed=False,
                        phase=phase,
                        feedback_valid=False,
                        position_m=None,
                    )
                error_um = abs(s["lift"]["current_um"] - target)
                completed = (
                    phase == "reached"
                    and ls["phase"] == "idle"
                    and ls.get("stationary") is True
                    and ls.get("session_present") is False
                    and ls.get("session") is False
                    and error_um <= 10
                )
                if completed:
                    self.pending = None
                return dict(
                    task_id=task_id,
                    completed=completed,
                    phase=phase,
                    position_m=s["lift"]["current_um"] / 1e6,
                    error_m=error_um / 1e6,
                )
            except Exception as exc:
                self._abort(exc)
                raise

    def wait(self, task_id, timeout=180.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            result = self.result(task_id)
            if result["completed"]:
                return result
            time.sleep(0.1)
        exc = SDKError("Lift arrival timed out")
        self._abort(exc)
        raise exc

    def stop(self, timeout=5.0):
        with self._operation:
            if not self.owner:
                return dict(stopped=True, commands_sent=False)
            self._halt.set()
            try:
                self._stop_once()
            except Exception as exc:
                self.stop_error = str(exc)
            if self._thread and self._thread is not threading.current_thread():
                self._thread.join(timeout=1.0)
            deadline = time.monotonic() + timeout
            last_error = self.stop_error
            while time.monotonic() < deadline:
                try:
                    s = self.snapshot()
                    ls = s["lift_status"]
                    if (
                        ls.get("enabled") is False
                        and ls.get("stationary") is True
                        and ls.get("session_present") is False
                    ):
                        self.owner = None
                        self.pending = None
                        return dict(
                            stopped=True,
                            position_m=s["lift"]["current_um"] / 1e6,
                            stop_ack_error=self.stop_error,
                        )
                except SDKError as exc:
                    last_error = str(exc)
                time.sleep(0.1)
            raise SDKError("Lift stop unconfirmed: " + str(last_error), uncertain=True)

    close = stop

    def __enter__(self):
        try:
            self.open()
        except Exception:
            try:
                self.close()
            except Exception:
                pass
            raise
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self.close()
        except Exception:
            if exc is None:
                raise
