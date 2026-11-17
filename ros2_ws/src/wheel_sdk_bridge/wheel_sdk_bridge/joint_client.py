"""SDK joint_move protocol adapter. GET checks never enable motors.

This API submits one 16-axis target, not a streamed trajectory. The robot keeps
executing after a client disconnect. No automatic retry, disable, or homing.
"""

import json
import math
import threading
import time
import uuid
import warnings
from pathlib import Path

from wheel_sdk_bridge.feedback_diagnostics import append_diagnostic, timing
from wheel_sdk_bridge.http_client import JSONClient, SDKError

FEEDBACK_MAX_AGE_S = 0.400


def vector(value, name):
    if (
        not isinstance(value, (list, tuple))
        or len(value) != 16
        or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
            for v in value
        )
    ):
        raise ValueError(name + " must contain 16 finite numbers")
    return list(map(float, value))


class JointClient:
    def __init__(self, url, policy, *, transport=None, feedback_transport=None):
        self.http = transport or JSONClient(url)
        self.policy = dict(policy)
        feedback_url = self.policy.get("feedback_url")
        self.feedback = feedback_transport or (
            JSONClient(feedback_url, timeout=0.5) if feedback_url else self.http
        )
        self.diagnostics_path = self.policy.get("feedback_diagnostics")
        if self.diagnostics_path and not Path(self.diagnostics_path).is_absolute():
            self.diagnostics_path = Path(__file__).resolve().parents[4] / self.diagnostics_path
        self.last_diagnostic = None
        self.pending_id = None
        self.lock = threading.Lock()

    @property
    def motion_allowed(self):
        # Compatibility capability flag; live readiness is checked per request.
        return True

    def snapshot(self):
        start = time.monotonic()
        s, error = None, None
        try:
            # No fallback to older HTTP data if the selected feedback source fails.
            path = self.policy.get("feedback_path", "/api/state")
            if path not in ("/api/state", "/api/stream_state"):
                raise SDKError("Unknown joint feedback endpoint")
            s = self.feedback.request("GET", path)
            elapsed = time.monotonic() - start
            self._validate_snapshot(s, elapsed)
            return s
        except Exception as exc:
            error = str(exc)
            raise
        finally:
            elapsed = time.monotonic() - start
            record = dict(
                wall_time_ns=time.time_ns(),
                accepted=error is None,
                error=error,
                timing=timing(s, elapsed),
                raw=s,
            )
            self.last_diagnostic = record
            if self.diagnostics_path:
                try:
                    append_diagnostic(self.diagnostics_path, record)
                except (OSError, ValueError) as exc:
                    warnings.warn(
                        "Feedback diagnostics could not be saved: " + str(exc), RuntimeWarning
                    )

    def _validate_snapshot(self, s, elapsed):
        if self.policy.get("feedback_url") and s.get("feedback_source") != "ros_joint_feedback_v1":
            raise SDKError("Unexpected joint feedback source")
        if s.get("system", {}).get("mock") is not False:
            raise SDKError("Real robot state required")
        arms = s.get("arms") or {}
        startup = s.get("arm_startup") or {}
        joint = startup.get("joint_control") or {}
        if (
            arms.get("fresh") is not True
            or startup.get("fresh") is not True
            or arms.get("joint_order") != "right_then_left"
            or joint.get("joint_order") != "right_then_left"
            or joint.get("unit") != "rad"
        ):
            raise SDKError("Fresh compatible joint feedback required")
        now = int(s["server_mono_ns"])
        maximum_age = self.policy.get("feedback_max_age_s", FEEDBACK_MAX_AGE_S)
        if not isinstance(maximum_age, (int, float)) or not 0 < maximum_age <= FEEDBACK_MAX_AGE_S:
            raise SDKError("Invalid joint feedback freshness policy")
        independent = self.policy.get("feedback_path") == "/api/stream_state"
        if independent and s.get("topic_pairing") != "independent_receive_monotonic":
            raise SDKError("Independent topic timestamps required for streaming feedback")
        for name, obj in (("arms", arms), ("arm_startup", startup)):
            age = (now - int(obj["receive_mono_ns"])) / 1e9 + elapsed
            if not 0 <= age <= maximum_age:
                raise SDKError(
                    "Joint feedback stale: %s effective=%.3f ms, request=%.3f ms"
                    % (name, age * 1000, elapsed * 1000)
                )
        skew = abs(int(arms["receive_mono_ns"]) - int(startup["receive_mono_ns"])) / 1e9
        maximum_skew = 0.15 if independent else 0.03
        if self.policy.get("feedback_url") and skew > maximum_skew:
            raise SDKError("Joint feedback/status timestamp mismatch: %.3f ms" % (skew * 1000))
        q = vector(arms["position"], "Feedback")
        vector(arms["velocity"], "Velocity")
        valid, ages = arms["valid"], arms["age_ns"]
        if len(valid) != 16 or len(ages) != 16 or any(v is not True for v in valid):
            raise SDKError("All 16 joints need valid feedback")
        snapshot_age = (now - int(arms["receive_mono_ns"])) / 1e9 + elapsed
        if any(
            not math.isfinite(float(a)) or not 0 <= float(a) / 1e9 + snapshot_age <= maximum_age
            for a in ages
        ):
            raise SDKError("Per-joint feedback stale")
        lower, upper = (
            vector(joint["lower"], "Lower limits"),
            vector(joint["upper"], "Upper limits"),
        )
        violations = [
            f"{'right' if index < 8 else 'left'} J{index % 8 + 1}: "
            f"measured={math.degrees(value):.5f}deg, "
            f"limits=[{math.degrees(lo):.5f}, {math.degrees(hi):.5f}]deg"
            for index, (lo, value, hi) in enumerate(zip(lower, q, upper))
            if not lo <= value <= hi or lo >= hi
        ]
        if violations:
            raise SDKError("Measured position outside SDK limits: " + "; ".join(violations))
        return s

    def preflight(self):
        s = self.snapshot()
        return dict(
            connected=True,
            motion_allowed=self.motion_allowed,
            blockers=[],
            enabled=s["arms"]["enabled"],
            arm_phase=s["arms"]["phase"],
            joint_control=s["arm_startup"]["joint_control"],
            positions_rad=s["arms"]["position"],
            startup_compensation_rad=s["arm_startup"].get("static_compensation_rad"),
            commands_sent=False,
        )

    def prepare(self, positions, speed_rad_s=0.02):
        q = vector(positions, "Target")
        if (
            isinstance(speed_rad_s, bool)
            or not isinstance(speed_rad_s, (int, float))
            or not math.isfinite(speed_rad_s)
        ):
            raise ValueError("Invalid speed")
        s = self.snapshot()
        j = s["arm_startup"]["joint_control"]
        if not max(0.02, j["min_speed_rad_s"]) <= speed_rad_s <= min(0.2, j["max_speed_rad_s"]):
            raise ValueError("Speed outside SDK bounds")
        if any(not lo <= v <= hi for lo, v, hi in zip(j["lower"], q, j["upper"])):
            raise ValueError("Target outside SDK limits")
        if j["phase"] in ("preparing", "moving"):
            raise SDKError("Robot already executing a joint target", 409)
        return dict(
            request_id="wheel-" + uuid.uuid4().hex,
            positions=q,
            unit="rad",
            speed_rad_s=float(speed_rad_s),
        )

    def move(self, positions, speed_rad_s=0.02, *, enable_if_needed=False):
        if type(enable_if_needed) is not bool:
            raise ValueError("enable_if_needed must be boolean")
        with self.lock:
            if self.pending_id:
                raise SDKError("Previous request must be reconciled before another move", 409)
            detail = self.prepare(positions, speed_rad_s)
            s = self.snapshot()
            enabled = s["arms"].get("enabled") is True
            if not enabled and not enable_if_needed:
                raise SDKError("Refusing implicit motor enable", 409)
            if any(abs(v) > 0.08 for v in s["arms"]["velocity"]):
                raise SDKError("Arms are not stationary", 409)
            if enabled:
                self._check_compensation_handover(s)
            # Explicit opt-in uses SDK joint_move's stable-feedback enable and
            # measured-position reanchor, never startup/homing or stale targets.
            return self._submit(detail)

    def _check_compensation_handover(self, s, required=False):
        b = s["arm_startup"]
        bias = vector(b.get("static_compensation_rad"), "Compensation")
        if not required and not any(abs(v) > 1e-6 for v in bias):
            return
        meta = b.get("static_compensation") or {}
        if (
            self.policy.get("compensated_handover_protocol") != "nominal_target_v1"
            or meta.get("scope") != "all_control_modes"
            or meta.get("arm_joints_enabled") is not True
            or meta.get("grippers_enabled") is not False
        ):
            raise SDKError("Startup support compensation handover is not validated", 409)
        nominal = vector(b.get("nominal_target"), "Nominal target")
        support = vector(b.get("support_target"), "Support target")
        lower = vector(b["joint_control"]["lower"], "Lower limits")
        upper = vector(b["joint_control"]["upper"], "Upper limits")
        if any(abs(v) > 0.200001 for v in bias) or any(abs(bias[i]) > 1e-9 for i in (7, 15)):
            raise SDKError("Compensation outside audited bounds", 409)
        if any(
            not lo <= n <= hi or not lo <= out <= hi
            for n, out, lo, hi in zip(nominal, support, lower, upper)
        ):
            raise SDKError("Compensated target outside SDK limits", 409)
        if any(abs(out - n - b) > 0.0001 for out, n, b in zip(support, nominal, bias)):
            raise SDKError("Compensated support output has not settled", 409)
        if b.get("manual_pose", {}).get("fault_latched") is not False or b["joint_control"][
            "phase"
        ] in ("preparing", "moving", "fault"):
            raise SDKError("Driver is not ready for compensated handover", 409)

    def takeover_hold(self):
        """Explicitly authorized nominal-target-only takeover; never enables motors.

        This separate gate cannot accept user-selected positions.
        No retry or implicit stop.
        """
        with self.lock:
            if self.policy.get("hold_takeover_enabled") is not True:
                raise SDKError("Hold takeover blocked", 409)
            if self.pending_id:
                raise SDKError("Previous request must be reconciled before takeover", 409)
            before = self.snapshot()
            self._check_compensation_handover(before, required=True)
            target = vector(before["arm_startup"]["nominal_target"], "Nominal target")
            detail = self.prepare(target, 0.02)
            s = self.snapshot()
            self._check_compensation_handover(s, required=True)
            if (
                s["arms"]["enabled"] is not True
                or s["arms"]["phase"] not in ("startup_ready", "manual_pose_hold")
                or any(abs(v) > 0.08 for v in s["arms"]["velocity"])
            ):
                raise SDKError("Takeover requires enabled stationary hold", 409)
            if (
                s["arm_startup"]["control_generation"]
                != before["arm_startup"]["control_generation"]
                or s["arm_startup"]["nominal_target"] != target
            ):
                raise SDKError("Control owner or nominal target changed before takeover", 409)
            if any(abs(a - b) > 0.02 for a, b in zip(s["arms"]["position"], target)):
                raise SDKError("Holding position has not settled before takeover", 409)
            return self._submit(detail)

    def _submit(self, detail):
        self.pending_id = detail["request_id"]
        try:
            result = self.http.request(
                "POST",
                "/api/arm_startup",
                dict(command="joint_move", detail=json.dumps(detail, allow_nan=False)),
            )
            if result.get("success") is not True:
                self.pending_id = None
                raise SDKError(str(result.get("message", "Joint request rejected")), 409)
        except SDKError as exc:
            if not exc.uncertain:
                self.pending_id = None
            raise
        return dict(
            request_id=self.pending_id,
            accepted=True,
            completed=False,
            message=result.get("message"),
        )

    def result(self, request_id):
        s = self.snapshot()
        j = s["arm_startup"]["joint_control"]
        if j.get("request_id") != request_id:
            return dict(request_id=request_id, phase="unknown_or_superseded", completed=False)
        error = max(
            abs(a - b)
            for a, b in zip(s["arms"]["position"], vector(j["target"], "Target feedback"))
        )
        completed = (
            j["phase"] == "ready"
            and s["arms"]["enabled"] is True
            and error <= 0.02
            and all(abs(v) < 0.08 for v in s["arms"]["velocity"])
        )
        if self.pending_id == request_id and (completed or j["phase"] in ("stopped", "fault")):
            self.pending_id = None
        return dict(
            request_id=request_id,
            phase=j["phase"],
            reason=j["reason"],
            completed=completed,
            max_error_rad=error,
            enabled=s["arms"]["enabled"],
        )

    def stop(self, request_id):
        with self.lock:
            s = self.snapshot()
            if (
                self.pending_id != request_id
                or s["arm_startup"]["joint_control"].get("request_id") != request_id
            ):
                raise SDKError("Refusing to stop an unowned or superseded request", 409)
            # This SDK stop is global, not request-scoped. The checked state can
            # race another client; this local check is not SDK ownership isolation.
            return self.http.request(
                "POST", "/api/arm_startup", dict(command="joint_stop", detail="")
            )
