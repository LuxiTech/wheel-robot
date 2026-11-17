"""One HTTP submission per continuous SDK arm phase; no endpoint-by-endpoint fallback."""

import json
import math
import uuid

from wheel_sdk_bridge.http_client import SDKError
from wheel_sdk_bridge.joint_client import vector


class JointTrajectoryClient:
    def __init__(self, joints):
        self.joints = joints

    def require_supported(self, state=None):
        state = self.joints.snapshot() if state is None else state
        protocol = state["arm_startup"]["joint_control"].get("trajectory_protocol")
        if type(protocol) is not int or protocol != 1:
            raise SDKError("SDK continuous trajectory protocol v1 is not installed", 409)
        return state

    def submit(self, waypoints, *, speed_rad_s=0.2, acceleration_rad_s2=0.5, settle_timeout_s=2.0):
        with self.joints.lock:
            if self.joints.pending_id:
                raise SDKError(
                    "Previous SDK request must be reconciled before trajectory submission", 409
                )
            state = self.require_supported()
            if state["arms"].get("enabled") is not True:
                raise SDKError("Continuous trajectory requires an already-enabled hold", 409)
            if any(abs(value) > 0.08 for value in state["arms"]["velocity"]):
                raise SDKError("Arms are not stationary before phase submission", 409)
            joint = state["arm_startup"]["joint_control"]
            if joint["phase"] in ("preparing", "moving", "fault"):
                raise SDKError("SDK is not ready for a continuous phase", 409)
            self.joints._check_compensation_handover(state)
            if (
                isinstance(speed_rad_s, bool)
                or not math.isfinite(speed_rad_s)
                or not max(0.02, joint["min_speed_rad_s"])
                <= speed_rad_s
                <= min(0.2, joint["max_speed_rad_s"])
            ):
                raise ValueError("Trajectory speed outside original SDK bounds")
            if (
                isinstance(acceleration_rad_s2, bool)
                or not math.isfinite(acceleration_rad_s2)
                or not 0 < acceleration_rad_s2 <= 2
            ):
                raise ValueError("Invalid bounded trajectory acceleration")
            if (
                isinstance(settle_timeout_s, bool)
                or not math.isfinite(settle_timeout_s)
                or not 0.5 <= settle_timeout_s <= 10
            ):
                raise ValueError("Invalid phase endpoint settle timeout")
            if not isinstance(waypoints, (list, tuple)) or not 2 <= len(waypoints) <= 4096:
                raise ValueError("Trajectory requires 2..4096 waypoints")
            points = [vector(row, "Trajectory waypoint") for row in waypoints]
            nominal = vector(state["arm_startup"]["nominal_target"], "Nominal hold")
            for row in points:
                for index in (7, 15):
                    row[index] = nominal[index]
                if any(
                    not low <= value <= high
                    for low, value, high in zip(joint["lower"], row, joint["upper"])
                ):
                    raise ValueError("Trajectory waypoint outside original SDK limits")
            if any(
                abs(a - b) > 0.015
                for index, (a, b) in enumerate(zip(points[0], state["arms"]["position"]))
                if index % 8 != 7
            ):
                raise SDKError("Trajectory no longer starts at measured arm joints", 409)
            detail = dict(
                request_id="wheel-path-" + uuid.uuid4().hex,
                unit="rad",
                waypoints=points,
                speed_rad_s=speed_rad_s,
                acceleration_rad_s2=acceleration_rad_s2,
                settle_timeout_s=settle_timeout_s,
            )
            self.joints.pending_id = detail["request_id"]
            try:
                response = self.joints.http.request(
                    "POST",
                    "/api/arm_startup",
                    dict(command="joint_trajectory", detail=json.dumps(detail, allow_nan=False)),
                )
                if response.get("success") is not True:
                    self.joints.pending_id = None
                    raise SDKError(str(response.get("message", "SDK trajectory rejected")), 409)
            except SDKError as exc:
                if not exc.uncertain:
                    self.joints.pending_id = None
                raise
            return dict(
                request_id=detail["request_id"],
                accepted=True,
                completed=False,
                message=response.get("message"),
            )

    def result(self, request_id, *, state=None):
        state = self.joints.snapshot() if state is None else state
        joint = state["arm_startup"]["joint_control"]
        if joint.get("request_id") != request_id:
            return dict(request_id=request_id, phase="unknown_or_superseded", completed=False)
        error = max(abs(a - b) for a, b in zip(state["arms"]["position"], joint["target"]))
        completed = (
            joint["phase"] == "ready"
            and state["arms"]["enabled"] is True
            and error <= 0.02
            and all(abs(value) < 0.08 for value in state["arms"]["velocity"])
        )
        terminal = joint["phase"] in ("ready", "needs_correction", "stopped", "fault")
        if self.joints.pending_id == request_id and terminal:
            self.joints.pending_id = None
        return dict(
            request_id=request_id,
            phase=joint["phase"],
            reason=joint["reason"],
            completed=completed,
            terminal=terminal,
            max_error_rad=error,
            duration_s=joint.get("trajectory_duration_s"),
            fraction=joint.get("trajectory_fraction"),
        )

    def stop(self, request_id):
        with self.joints.lock:
            # 停止不能依赖正常反馈：越限/反馈过期正是需要停止的情况。
            # 本地确认所有权，SDK 再原子校验 request_id，拒绝停止后来接管的动作。
            if not request_id or self.joints.pending_id != request_id:
                raise SDKError("Refusing to stop an unowned trajectory", 409)
            response = self.joints.http.request(
                "POST",
                "/api/arm_startup",
                dict(command="joint_stop", detail=json.dumps(dict(request_id=request_id))),
            )
            if response.get("success") is not True:
                raise SDKError(str(response.get("message", "Trajectory stop rejected")), 409)
            return response
