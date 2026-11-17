"""Owned Cartesian visual target stream over the real SDK command gateway."""

import json
import math
import uuid

from wheel_task_core.transforms import pose

from wheel_sdk_bridge.http_client import SDKError


class VisualServoClient:
    def __init__(self, joints):
        self.joints = joints
        self.sequence = 0
        self.last_stamp = 0
        self.last_pair = None

    def require_supported(self, state=None):
        state = self.joints.snapshot() if state is None else state
        protocol = state["arm_startup"]["joint_control"].get("visual_servo_protocol")
        if type(protocol) is not int or protocol != 1:
            raise SDKError("SDK pure visual servo protocol v1 is not installed", 409)
        return state

    def _post(self, command, detail):
        reply = self.joints.http.request(
            "POST",
            "/api/arm_startup",
            dict(command=command, detail=json.dumps(detail, allow_nan=False)),
        )
        if reply.get("success") is not True:
            raise SDKError(str(reply.get("message", "Visual servo rejected")), 409)
        return reply

    def start(self, *, speed_rad_s=0.2):
        with self.joints.lock:
            if self.joints.pending_id is not None:
                raise SDKError("Previous request must be reconciled before visual servo", 409)
            state = self.require_supported()
            joint = state["arm_startup"]["joint_control"]
            if state["arms"].get("enabled") is not True or joint["phase"] in (
                "preparing",
                "moving",
                "servoing",
                "fault",
            ):
                raise SDKError("Already-enabled non-faulted stationary hold required", 409)
            if any(abs(v) >= 0.08 for v in state["arms"]["velocity"]):
                raise SDKError("Stationary arms required before visual servo", 409)
            if (
                isinstance(speed_rad_s, bool)
                or not math.isfinite(speed_rad_s)
                or not 0.02 <= speed_rad_s <= 0.2
            ):
                raise ValueError("Visual servo speed outside SDK bounds")
            self.joints._check_compensation_handover(state)
            identity = "wheel-visual-" + uuid.uuid4().hex
            self.joints.pending_id = identity
            try:
                reply = self._post(
                    "visual_servo_start", dict(request_id=identity, speed_rad_s=speed_rad_s)
                )
            except SDKError as exc:
                if not exc.uncertain:
                    self.joints.pending_id = None
                raise
            self.sequence = self.last_stamp = 0
            self.last_pair = None
            return dict(
                request_id=identity, accepted=True, completed=False, message=reply.get("message")
            )

    def update(self, request_id, targets, *, pair_id, image_receive_mono_ns):
        with self.joints.lock:
            if self.joints.pending_id != request_id:
                raise SDKError("Unowned visual session", 409)
            stamp = image_receive_mono_ns
            if (
                type(stamp) is not int
                or stamp <= self.last_stamp
                or not pair_id
                or pair_id == self.last_pair
            ):
                raise ValueError("New image identity and advancing robot timestamp required")
            if set(targets) != {"left", "right"}:
                raise ValueError("Bilateral Cartesian targets required")
            rows = {side: pose(matrix).tolist() for side, matrix in targets.items()}
            packet = dict(
                request_id=request_id,
                sequence=self.sequence + 1,
                frame_id="arm_origin",
                targets=rows,
                pair_id=pair_id,
                image_receive_mono_ns=stamp,
            )
            # Never retry a lost write acknowledgement or mint a new timestamp
            # for an old camera frame. The board independently expires images.
            self._post("visual_servo_update", packet)
            self.sequence += 1
            self.last_stamp, self.last_pair = stamp, pair_id
            return dict(sequence=self.sequence, request_id=request_id)

    def stop(self, request_id):
        with self.joints.lock:
            state = self.joints.snapshot()
            if (
                self.joints.pending_id != request_id
                or state["arm_startup"]["joint_control"].get("request_id") != request_id
            ):
                raise SDKError("Refusing to stop an unowned request", 409)
            return self._post("joint_stop", dict(request_id=request_id))
