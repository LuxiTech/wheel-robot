"""Owned UDP joint references; HTTP is used only for session start/stop.

Every datagram expires using real robot feedback/image timestamps, never a
comparison between unsynchronized laptop and robot monotonic clocks.
"""

import hashlib
import json
import math
import socket
import struct
import uuid
from urllib.parse import urlsplit

from wheel_sdk_bridge.http_client import SDKError
from wheel_sdk_bridge.joint_client import vector

PACKET = struct.Struct("!8s32sQQQ16s32d")
MAGIC = b"WJSTRM1\0"


class JointStreamClient:
    def __init__(self, joints, *, socket_factory=socket.socket, trace=None):
        self.joints = joints
        self.trace = trace
        self.socket_factory = socket_factory
        self.socket = None
        self.key = None
        self.sequence = 0
        self.destination = None
        self.fixed_target = False

    def require_supported(self, state=None):
        state = self.joints.snapshot() if state is None else state
        control = state["arm_startup"]["joint_control"]
        if (
            type(control.get("joint_stream_protocol")) is not int
            or control["joint_stream_protocol"] != 1
        ):
            raise SDKError("SDK laptop joint stream protocol v1 is not installed", 409)
        port = control.get("joint_stream_port")
        if type(port) is not int or not 1024 <= port <= 65535:
            raise SDKError("SDK advertised an invalid joint stream port", 409)
        speeds = vector(control.get("joint_stream_speed_limits"), "SDK stream speed limits")
        if any(value <= 0 for value in speeds):
            raise SDKError("Positive original SDK stream speed limits required", 409)
        if control.get("joint_stream_image_max_age_ms") != 2000:
            raise SDKError("SDK image watchdog must advertise 2000ms before execution", 409)
        self.destination = (urlsplit(self.joints.http.base_url).hostname, port)
        return state

    def _post(self, command, detail):
        safe_detail = {key: value for key, value in detail.items() if key != "stream_key"}
        if self.trace is not None:
            try:
                self.trace.record("sdk_request", command=command, detail=safe_detail)
            except Exception:
                if command != "joint_stop":
                    raise
                # 停止动作不能依赖磁盘正常；日志损坏时仍停止自己的请求。
        reply = self.joints.http.request(
            "POST",
            "/api/arm_startup",
            dict(command=command, detail=json.dumps(detail, allow_nan=False)),
        )
        if self.trace is not None:
            try:
                self.trace.record("sdk_receipt", command=command, detail=safe_detail, reply=reply)
            except Exception as error:
                # A successful POST can already own hardware even if recording fails.
                raise SDKError(
                    "SDK receipt logging failed: " + str(error), uncertain=True
                ) from error
        if reply.get("success") is not True:
            raise SDKError(str(reply.get("message", "Joint stream rejected")), 409)
        return reply

    def start(self, *, speed_rad_s=0.2, acceleration_rad_s2=0.5, fixed_target=False):
        with self.joints.lock:
            if self.joints.pending_id is not None:
                raise SDKError("Previous request must be reconciled before joint stream", 409)
            state = self.require_supported()
            joint = state["arm_startup"]["joint_control"]
            if fixed_target and joint.get("joint_stream_fixed_target_protocol") != 1:
                raise SDKError("SDK fixed-target protocol is not installed", 409)
            self.fixed_target = fixed_target
            if (
                state["arms"].get("enabled") is not True
                or joint["phase"] not in ("ready", "idle", "stopped", "needs_correction")
                or any(abs(v) >= 0.08 for v in state["arms"]["velocity"])
            ):
                raise SDKError("Enabled non-faulted stationary hold required", 409)
            if (
                isinstance(speed_rad_s, bool)
                or not math.isfinite(speed_rad_s)
                or not 0.02 <= speed_rad_s <= 0.2
            ):
                raise ValueError("Stream speed outside SDK bounds")
            if (
                isinstance(acceleration_rad_s2, bool)
                or not math.isfinite(acceleration_rad_s2)
                or not 0 < acceleration_rad_s2 <= 2
            ):
                raise ValueError("Stream acceleration outside supported bounds")
            self.joints._check_compensation_handover(state)
            identity = "wheel-stream-" + uuid.uuid4().hex
            key = uuid.uuid4().hex
            sock = self.socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.setblocking(False)
                sock.connect(self.destination)
                self.joints.pending_id = identity
                reply = self._post(
                    "joint_stream_start",
                    dict(
                        request_id=identity,
                        stream_key=key,
                        speed_rad_s=speed_rad_s,
                        acceleration_rad_s2=acceleration_rad_s2,
                    ),
                )
            except BaseException as exc:
                sock.close()
                if not isinstance(exc, SDKError) or not exc.uncertain:
                    self.joints.pending_id = None
                raise
            self.socket, self.key, self.sequence = sock, key, 0
            return dict(
                request_id=identity, accepted=True, completed=False, message=reply.get("message")
            )

    def send(
        self, request_id, positions, velocities, *, feedback_stamp_ns, image_stamp_ns, pair_id
    ):
        if self.socket is None or self.joints.pending_id != request_id:
            raise SDKError("No owned joint stream", 409)
        positions = vector(positions, "Stream reference")
        velocities = vector(velocities, "Stream velocity")
        if (
            any(
                type(stamp) is not int or stamp <= 0
                for stamp in (feedback_stamp_ns, image_stamp_ns)
            )
            or not pair_id
        ):
            raise ValueError("Real robot feedback/image timestamps and pair identity required")
        sequence = self.sequence + 1
        # 图像时间戳来自真机，复用旧图时保持原值。序号递增不能为旧图续期。
        packet = PACKET.pack(
            b"WJFIXD1\0" if self.fixed_target else MAGIC,
            self.key.encode("ascii"),
            sequence,
            feedback_stamp_ns,
            image_stamp_ns,
            hashlib.sha256(str(pair_id).encode()).digest()[:16],
            *positions,
            *velocities,
        )
        # A failed send ends this session. Never queue/retry an old reference.
        if self.socket.send(packet) != len(packet):
            raise SDKError("Incomplete joint reference datagram", uncertain=True)
        self.sequence = sequence

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None
        self.key = None

    def stop(self, request_id):
        with self.joints.lock:
            # 停止不能依赖正常反馈：越限/反馈过期正是需要停止的情况。
            # 本地确认所有权，SDK 再原子校验 request_id，拒绝停止后来接管的动作。
            if not request_id or self.joints.pending_id != request_id:
                raise SDKError("Refusing to stop an unowned joint stream", 409)
            self.close()
            return self._post("joint_stop", dict(request_id=request_id))
