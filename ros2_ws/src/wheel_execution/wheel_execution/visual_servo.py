"""Pure visual feedback with live Cartesian targets, never a stored arm path."""

import copy
import threading
import time

import numpy as np
from wheel_sdk_bridge.visual_servo_client import VisualServoClient

from wheel_execution.phase_driver import SDKTrajectoryDriver
from wheel_execution.reuse_existing import runtime_blockers


class LatestObservation:
    """One producer, one latest slot; slow perception never creates a backlog."""

    def __init__(self, observe, *, clock=time.monotonic):
        self.observe, self.clock = observe, clock
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.value = self.error = None
        self.thread = None

    def start(self):
        def poll():
            while not self.stopped.is_set():
                try:
                    value = self.observe()
                    if value is not None:
                        with self.lock:
                            # 丢帧不为旧目标续期；原时间戳仍由 latest 的 2s 检查约束。
                            self.value = copy.deepcopy(value)
                except Exception as exc:
                    with self.lock:
                        self.error = exc
                    return
                self.stopped.wait(0.005)

        self.thread = threading.Thread(target=poll, daemon=True)
        self.thread.start()

    def latest(self, *, max_age_s=2.0):
        with self.lock:
            if self.error is not None:
                raise RuntimeError("Visual observation failed: " + str(self.error))
            value = copy.deepcopy(self.value)
        if value is None:
            return None
        age = self.clock() - value["observed_at_monotonic"]
        if not np.isfinite(age) or age < 0:
            raise ValueError("Invalid visual observation clock")
        if age > max_age_s:
            return None
        return value

    def close(self, timeout=0.5):
        self.stopped.set()
        if self.thread is not None:
            self.thread.join(timeout=timeout)


class SDKVisualServoDriver(SDKTrajectoryDriver):
    pure_visual_servo = True
    continuous_phases = False

    def __init__(self, joints, *args, servo_client=None, **kwargs):
        servo = servo_client or VisualServoClient(joints)
        super().__init__(joints, *args, trajectory_client=servo, **kwargs)
        self.servo = servo
        self.visual_events = []

    def execute_phase(self, *args, **kwargs):
        raise RuntimeError("Pure visual servo does not execute stored arm paths")

    def follow(self, observations, goal, *, name, complete=None, timeout_s=90, stable_s=0.5):
        if self.collision_policy != "onsite_supervised_skip":
            raise ValueError(
                "Pure visual servo requires a commissioned online validator or explicit onsite policy"
            )
        initial = observations.latest()
        if initial is None:
            raise ValueError("Fresh visual target required before starting servo")
        state, lift = self.measured()
        previous_id = state["arm_startup"]["joint_control"]["request_id"]
        response = self.servo.start(speed_rad_s=self.speed_rad_s)
        self._adopt_request(response, previous_id)
        deadline = self.clock() + timeout_s
        stable_since, stable_pairs, previous_pair = None, set(), None
        last_packet = None
        while self.clock() < deadline:
            state, lift = self.measured()
            joint = state["arm_startup"]["joint_control"]
            if joint["request_id"] != self.request_id or joint["phase"] != "servoing":
                raise RuntimeError(
                    "Visual servo stopped or superseded: " + str(joint.get("reason"))
                )
            target = observations.latest()
            if target is None:
                raise RuntimeError("Visual observations stale; retain hold")
            packet = goal(target, state, lift)
            stamp, pair = packet["image_receive_mono_ns"], packet["pair_id"]
            if stamp > self.servo.last_stamp and pair != self.servo.last_pair:
                wire = copy.deepcopy(packet["targets"])
                for transform in wire.values():
                    transform[2, 3] -= lift["current_um"] * 1e-6
                self.servo.update(self.request_id, wire, pair_id=pair, image_receive_mono_ns=stamp)
                last_packet = packet
            if last_packet is None:
                raise ValueError("No usable visual target packet")
            errors = self.endpoint_errors(last_packet["targets"], state, lift)
            if complete is None:
                arrived = all(
                    error["position_m"] <= self.position_tolerance_m
                    and error["angle_rad"] <= self.angle_tolerance_rad
                    for error in errors.values()
                )
            else:
                arrived = complete(target, state, lift, last_packet)
            settled = max(abs(v) for v in state["arms"]["velocity"]) < 0.08
            if arrived and settled:
                if stable_since is None:
                    stable_since = self.clock()
                confirmation = packet.get("confirmation_id", pair)
                if confirmation != previous_pair:
                    stable_pairs.add(confirmation)
                if len(stable_pairs) >= 3 and self.clock() - stable_since >= stable_s:
                    stopped = self._stop_and_settle()
                    event = dict(
                        phase=name,
                        completed=True,
                        updates=self.servo.sequence,
                        measured_errors=errors,
                        collision_checked=False,
                        collision_policy=self.collision_policy,
                    )
                    self.visual_events.append(event)
                    return stopped, event
            else:
                stable_since, stable_pairs = None, set()
            previous_pair = packet.get("confirmation_id", pair)
            self.sleep(0.01)
        raise TimeoutError("Visual servo phase deadline exceeded")


def visual_packet(target, targets, *, resource_policy="strict"):
    blockers = runtime_blockers(target.get("execution_blockers", []), resource_policy)
    if blockers or (resource_policy == "strict" and target.get("diagnostic_only") is not False):
        raise ValueError("Visual target remains unaccepted")
    if (
        target.get("frame_id") != "arm_origin"
        or target.get("tool_frame") != "cad_contact_midpoint_pinch_y"
    ):
        raise ValueError("Visual target convention mismatch")
    stamp = target.get("image_receive_mono_ns")
    if type(stamp) is not int or stamp <= 0:
        raise ValueError("Actual robot image timestamp required for visual servo")
    return dict(targets=targets, pair_id=target["pair_id"], image_receive_mono_ns=stamp)
