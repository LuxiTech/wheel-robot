"""Submit complete arm phases and correct only at their measured endpoints."""

import copy
import threading
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation
from wheel_sdk_bridge.joint_client import JointClient
from wheel_sdk_bridge.trajectory_client import JointTrajectoryClient
from wheel_task_core.transforms import pose

from wheel_execution.grasp_executor import SDKWaypointDriver


class SDKTrajectoryDriver(SDKWaypointDriver):
    continuous_phases = True

    def __init__(
        self,
        *args,
        speed_rad_s=0.2,
        max_corrections=2,
        collision_policy="required",
        position_tolerance_m=0.005,
        angle_tolerance_rad=0.02,
        acceleration_rad_s2=0.5,
        endpoint_settle_timeout_s=2.0,
        trajectory_client=None,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        if collision_policy not in ("required", "onsite_supervised_skip"):
            raise ValueError("Explicit collision policy required")
        if type(max_corrections) is not int or not 0 <= max_corrections <= 3:
            raise ValueError("Endpoint corrections must be bounded to 0..3 attempts")
        if not np.isfinite(speed_rad_s) or not 0.02 <= speed_rad_s <= 0.2:
            raise ValueError("Phase speed outside SDK bounds")
        if (
            not np.isfinite([position_tolerance_m, angle_tolerance_rad]).all()
            or not 0 < position_tolerance_m <= 0.15
            or not 0 < angle_tolerance_rad <= 0.02
        ):
            raise ValueError("Invalid measured endpoint tolerances")
        self.speed_rad_s = speed_rad_s
        self.max_corrections = max_corrections
        self.collision_policy = collision_policy
        self.position_tolerance_m = position_tolerance_m
        self.angle_tolerance_rad = angle_tolerance_rad
        if not np.isfinite(acceleration_rad_s2) or not 0 < acceleration_rad_s2 <= 2:
            raise ValueError("Invalid continuous phase acceleration bound")
        if not np.isfinite(endpoint_settle_timeout_s) or not 0.5 <= endpoint_settle_timeout_s <= 10:
            raise ValueError("Invalid continuous phase endpoint settle timeout")
        self.acceleration_rad_s2 = acceleration_rad_s2
        self.endpoint_settle_timeout_s = endpoint_settle_timeout_s
        self.trajectory = trajectory_client or JointTrajectoryClient(self.joints)
        self.history = deque(maxlen=256)
        self.phase_events = []
        self.continuous_request = False
        self.history_lock = threading.Lock()
        self.history_stop = threading.Event()
        self.history_worker = None
        self.history_error = None
        self.last_motion_end_robot_ns = None
        self.latest_feedback_sample = None

    def begin(self):
        # Missing board support is detected before any movement or gripper POST.
        self.trajectory.require_supported()
        super().begin()
        self.history.clear()
        self.measured()
        if isinstance(self.joints, JointClient):
            policy = self.feedback_reader_policy()
            reader = JointClient(self.joints.http.base_url, policy)

            def poll():
                while not self.history_stop.is_set():
                    try:
                        began = self.clock()
                        self._remember(reader.snapshot(), self.read_lift(), received_at=began)
                    except Exception as exc:
                        self.history_error = exc
                        # 验证失败的样本也必须留存；不能只记录最后一帧合格反馈。
                        trace = getattr(self, "trace", None)
                        if trace is not None:
                            try:
                                trace.record(
                                    "feedback_source_failed",
                                    error=str(exc),
                                    diagnostic=reader.last_diagnostic,
                                )
                            except Exception:
                                pass  # 保留原始故障，停止不能依赖日志可用。
                        return
                    self.history_stop.wait(0.02)

            self.history_stop.clear()
            self.history_worker = threading.Thread(target=poll, daemon=True)
            self.history_worker.start()

    def feedback_reader_policy(self):
        return dict(self.joints.policy, feedback_diagnostics=None)

    def _remember(self, state, lift, *, received_at=None):
        if "receive_mono_ns" not in state["arms"]:
            return
        stamp = int(state["arms"]["receive_mono_ns"])
        advanced = False
        with self.history_lock:
            if not self.history or stamp > self.history[-1][0]:
                self.history.append((stamp, copy.deepcopy(state), dict(lift)))
                received_at = self.clock() if received_at is None else received_at
                self.latest_feedback_sample = (copy.deepcopy(state), dict(lift), received_at)
                advanced = True
        trace = getattr(self, "trace", None)
        if advanced and trace is not None:
            trace.record("sdk_feedback", state=state, lift=lift)

    def cached_feedback(self):
        """No network IO in the laptop control loop; duplicate states do not renew age."""
        if self.history_error is not None:
            raise RuntimeError("Continuous feedback source failed: " + str(self.history_error))
        with self.history_lock:
            sample = copy.deepcopy(self.latest_feedback_sample)
        if sample is None or not 0 <= self.clock() - sample[2] <= 0.15:
            raise RuntimeError("Laptop joint feedback cache expired")
        return sample

    def finish(self):
        self.history_stop.set()
        if self.history_worker is not None:
            self.history_worker.join(timeout=0.5)

    def measured(self):
        if self.history_error is not None:
            raise RuntimeError("Continuous feedback source failed: " + str(self.history_error))
        state, lift = super().measured()
        self._remember(state, lift)
        return state, lift

    def state_at(self, robot_stamp_ns):
        """Interpolate raw SDK angles at image receive time, never invent zeros."""
        self.measured()
        with self.history_lock:
            before = [row for row in self.history if row[0] <= robot_stamp_ns]
            after = [row for row in self.history if row[0] >= robot_stamp_ns]
        if not before or not after:
            raise ValueError("No measured joint bracket for moving wrist image")
        left, right = before[-1], after[0]
        if max(robot_stamp_ns - left[0], right[0] - robot_stamp_ns) > 50_000_000:
            raise ValueError("Moving wrist joint bracket exceeds 50 ms")
        if (
            left[1]["arm_startup"]["control_generation"]
            != right[1]["arm_startup"]["control_generation"]
        ):
            raise ValueError("Control generation changed across moving wrist image")
        if abs(left[2]["current_um"] - right[2]["current_um"]) > 200:
            raise ValueError("Lift moved across wrist image")
        fraction = 0 if left[0] == right[0] else (robot_stamp_ns - left[0]) / (right[0] - left[0])
        result = copy.deepcopy(left[1])
        for key in ("position", "velocity"):
            a, b = np.asarray(left[1]["arms"][key]), np.asarray(right[1]["arms"][key])
            result["arms"][key] = (a + fraction * (b - a)).tolist()
        result["arms"]["receive_mono_ns"] = str(robot_stamp_ns)
        return result, left[2]

    def _adopt_request(self, response, previous_id, *, continuous=False):
        self.request_id = response["request_id"]
        trace = getattr(self, "trace", None)
        if trace is not None:
            trace.record("sdk_motion_receipt", response=response, previous_request_id=previous_id)
        self.continuous_request = continuous
        deadline = self.clock() + 2
        while self.clock() < deadline:
            state = self.joints.snapshot()
            observed = state["arm_startup"]["joint_control"]["request_id"]
            if observed == self.request_id:
                self.generation = state["arm_startup"]["control_generation"]
                return
            if observed != previous_id:
                raise RuntimeError("SDK phase superseded by another controller")
            self.sleep(0.02)
        raise TimeoutError("SDK phase acknowledgement did not reach feedback channel")

    def _stop_and_settle(self):
        request_id = self.request_id
        self.trajectory.stop(request_id)
        deadline = self.clock() + 3
        while self.clock() < deadline:
            state, _ = self.measured()
            joint = state["arm_startup"]["joint_control"]
            if joint["request_id"] != request_id:
                raise RuntimeError("Stop request superseded")
            if joint["phase"] == "fault":
                raise RuntimeError("SDK fault while stopping continuous phase")
            if (
                joint["phase"] in ("stopped", "ready")
                and max(abs(v) for v in state["arms"]["velocity"]) < 0.08
            ):
                self.joints.pending_id = None
                self.request_id = None
                self.last_motion_end_robot_ns = int(state["arms"]["receive_mono_ns"])
                return state
            self.sleep(0.02)
        raise TimeoutError("Measured phase stop not confirmed")

    def _wait(self, *, monitor=None):
        deadline = self.clock() + 620
        while self.clock() < deadline:
            state, lift = self.measured()
            result = self.trajectory.result(self.request_id, state=state)
            if result["phase"] in ("fault", "stopped", "unknown_or_superseded"):
                raise RuntimeError("SDK continuous phase failed: " + str(result))
            duration = result.get("duration_s")
            if duration is not None:
                deadline = min(
                    deadline,
                    self.clock() + max(0, duration * (1 - result.get("fraction", 0))) * 2 + 12,
                )
            if result["completed"] or result["phase"] == "needs_correction":
                self.request_id = None
                self.last_motion_end_robot_ns = int(state["arms"]["receive_mono_ns"])
                if result["completed"]:
                    self.last_arrival_robot_ns = self.last_motion_end_robot_ns
                return state, result
            if monitor is not None and monitor(state, lift):
                arrived = self._stop_and_settle()
                return arrived, dict(monitored_stop=True, completed=True)
            self.sleep(0.02)
        raise TimeoutError("Continuous phase did not finish within bounded deadline")

    def move(self, target):
        # Discrete open/contact requests still use native joint_move, but handle
        # the real asynchronous status propagation and own generation changes.
        state, _ = self.measured()
        previous_id = state["arm_startup"]["joint_control"]["request_id"]
        trace = getattr(self, "trace", None)
        if trace is not None:
            trace.record("sdk_joint_move_request", target=target, speed_rad_s=self.speed_rad_s)
        response = self.joints.move(target, self.speed_rad_s)
        self._adopt_request(response, previous_id)
        deadline = self.clock() + 45
        while self.clock() < deadline:
            state, _ = self.measured()
            result = self.joints.result(self.request_id)
            if result["phase"] in ("fault", "stopped", "unknown_or_superseded"):
                raise RuntimeError("SDK discrete gripper request failed: " + str(result))
            if result["completed"]:
                self.request_id = None
                self.last_arrival_robot_ns = int(state["arms"]["receive_mono_ns"])
                self.last_motion_end_robot_ns = self.last_arrival_robot_ns
                return state
            self.sleep(0.02)
        raise TimeoutError("SDK discrete request timeout")

    def move_until(self, target, stop_when):
        """One SDK gripper goal, continuously observed until contact or arrival."""
        state, _ = self.measured()
        previous_id = state["arm_startup"]["joint_control"]["request_id"]
        trace = getattr(self, "trace", None)
        if trace is not None:
            trace.record("sdk_contact_move_request", target=target, speed_rad_s=self.speed_rad_s)
        response = self.joints.move(target, self.speed_rad_s)
        self._adopt_request(response, previous_id)
        deadline = self.clock() + 45
        while self.clock() < deadline:
            state, _ = self.measured()
            joint = state["arm_startup"]["joint_control"]
            if joint["request_id"] != self.request_id or joint["phase"] in ("fault", "stopped"):
                raise RuntimeError("SDK gripper closing request failed or was superseded")
            if stop_when(state):
                return dict(contact_stop=True, state=self._stop_and_settle())
            result = self.joints.result(self.request_id)
            if result["completed"]:
                self.request_id = None
                self.last_arrival_robot_ns = int(state["arms"]["receive_mono_ns"])
                self.last_motion_end_robot_ns = self.last_arrival_robot_ns
                return dict(contact_stop=False, state=state)
            self.sleep(0.02)
        raise TimeoutError("SDK continuous gripper closure timeout")

    def _combine(self, stages):
        points = []
        for stage in stages:
            if abs(stage["fixed_lift_m"] * 1e6 - self.lift_um) > 200:
                raise ValueError("Phase lift context changed")
            rows = stage["arm_targets"]
            if len(rows) < 2:
                raise ValueError("Continuous phase needs full planned waypoints")
            if points and np.max(np.abs(np.asarray(points[-1]) - rows[0])) > 1e-6:
                raise ValueError("Discontinuous phase boundary")
            points.extend(rows if not points else rows[1:])
        return points

    def _validate_path(self, points):
        if self.collision_policy == "onsite_supervised_skip":
            return
        checker = self.planner.collision_check
        # A curve differs from a straight joint chord. Never reuse a stale
        # collision_checked flag as evidence for the continuous cubic.
        if checker is None or not hasattr(checker, "validate_continuous_phase"):
            raise ValueError("Continuous SDK curve collision validator required")
        if (
            checker.validate_continuous_phase(
                points, self.lift_um * 1e-6, self.speed_rad_s, self.acceleration_rad_s2
            )
            is not True
        ):
            raise ValueError("Continuous SDK phase collision rejected")

    def endpoint_errors(self, targets, state, lift):
        actual = self.planner.ik.fk(
            self.planner.model.values(state["arms"]["position"], lift["current_um"] * 1e-6)
        )
        return {
            side: dict(
                position_m=float(np.linalg.norm(actual[side][:3, 3] - pose(goal)[:3, 3])),
                angle_rad=float(
                    Rotation.from_matrix(pose(goal)[:3, :3] @ actual[side][:3, :3].T).magnitude()
                ),
            )
            for side, goal in targets.items()
        }

    def execute_phase(self, stages, *, name, monitor=None, max_corrections=None):
        stages = list(stages)
        if not stages:
            raise ValueError("Empty motion phase")
        goals = stages[-1]["targets"]
        points = self._combine(stages)
        budget = self.max_corrections if max_corrections is None else max_corrections
        if type(budget) is not int or not 0 <= budget <= self.max_corrections:
            raise ValueError("Invalid endpoint correction budget")
        previous_error = None
        for correction in range(budget + 1):
            state, lift = self.measured()
            if np.max(np.abs(np.asarray(points[0]) - state["arms"]["position"])) > 0.015:
                raise ValueError("Continuous phase no longer starts at measured joints")
            self._validate_path(points)
            previous_id = state["arm_startup"]["joint_control"]["request_id"]
            response = self.trajectory.submit(
                points,
                speed_rad_s=self.speed_rad_s,
                acceleration_rad_s2=self.acceleration_rad_s2,
                settle_timeout_s=self.endpoint_settle_timeout_s,
            )
            self._adopt_request(response, previous_id, continuous=True)
            state, result = self._wait(monitor=monitor)
            _, lift = self.measured()
            errors = self.endpoint_errors(goals, state, lift)
            row = dict(
                phase=name,
                correction=correction,
                result=result,
                errors=errors,
                waypoint_count=len(points),
                collision_checked=self.collision_policy == "required",
                collision_policy=self.collision_policy,
            )
            self.phase_events.append(row)
            if result.get("monitored_stop"):
                return row
            if monitor is not None:
                # A stopped insertion is judged by fresh bilateral rail
                # distances, never by treating a model endpoint as a grip.
                return row
            arrived = result["completed"] and all(
                error["position_m"] <= self.position_tolerance_m
                and error["angle_rad"] <= self.angle_tolerance_rad
                for error in errors.values()
            )
            if arrived:
                return row
            metric = max(
                max(
                    error["position_m"] / self.position_tolerance_m,
                    error["angle_rad"] / self.angle_tolerance_rad,
                )
                for error in errors.values()
            )
            metric = max(metric, result.get("max_error_rad", 0) / 0.02)
            if correction == budget:
                raise RuntimeError(
                    "Phase endpoint did not converge within correction limit: " + str(errors)
                )
            if previous_error is not None and metric >= previous_error * 0.95:
                raise RuntimeError("Phase correction made no measurable progress; retain hold")
            previous_error = metric
            # Re-solve from real feedback, rather than offsetting encoder values
            # or indefinitely resending the same unreachable joint endpoint.
            adjusted = self.planner.stage(
                goals, state["arms"]["position"], lift["current_um"] * 1e-6, steps=20
            )
            points = adjusted["arm_targets"]

    def execute(self, stage):
        return self.execute_phase([stage], name="verification")

    def hold(self):
        self.finish()
        self.cancelled.set()
        if self.request_id is not None and self.joints.pending_id == self.request_id:
            return self.trajectory.stop(self.request_id)
        return dict(commands_sent=False, reason="No owned active phase; retain hold")
