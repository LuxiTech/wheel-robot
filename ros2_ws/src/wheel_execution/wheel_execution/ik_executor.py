"""Bounded pair IK -> staged lift/arm SDK goals -> measured FK confirmation.

This executes an endpoint, not a time-synchronised Cartesian trajectory. Lift
moves first; arms then move at the new height. Scene clearance is a caller
responsibility. Coordinates are SDK EE control points in sdk_lift_zero.
"""

import json
import math
import threading
import time
import uuid
from copy import deepcopy
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from wheel_motion_core.live_kinematics import SDKAlignedModel
from wheel_sdk_bridge.http_client import SDKError


class SDKIKExecutor:
    def __init__(
        self,
        joints,
        lift,
        urdf,
        sdk_chains,
        *,
        max_lift_delta=0.04,
        max_joint_delta=0.55,
        arm_speed=0.03,
        output=None,
    ):
        if (
            not 0 < max_lift_delta <= 0.05
            or not 0 < max_joint_delta <= 0.6
            or not 0.02 <= arm_speed <= 0.05
        ):
            raise ValueError("Invalid bounded endpoint execution limits")
        self.joints, self.lift = joints, lift
        self.urdf, self.sdk_chains = urdf, sdk_chains
        self.max_lift_delta, self.max_joint_delta, self.arm_speed = (
            max_lift_delta,
            max_joint_delta,
            arm_speed,
        )
        self.output = Path(output) if output else None
        self._plans = {}
        self._lock = threading.Lock()
        self._cancel = threading.Event()

    def _save(self, name, value):
        if self.output:
            self.output.mkdir(parents=True, exist_ok=True)
            (self.output / name).write_text(
                json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
            )

    def _read(self):
        before = self.joints.snapshot()
        lift = self.lift.snapshot()
        after = self.joints.snapshot()
        if (
            before["arm_startup"]["control_generation"]
            != after["arm_startup"]["control_generation"]
            or max(
                abs(a - b) for a, b in zip(before["arms"]["position"], after["arms"]["position"])
            )
            > 0.01
        ):
            raise SDKError("Arms changed while pairing lift feedback")
        if (
            after["arms"].get("enabled") is not True
            or max(abs(v) for v in after["arms"]["velocity"]) > 0.08
            or after["arm_startup"]["joint_control"]["phase"] not in ("idle", "ready", "stopped")
        ):
            raise SDKError("Enabled stationary arms required before IK planning")
        if lift["lift_status"].get("stationary") is not True:
            raise SDKError("Stationary lift required before IK planning")
        return after, lift

    def plan_relative(self, left_delta_m, right_delta_m, *, lift_delta_bounds=None):
        arms, lift = self._read()
        model, ik, values = self._model(arms, lift)
        targets = ik.fk(values)
        for side, delta in [("left", left_delta_m), ("right", right_delta_m)]:
            delta = np.asarray(delta, float)
            if delta.shape != (3,) or not np.isfinite(delta).all():
                raise ValueError("Expected finite xyz translation")
            targets[side][:3, 3] += delta
        bounds = (
            None
            if lift_delta_bounds is None
            else tuple(values["lift_joint"] + float(v) for v in lift_delta_bounds)
        )
        return self._plan(arms, lift, model, ik, values, targets, bounds)

    def plan(self, targets, *, frame="sdk_lift_zero", lift_bounds=None):
        if frame != "sdk_lift_zero":
            raise ValueError("Targets must be registered to sdk_lift_zero")
        arms, lift = self._read()
        model, ik, values = self._model(arms, lift)
        return self._plan(arms, lift, model, ik, values, targets, lift_bounds)

    def _model(self, arms, lift):
        limits = arms["arm_startup"]["joint_control"]
        model = SDKAlignedModel(self.urdf, self.sdk_chains, limits["lower"], limits["upper"])
        values = model.values(arms["arms"]["position"], lift["lift"]["current_um"] / 1e6)
        return model, model.ik(), values

    def _plan(self, arms, lift, model, ik, values, targets, lift_bounds):
        h = values["lift_joint"]
        bounds = [max(-0.18, h - self.max_lift_delta), min(0.18, h + self.max_lift_delta)]
        if lift_bounds is not None:
            if len(lift_bounds) != 2 or not np.isfinite(lift_bounds).all():
                raise ValueError("Invalid lift bounds")
            bounds = [max(bounds[0], lift_bounds[0]), min(bounds[1], lift_bounds[1])]
        solution = ik.solve(targets, values, lift_bounds=bounds, max_nfev=500)
        if not solution.success:
            raise ValueError("Joint IK target infeasible")
        if any(
            abs(solution.joints[n] - values[n]) > self.max_joint_delta
            for n in ik.names
            if n != "lift_joint"
        ):
            raise ValueError("IK solution exceeds bounded joint excursion")
        hold = arms["arm_startup"]["nominal_target"]
        target = model.sdk_target(solution.joints, hold)
        # Runtime bounds are the intersection of SDK and URDF, never clipping.
        self.joints.prepare(target, self.arm_speed)
        plan_id = "ik-" + uuid.uuid4().hex
        plan = dict(
            plan_id=plan_id,
            frame=model.base,
            endpoint="SDK EE / URDF link7",
            execution_order=["lift", "arms"],
            targets={s: np.asarray(t).tolist() for s, t in targets.items()},
            measured=values,
            solution={n: float(v) for n, v in solution.joints.items()},
            arm_target=target,
            lift_target_m=float(solution.joints["lift_joint"]),
            position_errors_m=solution.position_errors,
            rotation_errors_rad=solution.rotation_errors,
            lift_delta_m=float(solution.joints["lift_joint"] - h),
            arm_delta_deg={
                n: math.degrees(solution.joints[n] - values[n])
                for n in ik.names
                if n != "lift_joint"
            },
            scene_collision_checked=False,
        )
        self._plans[plan_id] = (time.monotonic(), deepcopy(plan), arms, lift)
        self._save("plan.json", plan)
        self._save("before.json", dict(arms=arms, lift=lift))
        return deepcopy(plan)

    def cancel(self):
        self._cancel.set()

    def _check_cancel(self):
        if self._cancel.is_set():
            raise SDKError("IK execution cancelled")

    def _check_baseline(self, arms, lift, planned_arms, planned_lift):
        if (
            arms["arm_startup"]["control_generation"]
            != planned_arms["arm_startup"]["control_generation"]
            or arms["arm_startup"]["nominal_target"]
            != planned_arms["arm_startup"]["nominal_target"]
            or max(
                abs(a - b)
                for a, b in zip(arms["arms"]["position"], planned_arms["arms"]["position"])
            )
            > 0.015
            or abs(lift["lift"]["current_um"] - planned_lift["lift"]["current_um"]) > 500
            or lift["lift"]["process_started_mono_ns"]
            != planned_lift["lift"]["process_started_mono_ns"]
        ):
            raise SDKError("IK plan no longer matches live state")

    def execute(self, plan_id, *, scene_clear=False):
        if scene_clear is not True:
            raise ValueError("Confirm clearance for bounded staged motion with scene_clear=True")
        if not self._lock.acquire(blocking=False):
            raise SDKError("IK execution already active")
        report = dict(completed=False, plan_id=plan_id, commands_sent=False)
        lift_attempted = arm_attempted = False
        try:
            if self.lift.owner or self.joints.pending_id:
                raise SDKError("Existing control request must finish before IK execution")
            created, plan, prior_arms, prior_lift = self._plans.pop(plan_id)
            if time.monotonic() - created > 30:
                raise SDKError("IK plan expired; solve again from live state")
            self._cancel.clear()
            arms, lift = self._read()
            self._check_baseline(arms, lift, prior_arms, prior_lift)
            model, ik, _ = self._model(arms, lift)
            report["stage"] = "lift_open"
            report["commands_sent"] = True
            lift_attempted = True
            self.lift.open()
            self._check_cancel()
            current_arms, current_lift = self._read()
            self._check_baseline(current_arms, current_lift, prior_arms, prior_lift)
            report["stage"] = "lift_move"
            report["lift_command"] = self.lift.move(plan["lift_target_m"])
            self._save("execution.json", report)
            deadline = time.monotonic() + 60
            while time.monotonic() < deadline:
                self._check_cancel()
                status = self.lift.result(report["lift_command"]["task_id"])
                a = self.joints.snapshot()
                if (
                    a["arm_startup"]["control_generation"]
                    != prior_arms["arm_startup"]["control_generation"]
                    or max(
                        abs(x - y)
                        for x, y in zip(a["arms"]["position"], prior_arms["arms"]["position"])
                    )
                    > 0.03
                ):
                    raise SDKError("Arm hold changed during lift motion")
                if status["completed"]:
                    report["lift_arrival"] = status
                    break
                time.sleep(0.1)
            else:
                raise SDKError("IK lift stage timed out")
            self._check_cancel()
            report["stage"] = "arms_move"
            arm_attempted = True
            report["arm_command"] = self.joints.move(plan["arm_target"], self.arm_speed)
            self._save("execution.json", report)
            deadline = time.monotonic() + 60
            while time.monotonic() < deadline:
                self._check_cancel()
                status = self.joints.result(report["arm_command"]["request_id"])
                lift_state = self.lift.snapshot()
                self.lift._active()
                self.lift._owned(lift_state)
                if (
                    abs(lift_state["lift"]["current_um"] - round(plan["lift_target_m"] * 1e6)) > 500
                    or lift_state["lift_status"]["stationary"] is not True
                ):
                    raise SDKError("Lift did not hold solved height during arm motion")
                if status["phase"] in ("fault", "stopped", "unknown_or_superseded"):
                    raise SDKError("IK arm stage " + status["phase"])
                if status["completed"]:
                    report["arm_arrival"] = status
                    break
                time.sleep(0.1)
            else:
                raise SDKError("IK arm stage timed out")
            arms, lift = self._read()
            actual = ik.fk(model.values(arms["arms"]["position"], lift["lift"]["current_um"] / 1e6))
            pe, ae = {}, {}
            for side in ("left", "right"):
                target = np.asarray(plan["targets"][side])
                pe[side] = float(np.linalg.norm(actual[side][:3, 3] - target[:3, 3]))
                ae[side] = float(
                    Rotation.from_matrix(target[:3, :3] @ actual[side][:3, :3].T).magnitude()
                )
            report.update(
                position_errors_m=pe,
                rotation_errors_rad=ae,
                final_joints=model.values(
                    arms["arms"]["position"], lift["lift"]["current_um"] / 1e6
                ),
            )
            self._save("after.json", dict(arms=arms, lift=lift))
            if max(pe.values()) > 0.01 or max(ae.values()) > 0.05:
                raise SDKError("Measured FK does not meet endpoint tolerance")
            report["lift_stop"] = self.lift.stop()
            report.update(completed=True, stage="completed")
            return report
        except BaseException as exc:
            report["error"] = str(exc)
            if arm_attempted and self.joints.pending_id:
                try:
                    report["arm_stop"] = self.joints.stop(self.joints.pending_id)
                except Exception as stop_exc:
                    report["arm_stop_error"] = str(stop_exc)
            if lift_attempted and self.lift.owner:
                try:
                    report["lift_stop"] = self.lift.stop()
                except Exception as stop_exc:
                    report["lift_stop_error"] = str(stop_exc)
            raise
        finally:
            self._save("execution.json", report)
            self._lock.release()
