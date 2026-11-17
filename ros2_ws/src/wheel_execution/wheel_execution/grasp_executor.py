"""Stationary pick: visual targets -> SDK waypoint motion -> wrist stop -> grasp.

Uses injected live observation and bilateral verification sources. It never
moves/disables the lift, parks, zeros encoders, or applies a fitted joint bias.
A completed joint request is not a verified grasp or lift.
"""

import copy
import json
import threading
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from wheel_task_core.transforms import pose


class SDKWaypointDriver:
    def __init__(self, joints, read_lift, planner, *, clock=time.monotonic, sleep=time.sleep):
        self.joints, self.read_lift, self.planner = joints, read_lift, planner
        self.clock, self.sleep = clock, sleep
        self.cancelled = threading.Event()
        self.generation = None
        self.lift_um = None
        self.request_id = None
        self.feedback_guard = None
        self.last_arrival_robot_ns = None

    def begin(self):
        state = self.joints.snapshot()
        lift = self.read_lift()
        if state["arms"].get("enabled") is not True:
            raise ValueError("Already enabled arm hold required")
        if self.joints.pending_id:
            raise ValueError("Existing arm request must be reconciled first")
        self.generation = state["arm_startup"]["control_generation"]
        self.lift_um = lift["current_um"]
        self.cancelled.clear()

    def measured(self):
        if self.cancelled.is_set():
            raise RuntimeError("Grasp cancelled")
        state = self.joints.snapshot()
        lift = self.read_lift()
        if state["arm_startup"]["control_generation"] != self.generation:
            raise RuntimeError("Control generation changed")
        if lift.get("stationary") is not True or abs(lift["current_um"] - self.lift_um) > 200:
            raise RuntimeError("Fixed lift height changed")
        if state["arms"].get("enabled") is not True:
            raise RuntimeError("Arm hold lost")
        if self.feedback_guard is not None:
            self.feedback_guard(state)
        return state, lift

    def move(self, target):
        self.measured()
        response = self.joints.move(target, 0.03)
        self.request_id = response["request_id"]
        deadline = self.clock() + 45
        while self.clock() < deadline:
            self.measured()
            result = self.joints.result(self.request_id)
            if result["phase"] in ("stopped", "fault", "unknown_or_superseded"):
                raise RuntimeError("SDK waypoint failed: " + result["phase"])
            if result["completed"]:
                self.request_id = None
                arrived = self.measured()[0]
                self.last_arrival_robot_ns = (
                    int(arrived["arms"]["receive_mono_ns"])
                    if "receive_mono_ns" in arrived["arms"]
                    else None
                )
                return arrived
            self.sleep(0.05)
        raise TimeoutError("SDK waypoint did not reach measured completion")

    def execute(self, stage):
        state, lift = self.measured()
        if stage.get("collision_checked") is not True:
            raise ValueError("Collision-checked stage required")
        if abs(stage["fixed_lift_m"] * 1e6 - lift["current_um"]) > 200:
            raise ValueError("Plan lift height no longer matches feedback")
        if np.max(np.abs(np.asarray(stage["initial_q16"]) - state["arms"]["position"])) > 0.015:
            raise ValueError("Plan no longer starts at current measured joints")
        for target in stage["arm_targets"][1:]:
            state, lift = self.measured()
            checker = self.planner.collision_check
            if checker is None:
                raise ValueError("Scene collision source disconnected before execution")
            if hasattr(checker, "refresh"):
                checker.refresh(state["arms"]["position"])
            current = np.asarray(state["arms"]["position"])
            delta = np.asarray(target) - current
            steps = max(2, int(np.max(np.abs(delta)) / 0.01) + 2)
            for fraction in np.linspace(0, 1, steps):
                interpolated = current + fraction * delta
                values = self.planner.model.values(interpolated, lift["current_um"] * 1e-6)
                if not checker(values):
                    raise ValueError(
                        "Scene collision changed before SDK waypoint: "
                        + str(getattr(checker, "last_collision", None))
                    )
            # Requests are sequential SDK endpoints, not a claimed streaming
            # Cartesian controller. Each short segment is checked and settled.
            self.move(target)
        state, lift = self.measured()
        actual = self.planner.ik.fk(
            self.planner.model.values(state["arms"]["position"], lift["current_um"] * 1e-6)
        )
        for side in ("left", "right"):
            if np.linalg.norm(actual[side][:3, 3] - pose(stage["targets"][side])[:3, 3]) > 0.005:
                raise RuntimeError("Measured wrist FK failed stage arrival")

    def hold(self):
        self.cancelled.set()
        # Never issue a global stop for another client's request.
        pending = self.joints.pending_id
        if pending is not None:
            return self.joints.stop(pending)
        return dict(commands_sent=False, reason="No owned motion; retain current hold")


class StationaryGraspExecutor:
    def __init__(
        self,
        planner,
        driver,
        observe,
        wrist_stop,
        close_verify,
        lift_verify,
        *,
        open_verify,
        output=None,
        clock=time.monotonic,
        max_target_age_s=2.0,
        max_insertion_m=0.04,
    ):
        self.planner, self.driver, self.observe = planner, driver, observe
        self.wrist_stop, self.close_verify, self.lift_verify = wrist_stop, close_verify, lift_verify
        self.open_verify = open_verify
        if not np.isfinite(max_insertion_m) or not 0 < max_insertion_m <= 0.25:
            raise ValueError("Explicit bounded insertion travel required")
        self.max_insertion_m = max_insertion_m
        self.output = Path(output) if output else None
        self.clock, self.max_target_age_s = clock, max_target_age_s
        if not np.isfinite(max_target_age_s) or max_target_age_s <= 0:
            raise ValueError("Positive finite visual target timeout required")
        self.events = []
        self.may_be_holding = False
        self.lock = threading.Lock()

    def event(self, phase, **data):
        self.events.append(dict(phase=phase, monotonic=self.clock(), **data))
        trace = getattr(self.driver, "trace", None)
        if trace is not None:
            try:
                trace.record("task_state", phase=phase, **data)
            except Exception:
                if phase not in ("BLOCKED", "HOLD"):
                    raise
                # 停止阶段保留原始故障；结构化日志损坏也继续尝试保存执行结果。
        if self.output:
            self.output.mkdir(parents=True, exist_ok=True)
            (self.output / "execution.json").write_text(
                json.dumps(
                    dict(events=self.events, may_be_holding=self.may_be_holding),
                    indent=2,
                    allow_nan=False,
                )
                + "\n"
            )

    def target(self, target_id, previous_pair=None):
        target = self.observe(target_id)
        observed_at = target.get("observed_at_monotonic")
        if observed_at is None:
            raise ValueError("Live process-local monotonic visual timestamp required")
        age = self.clock() - observed_at
        if (
            target["target_id"] != target_id
            or not target["pair_id"]
            or target["pair_id"] == previous_pair
        ):
            raise ValueError("Wrong target or duplicate visual pair")
        if not np.isfinite(age) or not 0 <= age <= self.max_target_age_s:
            raise ValueError("Visual target expired during perception")
        if (
            target.get("frame_id") != "arm_origin"
            or target.get("tool_frame") != "cad_contact_midpoint_pinch_y"
        ):
            raise ValueError("Visual grasp frame/convention mismatch")
        for stage in ("pregrasp", "grasp"):
            if set(target[stage]) != {"left", "right"}:
                raise ValueError("Bilateral visual tool poses required")
            for value in target[stage].values():
                pose(value)
        if target.get("execution_blockers") or target.get("diagnostic_only") is not False:
            raise ValueError(
                "Visual target remains diagnostic: " + str(target.get("execution_blockers"))
            )
        return target

    def run(self, target_id):
        if not self.lock.acquire(blocking=False):
            raise RuntimeError("Grasp already running")
        began = False
        try:
            if self.may_be_holding or self.events:
                raise RuntimeError(
                    "Use a new commissioned pick session; automatic retry while possibly loaded is forbidden"
                )
            self.may_be_holding = False
            target = self.target(target_id)
            self.driver.begin()
            began = True
            state, lift = self.driver.measured()
            pregrasp = self.planner.wrist_targets(
                target, "pregrasp", lift["current_um"] * 1e-6, q16=state["arms"]["position"]
            )
            grasp = self.planner.wrist_targets(
                target, "grasp", lift["current_um"] * 1e-6, q16=state["arms"]["position"]
            )
            if any(
                pregrasp[side][2, 3] - grasp[side][2, 3] > self.max_insertion_m for side in pregrasp
            ):
                raise ValueError("Requested insertion exceeds trial bound before alignment")
            plan = self.planner.plan(target, state["arms"]["position"], lift["current_um"] * 1e-6)
            if not plan["execution_ready"]:
                raise ValueError("Grasp plan rejected: " + str(plan["execution_blockers"]))
            # Planning itself may consume the observation budget.
            if self.clock() - target["observed_at_monotonic"] > self.max_target_age_s:
                raise ValueError("Visual target expired during planning")
            for stage_name, event in (
                ("clearance", "CLEARANCE"),
                ("align", "ALIGN"),
                ("pregrasp", "PREGRASP"),
            ):
                if stage_name in plan["stages"]:
                    self.event(event)
                    self.driver.execute(plan["stages"][stage_name])
            target = self.target(target_id, target["pair_id"])
            self.event("REOBSERVE", pair_id=target["pair_id"])
            if hasattr(self.planner.collision_check, "update_target"):
                self.planner.collision_check.update_target(target)
            state, lift = self.driver.measured()
            expected = self.planner.wrist_targets(
                target, "pregrasp", lift["current_um"] * 1e-6, q16=state["arms"]["position"]
            )
            actual = self.planner.ik.fk(
                self.planner.model.values(state["arms"]["position"], lift["current_um"] * 1e-6)
            )
            for side in expected:
                position_error = np.linalg.norm(actual[side][:3, 3] - expected[side][:3, 3])
                rotation_error = Rotation.from_matrix(
                    expected[side][:3, :3] @ actual[side][:3, :3].T
                ).magnitude()
                if position_error > 0.005 or rotation_error > 0.02:
                    raise ValueError("Fresh target no longer aligned before opening: " + side)
            self.event("ALIGNED")
            self.event("OPEN")
            opened = self.open_verify(target)
            if opened.get("bilateral_open_verified") is not True:
                raise ValueError("Bilateral gripper opening not confirmed")
            self.event("OPEN_CONFIRMED")
            stable = 0
            previous_wrist_pairs = None
            start_state, _ = self.driver.measured()
            start = self.planner.ik.fk(
                self.planner.model.values(
                    start_state["arms"]["position"], lift["current_um"] * 1e-6
                )
            )
            approach_deadline = self.clock() + 60
            for _ in range(100):
                if self.clock() > approach_deadline:
                    raise TimeoutError("Wrist approach deadline exceeded")
                feedback = self.wrist_stop(target)
                if feedback.get("accepted") is not True or feedback.get("blockers"):
                    raise ValueError("Wrist insertion feedback rejected: " + str(feedback))
                wrist_age = self.clock() - feedback["observed_at_monotonic"]
                if not np.isfinite(wrist_age) or not 0 <= wrist_age <= 0.5:
                    raise ValueError("Wrist insertion feedback stale")
                pairs = feedback["pair_ids"]
                if (
                    set(pairs) != {"left", "right"}
                    or not all(pairs.values())
                    or (
                        previous_wrist_pairs
                        and any(pairs[s] == previous_wrist_pairs[s] for s in pairs)
                    )
                ):
                    raise ValueError("Fresh independent bilateral wrist pairs required")
                previous_wrist_pairs = dict(pairs)
                remaining = feedback["remaining_downward_m"]
                if (
                    set(remaining) != {"left", "right"}
                    or not np.isfinite(list(remaining.values())).all()
                    or min(remaining.values()) < -0.003
                ):
                    raise ValueError("Invalid or overshot rail clearance")
                reached = max(remaining.values()) <= 0
                stable = stable + 1 if reached else 0
                if stable >= 3:
                    break
                if reached:
                    continue
                state, lift = self.driver.measured()
                measured = self.planner.model.values(
                    state["arms"]["position"], lift["current_um"] * 1e-6
                )
                goals = self.planner.ik.fk(measured)
                for side in goals:
                    travel = max(0.0, min(0.002, remaining[side]))
                    goals[side][2, 3] -= travel
                    if start[side][2, 3] - goals[side][2, 3] > self.max_insertion_m:
                        raise ValueError("Insertion exceeded configured trial bound")
                self.event("APPROACH", remaining_downward_m=remaining)
                self.driver.execute(
                    self.planner.stage(
                        goals, state["arms"]["position"], lift["current_um"] * 1e-6, steps=2
                    )
                )
            else:
                raise TimeoutError("Wrist height condition did not settle")
            self.event("RAIL_STOP")
            self.may_be_holding = True
            self.event("CLOSE")
            closure = self.close_verify(target)
            if closure.get("bilateral_contact_verified") is not True:
                raise ValueError("Joint closure is not verified bilateral capture")
            state, lift = self.driver.measured()
            goals = self.planner.ik.fk(
                self.planner.model.values(state["arms"]["position"], lift["current_um"] * 1e-6)
            )
            for value in goals.values():
                value[2, 3] += 0.01
            self.event("TEST_LIFT")
            self.driver.execute(
                self.planner.stage(
                    goals, state["arms"]["position"], lift["current_um"] * 1e-6, steps=5
                )
            )
            verification = self.lift_verify(target, closure)
            if verification.get("box_followed_both_hands") is not True:
                raise ValueError("Box did not pass measured test-lift verification")
            self.event("COMPLETE")
            return dict(completed=True, may_be_holding=True, events=copy.deepcopy(self.events))
        except BaseException as exc:
            stop_error = None
            if began:
                try:
                    self.driver.hold()
                except Exception as hold_exc:
                    stop_error = str(hold_exc)
            self.event(
                "HOLD" if self.may_be_holding else "BLOCKED", error=str(exc), hold_error=stop_error
            )
            raise
        finally:
            self.lock.release()


class SDKContactCloser:
    """Incremental positive-toward-zero closing with bilateral measured contact.

    Effort and contact-angle limits MUST come from a commissioned policy.
    They are never guessed from a pose fit or disabled to get a demonstration.
    The caller retains the load on any failure; no implicit reopening.
    """

    def __init__(self, driver, policy):
        self.driver, self.policy = driver, copy.deepcopy(policy)
        if policy.get("hardware_validated") is not True:
            raise ValueError("Measured bilateral gripper-contact policy required")
        for side in ("left", "right"):
            item = policy[side]
            if (
                not np.isfinite(
                    [
                        item["contact_effort_min_nm"],
                        item["hard_effort_max_nm"],
                        item["contact_angle_min_rad"],
                        item["contact_angle_max_rad"],
                        item["step_rad"],
                    ]
                ).all()
                or not 0 < item["contact_effort_min_nm"] < item["hard_effort_max_nm"]
                or not item["contact_angle_min_rad"] < item["contact_angle_max_rad"] < 0
                or not 0 < item["step_rad"] <= 0.02
            ):
                raise ValueError("Invalid gripper contact/effort limits")

    def __call__(self, target):
        del target
        contacts = {side: 0 for side in ("left", "right")}
        previous_guard = self.driver.feedback_guard

        def guard(state):
            if previous_guard is not None:
                previous_guard(state)
            for side, index in (("left", 15), ("right", 7)):
                if not np.isfinite(state["arms"]["effort"][index]):
                    raise RuntimeError("Invalid gripper effort feedback: " + side)
                if abs(state["arms"]["effort"][index]) >= self.policy[side]["hard_effort_max_nm"]:
                    raise RuntimeError("Gripper hard effort limit: " + side)

        self.driver.feedback_guard = guard
        capture_verified = False
        try:
            for _ in range(150):
                state, _ = self.driver.measured()
                holding = list(state["arm_startup"]["nominal_target"])
                changed = False
                for side, index in (("left", 15), ("right", 7)):
                    item = self.policy[side]
                    angle = state["arms"]["position"][index]
                    effort = abs(state["arms"]["effort"][index])
                    contact = (
                        item["contact_angle_min_rad"] <= angle <= item["contact_angle_max_rad"]
                        and effort >= item["contact_effort_min_nm"]
                        and abs(state["arms"]["velocity"][index]) < 0.08
                    )
                    contacts[side] = contacts[side] + 1 if contact else 0
                    if contact:
                        continue
                    if angle >= -0.001:
                        raise ValueError(
                            "Gripper reached empty-closed region without rim contact: " + side
                        )
                    holding[index] = min(0.0, angle + item["step_rad"])
                    changed = True
                if min(contacts.values()) >= 3:
                    capture_verified = True
                    return dict(bilateral_contact_verified=True, measured_state=state)
                if changed:
                    self.driver.move(holding)
                else:
                    self.driver.sleep(0.05)
            raise TimeoutError("Bilateral gripper contact not confirmed")
        finally:
            if not capture_verified:
                self.driver.feedback_guard = previous_guard


class SDKGripperOpener:
    """Open at the aligned pose; retain all nominal arm targets and verify hold."""

    def __init__(self, driver, mapping, *, effort_limits, step_rad=0.02):
        self.driver = driver
        self.mapping = copy.deepcopy(mapping)
        self.effort_limits = dict(effort_limits)
        if step_rad is not None and (not np.isfinite(step_rad) or step_rad <= 0):
            raise ValueError("Positive gripper opening step or full-goal mode required")
        self.step_rad = step_rad
        if any(
            not np.isfinite(self.effort_limits[side]) or self.effort_limits[side] <= 0
            for side in ("left", "right")
        ):
            raise ValueError("Finite opening effort limits required")
        for side in ("left", "right"):
            config = self.mapping[side]
            if not np.isfinite(config["configured_open_rad"]) or config["opening_sign"] not in (
                -1,
                1,
            ):
                raise ValueError("Explicit SDK opening reference required")
            if (config["configured_open_rad"] - config["configured_closed_rad"]) * config[
                "opening_sign"
            ] <= 0:
                raise ValueError("SDK opening reference/direction disagree")

    def __call__(self, target):
        del target
        initial, lift = self.driver.measured()
        anchor = np.asarray(initial["arms"]["position"])
        limits = initial["arm_startup"]["joint_control"]
        goals = {side: self.mapping[side]["configured_open_rad"] for side in ("left", "right")}
        for side, index in (("left", 15), ("right", 7)):
            if not limits["lower"][index] <= goals[side] <= limits["upper"][index]:
                raise ValueError("Opening reference outside current SDK limits")
        wrist_anchor = self.driver.planner.ik.fk(
            self.driver.planner.model.values(anchor, lift["current_um"] * 1e-6)
        )
        previous_guard = self.driver.feedback_guard

        def hold_guard(state):
            if previous_guard is not None:
                previous_guard(state)
            for side, index in (("left", 15), ("right", 7)):
                effort = state["arms"]["effort"][index]
                if not np.isfinite(effort) or abs(effort) >= self.effort_limits[side]:
                    raise RuntimeError("Gripper load rejected while opening: " + side)
            actual = self.driver.planner.ik.fk(
                self.driver.planner.model.values(
                    state["arms"]["position"], lift["current_um"] * 1e-6
                )
            )
            for side in actual:
                if np.linalg.norm(actual[side][:3, 3] - wrist_anchor[side][:3, 3]) > 0.003:
                    raise RuntimeError("Arm alignment changed while opening: " + side)
                if (
                    Rotation.from_matrix(
                        actual[side][:3, :3] @ wrist_anchor[side][:3, :3].T
                    ).magnitude()
                    > 0.02
                ):
                    raise RuntimeError("Arm orientation changed while opening: " + side)

        self.driver.feedback_guard = hold_guard
        try:
            stable = 0
            for _ in range(150):
                state, _ = self.driver.measured()
                feedback = np.asarray(state["arms"]["position"])
                arrived = all(
                    abs(feedback[index] - goals[side]) <= 0.01
                    and abs(state["arms"]["velocity"][index]) < 0.08
                    for side, index in (("left", 15), ("right", 7))
                )
                stable = stable + 1 if arrived else 0
                if stable >= 3:
                    return dict(bilateral_open_verified=True, measured_state=state)
                if arrived:
                    self.driver.sleep(0.05)
                    continue
                commanded = list(state["arm_startup"]["nominal_target"])
                for side, index in (("left", 15), ("right", 7)):
                    if self.step_rad is None:
                        commanded[index] = float(goals[side])
                    else:
                        commanded[index] = float(
                            feedback[index]
                            + np.clip(goals[side] - feedback[index], -self.step_rad, self.step_rad)
                        )
                checker = self.driver.planner.collision_check
                collision_skip = (
                    getattr(self.driver, "collision_policy", "required") == "onsite_supervised_skip"
                )
                if checker is None and not collision_skip:
                    raise ValueError("Collision check required before opening")
                for fraction in () if collision_skip else np.linspace(0, 1, 5):
                    intermediate = feedback + fraction * (np.asarray(commanded) - feedback)
                    if hasattr(checker, "refresh"):
                        checker.refresh(intermediate)
                    if not checker(
                        self.driver.planner.model.values(intermediate, lift["current_um"] * 1e-6)
                    ):
                        raise ValueError("Gripper opening sweep collides")
                self.driver.move(commanded)
            raise TimeoutError("Gripper open feedback did not settle")
        finally:
            self.driver.feedback_guard = previous_guard
