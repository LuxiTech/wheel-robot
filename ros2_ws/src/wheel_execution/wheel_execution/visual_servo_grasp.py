"""Two grasp phases driven only by current visual observations and SDK feedback."""

import copy
import json

import numpy as np

from wheel_execution.grasp_executor import StationaryGraspExecutor
from wheel_execution.reuse_existing import REUSE_EXISTING
from wheel_execution.visual_servo import LatestObservation, visual_packet


class VisualServoGraspExecutor(StationaryGraspExecutor):
    def __init__(self, *args, resource_policy="strict", **kwargs):
        super().__init__(*args, **kwargs)
        self.resource_policy = resource_policy

    def packet(self, target, goals):
        return visual_packet(target, goals, resource_policy=self.resource_policy)

    def _wait_visual(self, stream, *, timeout=20):
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            value = stream.latest()
            if value is not None:
                return value
            self.driver.sleep(0.01)
        raise TimeoutError("No fresh visual observation; no motion started")

    def _validate(self, target, target_id):
        if target["target_id"] != target_id:
            raise ValueError("Visual target identity changed")
        self.packet(target, {})
        if abs(target.get("height_um", self.driver.lift_um) - self.driver.lift_um) > 200:
            raise ValueError("Visual target uses a different lift height")

    def _alignment(self, stream, target_id):
        # 对准阶段保持当前夹爪开口：先留净空，再等高平移/转向，最后到预抓位置。
        # 本次用户明确复用原先预抓取端点对准，不强制先锁住当前腕姿另做一段抬高。
        # 目标本身仍按整个夹爪高于箱沿的既有几何规则生成。
        phase = ["pregrasp" if self.resource_policy == REUSE_EXISTING else "clearance"]
        cached = {}

        def goal(target, state, lift):
            self._validate(target, target_id)
            if cached.get("pair") == target["pair_id"]:
                return cached["packet"]
            height = lift["current_um"] * 1e-6
            current = self.planner.ik.fk(
                self.planner.model.values(state["arms"]["position"], height)
            )
            if "rim_top" in target:
                options = self.planner.alignment_targets(target, state["arms"]["position"], height)
                if phase[0] == "clearance" and all(
                    current[side][2, 3] >= options["clearance"][side][2, 3] - 0.005
                    for side in current
                ):
                    phase[0] = "align"
                if phase[0] == "align":
                    errors = self.driver.endpoint_errors(options["align"], state, lift)
                    if all(
                        error["position_m"] <= 0.005 and error["angle_rad"] <= 0.02
                        for error in errors.values()
                    ):
                        phase[0] = "pregrasp"
                goals = options[phase[0]]
            else:
                phase[0] = "pregrasp"
                goals = self.planner.wrist_targets(
                    target, "pregrasp", height, q16=state["arms"]["position"]
                )
            packet = self.packet(target, goals)
            packet["alignment_phase"] = phase[0]
            cached.update(pair=target["pair_id"], packet=packet)
            return packet

        def complete(target, state, lift, packet):
            errors = self.driver.endpoint_errors(packet["targets"], state, lift)
            return packet["alignment_phase"] == "pregrasp" and all(
                error["position_m"] <= self.driver.position_tolerance_m
                and error["angle_rad"] <= self.driver.angle_tolerance_rad
                for error in errors.values()
            )

        self.event("ALIGN_VISUAL_SERVO")
        self.driver.follow(stream, goal, name="align", complete=complete)

    def _insert(self, stream, target_id, wrist):
        # 下插使用双腕实测间隙；任一侧到达停止高度就保持该侧，不能只看头部目标。
        start, lift = self.driver.measured()
        height = lift["current_um"] * 1e-6
        anchor = self.planner.ik.fk(self.planner.model.values(start["arms"]["position"], height))
        held = {}
        cached = {}

        def goal(target, state, lift):
            self._validate(target, target_id)
            feedback = wrist.latest(max_age_s=2.0)
            if feedback is None or feedback.get("accepted") is not True or feedback.get("blockers"):
                raise ValueError("Fresh bilateral wrist clearance required during insertion")
            pair_key = json.dumps([target["pair_id"], feedback["pair_ids"]], sort_keys=True)
            if cached.get("pair") == pair_key:
                return cached["packet"]
            remaining = feedback["remaining_downward_m"]
            if (
                set(remaining) != {"left", "right"}
                or not np.isfinite(list(remaining.values())).all()
                or min(remaining.values()) < -0.003
            ):
                raise ValueError("Invalid or overshot wrist clearance")
            goals = self.planner.wrist_targets(
                target, "grasp", lift["current_um"] * 1e-6, q16=state["arms"]["position"]
            )
            current = self.planner.ik.fk(
                self.planner.model.values(state["arms"]["position"], lift["current_um"] * 1e-6)
            )
            for side in goals:
                if side in held:
                    goals[side] = held[side].copy()
                    continue
                if target.get("orientation_policy") == "ground_vertical_pinch_y":
                    # Freeze the arrived horizontal position and orientation;
                    # wrist depth may shorten descent, never add a sideways correction.
                    goals[side][:2, 3] = anchor[side][:2, 3]
                    goals[side][:3, :3] = anchor[side][:3, :3]
                goals[side][2, 3] = feedback["sdk_wrist_z_m"][side] - max(0.0, remaining[side])
                if remaining[side] <= 0:
                    held[side] = current[side].copy()
                    goals[side] = held[side].copy()
                if (
                    anchor[side][2, 3] - min(goals[side][2, 3], current[side][2, 3])
                    > self.max_insertion_m
                ):
                    raise ValueError("Visual insertion exceeds configured trial bound")
            packet = self.packet(target, goals)
            packet["pair_id"] = json.dumps(
                [target["pair_id"], feedback["pair_ids"]], sort_keys=True
            )
            packet["image_receive_mono_ns"] = min(
                packet["image_receive_mono_ns"], feedback["image_receive_mono_ns"]
            )
            packet["rail_arrived"] = len(held) == 2 and max(remaining.values()) <= 0
            # Explicit wrist-stop events let the trajectory executor distinguish
            # a real stop from ordinary visual target jitter.
            packet["held_sides"] = sorted(held)
            packet["confirmation_id"] = json.dumps(feedback["pair_ids"], sort_keys=True)
            cached.update(pair=pair_key, packet=packet)
            return packet

        self.event("INSERT_VISUAL_SERVO")
        self.driver.follow(
            stream,
            goal,
            name="insert",
            timeout_s=60,
            complete=lambda target, state, lift, packet: packet["rail_arrived"],
        )

    def run(self, target_id):
        if not self.lock.acquire(blocking=False):
            raise RuntimeError("Grasp already running")
        began = False
        stream = wrist = None
        try:
            if self.events or self.may_be_holding:
                raise RuntimeError("A fresh visual pick session is required")
            if getattr(self.driver, "pure_visual_servo", False) is not True:
                raise ValueError("Pure visual servo driver required")
            if getattr(self.wrist_stop, "supports_motion", False) is not True:
                raise ValueError("Moving wrist time pairing required")
            self.driver.begin()
            began = True
            if hasattr(self.observe, "enable_streaming"):
                self.observe.enable_streaming()
            stream = LatestObservation(lambda: self.observe(target_id), clock=self.clock)
            stream.start()
            target = self._wait_visual(stream)
            self._validate(target, target_id)
            state, lift = self.driver.measured()
            before = self.planner.wrist_targets(
                target, "pregrasp", lift["current_um"] * 1e-6, q16=state["arms"]["position"]
            )
            final = self.planner.wrist_targets(
                target, "grasp", lift["current_um"] * 1e-6, q16=state["arms"]["position"]
            )
            if any(
                before[side][2, 3] - final[side][2, 3] > self.max_insertion_m for side in before
            ):
                raise ValueError("Requested visual insertion exceeds trial bound before motion")
            self._alignment(stream, target_id)
            self.event("OPEN")
            if self.open_verify(stream.latest()).get("bilateral_open_verified") is not True:
                raise ValueError("Bilateral opening not confirmed")
            wrist = LatestObservation(lambda: self.wrist_stop(stream.latest()), clock=self.clock)
            wrist.start()
            self._wait_visual(wrist, timeout=3)
            self._insert(stream, target_id, wrist)
            wrist.close()
            self.may_be_holding = True
            self.event("CLOSE")
            closure = self.close_verify(stream.latest())
            closure_key = (
                "bilateral_closure_observed"
                if self.resource_policy == REUSE_EXISTING
                else "bilateral_contact_verified"
            )
            if closure.get(closure_key) is not True:
                raise ValueError("Bilateral contact not confirmed")
            pre_lift_target = stream.latest()
            if pre_lift_target is None:
                raise ValueError("Fresh box observation required before loaded test lift")
            state, lift = self.driver.measured()
            raised = self.planner.ik.fk(
                self.planner.model.values(state["arms"]["position"], lift["current_um"] * 1e-6)
            )
            for matrix in raised.values():
                matrix[2, 3] += 0.01
            # A held box follows the hands. Anchor the 10mm trial to its start;
            # continually adding 10mm to the moving box would chase it upward.
            self.event("TEST_LIFT")
            self.driver.follow(
                stream,
                lambda target, state, lift: self.packet(target, copy.deepcopy(raised)),
                name="test_lift",
                timeout_s=30,
            )
            stream.close(timeout=3)
            if stream.thread.is_alive():
                raise RuntimeError(
                    "Previous visual capture still running before test-lift verification"
                )
            if (
                self.lift_verify(pre_lift_target, closure).get("box_followed_both_hands")
                is not True
            ):
                raise ValueError("Visual test lift not confirmed")
            self.event("COMPLETE")
            return dict(
                completed=True,
                may_be_holding=True,
                execution_mode=getattr(self.driver, "execution_mode", "pure_visual_servo"),
                events=copy.deepcopy(self.events),
                resource_policy=self.resource_policy,
                force_contact_calibrated=self.resource_policy != REUSE_EXISTING,
            )
        except BaseException as exc:
            stop_error = None
            if began:
                try:
                    self.driver.hold()
                except Exception as stop_exc:
                    stop_error = str(stop_exc)
            self.event(
                "HOLD" if self.may_be_holding else "BLOCKED", error=str(exc), hold_error=stop_error
            )
            raise
        finally:
            for source in (wrist, stream):
                if source is not None:
                    source.close()
            if began:
                self.driver.finish()
            if hasattr(self.observe, "disable_streaming"):
                self.observe.disable_streaming()
            self.lock.release()
