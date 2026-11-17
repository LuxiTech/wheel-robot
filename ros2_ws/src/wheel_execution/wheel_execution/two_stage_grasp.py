"""Two arm-motion phases: above-rim alignment, then continuous insertion/grasp."""

import copy

import numpy as np

from wheel_execution.grasp_executor import StationaryGraspExecutor


class TwoStageGraspExecutor(StationaryGraspExecutor):
    def _check_wrist(self, target, previous_pairs=None):
        feedback = self.wrist_stop(target)
        if feedback.get("accepted") is not True or feedback.get("blockers"):
            raise ValueError("Wrist insertion feedback rejected: " + str(feedback))
        age = self.clock() - feedback["observed_at_monotonic"]
        if not np.isfinite(age) or not 0 <= age <= 0.5:
            raise ValueError("Wrist insertion feedback stale")
        pairs = feedback["pair_ids"]
        if (
            set(pairs) != {"left", "right"}
            or not all(pairs.values())
            or (previous_pairs and any(pairs[side] == previous_pairs[side] for side in pairs))
        ):
            raise ValueError("Fresh independent bilateral wrist pairs required")
        remaining = feedback["remaining_downward_m"]
        if (
            set(remaining) != {"left", "right"}
            or not np.isfinite(list(remaining.values())).all()
            or min(remaining.values()) < -0.003
        ):
            raise ValueError("Invalid or overshot rail clearance")
        return feedback

    def _alignment(self, target):
        used_corrections = 0
        for correction in range(self.driver.max_corrections + 1):
            state, lift = self.driver.measured()
            if correction == 0:
                plan = self.planner.plan(
                    target, state["arms"]["position"], lift["current_um"] * 1e-6
                )
                blockers = list(plan["execution_blockers"])
                if self.driver.collision_policy == "onsite_supervised_skip":
                    # An explicit policy removes only the absent collision checker
                    # gate. Vision/geometry acceptance flags remain untouched.
                    blockers = [
                        reason
                        for reason in blockers
                        if reason != "scene_collision_checker_not_connected"
                    ]
                if blockers or plan["errors"]:
                    raise ValueError(
                        "Two-stage grasp plan rejected: " + str(blockers or plan["errors"])
                    )
                stages = [
                    plan["stages"][name]
                    for name in ("clearance", "align", "pregrasp")
                    if name in plan["stages"]
                ]
            else:
                # Fresh vision corrections start at the actual above-rim pose;
                # they must not replay the entire clearance/turn route.
                goals = self.planner.wrist_targets(
                    target, "pregrasp", lift["current_um"] * 1e-6, q16=state["arms"]["position"]
                )
                stages = [
                    self.planner.stage(goals, state["arms"]["position"], lift["current_um"] * 1e-6)
                ]
            if self.clock() - target["observed_at_monotonic"] > self.max_target_age_s:
                raise ValueError("Visual target expired during planning")
            self.event(
                "ALIGN_PHASE",
                correction=correction,
                internal_waypoints=sum(len(stage["arm_targets"]) - 1 for stage in stages),
            )
            result = self.driver.execute_phase(
                stages, name="align", max_corrections=self.driver.max_corrections - used_corrections
            )
            used_corrections += result["correction"]
            target = self.target(target["target_id"], target["pair_id"])
            self.event("REOBSERVE", pair_id=target["pair_id"])
            if hasattr(self.planner.collision_check, "update_target"):
                self.planner.collision_check.update_target(target)
            state, lift = self.driver.measured()
            goals = self.planner.wrist_targets(
                target, "pregrasp", lift["current_um"] * 1e-6, q16=state["arms"]["position"]
            )
            errors = self.driver.endpoint_errors(goals, state, lift)
            if all(
                error["position_m"] <= self.driver.position_tolerance_m
                and error["angle_rad"] <= self.driver.angle_tolerance_rad
                for error in errors.values()
            ):
                self.event("ALIGNED", measured_errors=errors)
                return target
            if used_corrections >= self.driver.max_corrections:
                raise ValueError("Fresh alignment did not converge before opening: " + str(errors))
            used_corrections += 1
        raise AssertionError("Unreachable alignment loop")

    def _insert(self, target):
        previous_pairs = None
        start_state, start_lift = self.driver.measured()
        start_pose = self.planner.ik.fk(
            self.planner.model.values(
                start_state["arms"]["position"], start_lift["current_um"] * 1e-6
            )
        )
        deadline = self.clock() + 60
        for correction in range(self.driver.max_corrections + 2):
            feedback = self._check_wrist(target, previous_pairs)
            previous_pairs = dict(feedback["pair_ids"])
            remaining = feedback["remaining_downward_m"]
            if max(remaining.values()) <= 0:
                # Confirmation frames occur while holding, not after every 2mm.
                for _ in range(2):
                    feedback = self._check_wrist(target, previous_pairs)
                    previous_pairs = dict(feedback["pair_ids"])
                    if max(feedback["remaining_downward_m"].values()) > 0:
                        break
                else:
                    self.event("RAIL_STOP", measurements=feedback)
                    return
                remaining = feedback["remaining_downward_m"]
            if correction > self.driver.max_corrections:
                raise ValueError("Bilateral rail stop did not converge within correction limit")
            if self.clock() > deadline:
                raise TimeoutError("Continuous insertion deadline exceeded")
            state, lift = self.driver.measured()
            goals = self.planner.ik.fk(
                self.planner.model.values(state["arms"]["position"], lift["current_um"] * 1e-6)
            )
            for side in goals:
                goals[side][2, 3] -= max(0.0, remaining[side])
                if start_pose[side][2, 3] - goals[side][2, 3] > self.max_insertion_m:
                    raise ValueError("Insertion exceeds configured trial bound")
            stage = self.planner.stage(
                goals, state["arms"]["position"], lift["current_um"] * 1e-6, steps=40
            )

            def monitor(measured, measured_lift):
                nonlocal previous_pairs
                if self.clock() > deadline:
                    raise TimeoutError("Continuous insertion deadline exceeded")
                actual = self.planner.ik.fk(
                    self.planner.model.values(
                        measured["arms"]["position"], measured_lift["current_um"] * 1e-6
                    )
                )
                if any(
                    start_pose[side][2, 3] - actual[side][2, 3] > self.max_insertion_m
                    for side in actual
                ):
                    raise ValueError("Measured insertion exceeded configured trial bound")
                current = self._check_wrist(target, previous_pairs)
                previous_pairs = dict(current["pair_ids"])
                # Stop the shared arm phase as soon as either rail reaches its
                # limit. A bounded correction can finish the other side alone.
                return min(current["remaining_downward_m"].values()) <= 0

            self.event("INSERT_PHASE", correction=correction, remaining_downward_m=remaining)
            self.driver.execute_phase([stage], name="insert", monitor=monitor)
        raise ValueError("Bilateral rail stop did not converge within correction limit")

    def run(self, target_id):
        if not self.lock.acquire(blocking=False):
            raise RuntimeError("Grasp already running")
        began = False
        try:
            if self.may_be_holding or self.events:
                raise RuntimeError("A fresh pick session is required; no automatic loaded retry")
            if getattr(self.driver, "continuous_phases", False) is not True:
                raise ValueError("Continuous SDK phase driver required")
            if getattr(self.wrist_stop, "supports_motion", False) is not True:
                raise ValueError("Time-paired moving wrist feedback required before motion")
            target = self.target(target_id)
            self.driver.begin()
            began = True
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
                raise ValueError("Requested insertion exceeds trial bound before alignment")
            self.event("TWO_STAGE_BEGIN", collision_policy=self.driver.collision_policy)
            target = self._alignment(target)
            self.event("GRASP_PHASE")
            self.event("OPEN")
            if self.open_verify(target).get("bilateral_open_verified") is not True:
                raise ValueError("Bilateral gripper opening not confirmed")
            self.event("OPEN_CONFIRMED")
            self._insert(target)
            self.may_be_holding = True
            self.event("CLOSE")
            closure = self.close_verify(target)
            if closure.get("bilateral_contact_verified") is not True:
                raise ValueError("Bilateral rim contact not confirmed")
            state, lift = self.driver.measured()
            goals = self.planner.ik.fk(
                self.planner.model.values(state["arms"]["position"], lift["current_um"] * 1e-6)
            )
            for goal in goals.values():
                goal[2, 3] += 0.01
            self.event("TEST_LIFT")
            self.driver.execute_phase(
                [
                    self.planner.stage(
                        goals, state["arms"]["position"], lift["current_um"] * 1e-6, steps=5
                    )
                ],
                name="test_lift",
            )
            if self.lift_verify(target, closure).get("box_followed_both_hands") is not True:
                raise ValueError("Box did not pass measured test-lift verification")
            self.event("COMPLETE")
            return dict(
                completed=True,
                may_be_holding=True,
                motion_phases=2,
                events=copy.deepcopy(self.events),
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
            if began:
                self.driver.finish()
            self.lock.release()
