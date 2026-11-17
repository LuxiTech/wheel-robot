"""User-authorized reuse of existing geometry and SDK gripper controls for a demo.

Resource acceptance flags stay unchanged. Runtime failures are never waived, and
an observed jaw stop is not reported as a calibrated force/contact measurement.
"""

import numpy as np
from scipy.spatial.transform import Rotation

REUSE_EXISTING = "user_confirmed_reuse"
RESOURCE_WARNINGS = frozenset(
    {
        "sdk_to_urdf_and_physical_tcp_not_registered",
        "stationary_lift_host_receive_alignment",
        "metric_depth_semantics_unvalidated",
        "local_top_rim_correspondence_unconfirmed",
        "optical_to_rail_correspondence_unconfirmed",
        "aligned_depth_metric_semantics_unconfirmed",
    }
)


def runtime_blockers(reasons, policy):
    if policy == "strict":
        return list(reasons)
    if policy != REUSE_EXISTING:
        raise ValueError("Unknown visual-grasp resource policy")
    return [
        reason
        for reason in reasons
        if reason not in RESOURCE_WARNINGS and not reason.startswith("selected_calibration:")
    ]


class SDKConfiguredGrippers:
    """Reuse the SDK open/closed endpoints and its existing POS_FORCE current cap."""

    def __init__(self, driver, mapping):
        self.driver = driver
        self.mapping = mapping

    def _record(self, kind, **detail):
        trace = getattr(self.driver, "trace", None)
        if trace is not None:
            trace.record(kind, **detail)

    def open(self, target):
        del target
        state, lift = self.driver.measured()
        anchor = self.driver.planner.ik.fk(
            self.driver.planner.model.values(state["arms"]["position"], lift["current_um"] * 1e-6)
        )
        previous_guard = self.driver.feedback_guard

        def hold_guard(measured):
            if previous_guard is not None:
                previous_guard(measured)
            actual = self.driver.planner.ik.fk(
                self.driver.planner.model.values(
                    measured["arms"]["position"], lift["current_um"] * 1e-6
                )
            )
            for side in actual:
                if (
                    np.linalg.norm(actual[side][:3, 3] - anchor[side][:3, 3])
                    > self.driver.position_tolerance_m
                    or Rotation.from_matrix(
                        actual[side][:3, :3] @ anchor[side][:3, :3].T
                    ).magnitude()
                    > self.driver.angle_tolerance_rad
                ):
                    raise RuntimeError("Arm alignment changed while opening: " + side)

        self.driver.feedback_guard = hold_guard
        try:
            reference = list(state["arm_startup"]["nominal_target"])
            for side, index in (("left", 15), ("right", 7)):
                reference[index] = self.mapping[side]["configured_open_rad"]
            self._record("sdk_configured_open", reference=reference)
            self.driver.move(reference)
            final, _ = self.driver.measured()
            if not all(
                abs(final["arms"]["position"][index] - reference[index]) <= 0.01
                and abs(final["arms"]["velocity"][index]) < 0.08
                for index in (7, 15)
            ):
                raise RuntimeError("SDK configured opening did not reach measured position")
            return dict(bilateral_open_verified=True, measured_state=final)
        finally:
            self.driver.feedback_guard = previous_guard

    def close(self, target):
        del target
        held = set()
        observations = {}
        final = None
        # 一侧先受阻就停止当前请求，保留该侧限流保持，再继续闭合另一侧。
        for _ in range(3):
            state, _ = self.driver.measured()
            reference = list(state["arm_startup"]["nominal_target"])
            for side, index in (("left", 15), ("right", 7)):
                if side not in held:
                    reference[index] = self.mapping[side]["configured_closed_rad"]
            still_since = {}
            anchors = {}
            last_stamp = [None]
            newly_held = set()

            def stopped_jaw(measured):
                stamp = int(measured["arms"]["receive_mono_ns"])
                if stamp == last_stamp[0]:
                    return False
                last_stamp[0] = stamp
                now = self.driver.clock()
                for side, index in (("left", 15), ("right", 7)):
                    if side in held:
                        continue
                    actual = measured["arms"]["position"][index]
                    nominal = measured["arm_startup"]["nominal_target"][index]
                    if not np.isfinite(measured["arms"]["effort"][index]):
                        raise RuntimeError("Nonfinite gripper feedback")
                    settled = abs(measured["arms"]["velocity"][index]) < 0.08
                    commanded_past = nominal - actual > 0.01
                    at_closed = abs(actual - reference[index]) <= 0.01
                    if not settled or not (commanded_past or at_closed):
                        still_since.pop(side, None)
                        anchors.pop(side, None)
                        continue
                    if side not in anchors or abs(actual - anchors[side]) > 0.003:
                        anchors[side] = actual
                        still_since[side] = now
                    if now - still_since[side] >= 0.3:
                        newly_held.add(side)
                        observations[side] = dict(
                            angle_rad=actual,
                            effort_nm=measured["arms"]["effort"][index],
                            closed_endpoint=at_closed,
                            motion_blocked=commanded_past,
                            force_contact_calibrated=False,
                        )
                return bool(newly_held)

            self._record("sdk_configured_close", reference=reference, held_sides=sorted(held))
            result = self.driver.move_until(reference, stopped_jaw)
            final = result["state"]
            if not result["contact_stop"]:
                # 正常到达原 SDK 闭合端点，只报告开合观察，抓箱结果由试抬随动决定。
                for side, index in (("left", 15), ("right", 7)):
                    if side not in held:
                        if abs(final["arms"]["position"][index] - reference[index]) > 0.02:
                            raise RuntimeError("SDK closing endpoint not reached")
                        newly_held.add(side)
                        observations[side] = dict(
                            angle_rad=final["arms"]["position"][index],
                            closed_endpoint=True,
                            force_contact_calibrated=False,
                        )
            held.update(newly_held)
            if len(held) == 2:
                evidence = dict(
                    bilateral_closure_observed=True,
                    bilateral_contact_verified=False,
                    measured_state=final,
                    observations=observations,
                )
                self._record("sdk_closure_observed", evidence=evidence)
                return evidence
        raise RuntimeError("Configured jaw closing did not settle on both sides")


class ObservedBoxFollowVerifier:
    def __init__(self, observe, driver):
        self.observe = observe
        self.driver = driver

    def __call__(self, target, closure):
        deadline = self.driver.clock() + 3
        while self.driver.clock() < deadline:
            fresh = self.observe(target["target_id"])
            if fresh["pair_id"] != target["pair_id"]:
                break
            self.driver.sleep(0.02)
        else:
            raise RuntimeError("No new box observation after test lift")
        if (
            fresh["target_id"] != target["target_id"]
            or fresh.get("frame_id") != "arm_origin"
            or runtime_blockers(fresh.get("execution_blockers", []), REUSE_EXISTING)
            or not 0 <= self.driver.clock() - fresh["observed_at_monotonic"] <= 2
        ):
            raise ValueError("Fresh same-object box observation required")
        delta = np.asarray(fresh["object_pose"])[:3, 3] - np.asarray(target["object_pose"])[:3, 3]
        followed = (
            closure.get("bilateral_closure_observed") is True
            and 0.005 <= delta[2] <= 0.015
            and np.linalg.norm(delta[:2]) <= 0.01
        )
        result = dict(
            box_followed_both_hands=bool(followed),
            observed_box_translation_m=delta.tolist(),
            pair_id=fresh["pair_id"],
            force_contact_calibrated=False,
        )
        trace = getattr(self.driver, "trace", None)
        if trace is not None:
            trace.record("observed_box_follow", result=result)
        return result
