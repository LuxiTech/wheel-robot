"""Continuous gripper goals stopped by actual bilateral contact feedback."""

import numpy as np

from wheel_execution.grasp_executor import SDKContactCloser


class SDKContinuousContactCloser(SDKContactCloser):
    def __init__(self, driver, policy, gripper_mapping):
        super().__init__(driver, policy)
        self.closed = {
            side: float(gripper_mapping[side]["configured_closed_rad"])
            for side in ("left", "right")
        }
        if not np.isfinite(list(self.closed.values())).all():
            raise ValueError("Finite SDK closed references required")

    def __call__(self, target):
        del target
        previous_guard = self.driver.feedback_guard

        def guard(state):
            if previous_guard is not None:
                previous_guard(state)
            for side, index in (("left", 15), ("right", 7)):
                effort = state["arms"]["effort"][index]
                if (
                    not np.isfinite(effort)
                    or abs(effort) >= self.policy[side]["hard_effort_max_nm"]
                ):
                    raise RuntimeError("Gripper hard effort limit: " + side)

        def contacts(state, *, settled):
            found = {}
            for side, index in (("left", 15), ("right", 7)):
                policy = self.policy[side]
                angle = state["arms"]["position"][index]
                found[side] = bool(
                    policy["contact_angle_min_rad"] <= angle <= policy["contact_angle_max_rad"]
                    and abs(state["arms"]["effort"][index]) >= policy["contact_effort_min_nm"]
                    and (not settled or abs(state["arms"]["velocity"][index]) < 0.08)
                )
            return found

        def stable_contacts():
            stable = dict(left=0, right=0)
            current = None
            for _ in range(3):
                current, _ = self.driver.measured()
                observed = contacts(current, settled=True)
                for side in stable:
                    stable[side] = stable[side] + 1 if observed[side] else 0
                self.driver.sleep(0.05)
            return {side: count == 3 for side, count in stable.items()}, current

        self.driver.feedback_guard = guard
        verified = False
        try:
            # Two jaws can reach the rim at different times. Stop at the first
            # contact, retain that jaw's existing SDK hold, then finish the other.
            # A third bounded attempt accommodates one transient contact sample.
            for _ in range(3):
                held, state = stable_contacts()
                if all(held.values()):
                    verified = True
                    return dict(bilateral_contact_verified=True, measured_state=state)
                target = list(state["arm_startup"]["nominal_target"])
                for side, index in (("left", 15), ("right", 7)):
                    if not held[side]:
                        if state["arms"]["position"][index] >= self.closed[side] - 0.001:
                            raise ValueError(
                                "Gripper reached empty-closed region without contact: " + side
                            )
                        target[index] = self.closed[side]

                def new_contact(current):
                    observed = contacts(current, settled=False)
                    return any(observed[side] and not held[side] for side in held)

                result = self.driver.move_until(target, new_contact)
                if not result["contact_stop"]:
                    raise ValueError("Closed endpoint reached without verified rim contact")
            held, state = stable_contacts()
            if all(held.values()):
                verified = True
                return dict(bilateral_contact_verified=True, measured_state=state)
            raise TimeoutError("Bilateral contact did not settle within closing attempt bound")
        finally:
            if not verified:
                self.driver.feedback_guard = previous_guard
