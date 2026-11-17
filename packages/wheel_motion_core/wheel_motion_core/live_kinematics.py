"""URDF arm geometry expressed in SDK coordinates with an encoder lift axis.

sdk_lift_zero is a virtual fixed frame: SDK arm_origin translated by encoder
height along +Z. It is NOT registered to base_footprint, a camera or the floor.
End frames are link7 / SDK EE control points, not calibrated finger TCPs.
"""

import numpy as np

from wheel_motion_core.kinematics import PairIK, RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics


class SDKAlignedModel:
    base = "sdk_lift_zero"

    def __init__(self, urdf, sdk_chains, lower, upper, lift_bounds=(-0.18, 0.18)):
        self.urdf = RobotModel(urdf)
        self.sdk = SDKKinematics(sdk_chains)
        self.joints = {"lift_joint": dict(bounds=tuple(lift_bounds), velocity=0.05)}
        self.bases = {}
        if len(lower) != 16 or len(upper) != 16:
            raise ValueError("Expected 16 SDK limit pairs")
        for side, start in [("right", 0), ("left", 8)]:
            self.bases[side] = self.sdk.transform("openarm_" + side + "_base_link")
            for i in range(7):
                name = f"openarm_{side}_joint{i + 1}"
                spec = self.urdf.joints[name]
                # SDK发布当前运行限位（如用户指定J7±52°），与原模型名义范围取交集。
                # 收紧IK范围不改变关节角、轴向或FK几何，也不改写用户提供的原URDF。
                lo, hi = (
                    max(spec["bounds"][0], lower[start + i]),
                    min(spec["bounds"][1], upper[start + i]),
                )
                if not np.isfinite([lo, hi]).all() or lo >= hi:
                    raise ValueError("URDF/SDK joint limit mismatch: " + name)
                self.joints[name] = dict(bounds=(lo, hi), velocity=spec["velocity"])
        self.verify_arm_geometry()

    def verify_arm_geometry(self):
        rng = np.random.default_rng(30)
        for side in ("left", "right"):
            base = f"openarm_{side}_base_link"
            for _ in range(12):
                values = {
                    f"openarm_{side}_joint{i}": float(
                        rng.uniform(*self.joints[f"openarm_{side}_joint{i}"]["bounds"])
                    )
                    for i in range(1, 8)
                }
                urdf_t = self.urdf.transform(f"openarm_{side}_link7", values, base=base)
                sdk_t = np.linalg.inv(self.bases[side]) @ self.sdk.transform(
                    side + "_ee_control_point", values
                )
                if not np.allclose(urdf_t, sdk_t, atol=1e-7, rtol=0):
                    raise ValueError("URDF and SDK arm geometry differ: " + side)

    def transform(self, frame, values, base="sdk_lift_zero"):
        if base != self.base:
            raise ValueError("Unregistered base frame: " + base)
        h = float(values["lift_joint"])
        if (
            not np.isfinite(h)
            or not self.joints["lift_joint"]["bounds"][0]
            <= h
            <= self.joints["lift_joint"]["bounds"][1]
        ):
            raise ValueError("Lift encoder position outside limits")
        for side in ("left", "right"):
            if frame == f"openarm_{side}_ee_base_link":
                t = np.eye(4)
                t[2, 3] = h
                return (
                    t
                    @ self.bases[side]
                    @ self.urdf.transform(
                        f"openarm_{side}_link7", values, base=f"openarm_{side}_base_link"
                    )
                )
        raise ValueError("Unknown live IK endpoint: " + frame)

    def ik(self, *, position_tolerance_m=0.002):
        if not np.isfinite(position_tolerance_m) or not 0 < position_tolerance_m <= 0.15:
            raise ValueError("Position tolerance must be in (0, 150] mm")
        return PairIK(
            self,
            {"left": np.eye(4), "right": np.eye(4)},
            base=self.base,
            position_tolerance=position_tolerance_m,
            angle_tolerance=0.02,
        )

    @staticmethod
    def values(q16, lift_m):
        values = SDKKinematics.from_sdk_vector(q16)
        values["lift_joint"] = float(lift_m)
        return values

    @staticmethod
    def sdk_target(values, holding_q16):
        target = list(map(float, holding_q16))
        for side, start in [("right", 0), ("left", 8)]:
            for i in range(7):
                target[start + i] = float(values[f"openarm_{side}_joint{i + 1}"])
        return target  # Gripper targets retain the existing nominal hold.
