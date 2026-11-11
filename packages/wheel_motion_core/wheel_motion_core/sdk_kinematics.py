"""FK from a frozen extract of the calibration SDK model, not the CAD URDF."""

import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


class SDKKinematics:
    def __init__(self, path):
        self.config = json.loads(Path(path).read_text())
        if (
            self.config.get("schema") != "wheel_sdk_chains_v1"
            or self.config.get("root_frame") != "arm_origin"
        ):
            raise ValueError("Unsupported SDK chain snapshot")
        self.chains = self.config["chains"]

    def transform(self, frame, joints=None):
        if frame == "arm_origin":
            return np.eye(4)
        for side, chain in self.chains.items():
            if frame not in [n["name"] for n in chain]:
                continue
            t = np.eye(4)
            for n in chain:
                shift = np.eye(4)
                shift[:3, 3] = n["translation_m"]
                t = t @ shift
                if "joint" in n:
                    j = n["joint"]
                    if joints is None or j["name"] not in joints:
                        raise ValueError("Missing SDK joint: " + j["name"])
                    angle = float(joints[j["name"]])
                    if not np.isfinite(angle):
                        raise ValueError("Nonfinite SDK joint")
                    # Like luxi::Ik::sync, evaluate measured q without clipping.
                    # This function is not a motion limit checker.
                    motion = np.eye(4)
                    motion[:3, :3] = Rotation.from_rotvec(np.asarray(j["axis"]) * angle).as_matrix()
                    t = t @ motion
                if n["name"] == frame:
                    return t
        raise ValueError("Unknown SDK frame: " + frame)

    def fk(self, joints):
        return {s: self.transform(s + "_ee_control_point", joints) for s in ("left", "right")}

    def gripper_transform(self, model, side, frame, joints):
        """Nominal full gripper in arm_origin, using each SDK shoulder separately.

        No encoder zero/mount correction or motor-to-opening mapping is inferred.
        Finger frames require a measured CAD prismatic opening in joints.
        """
        if side not in ("left", "right"):
            raise ValueError("Unknown arm side")
        wrist = f"openarm_{side}_link7"
        # Require a descendant of this wrist, not another arm or the chassis.
        model.chain(frame, base=wrist)
        base = f"openarm_{side}_base_link"
        measured = model.transform(wrist, joints, base=base)
        expected = np.linalg.inv(self.transform(base)) @ self.transform(
            side + "_ee_control_point", joints
        )
        if not np.allclose(measured, expected, atol=1e-7, rtol=0):
            raise ValueError("URDF wrist and SDK differ; gripper registration unavailable")
        return self.transform(side + "_ee_control_point", joints) @ model.transform(
            frame, joints, base=wrist
        )

    @staticmethod
    def from_sdk_vector(q):
        q = np.asarray(q, float)
        if q.shape != (16,) or not np.isfinite(q).all():
            raise ValueError("Expected finite SDK q[16], right first then left")
        return {
            f"openarm_{side}_joint{i + 1}": float(q[start + i])
            for side, start in [("right", 0), ("left", 8)]
            for i in range(7)
        }
