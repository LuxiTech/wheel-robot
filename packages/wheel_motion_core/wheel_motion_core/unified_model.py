"""Feedback adapter for a whole-robot MJCF; no device commands or inferred zeros."""

import numpy as np
from wheel_task_core.transforms import pose


class UnifiedModel:
    def __init__(self, path, *, lift_direction, lift_offset_m):
        import mujoco

        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        if lift_direction not in (-1, 1) or not np.isfinite(lift_offset_m):
            raise ValueError("Explicit lift mapping required")
        self.direction = lift_direction
        self.offset = lift_offset_m

    def update(self, q16, height_um, *, finger_displacements):
        from wheel_motion_core.sdk_kinematics import SDKKinematics

        values = SDKKinematics.from_sdk_vector(q16)
        values["lift_joint"] = self.direction * float(height_um) * 1e-6 + self.offset
        for side in ["left", "right"]:
            g = float(finger_displacements[side])
            values[f"openarm_{side}_finger_joint1"] = g
            values[f"openarm_{side}_finger_joint2"] = g
        if not np.isfinite(list(values.values())).all():
            raise ValueError("Nonfinite feedback")
        assignments = []
        for name, v in values.items():
            j = self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_JOINT, name)
            if j < 0:
                raise ValueError("Missing joint " + name)
            if (
                self.model.jnt_limited[j]
                and not self.model.jnt_range[j, 0] - 1e-7 <= v <= self.model.jnt_range[j, 1] + 1e-7
            ):
                raise ValueError("Feedback outside unified model limits: " + name)
            assignments.append((self.model.jnt_qposadr[j], v))
        for address, v in assignments:
            self.data.qpos[address] = v
        self.mj.mj_forward(self.model, self.data)
        return values

    def site(self, name):
        i = self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_SITE, name)
        if i < 0:
            raise ValueError("Missing site " + name)
        t = np.eye(4)
        t[:3, :3] = self.data.site_xmat[i].reshape(3, 3)
        t[:3, 3] = self.data.site_xpos[i]
        return t


def registered_camera(urdf, sdk, calibration, height_um, lift_joint_m):
    """Transfer selected left-arm calibration into whole-URDF base_footprint.

    Left SDK base and supplied URDF left base correspondence is an explicit
    registration assumption, not a newly measured camera calibration.
    """
    base = urdf.transform("openarm_left_base_link", {"lift_joint": lift_joint_m})
    return pose(
        base
        @ np.linalg.inv(sdk.transform("openarm_left_base_link"))
        @ calibration.camera_to_origin(height_um)
    )
