"""Selected head calibration in SDK coordinates; never publishes TF or commands motion."""

import json
from pathlib import Path

import numpy as np
from wheel_task_core.transforms import pose


class HeadExtrinsics:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.config = json.loads(self.path.read_text())
        c = self.config
        if (
            c.get("schema") != "wheel_head_extrinsics_v1"
            or c.get("parent_frame") != "arm_origin"
            or c.get("units") != "metres"
        ):
            raise ValueError("Unsupported calibration convention")
        self.reference = pose(c["T_origin_camera"]).copy()
        self.axis = np.asarray(c["lift_axis_in_origin"], float)
        if (
            self.axis.shape != (3,)
            or not np.isfinite(self.axis).all()
            or not np.isclose(np.linalg.norm(self.axis), 1)
        ):
            raise ValueError("Invalid lift axis")
        self.height_range = np.asarray(c["height_range_um"], float)
        if (
            self.height_range.shape != (2,)
            or not np.isfinite(self.height_range).all()
            or not self.height_range[0] <= c["reference_height_um"] <= self.height_range[1]
        ):
            raise ValueError("Invalid reference height/range")

    def camera_to_origin(self, height_um):
        """Relative lift displacement, not an assumed URDF joint zero."""
        h = float(height_um)
        if not np.isfinite(h) or not self.height_range[0] <= h <= self.height_range[1]:
            raise ValueError("Lift outside recorded calibration range")
        t = self.reference.copy()
        t[:3, 3] -= self.axis * ((h - self.config["reference_height_um"]) * 1e-6)
        return t

    def check_camera(self, camera_id, frame_id, serial):
        c = self.config["camera"]
        if camera_id != c["id"] or frame_id not in c["optical_frames"]:
            raise ValueError("Camera/frame is not the calibrated color optical source")
        if str(serial) not in c["serials"]:
            raise ValueError("Camera serial does not match calibration")

    def transform(self, height_um, *, target_frame="arm_origin", sdk_fk=None, joints=None):
        x = self.camera_to_origin(height_um)
        if target_frame == "arm_origin":
            return x
        if target_frame in (
            "left_ee_control_point",
            "right_ee_control_point",
            "openarm_left_base_link",
            "openarm_right_base_link",
        ):
            if sdk_fk is None:
                raise ValueError("SDK kinematics required for requested frame")
            return np.linalg.inv(sdk_fk.transform(target_frame, joints)) @ x
        # An imported CAD camera joint is deliberately not used as calibration.
        raise ValueError("Unregistered target frame: " + target_frame)

    def object_pose(self, camera_pose, height_um, **kwargs):
        return self.transform(height_um, **kwargs) @ pose(camera_pose)

    def point(self, camera_point, height_um, **kwargs):
        p = np.asarray(camera_point, float)
        if p.shape != (3,) or not np.isfinite(p).all():
            raise ValueError("Expected finite camera XYZ in metres")
        t = self.transform(height_um, **kwargs)
        return t[:3, :3] @ p + t[:3, 3]

    def sdk_ee_target(self, origin_T_tcp, side):
        """Desired TCP pose -> SDK EE target; a board mount is NOT a TCP."""
        if side not in ("left", "right"):
            raise ValueError("Unknown arm")
        ee_T_tcp = self.config["ee_T_tcp"][side]
        if ee_T_tcp is None:
            raise ValueError("Measured EE-to-TCP transform missing for " + side)
        return pose(origin_T_tcp) @ np.linalg.inv(pose(ee_T_tcp))


class CalibratedHeadTransform:
    """Pipeline adapter. Context is produced by timestamped capture, not latest height."""

    output_frame = "arm_origin"

    def __init__(self, calibration):
        self.calibration = calibration

    def diagnostic_candidates(self, observation, context, grasp, pregrasp):
        """Convert same-frame camera candidates; never infer physical TCP or arm assignment."""
        transform, reasons = self.resolve(observation, context)
        return dict(
            frame_id=self.output_frame,
            pair_id=observation.metadata["pair_id"],
            height_um=context["height_um"],
            diagnostic_only=True,
            executable=False,
            arm_assignment_validated=False,
            reasons=reasons + ["unvalidated_gripper_contacts"],
            grasp={key: (transform @ pose(value)).tolist() for key, value in grasp.items()},
            pregrasp={key: (transform @ pose(value)).tolist() for key, value in pregrasp.items()},
        )

    def resolve(self, observation, context):
        if not isinstance(context, dict) or context.get("schema") not in (
            "wheel_lift_capture_v1",
            "wheel_lift_stream_capture_v1",
        ):
            raise ValueError("Timestamped lift capture context required")
        m = observation.metadata
        if context.get("pair_id") != m["pair_id"]:
            raise ValueError("Lift context belongs to another image")
        if int(context["image_receive_mono_ns"]) != int(m["receive_mono_ns"]):
            raise ValueError("Image/lift timestamp mismatch")
        if (
            context.get("stationary") is not True
            or context.get("time_basis") != "robot_receive_monotonic"
        ):
            raise ValueError("Require stationary lift bracket in robot clock domain")
        span = float(context["height_span_um"])
        if not np.isfinite(span) or not 0 <= span <= 200 or int(context["sample_count"]) < 2:
            raise ValueError("Unstable or incomplete lift capture")
        a, b = context["bracket_mono_ns"]
        stamp = int(m["receive_mono_ns"])
        streaming = context["schema"] == "wheel_lift_stream_capture_v1"
        if streaming:
            window = context.get("stationary_window_mono_ns")
            if (
                not isinstance(window, list)
                or len(window) != 2
                or not int(window[0]) <= stamp - 300_000_000
                or int(window[1]) != int(b)
                or context.get("freshness_policy") != "strict"
                or len(context.get("sdk_valid", [])) != int(context["sample_count"])
                or not all(v is True for v in context.get("sdk_valid", []))
                or int(context["sample_count"]) < 3
            ):
                raise ValueError("Native-valid fixed-lift history required for streaming")
        margin = 0 if streaming else 100_000_000
        if (
            not int(a) <= stamp - margin <= stamp + margin <= int(b)
            or not 0 <= int(b) - int(a) <= 500_000_000
        ):
            raise ValueError("Missing or excessive lift bracket")
        self.calibration.check_camera(m["camera_id"], m["frame_id"], context["camera_serial"])
        return self.calibration.camera_to_origin(context["height_um"]), [
            "selected_calibration:" + self.calibration.config.get("selection", "unspecified"),
            "stationary_lift_host_receive_alignment",
            "sdk_to_urdf_and_physical_tcp_not_registered",
        ]
