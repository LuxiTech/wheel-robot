#!/usr/bin/env python3
"""Read-only stationary capture and nominal full-gripper FK in SDK coordinates."""

import argparse
import datetime
import hashlib
import json
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure
from _robot_model import selected_model

configure(ROOT)
from wheel_motion_core.gripper import finger_displacement
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics
from wheel_perception_core.extrinsics import CalibratedHeadTransform, HeadExtrinsics
from wheel_perception_core.geometry import prepare_input
from wheel_perception_core.rgbd import save_observation
from wheel_sdk_bridge.calibrated_capture import ReadOnlyHeadCapture
from wheel_sdk_bridge.joint_client import JointClient


class LoggedHeadCapture(ReadOnlyHeadCapture):
    """Retain the unmodified state used by each capture gate, including failures."""

    def __init__(self, *args, diagnostic_path, **kwargs):
        super().__init__(*args, **kwargs)
        self.diagnostic_path = diagnostic_path

    def state(self):
        state, duration = super().state()
        with self.diagnostic_path.open("a") as trace:
            trace.write(json.dumps(dict(request_seconds=duration, state=state)) + "\n")
        return state, duration


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument(
        "--capture-url", help="Optional SDK camera/state endpoint for this read-only capture"
    )
    p.add_argument(
        "--calibration",
        type=Path,
        default=ROOT / "config/robots/calibration/head_camera_j4_rezero_20261104.json",
    )
    p.add_argument(
        "--image-max-age-ms",
        type=float,
        default=250,
        help="Explicit image age allowance for this read-only stationary diagnostic; no motion gate changes",
    )
    p.add_argument(
        "--contacts",
        type=Path,
        default=selected_model(ROOT)["contacts"],
        help="CAD contact candidates matching the selected model",
    )
    a = p.parse_args()
    out = a.output.resolve()
    out.mkdir(parents=True, exist_ok=False)

    def dump(name, value):
        (out / name).write_text(
            json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        )

    report = dict(
        created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        executable=False,
        commands_sent=False,
        status="capturing",
        frame="arm_origin",
    )
    samples = []
    errors = []
    stop = threading.Event()
    cfg = json.loads((ROOT / "config/robots/joint_control.json").read_text())
    cfg["feedback_diagnostics"] = str(out / "joint_feedback.jsonl")
    client = JointClient(cfg["base_url"], cfg)

    def poll():
        while not stop.is_set():
            try:
                samples.append(client.snapshot())
            except Exception as exc:
                errors.append(str(exc))
            stop.wait(0.025)

    worker = threading.Thread(target=poll, daemon=True)
    try:
        worker.start()
        calibration = HeadExtrinsics(a.calibration)
        obs, ctx = LoggedHeadCapture(
            a.capture_url or cfg["base_url"],
            calibration,
            stationary_diagnostic=True,
            max_age_ms=a.image_max_age_ms,
            diagnostic_path=out / "capture_state.jsonl",
        ).capture_sync()
        time.sleep(0.15)
        stop.set()
        worker.join(timeout=2)
        if worker.is_alive():
            raise ValueError("Joint feedback polling did not finish")
        save_observation(out / "observation.npz", obs)
        dump("lift_context.json", ctx)
        inp = prepare_input(obs)
        cv2.imwrite(str(out / "rgb.png"), cv2.cvtColor(inp.rgb, cv2.COLOR_RGB2BGR))
        stamp = int(obs.metadata["receive_mono_ns"])
        nearby = [
            s for s in samples if abs(int(s["arms"]["receive_mono_ns"]) - stamp) <= 200_000_000
        ]
        before = [s for s in nearby if int(s["arms"]["receive_mono_ns"]) <= stamp]
        after = [s for s in nearby if int(s["arms"]["receive_mono_ns"]) >= stamp]
        if not before or not after:
            raise ValueError("No fresh joint bracket around image receive time")
        if len({s["arm_startup"]["control_generation"] for s in nearby}) != 1:
            raise ValueError("Control generation changed during image bracket")
        span = np.ptp(np.array([s["arms"]["position"] for s in nearby]), axis=0)
        if max(span) > 0.006 or max(abs(v) for s in nearby for v in s["arms"]["velocity"]) > 0.08:
            raise ValueError("Arms not stationary around image")
        state = min(nearby, key=lambda s: abs(int(s["arms"]["receive_mono_ns"]) - stamp))
        dump("paired_joint_state.json", state)
        camera, reasons = CalibratedHeadTransform(calibration).resolve(obs, ctx)
        resources = selected_model(ROOT)
        manifest = json.loads(resources["source"].read_text())
        urdf = ROOT / manifest["urdf"]
        digest = hashlib.sha256(urdf.read_bytes()).hexdigest()
        contacts = json.loads(a.contacts.read_text())
        if digest != manifest["urdf_sha256"] or digest != contacts["urdf_sha256"]:
            raise ValueError("URDF/contact provenance mismatch")
        xml = ET.parse(urdf).getroot()
        for data in contacts["sides"].values():
            for surface in data["contacts"]:
                mesh = xml.find(f"link[@name='{surface['link']}']/visual/geometry/mesh")
                if (
                    mesh is None
                    or hashlib.sha256((urdf.parent / mesh.get("filename")).read_bytes()).hexdigest()
                    != surface["mesh_sha256"]
                ):
                    raise ValueError("Contact mesh provenance mismatch")
        model = RobotModel(urdf)
        sdk = SDKKinematics(resources["sdk_chains"])
        values = sdk.from_sdk_vector(state["arms"]["position"])
        angle_limit_exceedances = {
            name: dict(measured_rad=value, limits_rad=list(model.joints[name]["bounds"]))
            for name, value in values.items()
            if model.joints[name]["bounds"]
            and not model.joints[name]["bounds"][0] <= value <= model.joints[name]["bounds"][1]
        }
        # Read-only geometry must evaluate actual encoder feedback, including
        # negative half-count feedback after setting a lower-bound joint zero.
        # Keep motion/planning guards intact and never clip the measured angles.
        for joint in model.joints.values():
            joint["bounds"] = None

        def describe(t):
            c = np.linalg.inv(camera) @ t
            return dict(
                T_origin=t.tolist(),
                position_origin_mm=(t[:3, 3] * 1000).tolist(),
                quaternion_xyzw=Rotation.from_matrix(t[:3, :3]).as_quat().tolist(),
                T_camera=c.tolist(),
            )

        arms = {}
        for side, index in [("left", 15), ("right", 7)]:
            base_frame = f"openarm_{side}_ee_base_link"
            wrist = sdk.transform(side + "_ee_control_point", values)
            body = sdk.gripper_transform(model, side, base_frame, values)
            data = contacts["sides"][side]
            ends = []
            mapping_config = json.loads(resources["gripper"].read_text())["sides"][side]
            mapping = finger_displacement(
                state["arms"]["position"][index],
                offset_rad=mapping_config["offset_rad"],
                opening_sign=mapping_config["opening_sign"],
                closed_feedback_rad=mapping_config.get("closed_feedback_rad"),
                closed_displacement_m=mapping_config.get("closed_displacement_m"),
                bounds=data["finger_displacement_limits_m"],
            )
            estimated = None
            if mapping["within_urdf_limits"]:
                q = dict(values)
                q[f"openarm_{side}_finger_joint1"] = mapping["estimated_finger_displacement_m"]
                estimated = {}
                for surface in data["contacts"]:
                    t = sdk.gripper_transform(model, side, surface["link"], q)
                    point = (t @ np.r_[surface["point_in_finger_m"], 1])[:3]
                    estimated[surface["link"]] = dict(
                        pose=describe(t),
                        cad_hook_contact_reference_origin_mm=(point * 1000).tolist(),
                    )
            for opening in data["finger_displacement_limits_m"]:
                q = dict(values)
                q[f"openarm_{side}_finger_joint1"] = opening
                points = []
                finger_poses = {}
                for surface in data["contacts"]:
                    t = sdk.gripper_transform(model, side, surface["link"], q)
                    points.append((t @ np.r_[surface["point_in_finger_m"], 1])[:3].tolist())
                    finger_poses[surface["link"]] = describe(t)
                ends.append(
                    dict(
                        assumed_finger_displacement_m=opening,
                        contact_points_origin_m=points,
                        finger_poses=finger_poses,
                    )
                )
            midpoint = (wrist @ np.r_[data["contact_midpoint_in_wrist_m"], 1])[:3]
            arms[side] = dict(
                wrist=describe(wrist),
                gripper_body=describe(body),
                measured_arm_angles_rad=[values[f"openarm_{side}_joint{i}"] for i in range(1, 8)],
                measured_gripper_motor_rad=state["arms"]["position"][index],
                actual_finger_displacement_m=None,
                official_approximation=mapping,
                estimated_fingers=estimated,
                opening_range_candidates=ends,
                cad_contact_midpoint_origin_mm=(midpoint * 1000).tolist(),
                contact_definition=contacts["contact_definition"],
            )
        report.update(
            status="nominal_gripper_fk_computed",
            pair_id=obs.metadata["pair_id"],
            height_um=ctx["height_um"],
            urdf_sha256=digest,
            model_resources={name: str(path.relative_to(ROOT)) for name, path in resources.items()},
            arms=arms,
            arm_origin_T_camera=camera.tolist(),
            calibration_path=str(calibration.path),
            calibration_selection=calibration.config["selection"],
            image_max_age_ms=a.image_max_age_ms,
            capture_url=a.capture_url or cfg["base_url"],
            source_sha256={
                str(path.relative_to(ROOT) if path.is_relative_to(ROOT) else path): hashlib.sha256(
                    path.read_bytes()
                ).hexdigest()
                for path in [
                    calibration.path,
                    resources["sdk_chains"],
                    resources["gripper"],
                ]
            },
            pairing=dict(
                sample_count=len(nearby),
                joint_span_rad=span.tolist(),
                nearest_joint_image_delta_ms=(int(state["arms"]["receive_mono_ns"]) - stamp) / 1e6,
                hardware_exposure_synchronized=False,
            ),
            enabled=state["arms"]["enabled"],
            joint_control_phase=state["arm_startup"]["joint_control"]["phase"],
            geometry_only=True,
            measured_arm_angles_modified=False,
            urdf_angle_limit_exceedances=angle_limit_exceedances,
            limitations=reasons
            + [
                "Finger endpoints are CAD candidates, not measured opening or accepted hook TCP.",
                "Contact midpoint is a geometric reference, not the hook insertion tip.",
            ],
        )
    except Exception as exc:
        report.update(status="blocked", error=str(exc))
        raise
    finally:
        stop.set()
        if worker.is_alive():
            worker.join(timeout=2)
        dump("joint_samples.json", dict(samples=samples, errors=errors))
        dump("pose_report.json", report)
        print(
            json.dumps(
                dict(output=str(out), status=report["status"], error=report.get("error")),
                ensure_ascii=False,
            )
        )


if __name__ == "__main__":
    main()
