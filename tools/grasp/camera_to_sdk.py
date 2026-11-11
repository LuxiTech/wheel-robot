#!/usr/bin/env python3
"""Read-only coordinate conversion: camera -> compensated SDK arm_origin/bases/EE.

Offline height is explicit and never presented as live feedback. --live captures
RGB-D and a stationary lift bracket using GET only. No TF or motor publishers.
"""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure
from _robot_model import selected_model

configure(ROOT)
from wheel_motion_core.sdk_kinematics import SDKKinematics
from wheel_perception_core.extrinsics import CalibratedHeadTransform, HeadExtrinsics


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--calibration",
        default=str(ROOT / "config/robots/calibration/head_camera_j4_rezero_20261104.json"),
    )
    p.add_argument("--sdk-chains", default=str(selected_model(ROOT)["sdk_chains"]))
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--height-um", type=float)
    mode.add_argument("--live", action="store_true")
    p.add_argument("--url", default="http://192.168.0.245:18030")
    p.add_argument("--frame", help="Defaults to the selected calibration's child frame")
    p.add_argument("--camera-serial", help="Required offline; live reads verified driver identity")
    data = p.add_mutually_exclusive_group()
    data.add_argument(
        "--point", nargs=3, type=float, help="XYZ in metres in the selected color optical frame"
    )
    data.add_argument("--pose-json", help="JSON 4x4 camera_T_object")
    p.add_argument("--joints", help="Offline SDK q[16] JSON; enables EE-relative output")
    p.add_argument("--output", help="Write result JSON; existing files are not overwritten")
    args = p.parse_args(argv)
    c = HeadExtrinsics(args.calibration)
    k = SDKKinematics(args.sdk_chains)
    try:
        context = None
        if args.live:
            if args.joints or args.point or args.pose_json:
                raise ValueError(
                    "Live mode outputs camera transforms only; use a timestamped perception result for object/EE poses"
                )
            from wheel_sdk_bridge.calibrated_capture import ReadOnlyHeadCapture

            obs, context = ReadOnlyHeadCapture(
                args.url, c, stationary_diagnostic=True
            ).capture_sync()
            CalibratedHeadTransform(c).resolve(obs, context)
            height = context["height_um"]
        else:
            c.check_camera("head_rgb", args.frame or c.config["child_frame"], args.camera_serial)
            height = args.height_um
        joints = (
            None
            if not args.joints
            else k.from_sdk_vector(json.loads(Path(args.joints).read_text()))
        )
        frames = ["arm_origin", "openarm_left_base_link", "openarm_right_base_link"]
        if joints is not None:
            frames += ["left_ee_control_point", "right_ee_control_point"]
        camera_pose = None if not args.pose_json else json.loads(Path(args.pose_json).read_text())
        output = {}
        for frame in frames:
            kw = dict(target_frame=frame, sdk_fk=k, joints=joints)
            item = {"T_target_camera": c.transform(height, **kw).tolist()}
            if args.point is not None:
                item["point_m"] = c.point(args.point, height, **kw).tolist()
            if camera_pose is not None:
                item["T_target_object"] = c.object_pose(camera_pose, height, **kw).tolist()
            output[frame] = item
        result = dict(
            status="converted_diagnostic",
            executable=False,
            units="metres",
            input_source="live_stationary_capture" if args.live else "explicit_offline_input",
            height_um=height,
            reference_height_um=c.config["reference_height_um"],
            calibration_path=str(c.path),
            calibration_selection=c.config["selection"],
            frames=output,
            lift_context=context,
            unavailable=[
                "base_link/base_footprint: SDK registration unresolved",
                "physical grasp TCP: measured EE-to-TCP missing",
            ],
        )
        code = 0
    except (ValueError, KeyError, OSError) as exc:
        result = dict(status="blocked", reason=str(exc), executable=False)
        code = 1
    text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if args.output:
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("x") as f:
            f.write(text)
    print(text, end="")
    return code


if __name__ == "__main__":
    sys.exit(main())
