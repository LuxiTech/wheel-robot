#!/usr/bin/env python3
"""Solve a bounded SDK-frame pair endpoint; execute only with --execute.

Deltas are metres in sdk_lift_zero (+Z up, +Y SDK left). Targets are link7 /
SDK control points. No camera-space grasp or floor-frame pose is implied.
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
from wheel_execution.ik_executor import SDKIKExecutor
from wheel_sdk_bridge.joint_client import JointClient
from wheel_sdk_bridge.lift_motion_client import LiftMotionClient


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--left-delta", type=float, nargs=3, default=[0, 0, 0], metavar=("X", "Y", "Z"))
    p.add_argument("--right-delta", type=float, nargs=3, default=[0, 0, 0], metavar=("X", "Y", "Z"))
    p.add_argument("--lift-delta-bounds", type=float, nargs=2)
    p.add_argument(
        "--execute",
        action="store_true",
        help="Execute the solved bounded endpoint in a clear workspace",
    )
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    jp = json.loads((ROOT / "config/robots/joint_control.json").read_text())
    lp = json.loads((ROOT / "config/robots/lift_control.json").read_text())
    jp["feedback_diagnostics"] = str(a.output.resolve() / "joint_feedback.jsonl")
    e = SDKIKExecutor(
        JointClient(jp["base_url"], jp),
        LiftMotionClient(lp["base_url"], feedback_max_age_s=lp["feedback_max_age_s"]),
        selected_model(ROOT)["urdf"],
        selected_model(ROOT)["sdk_chains"],
        output=a.output,
    )
    plan = e.plan_relative(a.left_delta, a.right_delta, lift_delta_bounds=a.lift_delta_bounds)
    print(json.dumps(plan, ensure_ascii=False, indent=2), flush=True)
    if a.execute:
        print(
            json.dumps(e.execute(plan["plan_id"], scene_clear=True), ensure_ascii=False, indent=2),
            flush=True,
        )


if __name__ == "__main__":
    main()
