#!/usr/bin/env python3
"""Inspect lift, or explicitly execute one leased absolute encoder target."""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure

configure(ROOT)
from wheel_sdk_bridge.lift_motion_client import LiftMotionClient


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", type=Path, default=ROOT / "config/robots/lift_control.json")
    group = p.add_mutually_exclusive_group()
    group.add_argument(
        "--target-m", type=float, help="Absolute SDK encoder position in metres; executes motion"
    )
    group.add_argument(
        "--hold-current",
        action="store_true",
        help="Submit current encoder position and verify arrival",
    )
    p.add_argument("--timeout", type=float, default=180.0)
    p.add_argument("--output", type=Path, help="New JSON result file")
    a = p.parse_args()
    if a.output and a.output.exists():
        p.error("Output already exists")
    cfg = json.loads(a.config.read_text())
    c = LiftMotionClient(cfg["base_url"], feedback_max_age_s=cfg["feedback_max_age_s"])
    result = {"commands_sent": False}
    try:
        result["preflight"] = c.preflight()
        if a.target_m is not None or a.hold_current:
            with c:
                target = c.snapshot()["lift"]["current_um"] / 1e6 if a.hold_current else a.target_m
                result["commands_sent"] = True
                result["accepted"] = c.move(target)
                result["arrival"] = c.wait(result["accepted"]["task_id"], a.timeout)
            result["stopped"] = True
    except Exception as exc:
        result.update(error=str(exc), session_error=c.error, stop_error=c.stop_error)
        raise
    finally:
        result["commands_sent"] = c.sequence > 0
        text = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
        if a.output:
            a.output.parent.mkdir(parents=True, exist_ok=True)
            with a.output.open("x") as f:
                f.write(text)
        print(text, end="")


if __name__ == "__main__":
    main()
