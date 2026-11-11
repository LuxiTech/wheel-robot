#!/usr/bin/env python3
"""Send the SDK one all-zero arm target and keep the lift untouched.

Default is GET-only preview. --execute explicitly authorizes SDK joint_move,
including its measured-position enable when outputs are disabled. This never
sets encoder zeros, starts homing, changes calibration, or commands the lift.
"""

import argparse
import datetime
import fcntl
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure

configure(ROOT)
from wheel_sdk_bridge.joint_client import JointClient

TARGET = [0.0] * 16


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def wait_for_zero(
    client,
    request_id,
    *,
    output,
    timeout=120.0,
    tolerance=0.01,
    clock=time.monotonic,
    sleep=time.sleep,
):
    started = clock()
    settled_since = None
    observed_request = False
    next_progress = started
    with (output / "motion.jsonl").open("x") as trace:
        while clock() - started < timeout:
            state = client.snapshot()
            joint = state["arm_startup"]["joint_control"]
            trace.write(json.dumps({"joint_state": state}) + "\n")
            trace.flush()
            if joint.get("request_id") != request_id:
                if observed_request or clock() - started > 1.0:
                    raise RuntimeError(
                        "SDK request superseded or not observed; do not overwrite it"
                    )
                sleep(0.1)
                continue
            observed_request = True
            if joint["phase"] in ("fault", "stopped"):
                raise RuntimeError(f"SDK {joint['phase']}: {joint.get('reason', '')}")
            if joint.get("target") != TARGET:
                raise RuntimeError("SDK target changed; expected exactly sixteen zeros")
            positions = state["arms"]["position"]
            error = max(abs(value) for value in positions)
            settled = (
                joint["phase"] == "ready"
                and state["arms"]["enabled"] is True
                and error <= tolerance
                and max(abs(value) for value in state["arms"]["velocity"]) < 0.08
            )
            now = clock()
            if settled:
                if settled_since is None:
                    settled_since = now
                if now - settled_since >= 1.0:
                    client.pending_id = None
                    save(output / "arrived_joint_state.json", state)
                    return state
            else:
                settled_since = None
            if now >= next_progress:
                print(
                    f"{joint['phase']}: max |q|={math.degrees(error):.3f} deg",
                    flush=True,
                )
                next_progress = now + 2.0
            sleep(0.1)
    raise TimeoutError("All-zero target did not reach the measured settling tolerance")


def stop_owned_request(client, output):
    """Reconcile an uncertain write without retrying or stopping another owner."""
    if not client.pending_id:
        return
    try:
        state = client.snapshot()
        joint = state["arm_startup"]["joint_control"]
        if joint.get("request_id") == client.pending_id:
            save(output / "stop_result.json", client.stop(client.pending_id))
        else:
            save(output / "stop_result.json", {"stop_sent": False, "reason": "unowned_request"})
    except Exception as error:
        save(output / "stop_error.json", {"error": str(error), "stop_confirmed": False})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Send zero target; SDK may enable from measured positions",
    )
    parser.add_argument(
        "--speed", type=float, default=0.04, help="SDK maximum joint speed in rad/s"
    )
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--config", type=Path, default=ROOT / "config/robots/joint_control.json")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout < 5:
        parser.error("--timeout must be finite and at least 5 seconds")
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = (args.output or ROOT / "outputs/grasp/arm-zero-j2" / stamp).resolve()
    output.mkdir(parents=True, exist_ok=False)
    lock = ROOT / ".runtime/arm-zero-j2.lock"
    lock.parent.mkdir(exist_ok=True)
    lock_file = lock.open("a")
    fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    config = json.loads(args.config.read_text())
    config["feedback_diagnostics"] = str(output / "joint_feedback.jsonl")
    client = JointClient(config["base_url"], config)
    report = {
        "status": "preflight",
        "created_utc": stamp,
        "commands_sent": False,
        "lift_commands_sent": False,
        "encoder_zero_changed": False,
        "calibration_or_offset_changed": False,
        "target_rad": TARGET,
        "settling_tolerance_rad": 0.01,
    }
    try:
        initial = client.snapshot()
        save(output / "initial_joint_state.json", initial)
        if initial["arm_startup"].get("manual_pose", {}).get("fault_latched") is True:
            raise RuntimeError("SDK fault is latched; do not clear it automatically")
        if max(abs(value) for value in initial["arms"]["velocity"]) >= 0.08:
            raise RuntimeError("Arms must be stationary before the zero target")
        prepared = client.prepare(TARGET, args.speed)
        save(output / "unsent_preview.json", prepared)
        print(f"Logs: {output}", flush=True)
        print(
            f"Target: 16 zeros; speed={args.speed} rad/s",
            flush=True,
        )
        if not args.execute:
            report["status"] = "preview_only_no_commands"
            return 0
        # This is explicit execution of exactly the requested target, using the
        # deployed SDK's stable-feedback enable/reanchor; never startup/home.
        report["command_submission_attempted"] = True
        report["status"] = "submitting_zero_target"
        save(output / "report.json", report)
        accepted = client.move(TARGET, args.speed, enable_if_needed=True)
        report["commands_sent"] = True
        save(output / "accepted.json", accepted)
        state = wait_for_zero(
            client,
            accepted["request_id"],
            output=output,
            timeout=args.timeout,
        )
        report.update(
            status="zero_target_reached",
            request_id=accepted["request_id"],
            actual_positions_rad=state["arms"]["position"],
            max_abs_angle_deg=math.degrees(max(abs(value) for value in state["arms"]["position"])),
        )
        print(
            "Zero target reached.",
            flush=True,
        )
        return 0
    except (Exception, KeyboardInterrupt) as error:
        report.update(status="aborted", error=str(error) or type(error).__name__)
        if client.pending_id and report["commands_sent"] is False:
            report.update(
                commands_sent=None,
                submission_uncertain=True,
                pending_request_id=client.pending_id,
            )
        stop_owned_request(client, output)
        print(f"Stopped: {report['error']}; see {output}", file=sys.stderr, flush=True)
        return 1
    finally:
        save(output / "report.json", report)
        lock_file.close()


if __name__ == "__main__":
    raise SystemExit(main())
