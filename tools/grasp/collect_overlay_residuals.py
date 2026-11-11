#!/usr/bin/env python3
"""Collect small, bounded arm-pose perturbations; never commands or disables the lift.

The default writes a reviewable plan only. Execute uses the existing SDK joint
client, requires an already enabled stationary hold, and records measured angles
with every camera sample. It never applies any fitted correction.
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure

configure(ROOT)
from grasp.zero_arms_check_j2 import stop_owned_request
from wheel_sdk_bridge.joint_client import JointClient


def save(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")


def wait_stationary(client, timeout=12):
    started = time.monotonic()
    settled = None
    while time.monotonic() - started < timeout:
        state = client.snapshot()
        if np.max(np.abs(state["arms"]["velocity"])) < 0.08:
            settled = settled or time.monotonic()
            if time.monotonic() - settled > 0.4:
                return
        else:
            settled = None
        time.sleep(0.05)
    raise RuntimeError("SDK stationary gate has not settled; no move submitted")


def submit_when_stationary(client, target, speed):
    for attempt in range(3):
        wait_stationary(client)
        try:
            return client.move(target, speed)
        except Exception as exc:
            # A pending ID is assigned before any POST; never retry a possibly
            # submitted write. Only this known local pre-submit gate is retried.
            if client.pending_id is not None or str(exc) != "Arms are not stationary":
                raise
            if attempt == 2:
                raise
            print("Local stationary gate rejected before POST; waiting", flush=True)


def capture_pose(output, name, url):
    for attempt in range(5):
        directory = "capture" if attempt == 0 else f"capture-retry{attempt}"
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "tools/grasp/current_gripper_pose.py"),
                "--output",
                str(output / name / directory),
                "--capture-url",
                url,
                "--image-max-age-ms",
                "500",
            ],
            capture_output=True,
            text=True,
        )
        save(
            output / (name + f"_capture_result{attempt}.json"),
            dict(returncode=result.returncode, stdout=result.stdout, stderr=result.stderr),
        )
        print("CAPTURE", name, result.returncode, result.stdout.strip(), flush=True)
        if result.returncode == 0:
            return
        report_path = output / name / directory / "pose_report.json"
        report = json.loads(report_path.read_text()) if report_path.exists() else {}
        if not str(report.get("error", "")).startswith(
            ("Arms not stationary around image", "Capture timed out")
        ):
            raise RuntimeError("Nontransient capture failure: " + str(report))
    raise RuntimeError("Capture rejected; stop collection at the settled hold")


def arrive(client, request_id, target, trace, timeout=25):
    started = time.monotonic()
    settled = None
    while time.monotonic() - started < timeout:
        state = client.snapshot()
        trace.write(json.dumps(state) + "\n")
        trace.flush()
        control = state["arm_startup"]["joint_control"]
        if control["request_id"] != request_id:
            if time.monotonic() - started > 1:
                raise RuntimeError("Request ownership changed")
            time.sleep(0.1)
            continue
        if control["phase"] in ("fault", "stopped"):
            raise RuntimeError("SDK motion failed: " + str(control))
        if not np.allclose(control["target"], target, atol=1e-9, rtol=0):
            raise RuntimeError("SDK target changed")
        error = np.max(np.abs(np.array(state["arms"]["position"]) - target))
        stable = (
            control["phase"] == "ready"
            and error < 0.012
            and np.max(np.abs(state["arms"]["velocity"])) < 0.08
        )
        if stable:
            settled = settled or time.monotonic()
            if time.monotonic() - settled > 1:
                client.pending_id = None
                return state
        else:
            settled = None
        time.sleep(0.1)
    raise TimeoutError("Bounded sample did not settle")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Continue the same reviewed plan after a confirmed owned stop",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=args.execute)
    config = json.loads((ROOT / "config/robots/joint_control.json").read_text())
    config["feedback_diagnostics"] = str((args.output / "joint_feedback.jsonl").resolve())
    client = JointClient(config["base_url"], config)
    state = client.snapshot()
    if args.resume:
        stopped = json.loads((args.output / "after_stop_state.json").read_text())
        if (
            state["arm_startup"]["joint_control"]["request_id"]
            != stopped["arm_startup"]["joint_control"]["request_id"]
        ):
            raise RuntimeError("Control owner changed after the recorded stop")
        if (
            state["arm_startup"]["joint_control"]["phase"] not in ("stopped", "ready")
            or state["arm_startup"]["manual_pose"].get("fault_latched") is not False
        ):
            raise RuntimeError(
                "Resume requires a confirmed stopped or arrived hold without a latched fault"
            )
    if state["arms"]["enabled"] is not True:
        raise RuntimeError("An already enabled hold is required")
    client._check_compensation_handover(state, required=True)
    current_hold = np.array(state["arm_startup"]["nominal_target"])
    anchor = (
        np.array(json.loads((args.output / "plan.json").read_text())["anchor"])
        if args.resume
        else current_hold
    )
    if np.max(np.abs(current_hold - state["arms"]["position"])) > 0.012:
        raise RuntimeError("Existing hold has not settled")
    poses = [dict(name="anchor", positions=anchor.tolist())]
    # 0.06 rad (3.44 degrees) per joint, one corresponding joint on each arm.
    # Keep all other targets and both opening targets unchanged.
    for joint in range(7):
        for sign in (1, -1):
            target = anchor.copy()
            target[[joint, joint + 8]] += sign * 0.06
            poses.append(
                dict(
                    name=f"J{joint + 1}_{'plus' if sign > 0 else 'minus'}",
                    positions=target.tolist(),
                )
            )
    plan = dict(
        anchor=anchor.tolist(),
        poses=poses,
        speed_rad_s=0.03,
        lift_commands_sent=False,
        encoder_zero_changed=False,
        fitting_corrections_applied=False,
        scope="Small perturbations around the observed clear raised pose; not a workspace collision certificate",
    )
    for pose in poses:
        client.prepare(pose["positions"], plan["speed_rad_s"])
    plan_path = args.output / "plan.json"
    if args.execute:
        previous = json.loads(plan_path.read_text())
        if np.max(np.abs(np.array(previous["anchor"]) - anchor)) > 0.005:
            raise RuntimeError("Hold changed since plan review")
        plan = previous
        poses = plan["poses"]
    else:
        save(plan_path, plan)
        print(json.dumps(plan, ensure_ascii=False), flush=True)
        return
    try:
        with (args.output / "motion.jsonl").open("a" if args.resume else "x") as trace:
            for pose in poses:
                completed = [
                    p
                    for p in (args.output / pose["name"]).glob("capture*/pose_report.json")
                    if json.loads(p.read_text()).get("status") == "nominal_gripper_fk_computed"
                ]
                if args.resume and completed:
                    print("KEEP existing paired capture", pose["name"], flush=True)
                    continue
                print("MOVE", pose["name"], flush=True)
                target = np.array(pose["positions"])
                holding = client.snapshot()
                already_holding = (
                    np.max(np.abs(np.array(holding["arm_startup"]["nominal_target"]) - target))
                    < 1e-9
                    and np.max(np.abs(np.array(holding["arms"]["position"]) - target)) < 0.012
                )
                if already_holding:
                    save(
                        args.output / (pose["name"] + "_already_holding.json"),
                        dict(commands_sent=False, reason="Already holding anchor"),
                    )
                else:
                    response = submit_when_stationary(client, target.tolist(), plan["speed_rad_s"])
                    request_path = args.output / (pose["name"] + "_request.json")
                    if request_path.exists():
                        request_path = args.output / (
                            pose["name"] + f"_request_retry_{time.time_ns()}.json"
                        )
                    save(request_path, response)
                    arrive(client, client.pending_id, target, trace)
                capture_pose(args.output, pose["name"], config["base_url"])
            print("RETURN anchor", flush=True)
            response = submit_when_stationary(client, anchor.tolist(), plan["speed_rad_s"])
            save(args.output / "return_request.json", response)
            final = arrive(client, client.pending_id, anchor, trace)
            save(args.output / "final_joint_state.json", final)
        save(
            args.output / "completed.json",
            dict(captures=len(poses), lift_commands_sent=False, returned_to_anchor=True),
        )
    except Exception as exc:
        stop_owned_request(client, args.output)
        save(args.output / "blocked.json", dict(error=str(exc), lift_commands_sent=False))
        raise


if __name__ == "__main__":
    main()
