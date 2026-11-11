#!/usr/bin/env python3
"""Session-authorized, mechanically supported LEFT wrist test. Global policy unchanged.
Only +2 degrees and a verified return, one of J6/J7; no enable/disable/retry.
"""

import argparse
import datetime
import fcntl
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure

configure(ROOT)
from wheel_perception_core.extrinsics import HeadExtrinsics
from wheel_perception_core.geometry import prepare_input
from wheel_perception_core.rgbd import save_observation
from wheel_sdk_bridge.calibrated_capture import ReadOnlyHeadCapture
from wheel_sdk_bridge.joint_client import JointClient


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--joint", type=int, choices=[6, 7], required=True)
    p.add_argument("--supported-test-authorized", action="store_true", required=True)
    a = p.parse_args()
    lock = open("/tmp/wheel-supported-wrist.lock", "w")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    out = (
        ROOT
        / "outputs/grasp/wrist_direction_check"
        / datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    )
    out.mkdir()

    def dump(name, value):
        (out / name).write_text(
            json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        )

    cfg = json.loads((ROOT / "config/robots/joint_control.json").read_text())
    dump("global_policy_unchanged.json", cfg)
    # Narrow session authorization: user explicitly confirmed mechanical support.
    cfg.update(feedback_diagnostics=str(out / "feedback.jsonl"))
    client = JointClient(cfg["base_url"], cfg)
    index = 8 + a.joint - 1
    delta = np.deg2rad(2)
    speed = 0.02
    active = None
    report = {
        "status": "preflight",
        "commands_sent": False,
        "scope": "left J%d +2deg then return only" % a.joint,
        "mechanical_support": "user_confirmed",
        "global_policy_changed": False,
    }

    def state():
        s = client.snapshot()
        b = s["arm_startup"]
        if s["arms"]["enabled"] is not True or b["manual_pose"]["fault_latched"] is not False:
            raise RuntimeError("Hold unavailable/fault")
        if b["output_disable_count"] != disable_count:
            raise RuntimeError("Motor output was disabled")
        return s

    def capture(name):
        before = state()
        obs, ctx = ReadOnlyHeadCapture(
            cfg["base_url"],
            HeadExtrinsics(ROOT / "config/robots/calibration/head_camera_j4_rezero_20261104.json"),
            stationary_diagnostic=True,
            max_age_ms=1000,
        ).capture_sync()
        after = state()
        if max(abs(np.array(after["arms"]["position"]) - before["arms"]["position"])) > 0.006:
            raise RuntimeError("Motion during stationary capture")
        if (
            after["arm_startup"]["control_generation"]
            != before["arm_startup"]["control_generation"]
        ):
            raise RuntimeError("Control owner changed during capture")
        save_observation(out / (name + ".npz"), obs)
        dump(name + "_states.json", {"before": before, "after": after, "lift": ctx})
        cv2.imwrite(
            str(out / (name + ".png")), cv2.cvtColor(prepare_input(obs).rgb, cv2.COLOR_RGB2BGR)
        )
        return after

    def move(target, label):
        nonlocal active
        s = state()
        current = np.array(s["arm_startup"]["nominal_target"])
        changes = np.array(target) - current
        if np.any(np.abs(np.delete(changes, index)) > 1e-8) or abs(changes[index]) > delta + 1e-8:
            raise RuntimeError("Outside single-joint 2deg scope")
        if active is not None and s["arm_startup"]["joint_control"]["request_id"] != active:
            raise RuntimeError("Control superseded")
        client._check_compensation_handover(s, required=True)
        dump(label + "_target.json", {"target": target, "speed_rad_s": speed, "baseline": s})
        report["commands_sent"] = True
        report["status"] = "submitting_" + label
        dump("report.json", report)
        result = client.move(target, speed)
        active = result["request_id"]
        dump(label + "_accepted.json", result)
        start = time.monotonic()
        stable = None
        while time.monotonic() - start < 15:
            s = state()
            b = s["arm_startup"]
            j = b["joint_control"]
            q = np.array(s["arms"]["position"])
            if j["request_id"] != active:
                raise RuntimeError("Control superseded")
            if j["phase"] in ["fault", "stopped"]:
                raise RuntimeError("Driver " + j["phase"] + ": " + j["reason"])
            if np.max(np.abs(np.delete(q - baseline_q, index))) > 0.012:
                raise RuntimeError("Untested joint moved >0.012 rad")
            if abs(q[index] - baseline_q[index]) > delta + 0.012:
                raise RuntimeError("Wrist excursion exceeded bound")
            settled = (
                j["phase"] == "ready"
                and np.max(np.abs(q - target)) < 0.006
                and max(abs(v) for v in s["arms"]["velocity"]) < 0.08
            )
            stable = (stable or time.monotonic()) if settled else None
            if stable and time.monotonic() - stable > 1.0:
                client.pending_id = None
                dump(label + "_settled.json", s)
                return s
            time.sleep(0.05)
        raise RuntimeError("Test settling timed out")

    try:
        initial = client.snapshot()
        disable_count = initial["arm_startup"]["output_disable_count"]
        baseline_q = np.array(initial["arms"]["position"])
        nominal = np.array(initial["arm_startup"]["nominal_target"])
        if initial["arm_startup"]["joint_control"]["phase"] not in ["idle", "ready"]:
            raise RuntimeError("Not idle/ready")
        client._check_compensation_handover(initial, required=True)
        if max(abs(nominal - baseline_q)) > 0.006:
            raise RuntimeError("Baseline not settled")
        baseline = capture("before")
        baseline_q = np.array(baseline["arms"]["position"])
        if (
            baseline["arm_startup"]["control_generation"]
            != initial["arm_startup"]["control_generation"]
        ):
            raise RuntimeError("Owner changed before first move")
        target = nominal.copy()
        target[index] += delta
        move(target.tolist(), "positive")
        positive = capture("positive")
        achieved = np.array(positive["arms"]["position"]) - baseline_q
        if not np.deg2rad(1.5) < achieved[index] < np.deg2rad(2.5):
            raise RuntimeError("Observed encoder delta not 2deg")
        move(nominal.tolist(), "return")
        returned = capture("returned")
        report.update(
            status="motion_and_return_completed_direction_analysis_pending",
            joint_index=index,
            achieved_delta_deg=float(np.rad2deg(achieved[index])),
            other_joint_max_change_deg=float(
                np.rad2deg(np.max(np.abs(np.delete(achieved, index))))
            ),
            return_error_deg=float(
                np.rad2deg(np.max(np.abs(np.array(returned["arms"]["position"]) - baseline_q)))
            ),
            output_disable_count=returned["arm_startup"]["output_disable_count"],
        )
    except Exception as e:
        report.update(status="aborted", error=str(e))
        try:
            if client.pending_id:
                s = client.snapshot()
                if s["arm_startup"]["joint_control"]["request_id"] == client.pending_id:
                    report["stop_result"] = client.stop(client.pending_id)
        except Exception as stop_error:
            report["stop_error"] = str(stop_error)
        dump("report.json", report)
        print(out, report, flush=True)
        raise
    dump("report.json", report)
    print(out, report, flush=True)


if __name__ == "__main__":
    main()
