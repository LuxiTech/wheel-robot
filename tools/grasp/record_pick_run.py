#!/usr/bin/env python3
"""Record real feedback and atomic RGB-D alongside a pick; sends GETs only."""

import argparse
import concurrent.futures
import datetime
import json
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure

configure(ROOT)
import cv2
from runlog import write_json
from wheel_perception_core.rgbd import CAMERAS, RGBDClient, save_observation
from wheel_sdk_bridge.http_client import JSONClient
from wheel_sdk_bridge.joint_client import JointClient


def record(args):
    args.output.mkdir(parents=True, exist_ok=False)
    stop = threading.Event()
    counts = {}
    errors = {}
    start = time.monotonic()
    deadline = start + args.duration
    policy = json.loads((ROOT / "config/robots/joint_control.json").read_text())
    policy["feedback_diagnostics"] = str(args.output / "preflight_feedback.jsonl")
    joints = JointClient(policy["base_url"], policy)
    summary = dict(
        started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        commands_sent=False,
        recording_only=True,
        feedback_interval_s=0.1,
        camera_interval_s=2.0,
        errors=errors,
        counts=counts,
    )

    def preflight(label):
        try:
            summary[label] = joints.preflight()
        except Exception as exc:
            summary[label] = dict(accepted=False, error=str(exc), commands_sent=False)
        write_json(args.output / "summary.json", summary)

    def poll_state(name, url, interval):
        client = JSONClient(url, timeout=1)
        count = 0
        failures = 0
        with (args.output / (name + ".jsonl")).open("x") as stream:
            while not stop.is_set() and time.monotonic() < deadline:
                began = time.monotonic()
                row = dict(wall_time_ns=time.time_ns(), monotonic_ns=time.monotonic_ns())
                try:
                    row["state"] = client.request("GET", "/api/state")
                    count += 1
                except Exception as exc:
                    row["error"] = str(exc)
                    failures += 1
                row["request_seconds"] = time.monotonic() - began
                stream.write(json.dumps(row, allow_nan=False) + "\n")
                stream.flush()
                stop.wait(max(0, interval - row["request_seconds"]))
        counts[name] = count
        errors[name] = failures

    def poll_camera(camera):
        directory = args.output / camera
        directory.mkdir()
        client = RGBDClient(policy["base_url"], timeout=1)
        count = 0
        failures = 0
        with (directory / "frames.jsonl").open("x") as stream:
            while not stop.is_set() and time.monotonic() < deadline:
                began = time.monotonic()
                row = dict(wall_time_ns=time.time_ns(), monotonic_ns=time.monotonic_ns())
                try:
                    observation = client.receive(camera)
                    stem = f"{count:05d}"
                    save_observation(directory / (stem + ".npz"), observation)
                    image = cv2.cvtColor(observation.rgb, cv2.COLOR_RGB2BGR)
                    if not cv2.imwrite(str(directory / (stem + ".jpg")), image):
                        raise OSError("Could not save camera image")
                    row.update(file=stem + ".npz", metadata=observation.metadata)
                    count += 1
                except Exception as exc:
                    row["error"] = str(exc)
                    failures += 1
                stream.write(json.dumps(row, allow_nan=False) + "\n")
                stream.flush()
                stop.wait(max(0, 2.0 - (time.monotonic() - began)))
        counts[camera] = count
        errors[camera] = failures

    preflight("preflight_before")
    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=5) as pool:
            futures = [
                pool.submit(poll_state, "joint_feedback", policy["feedback_url"], 0.1),
                pool.submit(poll_state, "sdk_state", policy["base_url"], 0.5),
                *(pool.submit(poll_camera, camera) for camera in sorted(CAMERAS)),
            ]
            try:
                for future in futures:
                    future.result()
            finally:
                stop.set()
    finally:
        stop.set()
        summary["duration_s"] = time.monotonic() - start
        preflight("preflight_after")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=30)
    arguments = parser.parse_args()
    if not 0 < arguments.duration <= 3600:
        parser.error("duration must be in (0, 3600] seconds")
    record(arguments)
