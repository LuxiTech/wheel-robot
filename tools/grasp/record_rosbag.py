#!/usr/bin/env python3
"""Supervise a robot-local rosbag recorder around one visual grasp process."""

import argparse
import base64
import json
import os
import select
import shlex
import signal
import sqlite3
import subprocess
import sys
import tempfile
import time
from pathlib import Path

SDK_ROOT = "/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2"
TOPICS = r"^(/luxi/.*|/camera/.*|/tf|/tf_static|/joint_states|/dynamic_joint_states|/arm_controller/.*|/controller_manager/.*|/rosout|/diagnostics|/planning_scene.*|/monitored_planning_scene|/display_planned_path)$"


def write_report(path, report):
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")


def interrupt(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)


def close_recorder(process):
    interrupt(process)
    try:
        return process.wait(timeout=30)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        return 1


def bag_counts(directory):
    counts = {}
    for database in directory.glob("*.db3"):
        with sqlite3.connect(f"file:{database}?mode=ro", uri=True) as connection:
            for topic, count in connection.execute(
                "SELECT topics.name, COUNT(messages.id) FROM topics "
                "LEFT JOIN messages ON topics.id=messages.topic_id GROUP BY topics.name"
            ):
                counts[topic] = counts.get(topic, 0) + count
    return counts


def worker(output):
    """Run on the board; stdin heartbeats bound lifetime after a lost SSH client."""
    output.mkdir(parents=True, exist_ok=False)
    bag = output / "bag"
    log_path = output / "recorder.log"
    report = {"started_ns": time.time_ns(), "topics_regex": TOPICS, "ready": False}
    process = None
    stop = False

    def on_signal(signum, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        with log_path.open("w") as log:
            process = subprocess.Popen(
                [
                    "ros2",
                    "bag",
                    "record",
                    "--storage",
                    "sqlite3",
                    "--output",
                    str(bag),
                    "--max-bag-size",
                    "1073741824",
                    "--include-hidden-topics",
                    "--regex",
                    TOPICS,
                ],
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            started = heartbeat = time.monotonic()
            while not stop:
                if process.poll() is not None:
                    raise RuntimeError(f"Recorder exited unexpectedly: {process.returncode}")
                readable, _, _ = select.select([sys.stdin], [], [], 0.2)
                if readable:
                    if not os.read(sys.stdin.fileno(), 4096):
                        report["stop_reason"] = "client_closed"
                        break
                    heartbeat = time.monotonic()
                if time.monotonic() - heartbeat > 10:
                    raise RuntimeError("Recorder client heartbeat lost")
                if not report["ready"]:
                    log_text = log_path.read_text(errors="replace")
                    if "/luxi/arms/state" in log_text and list(bag.glob("*.db3")):
                        report["ready"] = True
                        print(json.dumps({"ready": True, "remote_path": str(output)}), flush=True)
                    elif time.monotonic() - started > 30:
                        raise RuntimeError(
                            "Recorder did not subscribe to /luxi/arms/state within 30s"
                        )
            if stop:
                report["stop_reason"] = "signal"
    except Exception as error:
        report["error"] = str(error)
    finally:
        if process is not None:
            report["recorder_exit_code"] = close_recorder(process)
        try:
            report["message_counts"] = bag_counts(bag)
        except Exception as error:
            report["validation_error"] = str(error)
        report["complete"] = bool(
            report["ready"]
            and not report.get("error")
            and report.get("recorder_exit_code") in (0, -signal.SIGINT, 130)
            and (bag / "metadata.yaml").is_file()
            and report.get("message_counts", {}).get("/luxi/arms/state", 0) > 0
        )
        report["ended_ns"] = time.time_ns()
        write_report(output / "report.json", report)
        print(json.dumps(report), flush=True)
    return 0 if report["complete"] else 1


def supervise(output, command):
    output.mkdir(parents=True, exist_ok=True)
    host = os.environ.get("WHEEL_ROBOT_SSH", "sunrise@192.168.2.199")
    sdk_root = os.environ.get("WHEEL_ROBOT_SDK_ROOT", SDK_ROOT)
    remote_path = f"{sdk_root}/data/visual-grasp-runs/{output.parent.name}/rosbag"
    report = {"host": host, "remote_path": remote_path, "command": command}
    child = recorder = None
    interrupted = False
    exit_code = 1

    def on_signal(signum, frame):
        nonlocal interrupted
        interrupted = True
        interrupt(child)

    previous_handlers = {
        sig: signal.signal(sig, on_signal) for sig in (signal.SIGINT, signal.SIGTERM)
    }
    # A short private socket path lets scp reuse the interactive SSH authentication.
    with tempfile.TemporaryDirectory(prefix="wheel-bag-") as control:
        socket_path = str(Path(control) / "ssh")
        options = [
            "-o",
            "ConnectTimeout=10",
            "-o",
            "ServerAliveInterval=3",
            "-o",
            "ServerAliveCountMax=3",
            "-o",
            f"ControlPath={socket_path}",
        ]
        source = base64.b64encode(Path(__file__).read_bytes()).decode()
        script = (
            f"import base64; exec(compile(base64.b64decode({source!r}), '<rosbag-worker>', 'exec'))"
        )
        remote = (
            "set -e; source /opt/ros/humble/setup.bash; "
            f"source {shlex.quote(sdk_root + '/build/install/setup.bash')}; "
            "export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1; exec "
            + shlex.join(["python3", "-u", "-c", script, "--worker", "--output", remote_path])
        )
        try:
            recorder = subprocess.Popen(
                [
                    "ssh",
                    *options,
                    "-o",
                    "ControlMaster=auto",
                    "-o",
                    "ControlPersist=60",
                    host,
                    "bash -c " + shlex.quote(remote),
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                # Authentication uses the terminal; stdin is reserved for heartbeat.
                start_new_session=False,
            )
            started = time.monotonic()
            pending = b""
            ready = False
            with (output / "session.jsonl").open("ab") as log:
                while True:
                    if recorder.poll() is not None:
                        raise RuntimeError("ROS bag recorder connection exited; see recorder logs")
                    recorder.stdin.write(b"heartbeat\n")
                    recorder.stdin.flush()
                    readable, _, _ = select.select([recorder.stdout], [], [], 0.5)
                    if readable:
                        data = os.read(recorder.stdout.fileno(), 65536)
                        log.write(data)
                        log.flush()
                        pending += data
                        while b"\n" in pending:
                            line, pending = pending.split(b"\n", 1)
                            try:
                                event = json.loads(line)
                            except ValueError:
                                continue
                            if event.get("ready") and "complete" not in event:
                                ready = True
                    if interrupted:
                        exit_code = 130
                        if child is None or child.poll() is not None:
                            break
                    if ready and child is None and not interrupted:
                        print(f"ROS bag recording: {host}:{remote_path}", flush=True)
                        child = subprocess.Popen(command, start_new_session=True)
                    if child is not None and child.poll() is not None:
                        exit_code = child.returncode
                        break
                    if not ready and time.monotonic() - started > 60:
                        raise RuntimeError("ROS bag startup timed out; grasp was not started")
        except Exception as error:
            report["error"] = str(error)
            print(f"ROS bag error: {error}", file=sys.stderr, flush=True)
            interrupt(child)
            if child is not None:
                # Let the existing grasp shutdown send its owned stop before bag shutdown.
                child.wait()
        finally:
            if recorder is not None:
                try:
                    recorder.stdin.close()
                except BrokenPipeError:
                    pass
                try:
                    recorder.wait(timeout=45)
                except subprocess.TimeoutExpired:
                    recorder.terminate()
                    recorder.wait(timeout=15)
                report["remote_exit_code"] = recorder.returncode
                if recorder.returncode != 0 and exit_code == 0:
                    exit_code = 1
                # Preserve remote data on transfer failure; never delete a recording.
                transfer = subprocess.run(
                    [
                        "scp",
                        *options,
                        "-o",
                        "BatchMode=yes",
                        "-r",
                        f"{host}:{remote_path}",
                        str(output / "download"),
                    ],
                    check=False,
                )
                report["downloaded"] = transfer.returncode == 0
                if not report["downloaded"] and exit_code == 0:
                    exit_code = 1
            report["exit_code"] = exit_code if exit_code >= 0 else 128 - exit_code
            write_report(output / "session.json", report)
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)
    return report["exit_code"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.worker:
        return worker(args.output)
    command = args.command
    if command[:1] == ["--"]:
        command = command[1:]
    if not command:
        parser.error("A grasp command is required after --")
    return supervise(args.output.resolve(), command)


if __name__ == "__main__":
    raise SystemExit(main())
