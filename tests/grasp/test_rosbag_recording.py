"""Exercise recorder lifecycle without ROS, SSH, or robot motion."""

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools/grasp/record_rosbag.py"


def executable(path, source):
    path.write_text(f"#!{sys.executable}\n" + source)
    path.chmod(0o755)


@pytest.fixture
def recorder_environment(tmp_path):
    binaries = tmp_path / "bin"
    binaries.mkdir()
    executable(
        binaries / "ros2",
        """import os, pathlib, signal, sqlite3, sys, time
if os.environ.get("FAIL_RECORDER") == "1":
    sys.exit(3)
bag = pathlib.Path(sys.argv[sys.argv.index("--output") + 1])
bag.mkdir()
with sqlite3.connect(bag / "take.db3") as db:
    db.execute("CREATE TABLE topics (id INTEGER, name TEXT)")
    db.execute("CREATE TABLE messages (id INTEGER, topic_id INTEGER)")
    db.execute("INSERT INTO topics VALUES (1, '/luxi/arms/state')")
    db.execute("INSERT INTO messages VALUES (1, 1)")
def finish(*args):
    (bag / "metadata.yaml").write_text("closed: true")
    sys.exit(0)
signal.signal(signal.SIGINT, finish)
print("Subscribed to topic '/luxi/arms/state'", flush=True)
if os.environ.get("CRASH_RECORDER") == "1":
    time.sleep(2)
    sys.exit(3)
while True:
    time.sleep(.05)
""",
    )
    executable(
        binaries / "ssh",
        """import os, shlex, shutil, sys
remote = shlex.split(sys.argv[-1])[2]
command = shlex.split(remote.split("; exec ", 1)[1])
os.execv(shutil.which(command[0]), command)
""",
    )
    executable(
        binaries / "scp",
        """import shutil, sys
shutil.copytree(sys.argv[-2].split(":", 1)[1], sys.argv[-1])
""",
    )
    return {
        **os.environ,
        "PATH": str(binaries) + os.pathsep + os.environ["PATH"],
        "WHEEL_ROBOT_SDK_ROOT": str(tmp_path / "robot"),
    }


@pytest.mark.parametrize("code", [0, 7, 130])
def test_process_exit_closes_bag_and_preserves_status(tmp_path, recorder_environment, code):
    output = tmp_path / "run-id/rosbag"
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--output",
            str(output),
            "--",
            sys.executable,
            "-c",
            f"raise SystemExit({code})",
        ],
        env=recorder_environment,
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode == code, result.stderr
    report = json.loads((output / "download/report.json").read_text())
    assert report["complete"] is True
    assert report["message_counts"]["/luxi/arms/state"] == 1
    assert (output / "download/bag/metadata.yaml").is_file()


def test_failed_recorder_never_launches_grasp(tmp_path, recorder_environment):
    marker = tmp_path / "motion-started"
    output = tmp_path / "run-id/rosbag"
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--output",
            str(output),
            "--",
            sys.executable,
            "-c",
            f"from pathlib import Path; Path({str(marker)!r}).touch()",
        ],
        env={**recorder_environment, "FAIL_RECORDER": "1"},
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode != 0
    assert not marker.exists()
    assert json.loads((output / "download/report.json").read_text())["complete"] is False


def test_recorder_crash_interrupts_grasp_before_return(tmp_path, recorder_environment):
    output = tmp_path / "run-id/rosbag"
    stopped = tmp_path / "owned-stop"
    command = (
        "import signal,time; from pathlib import Path; "
        f"signal.signal(signal.SIGINT, lambda *args: (Path({str(stopped)!r}).touch(), exit(0))); "
        "time.sleep(30)"
    )
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--output", str(output), "--", sys.executable, "-c", command],
        env={**recorder_environment, "CRASH_RECORDER": "1"},
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode != 0
    assert stopped.exists()


def test_worker_stops_when_client_pipe_closes(tmp_path, recorder_environment):
    output = tmp_path / "remote"
    process = subprocess.Popen(
        [sys.executable, str(SCRIPT), "--worker", "--output", str(output)],
        env=recorder_environment,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
    )
    try:
        assert json.loads(process.stdout.readline())["ready"] is True
        process.stdin.close()
        assert process.wait(timeout=5) == 0
        assert json.loads((output / "report.json").read_text())["complete"] is True
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()


def test_sigterm_waits_for_grasp_shutdown_and_closes_bag(tmp_path, recorder_environment):
    output = tmp_path / "run-id/rosbag"
    marker = tmp_path / "started"
    command = (
        "import signal,time; from pathlib import Path; "
        "signal.signal(signal.SIGINT, lambda *args: exit(0)); "
        f"Path({str(marker)!r}).touch(); time.sleep(30)"
    )
    process = subprocess.Popen(
        [sys.executable, str(SCRIPT), "--output", str(output), "--", sys.executable, "-c", command],
        env=recorder_environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    try:
        deadline = time.monotonic() + 10
        while not marker.exists() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert marker.exists()
        process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=10) == 130
        assert json.loads((output / "download/report.json").read_text())["complete"] is True
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()


def test_worker_heartbeat_timeout_closes_bag(tmp_path, recorder_environment):
    output = tmp_path / "remote"
    process = subprocess.Popen(
        [sys.executable, str(SCRIPT), "--worker", "--output", str(output)],
        env=recorder_environment,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
    )
    try:
        assert json.loads(process.stdout.readline())["ready"] is True
        # Keep the pipe open, but send no heartbeat: simulate a hung SSH client.
        assert process.wait(timeout=13) != 0
        report = json.loads((output / "report.json").read_text())
        assert report["error"] == "Recorder client heartbeat lost"
        assert (output / "bag/metadata.yaml").is_file()
        assert report["message_counts"]["/luxi/arms/state"] == 1
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        process.stdin.close()


def test_download_failure_preserves_remote_bag(tmp_path, recorder_environment):
    executable(tmp_path / "bin/scp", "raise SystemExit(1)\n")
    output = tmp_path / "run-id/rosbag"
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--output", str(output), "--", sys.executable, "-c", "pass"],
        env=recorder_environment,
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode != 0
    session = json.loads((output / "session.json").read_text())
    assert session["downloaded"] is False
    remote = Path(session["remote_path"])
    assert (remote / "bag/metadata.yaml").is_file()
    assert json.loads((remote / "report.json").read_text())["complete"] is True
