#!/usr/bin/env python3
"""Record the physically supported pose as motor hardware zero; default is preview.

Uses the robot's installed OpenArm CLI, not joint_move. No lift/camera commands.
The arm monitor is left stopped after execution, preventing old targets from
being reused before coordinate-dependent settings and calibrations are reviewed.
"""

import argparse
import datetime
import fcntl
import hashlib
import json
import math
import os
import re
import shlex
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path
from urllib.request import ProxyHandler, build_opener

SDK_ROOT = Path("/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2")
SERVICE = "luxi-openarm-monitor.service"
CLI = "/usr/bin/openarm-can-cli"


def run(command, **kwargs):
    return subprocess.run(command, capture_output=True, text=True, timeout=30, **kwargs)


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def sdk_state():
    opener = build_opener(ProxyHandler({}))
    # The deployed feedback server binds this LAN address, not loopback.
    with opener.open("http://192.168.0.245:18032/api/state", timeout=3) as response:
        return json.load(response)


def check_disabled(state):
    arms = state.get("arms") or {}
    if state.get("system", {}).get("mock") is not False or arms.get("enabled") is not False:
        raise RuntimeError("Disable both arm outputs before hardware zero; no automatic disable")
    positions = arms.get("position", [])
    velocities = arms.get("velocity", [])
    ages = arms.get("age_ns", [])
    if (
        arms.get("fresh") is not True
        or arms.get("joint_order") != "right_then_left"
        or len(positions) != 16
        or len(velocities) != 16
        or len(ages) != 16
        or arms.get("valid") != [True] * 16
        or not all(math.isfinite(q) for q in positions + velocities)
        or max(abs(v) for v in velocities) > 0.05
    ):
        raise RuntimeError("Require all sixteen fresh, valid, stationary arm readings")
    server = int(state["server_mono_ns"])
    cache_age = (server - int(arms["receive_mono_ns"])) / 1e9
    if not 0 <= cache_age <= 0.4 or any(not 0 <= age / 1e9 + cache_age <= 0.4 for age in ages):
        raise RuntimeError("Arm feedback is stale")


def zero_command(interface, joints):
    if interface not in ("luxi_left", "luxi_right"):
        raise ValueError("Unexpected arm CAN interface")
    if not joints or any(type(j) is not int or not 1 <= j <= 8 for j in joints):
        raise ValueError("Motor IDs must be in 1..8")
    # --no-arm is essential: --id alone otherwise also selects all IDs 1..8.
    return [CLI, "-i", interface, "set_zero", "--no-arm", "--id", *map(str, joints)]


def read_can(interface, joints):
    """SDK's 0xCC refresh request; no torque enable or position target is sent."""
    result = {}
    with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        # The deployed adapter returns CAN FD frames, even for an 8-byte payload.
        # Enable reception of both frame types; refresh requests remain classical CAN.
        bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
        bus.bind((interface,))
        bus.settimeout(0.1)
        for motor in joints:
            bus.send(struct.pack("=IB3x8s", 0x7FF, 8, bytes([motor, 0, 0xCC, 0, 0, 0, 0, 0])))
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                try:
                    raw = bus.recv(72)
                except socket.timeout:
                    continue
                if len(raw) not in (16, 72):
                    continue
                can_id, size = struct.unpack_from("=IB", raw)
                payload = raw[8:16]
                if can_id != motor + 16 or size != 8 or payload[0] & 15 != motor:
                    continue
                code = int.from_bytes(payload[1:3], "big")
                result[motor] = {
                    "motor_id": motor,
                    "status_nibble": payload[0] >> 4,
                    "position_code": code,
                    "nominal_position_rad_at_pmax12_5": code * 25.0 / 65535 - 12.5,
                    "data_hex": payload.hex(),
                }
                break
            if motor not in result:
                raise RuntimeError(f"No fresh CAN response: {interface} motor {motor}")
    return result


def check_can_disabled(readings):
    if any(item["status_nibble"] != 0 for item in readings.values()):
        raise RuntimeError("CAN reports enabled/faulted motor; do not write zero")


def check_new_zero(readings):
    check_can_disabled(readings)
    if any(abs(item["position_code"] - 32767.5) > 4 for item in readings.values()):
        raise RuntimeError("Set-zero command was sent but raw feedback is not near zero")


def robot_main(args):
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = SDK_ROOT / "work/arm-encoder-zero" / stamp
    output.mkdir(parents=True, exist_ok=False)
    print(f"Robot logs: {output}", flush=True)
    # A preview can create this file as sunrise. Linux protected_regular may
    # reject sudo opening that existing /tmp file with O_CREAT (append mode).
    try:
        lock_file = open("/tmp/luxi-arm-encoder-zero.lock", "x")
    except FileExistsError:
        lock_file = open("/tmp/luxi-arm-encoder-zero.lock", "r")
    fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    sides = ["right", "left"] if args.side == "both" else [args.side]
    config_root = SDK_ROOT / "Project/robot_bringup/config"
    configs = {
        name: json.loads((config_root / name).read_text())
        for name in ["arms.json", "arms-kh.json", "arms-teleop.json"]
    }
    state = sdk_state()
    save(output / "sdk_before.json", state)
    blockers = []
    try:
        check_disabled(state)
    except RuntimeError as error:
        blockers.append(str(error))
    for name, config in configs.items():
        save(output / name, config)
        for side in sides:
            offsets = config[side]["offsets"]
            if len(offsets) != 8 or any(offsets[j - 1] != 0 for j in args.joints):
                raise RuntimeError(
                    f"{name}/{side} has software offsets; resolve before hardware zero"
                )
    active = configs["arms-kh.json"]
    commands = {}
    for side in sides:
        interface = active[side]["interface"]
        if interface != "luxi_" + side:
            raise RuntimeError("Unexpected SDK bus mapping")
        commands[side] = zero_command(interface, args.joints)
    report = {
        "created_utc": stamp,
        "status": "preview_only",
        "execution_blockers": blockers,
        "sdk_arms_enabled": state.get("arms", {}).get("enabled"),
        "selected_sides": sides,
        "selected_motor_ids": args.joints,
        "motor8_meaning": "gripper, not J7",
        "commands": commands,
        "cli_sha256": hashlib.sha256(Path(CLI).read_bytes()).hexdigest(),
        "source_sha256": hashlib.sha256(
            (
                SDK_ROOT
                / "device/openarm/openarm_can/setup/cli/commands/zero_position_commands.cpp"
            ).read_bytes()
        ).hexdigest(),
        "zero_writes_attempted": [],
        "monitor_left_stopped": False,
        "post_zero_coordinate_review_required": [
            "joint limits/home/support targets",
            "camera calibration and historical joint recordings",
            "gripper mapping if motor8 reset",
        ],
    }
    save(output / "report.json", report)
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    if not args.execute:
        return 0
    try:
        if os.geteuid() != 0:
            raise RuntimeError("Execution needs sudo on robot")
        check_disabled(sdk_state())
        if run(["systemctl", "is-active", "luxi-robot.service"]).returncode == 0:
            raise RuntimeError("Full robot controller is active; do not compete for CAN")
        result = run(["systemctl", "stop", SERVICE])
        if result.returncode or run(["systemctl", "is-active", SERVICE]).returncode == 0:
            raise RuntimeError("Could not stop arm monitor")
        report["monitor_left_stopped"] = True
        report["status"] = "monitor_stopped"
        save(output / "report.json", report)
        for side in sides:
            before = read_can("luxi_" + side, args.joints)
            save(output / f"{side}_raw_before.json", before)
            check_can_disabled(before)
        for side in sides:
            report["zero_writes_attempted"].append(side)
            save(output / "report.json", report)
            result = run(commands[side])
            save(
                output / f"{side}_set_zero.json",
                {"returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr},
            )
            if result.returncode:
                raise RuntimeError("Set-zero CLI failed; do not retry automatically")
            after = read_can("luxi_" + side, args.joints)
            save(output / f"{side}_raw_after.json", after)
            check_new_zero(after)
        report["status"] = "hardware_zero_verified_raw_feedback"
        print("Hardware zero verified. Motors disabled; arm monitor remains stopped.", flush=True)
        print(
            "Review changed coordinates before restarting control; no motion was sent.", flush=True
        )
        return 0
    except (Exception, KeyboardInterrupt) as error:
        report.update(status="aborted_may_be_partial", error=str(error) or type(error).__name__)
        print(report["error"], file=sys.stderr, flush=True)
        return 1
    finally:
        save(output / "report.json", report)
        lock_file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="sunrise@192.168.0.245")
    parser.add_argument("--side", choices=["left", "right", "both"], default="both")
    parser.add_argument(
        "--joints", type=int, nargs="+", choices=range(1, 9), default=list(range(1, 9))
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--pose-supported",
        action="store_true",
        help="Confirm pose is physically supported and both arm outputs disabled",
    )
    parser.add_argument("--on-robot", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    args.joints = sorted(set(args.joints))
    if args.execute and not args.pose_supported:
        parser.error(
            "--execute requires --pose-supported: secure the pose before motors are disabled"
        )
    if args.on_robot:
        try:
            return robot_main(args)
        except Exception as error:
            print(f"Preflight failed: {error}; no automatic retry", file=sys.stderr)
            return 1
    if not re.fullmatch(r"[A-Za-z0-9_.-]+@[A-Za-z0-9_.-]+", args.host):
        parser.error("--host must be user@hostname")
    remote = ["python3", "-", "--on-robot", "--side", args.side, "--joints", *map(str, args.joints)]
    if args.execute:
        remote = ["sudo", "-n", *remote, "--execute", "--pose-supported"]
    command = ["ssh", "-o", "ConnectTimeout=5", "--", args.host, shlex.join(remote)]
    # SSH authenticates using the user's terminal or agent; no password is saved.
    return subprocess.run(command, input=Path(__file__).read_text(), text=True).returncode


if __name__ == "__main__":
    raise SystemExit(main())
