#!/usr/bin/env python3
"""Verify the actual SDK candidate on isolated ROS + mock motors, never CAN.

All joint references and images here are synthetic. This does not inspect or
command the running hardware adapter, reset a fault or change production files.
"""

import argparse
import hashlib
import json
import math
import os
import socket
import struct
import subprocess
import time
from pathlib import Path

PACKET = struct.Struct("!8s32sQQQ16s32d")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--domain", type=int, default=224)
    parser.add_argument("--expected-j7-limit-deg", type=float)
    parser.add_argument("--fixed-target", action="store_true")
    parser.add_argument("--planned-trajectory", action="store_true")
    args = parser.parse_args()
    if args.expected_j7_limit_deg is not None and not 0 < args.expected_j7_limit_deg <= 90:
        raise ValueError("Expected J7 limit must be in (0, 90] degrees")
    if not 150 <= args.domain <= 225 or "candidate" not in str(args.candidate):
        raise ValueError("Independent candidate and isolated ROS domain required")
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            command = (process / "cmdline").read_bytes()
            if b"--ros-args" not in command and b"ros2" not in command:
                continue
            values = dict(
                row.split(b"=", 1)
                for row in (process / "environ").read_bytes().split(b"\0")
                if b"=" in row
            )
            if int(values.get(b"ROS_DOMAIN_ID", b"0")) == args.domain:
                raise RuntimeError("Requested domain is occupied by another ROS process")
        except (PermissionError, ProcessLookupError, FileNotFoundError):
            continue
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    import rclpy
    from luxi_interfaces.msg import ArmState
    from luxi_interfaces.srv import Command
    from std_msgs.msg import String

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node("wheel_joint_stream_mock_verifier")
    status, measured, history = {}, {}, []
    process, log = None, None
    checks = []

    def on_status(message):
        status.clear()
        status.update(json.loads(message.data))
        history.append(dict(at=time.monotonic(), control=dict(status["joint_control"])))

    def on_state(message):
        measured.update(
            position=list(message.position),
            velocity=list(message.velocity),
            received_ns=time.monotonic_ns(),
        )

    subscriptions = [
        node.create_subscription(String, "/luxi/arms/startup_status", on_status, 100),
        node.create_subscription(ArmState, "/luxi/arms/state", on_state, 100),
    ]
    publisher = node.create_publisher(String, "/luxi/control_status", 10)
    authority = ["console"]

    def heartbeat():
        message = String()
        message.data = json.dumps(
            dict(
                control_source=authority[0],
                phase="disabled",
                system_phase="disabled",
                shared_enable=True,
            )
        )
        publisher.publish(message)

    timer = node.create_timer(0.025, heartbeat)
    client = node.create_client(Command, "/luxi/arms/startup")

    def wait(predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process is not None and process.poll() is not None:
                raise RuntimeError("Candidate exited; inspect adapter.log")
            rclpy.spin_once(node, timeout_sec=0.001)
            if predicate():
                return
        raise TimeoutError("Mock condition timed out: " + str(status.get("joint_control")))

    def call(command, detail, expected=True):
        request = Command.Request()
        request.command, request.detail = command, json.dumps(detail)
        future = client.call_async(request)
        wait(future.done)
        reply = future.result()
        if reply.success is not expected:
            raise AssertionError((command, reply.message, expected))

    stream = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stream.connect(("127.0.0.1", port))

    def begin(identity, key):
        # A larger regression trajectory needs time to settle before takeover.
        # Respect the real SDK readiness check instead of relaxing it for mock.
        settled_since = None

        def stationary():
            nonlocal settled_since
            if not measured or max(abs(v) for v in measured["velocity"]) > 0.08:
                settled_since = None
                return False
            if settled_since is None:
                settled_since = time.monotonic()
            return time.monotonic() - settled_since >= 0.3

        wait(stationary)
        call(
            "joint_stream_start",
            dict(
                request_id=identity,
                stream_key=key,
                speed_rad_s=0.2,
                acceleration_rad_s2=0.5,
            ),
        )
        wait(lambda: status["joint_control"]["phase"] == "streaming")

    def send(sequence, key, positions, *, image_ns=None, pair=None, fixed=False, planned=False):
        now = time.monotonic_ns()
        image = image_ns or now
        identity = str(image) if pair is None else pair
        packet = PACKET.pack(
            b"WJPLAN1\0" if planned else (b"WJFIXD1\0" if fixed else b"WJSTRM1\0"),
            key.encode(),
            sequence,
            measured["received_ns"],
            image,
            hashlib.sha256(identity.encode()).digest()[:16],
            *positions,
            *([0.0] * 16),
        )
        stream.send(packet)
        return packet

    try:
        log = (args.output / "adapter.log").open("w")
        command = [
            str(args.candidate / "build/openarm_adapter_stream_candidate"),
            "--ros-args",
            "-r",
            "__node:=wheel_joint_stream_mock_adapter",
            "-p",
            "config:=" + str(args.config),
            "-p",
            "mock:=true",
            "-p",
            "allow_hardware:=false",
            "-p",
            "joint_stream_bind:=127.0.0.1",
            "-p",
            "joint_stream_port:=" + str(port),
        ]
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        wait(lambda: client.service_is_ready() and bool(status) and bool(measured))
        if status["joint_control"]["joint_stream_protocol"] != 1:
            raise AssertionError("Missing stream protocol")
        if status["joint_control"].get("joint_stream_image_max_age_ms") != 2000:
            raise AssertionError("Image watchdog must advertise 2000ms")
        j7_audit = None
        if args.expected_j7_limit_deg is not None:
            bound = math.radians(args.expected_j7_limit_deg)
            for index in (6, 14):
                control = status["joint_control"]
                if not (
                    abs(control["lower"][index] + bound) < 1e-12
                    and abs(control["upper"][index] - bound) < 1e-12
                ):
                    raise AssertionError("SDK did not publish the configured J7 limits")
                for sign in (-1, 1):
                    positions = list(measured["position"])
                    positions[index] = sign * (bound + 1e-5)
                    call(
                        "joint_move",
                        dict(
                            request_id=f"j7-outside-{index}-{sign}",
                            unit="rad",
                            positions=positions,
                            speed_rad_s=0.2,
                        ),
                        expected=False,
                    )
            j7_audit = dict(degrees=args.expected_j7_limit_deg, outside_goals_rejected=4)
        call(
            "joint_move",
            dict(
                request_id="mock-explicit-enable",
                unit="rad",
                positions=measured["position"],
                speed_rad_s=0.2,
            ),
        )
        wait(lambda: status["joint_control"]["phase"] == "ready")
        checks.append("mock_enable_only_no_hardware")
        if j7_audit is not None:
            for sign in (-1, 1):
                positions = list(measured["position"])
                positions[6], positions[14] = sign * bound, -sign * bound
                identity = f"j7-exact-boundaries-{sign}"
                call(
                    "joint_move",
                    dict(request_id=identity, unit="rad", positions=positions, speed_rad_s=0.2),
                )
                call("joint_stop", dict(request_id=identity))
                wait(lambda: status["joint_control"]["phase"] == "stopped")
            j7_audit["exact_boundary_goals_accepted"] = 2

        begin("stream-one", "a" * 32)
        generation = status["control_generation"]
        initial = list(status["nominal_target"])
        deadline = time.monotonic()
        next_image = deadline
        image = time.monotonic_ns()
        sent_at = []
        image_ids = set()
        for sequence in range(1, 101):
            if time.monotonic() >= next_image:
                image = time.monotonic_ns()
                next_image += 1 / 30.0
            image_ids.add(image)
            positions = list(initial)
            # Exercise acceleration lag: valid 0.002rad steps must not be
            # rejected merely because the board output is still ramping up.
            positions[0] += 0.002 * min(sequence, 60)
            sent_at.append(time.monotonic())
            send(sequence, "a" * 32, positions, image_ns=image)
            deadline += 0.01
            wait(lambda: time.monotonic() >= deadline)
            if status["joint_control"]["phase"] != "streaming":
                raise AssertionError(status["joint_control"])
            if status["control_generation"] != generation:
                raise AssertionError("Stream changed control generation")
        wait(lambda: status["joint_control"]["joint_stream_sequence"] == 100)
        if abs(measured["position"][0] - initial[0]) <= 0.001:
            raise AssertionError("Mock motor did not follow continuous references")
        checks.append("100hz_udp_references_with_30hz_synthetic_images")
        send(99, "a" * 32, initial)
        send(100, "a" * 32, initial)
        send(101, "b" * 32, initial)
        send(102, "a" * 32, [float("nan")] + initial[1:])
        call("joint_stop", dict(request_id="not-owner"), expected=False)
        call("joint_stop", dict(request_id="stream-one"))
        wait(lambda: status["joint_control"]["phase"] == "stopped")
        if status["joint_control"]["joint_stream_sequence"] != 100:
            raise AssertionError("Invalid datagram changed the accepted sequence")
        checks.append("invalid_duplicate_old_session_packets_and_unowned_stop_rejected")

        begin("stream-command-expire", "c" * 32)
        send(1, "c" * 32, status["nominal_target"])
        wait(lambda: status["joint_control"]["phase"] == "stopped")
        if status["joint_control"]["reason"] not in (
            "joint_stream_command_timeout",
            "joint_stream_feedback_timeout",
        ):
            raise AssertionError(status["joint_control"])
        send(2, "c" * 32, status["nominal_target"])
        until = time.monotonic() + 0.08
        wait(lambda: time.monotonic() > until)
        if status["joint_control"]["phase"] != "stopped":
            raise AssertionError("Late packet resumed an expired session")
        checks.append("command_loss_holds_without_automatic_resume")

        begin("stream-image-expire", "d" * 32)
        image = time.monotonic_ns()
        initial = list(status["nominal_target"])
        deadline = time.monotonic()
        for sequence in range(1, 221):
            send(sequence, "d" * 32, initial, image_ns=image, pair="frozen-mock-image")
            deadline += 0.01
            wait(lambda: time.monotonic() >= deadline)
            if sequence == 60 and status["joint_control"]["phase"] != "streaming":
                raise AssertionError("Image expired before the requested 2s window")
        wait(lambda: status["joint_control"]["phase"] == "stopped")
        if status["joint_control"]["reason"] != "joint_stream_image_timeout":
            raise AssertionError(status["joint_control"])
        checks.append("new_commands_do_not_refresh_old_image_time")

        if args.fixed_target:
            if status["joint_control"].get("joint_stream_fixed_target_protocol") != 1:
                raise AssertionError("Fixed target capability missing")
            begin("stream-fixed", "b" * 32)
            image = time.monotonic_ns()
            initial = list(status["nominal_target"])
            deadline = time.monotonic()
            for sequence in range(1, 301):
                send(sequence, "b" * 32, initial, image_ns=image, pair="fixed", fixed=True)
                deadline += 0.01
                wait(lambda: time.monotonic() >= deadline)
                if status["joint_control"]["phase"] != "streaming":
                    raise AssertionError(status["joint_control"])
            call("joint_stop", dict(request_id="stream-fixed"))
            wait(lambda: status["joint_control"]["phase"] == "stopped")

        if args.planned_trajectory:
            if status["joint_control"].get("joint_stream_planned_trajectory_protocol") != 1:
                raise AssertionError("Planned trajectory capability missing")
            begin("stream-planned", "b" * 32)
            admission = time.monotonic_ns()
            initial = list(status["nominal_target"])
            deadline = time.monotonic()
            for sequence in range(1, 301):
                send(sequence, "b" * 32, initial, image_ns=admission, pair="plan", planned=True)
                deadline += 0.01
                wait(lambda: time.monotonic() >= deadline)
                if status["joint_control"]["phase"] != "streaming":
                    raise AssertionError(status["joint_control"])
            call("joint_stop", dict(request_id="stream-planned"))
            wait(lambda: status["joint_control"]["phase"] == "stopped")
            checks.append("planned_trajectory_session_survives_three_seconds")

        begin("stream-authority", "e" * 32)
        initial = list(status["nominal_target"])
        authority[0] = "teleop"
        deadline = time.monotonic()
        # Keep commands fresh until the independently published authority change
        # is observed. Ending at 150 ms races the command watchdog with that topic.
        for sequence in range(1, 101):
            send(sequence, "e" * 32, initial)
            deadline += 0.01
            wait(lambda: time.monotonic() >= deadline)
            if status["joint_control"]["phase"] == "stopped":
                break
        wait(lambda: status["joint_control"]["phase"] == "stopped")
        if status["joint_control"]["reason"] != "joint_stream_authority_lost":
            raise AssertionError(status["joint_control"])
        call(
            "joint_stream_start",
            dict(request_id="wrong-authority", stream_key="f" * 32),
            expected=False,
        )
        checks.append("authority_loss_stops_and_prevents_restart")
        if status["output_enable_count"] != 1 or status["output_disable_count"] != 0:
            raise AssertionError("Unexpected mock enable/disable count")
        result = dict(
            passed=True,
            checks=checks,
            domain=args.domain,
            udp_port=port,
            mock=True,
            allow_hardware=False,
            real_motor_commands_sent=False,
            enable_count=status["output_enable_count"],
            disable_count=status["output_disable_count"],
            j7_limit_audit=j7_audit,
            fixed_target_three_seconds_passed=args.fixed_target,
            verified_sha256={
                relative: hashlib.sha256((args.candidate / relative).read_bytes()).hexdigest()
                for relative in (
                    "openarm_adapter.cpp",
                    "joint_stream.hpp",
                    "build/openarm_adapter_stream_candidate",
                )
            },
            reference_timing=dict(
                sent=len(sent_at),
                distinct_synthetic_images=len(image_ids),
                observed_send_hz=(len(sent_at) - 1) / (sent_at[-1] - sent_at[0]),
                max_send_period_ms=max(b - a for a, b in zip(sent_at, sent_at[1:])) * 1000,
                scope="Synthetic references on robot CPU, loopback UDP and mock motors only",
            ),
        )
        (args.output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result))
    finally:
        (args.output / "history.json").write_text(json.dumps(history) + "\n")
        if process is not None and process.poll() is None:
            process.terminate()
            process.wait(timeout=5)
        if log is not None:
            log.close()
        stream.close()
        node.destroy_node()
        rclpy.shutdown()
        del subscriptions, timer


if __name__ == "__main__":
    main()
