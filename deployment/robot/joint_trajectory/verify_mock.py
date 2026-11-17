#!/usr/bin/env python3
"""Exercise the compiled live-SDK candidate in an isolated ROS domain, mock only."""

import argparse
import json
import os
import subprocess
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--domain", type=int, default=221)
    args = parser.parse_args()
    if "candidate" not in args.binary.name or not 150 <= args.domain <= 225:
        raise ValueError("A separate candidate executable and isolated test domain are required")
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            command = (process / "cmdline").read_bytes()
            if b"openarm_adapter" in command and b"mock:=false" in command:
                environment = dict(
                    entry.split(b"=", 1)
                    for entry in (process / "environ").read_bytes().split(b"\0")
                    if b"=" in entry
                )
                if int(environment.get(b"ROS_DOMAIN_ID", b"0")) == args.domain:
                    raise RuntimeError("Refusing to share a ROS domain with real motors")
        except (PermissionError, FileNotFoundError, ProcessLookupError):
            continue
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    import rclpy
    from luxi_interfaces.msg import ArmCommand, ArmState
    from luxi_interfaces.srv import Command
    from std_msgs.msg import String

    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node("wheel_trajectory_mock_verifier")
    status, measured, history, applied = {}, {}, [], []

    def on_status(message):
        status.clear()
        status.update(json.loads(message.data))
        history.append((time.monotonic(), dict(status["joint_control"])))

    def on_state(message):
        measured["position"] = list(message.position)
        measured["velocity"] = list(message.velocity)

    def on_applied(message):
        applied.append((int(message.source_mono_ns), list(message.position)))

    subscriptions = [
        node.create_subscription(String, "/luxi/arms/startup_status", on_status, 100),
        node.create_subscription(ArmState, "/luxi/arms/state", on_state, 100),
        node.create_subscription(ArmCommand, "/luxi/applied", on_applied, 100),
    ]
    client = node.create_client(Command, "/luxi/arms/startup")
    control = node.create_publisher(String, "/luxi/control_status", 10)
    log = (args.output / "adapter.log").open("w")
    process = subprocess.Popen(
        [
            str(args.binary.resolve()),
            "--ros-args",
            "-p",
            "config:=" + str(args.config.resolve()),
            "-p",
            "mock:=true",
            "-p",
            "allow_hardware:=false",
            "-r",
            "__node:=wheel_trajectory_mock_adapter",
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
    )

    def wait(predicate, timeout=12):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("Candidate exited; inspect adapter.log")
            rclpy.spin_once(node, timeout_sec=0.005)
            if predicate():
                return
        raise TimeoutError("Mock predicate timed out: " + str(status.get("joint_control")))

    def call(command, detail=None, expected=True):
        request = Command.Request()
        request.command = command
        request.detail = "" if detail is None else json.dumps(detail)
        future = client.call_async(request)
        wait(future.done)
        reply = future.result()
        assert reply.success is expected, (command, reply.success, reply.message)
        return reply

    checks = []
    try:
        wait(lambda: client.service_is_ready() and bool(status) and bool(measured))
        assert status["joint_control"]["trajectory_protocol"] == 1
        current = measured["position"].copy()
        call(
            "joint_move",
            dict(request_id="mock-enable", unit="rad", positions=current, speed_rad_s=0.2),
        )
        wait(
            lambda: (
                status["joint_control"]["request_id"] == "mock-enable"
                and status["joint_control"]["phase"] == "ready"
            )
        )
        checks.append("explicit_mock_enable")
        current = measured["position"].copy()
        first, final = current.copy(), current.copy()
        first[0] += 0.04
        final[0] += 0.08
        packet = dict(
            request_id="mock-complete-phase",
            unit="rad",
            waypoints=[current, first, final],
            speed_rad_s=0.2,
        )
        call("joint_trajectory", packet)
        wait(
            lambda: (
                status["joint_control"]["request_id"] == packet["request_id"]
                and status["joint_control"]["phase"] == "ready"
            )
        )
        duration = status["joint_control"]["trajectory_duration_s"]
        assert abs(measured["position"][0] - final[0]) <= 0.02
        passage = [
            (stamp, point[0])
            for stamp, point in applied
            if current[0] + 0.03 <= point[0] <= current[0] + 0.05
        ]
        assert len(passage) >= 3
        velocities = [
            (b[1] - a[1]) / ((b[0] - a[0]) * 1e-9)
            for a, b in zip(passage, passage[1:])
            if b[0] > a[0]
        ]
        assert min(velocities) > 0.005, velocities
        assert max(velocities) <= 0.205
        checks.append("continuous_internal_waypoint_passage_and_final_arrival")
        generation = status["control_generation"]
        call("joint_trajectory", packet)
        wait(lambda: status["joint_control"]["phase"] == "ready")
        assert status["control_generation"] == generation
        modified = dict(packet, waypoints=[current, final, final])
        call("joint_trajectory", modified, expected=False)
        assert status["control_generation"] == generation
        checks.append("full_path_idempotency_not_just_final_endpoint")
        invalid = [row.copy() for row in packet["waypoints"]]
        invalid[1][2] = status["joint_control"]["upper"][2] + 0.1
        call(
            "joint_trajectory",
            dict(packet, request_id="mock-invalid", waypoints=invalid),
            expected=False,
        )
        assert status["control_generation"] == generation
        checks.append("invalid_interior_waypoint_rejected_before_mutation")
        message = String()
        message.data = json.dumps(
            dict(control_source="teleop", phase="hold", system_phase="disabled", shared_enable=True)
        )
        control.publish(message)
        until = time.monotonic() + 0.1
        wait(lambda: time.monotonic() > until)
        call("joint_trajectory", dict(packet, request_id="mock-no-console"), expected=False)
        message.data = json.dumps(
            dict(
                control_source="console",
                phase="disabled",
                system_phase="disabled",
                shared_enable=True,
            )
        )
        control.publish(message)
        until = time.monotonic() + 0.1
        wait(lambda: time.monotonic() > until)
        checks.append("explicit_console_authority_preserved")
        current = measured["position"].copy()
        final = current.copy()
        final[0] += 0.2
        stopping = dict(packet, request_id="mock-stop", waypoints=[current, final])
        call("joint_trajectory", stopping)
        wait(
            lambda: (
                status["joint_control"]["request_id"] == "mock-stop"
                and status["joint_control"]["phase"] == "moving"
            )
        )
        call("joint_stop", dict(request_id="another-controller"), expected=False)
        call("joint_stop", dict(request_id="mock-stop"))
        wait(
            lambda: (
                status["joint_control"]["phase"] == "stopped"
                and max(abs(value) for value in measured["velocity"]) < 0.08
            )
        )
        assert status["output_enable_count"] == 1 and status["output_disable_count"] == 0
        checks.append("atomic_owned_stop_retains_enable")
        current = measured["position"].copy()
        gripper_goal = current.copy()
        gripper_goal[7] = 0.5 * (
            status["joint_control"]["lower"][7] + status["joint_control"]["upper"][7]
        )
        call(
            "joint_move",
            dict(request_id="mock-gripper", unit="rad", positions=gripper_goal, speed_rad_s=0.2),
        )
        wait(
            lambda: (
                status["joint_control"]["request_id"] == "mock-gripper"
                and status["joint_control"]["phase"] == "moving"
            )
        )
        call("joint_stop", dict(request_id="mock-stop"), expected=False)
        call("joint_stop", dict(request_id="mock-gripper"))
        wait(
            lambda: (
                status["joint_control"]["phase"] == "stopped"
                and max(abs(value) for value in measured["velocity"]) < 0.08
            )
        )
        assert status["output_enable_count"] == 1 and status["output_disable_count"] == 0
        checks.append("owned_gripper_contact_stop_retains_enable")
        report = dict(
            passed=True,
            checks=checks,
            domain=args.domain,
            mock=True,
            allow_hardware=False,
            real_motor_commands_sent=False,
            trajectory_duration_s=duration,
            intermediate_velocity_min_rad_s=min(velocities),
            intermediate_velocity_max_rad_s=max(velocities),
            final_enable_count=status["output_enable_count"],
            final_disable_count=status["output_disable_count"],
        )
        (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report))
    finally:
        (args.output / "status_history.json").write_text(json.dumps(history, indent=2) + "\n")
        (args.output / "applied_history.json").write_text(json.dumps(applied) + "\n")
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        del subscriptions


if __name__ == "__main__":
    main()
