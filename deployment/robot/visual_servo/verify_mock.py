#!/usr/bin/env python3
"""Actual SDK IK + adapter streaming regression, isolated mock motors only."""

import argparse
import copy
import json
import os
import subprocess
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--ik-config", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--domain", type=int, default=217)
    args = parser.parse_args()
    if not 150 <= args.domain <= 225 or "candidate" not in str(args.candidate):
        raise ValueError("Independent candidate and isolated ROS domain required")
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            command = (process / "cmdline").read_bytes()
            if b"openarm_adapter" in command and b"mock:=false" in command:
                values = dict(
                    row.split(b"=", 1)
                    for row in (process / "environ").read_bytes().split(b"\0")
                    if b"=" in row
                )
                if int(values.get(b"ROS_DOMAIN_ID", b"0")) == args.domain:
                    raise RuntimeError("Domain is occupied by a real motor adapter")
        except (PermissionError, ProcessLookupError, FileNotFoundError):
            continue
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    import rclpy
    from luxi_interfaces.msg import ArmState
    from luxi_interfaces.srv import Command
    from std_msgs.msg import String

    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node("wheel_visual_mock_verifier")
    status, measured, history = {}, {}, []
    processes, logs = [], []

    def on_status(message):
        status.clear()
        status.update(json.loads(message.data))
        history.append(dict(at=time.monotonic(), control=dict(status["joint_control"])))

    def on_state(message):
        measured["position"] = list(message.position)
        measured["velocity"] = list(message.velocity)

    subscriptions = [
        node.create_subscription(String, "/luxi/arms/startup_status", on_status, 100),
        node.create_subscription(ArmState, "/luxi/arms/state", on_state, 100),
    ]
    control = node.create_publisher(String, "/luxi/control_status", 10)
    authority = ["console"]

    def publish_authority():
        message = String()
        message.data = json.dumps(
            dict(
                control_source=authority[0],
                phase="disabled",
                system_phase="disabled",
                shared_enable=True,
            )
        )
        control.publish(message)

    timer = node.create_timer(0.05, publish_authority)
    client = node.create_client(Command, "/luxi/arms/startup")

    def launch(binary, name, parameters):
        log = (args.output / (name + ".log")).open("w")
        command = [str(args.candidate / "build" / binary), "--ros-args", "-r", "__node:=" + name]
        for key, value in parameters.items():
            command.extend(["-p", key + ":=" + str(value)])
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        processes.append(process)
        logs.append(log)
        return process

    def wait(predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if any(process.poll() is not None for process in processes):
                raise RuntimeError("Candidate process exited; inspect logs")
            rclpy.spin_once(node, timeout_sec=0.005)
            if predicate():
                return
        raise TimeoutError("Mock condition timed out: " + str(status.get("joint_control")))

    def call(command, detail, expected=True):
        request = Command.Request()
        request.command, request.detail = command, json.dumps(detail)
        future = client.call_async(request)
        wait(future.done)
        response = future.result()
        assert response.success is expected, (command, detail, response.message)

    checks = []
    try:
        launch(
            "openarm_adapter_visual_candidate",
            "wheel_visual_mock_adapter",
            dict(config=args.config, mock="true", allow_hardware="false"),
        )
        launch(
            "wheel_visual_ik_candidate",
            "wheel_visual_mock_ik",
            dict(config=args.ik_config, model=args.model),
        )
        wait(lambda: client.service_is_ready() and bool(measured) and bool(status))
        current = measured["position"]
        call(
            "joint_move",
            dict(request_id="mock-enable", unit="rad", positions=current, speed_rad_s=0.2),
        )
        wait(lambda: status["joint_control"]["phase"] == "ready")
        assert status["joint_control"]["visual_servo_protocol"] == 1
        checks.append("explicit_mock_enable_only")
        worker = args.candidate / "build/wheel_visual_ik_candidate"
        poses = json.loads(
            subprocess.check_output(
                [
                    str(worker),
                    "--fk",
                    str(args.ik_config),
                    str(args.model),
                    json.dumps(measured["position"]),
                ],
                text=True,
            )
        )
        goal = copy.deepcopy(poses)
        for side in goal:
            goal[side][2][3] += 0.015
        call("visual_servo_start", dict(request_id="visual-one", speed_rad_s=0.2))
        wait(lambda: status["joint_control"]["phase"] == "servoing")
        generation = status["control_generation"]
        start = list(measured["position"])
        packet = None
        for sequence in range(1, 21):
            packet = dict(
                request_id="visual-one",
                sequence=sequence,
                frame_id="arm_origin",
                targets=goal,
                pair_id="mock-image-" + str(sequence),
                image_receive_mono_ns=time.monotonic_ns(),
            )
            call("visual_servo_update", packet)
            until = time.monotonic() + 0.06
            wait(lambda: time.monotonic() >= until)
            assert status["joint_control"]["phase"] == "servoing"
            assert status["control_generation"] == generation
        assert max(abs(a - b) for a, b in zip(start, measured["position"])) > 0.001
        assert status["joint_control"]["visual_intent_mono_ns"] > 0
        checks.append("real_sdk_ik_continuously_tracks_distinct_visual_frames")
        call("visual_servo_update", packet, expected=False)
        invalid = copy.deepcopy(packet)
        invalid.update(sequence=21, pair_id="invalid", image_receive_mono_ns=time.monotonic_ns())
        invalid["targets"]["left"][0][0] = 4
        call("visual_servo_update", invalid, expected=False)
        assert status["control_generation"] == generation
        checks.append("repeated_frames_and_nonrigid_targets_rejected")
        call("joint_stop", dict(request_id="not-owner"), expected=False)
        call("joint_stop", dict(request_id="visual-one"))
        wait(lambda: status["joint_control"]["phase"] == "stopped")
        checks.append("atomic_owned_stop_retains_hold")
        call("visual_servo_start", dict(request_id="visual-expire", speed_rad_s=0.2))
        wait(
            lambda: (
                status["joint_control"]["phase"] == "stopped"
                and status["joint_control"]["reason"] == "visual_target_timeout"
            )
        )
        call(
            "visual_servo_update",
            dict(packet, request_id="visual-expire", sequence=30),
            expected=False,
        )
        checks.append("image_timeout_holds_without_automatic_resume")
        authority[0] = "teleop"
        until = time.monotonic() + 0.1
        wait(lambda: time.monotonic() > until)
        call("visual_servo_start", dict(request_id="wrong-authority"), expected=False)
        checks.append("console_and_teleop_ownership_isolation")
        assert status["output_enable_count"] == 1 and status["output_disable_count"] == 0
        report = dict(
            passed=True,
            checks=checks,
            mock=True,
            allow_hardware=False,
            real_motor_commands_sent=False,
            domain=args.domain,
            source_ik_used=True,
            enable_count=status["output_enable_count"],
            disable_count=status["output_disable_count"],
        )
        (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report))
    finally:
        (args.output / "history.json").write_text(json.dumps(history) + "\n")
        for process in processes:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
        del subscriptions, timer


if __name__ == "__main__":
    main()
