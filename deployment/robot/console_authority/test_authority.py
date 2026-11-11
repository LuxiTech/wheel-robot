#!/usr/bin/env python3
"""Exercise real candidate nodes with mock hardware on ROS domain 97."""

import argparse
import json
import os
import signal
import socket
import subprocess
import tempfile
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("work", type=Path)
parser.add_argument("config", type=Path)
args, remaining = parser.parse_known_args()
os.environ["ROS_DOMAIN_ID"] = "97"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from luxi_interfaces.msg import ArmCommand, QuestInput
from luxi_interfaces.srv import Command
from std_msgs.msg import String


class ExplicitAuthority(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="explicit-authority-mock-")
        self.log = open(Path(self.tmp.name) / "nodes.log", "w+")
        self.node = rclpy.create_node("explicit_authority_test")
        self.prefix = "/authority_" + str(time.monotonic_ns())
        self.processes = []
        self.status = {}
        self.base = {}
        self.startup = {}
        self.arms = None
        self.commands = []
        self.lifecycle = []
        self.quest = QuestInput()
        self.quest_pub = self.node.create_publisher(QuestInput, self.prefix + "/quest", 1)
        self.lift_pub = self.node.create_publisher(String, self.prefix + "/lift/status", 1)
        for name, destination in [
            ("control_status", self.status),
            ("base/status", self.base),
            ("arms/startup_status", self.startup),
        ]:
            self.node.create_subscription(
                String,
                self.prefix + "/" + name,
                lambda message, destination=destination: destination.update(
                    json.loads(message.data)
                ),
                rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.VOLATILE),
            )
        from luxi_interfaces.msg import ArmState

        self.node.create_subscription(ArmState, self.prefix + "/arms/state", self.receive_arms, 10)
        self.node.create_subscription(
            ArmCommand, self.prefix + "/command", self.commands.append, 10
        )
        self.node.create_subscription(
            String,
            self.prefix + "/driver_lifecycle",
            lambda message: self.lifecycle.append(message.data),
            10,
        )
        self.control = self.node.create_client(Command, self.prefix + "/control")
        self.base_client = self.node.create_client(Command, self.prefix + "/base/control")
        self.arm_client = self.node.create_client(Command, self.prefix + "/arms/startup")
        self.addCleanup(self.cleanup)
        self.launch(
            "control_manager",
            ["shared_enable:=true", "explicit_control_source:=true", "config:=" + str(args.config)],
        )
        self.launch(
            "r680_adapter", ["mock:=true", "shared_enable:=true", "require_left_grip:=false"]
        )
        self.launch(
            "openarm_adapter", ["mock:=true", "monitor_only:=false", "config:=" + str(args.config)]
        )
        self.until(
            lambda: (
                self.control.service_is_ready()
                and self.base_client.service_is_ready()
                and self.arm_client.service_is_ready()
                and self.arms is not None
                and self.startup.get("fresh", True)
                and self.status.get("control_source") == "console"
                and self.base.get("keyboard_takeover_available")
                and self.base.get("stationary")
                and self.base.get("feedback_fresh")
            ),
            "default console ready",
            12,
        )

    def receive_arms(self, message):
        self.arms = message

    def launch(self, target, parameters):
        command = [str(args.work / "bin" / target), "--ros-args"]
        for parameter in parameters:
            command.extend(["-p", parameter])
        topics = [
            "quest",
            "intent",
            "command",
            "control",
            "control_status",
            "driver_lifecycle",
            "coordinated_control",
            "recorder/status",
            "base/status",
            "base/control",
            "base/state",
            "base/can_rx",
            "base/applied",
            "base/external_control",
            "lift/status",
            "lift/state",
            "arms/state",
            "arms/startup_status",
            "arms/startup",
            "arms/command",
            "applied",
            "faults",
        ]
        for topic in topics:
            command.extend(["-r", "/luxi/" + topic + ":=" + self.prefix + "/" + topic])
        for topic in ["velocity", "imu", "battery"]:
            command.extend(["-r", "/base/" + topic + ":=" + self.prefix + "/base/" + topic])
        command.extend(["-r", "/cmd_vel:=" + self.prefix + "/cmd_vel"])
        self.processes.append(
            subprocess.Popen(
                command,
                stdout=self.log,
                stderr=subprocess.STDOUT,
                env=dict(os.environ, ROS_LOG_DIR=self.tmp.name),
            )
        )

    def step(self):
        now = time.monotonic_ns()
        self.quest.receive_mono_ns = now
        self.quest_pub.publish(self.quest)
        lift = String()
        lift.data = json.dumps(
            {
                "receive_mono_ns": str(now),
                "shared_enable": True,
                "enabled": False,
                "stationary": True,
                "session_present": False,
                "feedback_healthy": True,
                "allow_motion": True,
                "phase": "disabled",
                "shared_ready": True,
                "shared_failed": False,
            }
        )
        self.lift_pub.publish(lift)
        for _ in range(12):
            rclpy.spin_once(self.node, timeout_sec=0)
        time.sleep(0.008)

    def run_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.step()

    def until(self, condition, description, timeout=4):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            self.step()
        self.assertTrue(condition(), f"{description}: {self.status}, {self.base}")

    def request(self, client, command, detail=""):
        request = Command.Request()
        request.command = command
        request.detail = detail
        future = client.call_async(request)
        self.until(future.done, command)
        return future.result()

    def test_connected_quest_and_long_x_do_not_acquire_console(self):
        self.quest.buttons[2] = True
        self.run_for(1.8)
        self.assertEqual(self.status["control_source"], "console")
        self.assertEqual(self.status["system_phase"], "disabled")
        self.assertEqual(self.commands, [])
        self.assertEqual(self.lifecycle, [])
        self.assertFalse(self.arms.enabled)
        rejected = self.request(self.control, "start")
        self.assertFalse(rejected.success)
        self.assertIn("console_control_selected", rejected.message)
        self.quest.sticks = [0.0, 0.9, 0.0, 0.0]
        self.assertTrue(self.request(self.base_client, "keyboard_start", "web-console").success)
        self.run_for(0.2)
        self.assertEqual(self.base["keyboard_owner"], "web-console")
        self.assertTrue(self.base["enabled"])
        self.assertEqual(self.commands, [])
        self.assertTrue(self.request(self.base_client, "stop", "web-console").success)

    def test_explicit_switch_is_nonmoving_and_gates_driver_commands(self):
        self.run_for(0.2)
        self.assertTrue(self.request(self.control, "select_teleop").success)
        self.until(
            lambda: (
                self.status.get("control_source") == "teleop"
                and self.base.get("control_source") == "teleop"
            ),
            "teleop permission propagated",
        )
        self.run_for(0.1)
        self.assertEqual(self.commands, [])
        self.assertEqual(self.lifecycle, [])
        self.assertFalse(self.arms.enabled)
        self.assertFalse(self.request(self.base_client, "keyboard_start", "web-blocked").success)
        rejected = self.request(self.arm_client, "joint_move", "{}")
        self.assertFalse(rejected.success)
        self.assertIn("控制台权限", rejected.message)
        started = self.request(self.control, "start")
        self.assertTrue(started.success, started.message)
        self.assertTrue(self.request(self.control, "halt").success)
        self.run_for(0.1)
        self.assertTrue(self.request(self.control, "select_console").success)
        self.until(
            lambda: (
                self.status.get("control_source") == "console"
                and self.base.get("keyboard_takeover_available")
            ),
            "console restored",
        )
        self.run_for(0.1)
        self.assertTrue(self.request(self.base_client, "keyboard_start", "web-restored").success)
        self.assertTrue(self.request(self.base_client, "stop", "web-restored").success)

    def test_switch_refuses_active_web_session(self):
        response = self.request(self.base_client, "keyboard_start", "web-hold")
        self.assertTrue(response.success, response.message)
        self.until(lambda: self.base.get("enabled"), "base enabled")
        self.run_for(0.1)
        self.assertFalse(self.request(self.control, "select_teleop").success)
        self.assertEqual(self.status["control_source"], "console")
        self.assertEqual(self.base["keyboard_owner"], "web-hold")
        self.assertTrue(self.request(self.base_client, "stop", "web-hold").success)

    def test_arm_motion_does_not_cancel_console_base_owner(self):
        self.assertTrue(self.request(self.base_client, "keyboard_start", "web-independent").success)
        detail = json.dumps(
            {
                "positions": list(self.arms.position),
                "speed_rad_s": 0.02,
                "unit": "rad",
                "request_id": "test-console-joint",
            }
        )
        response = self.request(self.arm_client, "joint_move", detail)
        self.assertTrue(response.success, response.message)
        self.run_for(0.25)
        self.assertEqual(self.status["control_source"], "console")
        self.assertEqual(self.base["keyboard_owner"], "web-independent")
        self.assertTrue(self.base["enabled"])
        self.assertEqual(self.commands, [])
        self.assertTrue(self.request(self.arm_client, "joint_stop").success)
        self.assertTrue(self.request(self.base_client, "stop", "web-independent").success)

    def test_http_enforces_same_source_for_all_actuators(self):
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
        self.launch(
            "web_ctrl",
            [
                "mock:=true",
                "host:=127.0.0.1",
                "port:=" + str(port),
                "no_cameras:=true",
                "directory:=" + self.tmp.name,
            ],
        )
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        url = "http://127.0.0.1:" + str(port)

        def source():
            try:
                with opener.open(url + "/api/state", timeout=0.5) as response:
                    return (json.load(response).get("control") or {}).get("control_source")
            except OSError:
                return None

        self.until(lambda: source() == "console", "HTTP sees console authority", 8)
        self.assertTrue(self.request(self.control, "select_teleop").success)
        self.until(lambda: source() == "teleop", "HTTP sees teleop authority")
        for path, command in [
            ("arm_startup", "joint_move"),
            ("keyboard", "keyboard_start"),
            ("lift", "position_arm"),
        ]:
            request = urllib.request.Request(
                url + "/api/" + path,
                data=json.dumps({"command": command, "detail": "{}"}).encode(),
                headers={"Content-Type": "application/json"},
            )
            with self.assertRaises(urllib.error.HTTPError) as error:
                opener.open(request, timeout=2)
            self.assertEqual(error.exception.code, 409)
            self.assertIn("控制权限", json.loads(error.exception.read())["message"])
        self.assertEqual(self.commands, [])

    def cleanup(self):
        for process in reversed(self.processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(4)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        self.log.seek(0)
        if any(process.returncode not in [0, -signal.SIGINT] for process in self.processes):
            print(self.log.read())
        self.node.destroy_node()
        self.log.close()
        self.tmp.cleanup()


if __name__ == "__main__":
    rclpy.init()
    try:
        unittest.main(argv=[__file__, *remaining], verbosity=2)
    finally:
        rclpy.shutdown()
