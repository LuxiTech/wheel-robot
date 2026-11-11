#!/usr/bin/env python3
"""Exercise web/Quest ownership with mock adapters on an isolated ROS domain."""

import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = "97"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from luxi_interfaces.msg import QuestInput
from luxi_interfaces.srv import Command
from std_msgs.msg import String

BASE_BINARY = Path(sys.argv.pop(1)).resolve()
MANAGER_BINARY = Path(sys.argv.pop(1)).resolve()
CONFIG = Path(sys.argv.pop(1)).resolve()
WEB_LINEAR_LIMIT = float(os.environ.get("WEB_BASE_LINEAR_LIMIT", "0.05"))


class WebBaseOwnership(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="web-base-mock-")
        self.log = open(Path(self.directory.name) / "adapter.log", "w+")
        self.node = rclpy.create_node("web_base_ownership_test")
        self.prefix = "/web_base_" + str(time.monotonic_ns())
        self.status = {}
        self.applied = {}
        self.epoch = "idle-epoch"
        self.phase = "disabled"
        self.coordination_enabled = True
        self.quest = QuestInput()
        self.quest_enabled = False
        self.owner = ""
        self.velocity = [0.0, 0.0, 0.0]
        self.keepalive = True
        self.sent_at = 0.0
        self.sequence = 0
        self.pending = []
        self.processes = []
        self.coordination = self.node.create_publisher(
            String, self.prefix + "/coordinated_control", 1
        )
        self.quest_pub = self.node.create_publisher(QuestInput, self.prefix + "/quest", 1)
        self.node.create_subscription(
            String,
            self.prefix + "/base/status",
            lambda message: self.status.update(json.loads(message.data)),
            10,
        )
        self.node.create_subscription(
            String,
            self.prefix + "/base/applied",
            lambda message: self.applied.update(json.loads(message.data)),
            10,
        )
        self.client = self.node.create_client(Command, self.prefix + "/base/control")
        args = [
            str(BASE_BINARY),
            "--ros-args",
            "-p",
            "mock:=true",
            "-p",
            "shared_enable:=true",
            "-p",
            "require_left_grip:=false",
            "-p",
            "max_linear:=0.5",
            "-p",
            f"web_keyboard_linear_limit:={WEB_LINEAR_LIMIT}",
        ]
        for topic in [
            "quest",
            "recorder/status",
            "base/state",
            "base/can_rx",
            "base/applied",
            "base/status",
            "base/control",
            "coordinated_control",
            "base/external_control",
        ]:
            args.extend(["-r", f"/luxi/{topic}:={self.prefix}/{topic}"])
        for topic in ["velocity", "imu", "battery"]:
            args.extend(["-r", f"/base/{topic}:={self.prefix}/{topic}"])
        args.extend(["-r", f"/cmd_vel:={self.prefix}/cmd_vel"])
        self.processes.append(
            subprocess.Popen(
                args,
                env=dict(os.environ, ROS_LOG_DIR=self.directory.name),
                stdout=self.log,
                stderr=subprocess.STDOUT,
            )
        )
        self.until(
            lambda: (
                self.client.service_is_ready()
                and self.status.get("keyboard_takeover_available")
                and self.status.get("stationary")
            ),
            "mock adapter ready without Quest",
            6,
        )
        self.assertEqual(self.status.get("web_keyboard_command_timeout_ms"), 500)
        self.assertTrue(self.status.get("web_keyboard_manual_release"))

    def step(self):
        coordination = String()
        coordination.data = json.dumps(
            {"epoch": self.epoch, "phase": self.phase, "sent_mono_ns": str(time.monotonic_ns())}
        )
        if self.coordination_enabled:
            self.coordination.publish(coordination)
        if self.quest_enabled:
            self.quest.receive_mono_ns = time.monotonic_ns()
            self.quest_pub.publish(self.quest)
        if self.owner and self.keepalive and time.monotonic() - self.sent_at >= 0.04:
            request = Command.Request()
            request.command = "keyboard"
            self.sequence += 1
            request.detail = json.dumps(
                {"owner": self.owner, "sequence": self.sequence, "velocity": self.velocity}
            )
            self.pending.append(self.client.call_async(request))
            self.sent_at = time.monotonic()
        self.pending = [future for future in self.pending if not future.done()]
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0)
        time.sleep(0.006)

    def run_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.step()

    def until(self, condition, description, seconds=2):
        deadline = time.monotonic() + seconds
        while not condition() and time.monotonic() < deadline:
            self.step()
        self.assertTrue(condition(), f"{description}: {self.status}, {self.applied}")

    def request(self, command, detail=""):
        request = Command.Request()
        request.command = command
        request.detail = detail
        future = self.client.call_async(request)
        self.until(future.done, "service response")
        return future.result()

    def acquire(self, owner):
        self.sequence = 0
        response = self.request("keyboard_start", owner)
        self.assertTrue(response.success, response.message)
        self.owner = owner
        self.until(lambda: self.status.get("keyboard_owner") == owner, "web owner visible")

    def release(self):
        owner = self.owner
        self.owner = ""
        self.assertTrue(self.request("stop", owner).success)
        self.until(lambda: not self.status.get("enabled"), "released and stopped")
        self.until(lambda: self.applied.get("velocity") == [0.0, 0.0, 0.0], "zero output")

    def test_web_without_quest_is_limited_and_scoped(self):
        self.acquire("web-first")
        self.velocity = [1.0, 1.0, 0.0]
        self.until(
            lambda: self.applied.get("velocity", [0.0])[0] > 0.02,
            "WASD produces mock velocity",
        )
        self.assertLessEqual(
            sum(value * value for value in self.applied["velocity"][:2]) ** 0.5,
            WEB_LINEAR_LIMIT + 0.00001,
        )
        self.assertFalse(self.request("keyboard_start", "web-other").success)
        self.release()
        self.until(lambda: self.status.get("stationary"), "measured stopped feedback")
        self.acquire("web-second")
        self.assertTrue(self.request("stop", "web-first").success)
        self.run_for(0.15)
        self.assertTrue(self.status["enabled"], "stale page cannot stop the new web owner")
        self.assertFalse(self.request("keyboard_start", "web-first").success)
        self.release()

    def test_selected_speeds_reach_mock_output(self):
        self.assertAlmostEqual(self.status["web_keyboard_linear_limit"], WEB_LINEAR_LIMIT)
        self.acquire("web-speed-selector")
        for ratio in [0.1, 0.2, 0.6, 1.0]:
            selected_speed = WEB_LINEAR_LIMIT * ratio
            self.velocity = [ratio, 0.0, 0.0]
            self.until(
                lambda: abs(self.applied.get("velocity", [0.0])[0] - selected_speed) < 0.001,
                f"selected speed {selected_speed} reaches simulated base",
            )
            self.assertLessEqual(abs(self.applied["velocity"][0]), WEB_LINEAR_LIMIT + 0.00001)
        self.release()

    def test_quest_input_cannot_steal_web_and_stale_web_cannot_stop_quest(self):
        self.acquire("web-mutex")
        self.quest_enabled = True
        self.quest.sticks = [0.0, 1.0, 0.0, 0.0]
        self.run_for(0.2)
        self.assertEqual(self.status["input_source"], "keyboard")
        self.assertEqual(self.applied["velocity"], [0.0, 0.0, 0.0])
        self.epoch = "rejected-quest-epoch"
        self.phase = "prepare"
        self.run_for(0.15)
        self.phase = "active"
        self.run_for(0.15)
        self.assertTrue(self.status["shared_failed"])
        self.assertEqual(self.status["input_source"], "keyboard")
        self.assertEqual(self.applied["velocity"], [0.0, 0.0, 0.0])
        self.release()
        self.run_for(0.15)
        self.assertFalse(self.status["enabled"], "old active heartbeat cannot resume Quest")
        self.quest.sticks = [0.0] * 4
        self.epoch = "fresh-explicit-quest-epoch"
        self.phase = "prepare"
        self.until(lambda: self.status.get("shared_ready"), "fresh neutral Quest ready")
        self.phase = "active"
        self.until(lambda: self.status.get("enabled"), "Quest explicitly enabled")
        self.assertFalse(self.request("keyboard_start", "web-during-quest").success)
        self.assertTrue(self.request("stop", "web-mutex").success)
        self.run_for(0.12)
        self.assertTrue(self.status["enabled"], "old web release cannot stop active Quest")
        self.assertEqual(self.status["input_source"], "quest")

    def test_watchdog_stops_but_retains_manual_ownership(self):
        self.acquire("web-timeout")
        self.velocity = [1.0, 0.0, 0.0]
        self.until(lambda: self.applied.get("velocity", [0.0])[0] > 0.02, "mock motion")
        self.keepalive = False
        self.run_for(0.3)
        self.assertFalse(self.status["web_keyboard_paused"], "300 ms is within the new deadline")
        self.until(lambda: self.status.get("web_keyboard_paused"), "500 ms watchdog stops motion")
        self.assertTrue(self.status["enabled"])
        self.assertEqual(self.status["keyboard_owner"], "web-timeout")
        self.assertEqual(self.applied["velocity"], [0.0, 0.0, 0.0])
        self.run_for(0.6)
        self.assertEqual(self.status["keyboard_owner"], "web-timeout", "long gaps retain ownership")
        self.keepalive = True
        self.run_for(0.15)
        self.assertTrue(self.status["web_keyboard_paused"])
        self.assertEqual(
            self.applied["velocity"], [0.0, 0.0, 0.0], "held keys cannot replay motion"
        )
        self.velocity = [0.0, 0.0, 0.0]
        self.until(
            lambda: not self.status.get("web_keyboard_paused"), "fresh neutral input clears pause"
        )
        self.velocity = [1.0, 0.0, 0.0]
        self.until(lambda: self.applied.get("velocity", [0.0])[0] > 0.02, "new press resumes")
        self.release()

    def test_stale_sequence_cannot_replay_a_previous_movement(self):
        self.acquire("web-sequence")
        self.run_for(0.2)
        self.keepalive = False
        response = self.request(
            "keyboard",
            json.dumps({"owner": self.owner, "sequence": 1, "velocity": [1, 0, 0]}),
        )
        self.assertFalse(response.success)
        self.assertIn("sequence", response.message)
        self.assertTrue(self.status["enabled"])
        self.assertEqual(self.applied["velocity"], [0.0, 0.0, 0.0])

    def test_web_yaw_turns_without_translation(self):
        self.acquire("web-turn")
        for direction in [1.0, -1.0]:
            self.velocity = [0.0, 0.0, direction]
            self.until(
                lambda: self.applied.get("velocity", [0.0] * 3)[2] * direction > 0.1,
                "pure yaw command",
            )
            self.assertEqual(self.applied["velocity"][:2], [0.0, 0.0])
            self.assertLessEqual(abs(self.applied["velocity"][2]), 0.30001)
            self.velocity = [0.0, 0.0, 0.0]
            self.until(
                lambda: self.applied.get("velocity") == [0.0, 0.0, 0.0],
                "released turn stops",
            )
        self.release()

    def test_operator_release_cleans_an_abandoned_page(self):
        self.acquire("web-abandoned")
        self.keepalive = False
        self.until(lambda: self.status.get("web_keyboard_paused"), "abandoned page stops")
        self.assertTrue(self.request("stop", "__web_operator_release__").success)
        self.owner = ""
        self.until(lambda: not self.status.get("enabled"), "operator releases abandoned owner")
        self.assertEqual(self.status["keyboard_owner"], "")

    def test_coordinator_gap_pauses_without_giving_up_web_owner(self):
        self.acquire("web-coordinator-gap")
        self.coordination_enabled = False
        self.until(lambda: self.status.get("web_keyboard_paused"), "coordination gap stops motion")
        self.assertEqual(self.status["keyboard_owner"], "web-coordinator-gap")
        self.assertTrue(self.status["enabled"])
        self.assertEqual(self.applied["velocity"], [0.0, 0.0, 0.0])
        self.coordination_enabled = True
        self.until(
            lambda: not self.status.get("web_keyboard_paused"), "neutral fresh input restores ready"
        )

    def test_quest_b_still_stops_web(self):
        self.acquire("web-b-stop")
        self.quest_enabled = True
        self.quest.buttons[1] = True
        self.until(lambda: not self.status.get("enabled"), "B stops web control")
        self.assertEqual(self.applied["velocity"], [0.0, 0.0, 0.0])

    def test_global_halt_stops_web_and_idle_heartbeats_do_not_rearm(self):
        self.acquire("web-global-stop")
        self.phase = "halt"
        self.until(lambda: not self.status.get("enabled"), "global stop releases web")
        self.owner = ""
        self.assertEqual(self.status["keyboard_owner"], "")
        self.until(
            lambda: self.status.get("keyboard_takeover_available"), "stopped coordinator idle"
        )
        self.acquire("web-after-stop")
        self.run_for(0.3)
        self.assertTrue(self.status["enabled"], "repeated idle halt heartbeat cannot cancel web")

    def test_manager_rejects_quest_start_while_web_owns_base(self):
        self.acquire("web-manager")
        manager_prefix = self.prefix + "/manager"
        args = [
            str(MANAGER_BINARY),
            "--ros-args",
            "-p",
            "shared_enable:=true",
            "-p",
            "config:=" + str(CONFIG),
        ]
        for topic in [
            "base/status",
            "lift/status",
            "arms/state",
            "arms/startup_status",
            "arms/startup",
            "arms/command",
            "quest",
            "intent",
            "control",
            "control_status",
            "driver_lifecycle",
            "coordinated_control",
            "recorder/status",
        ]:
            args.extend(["-r", f"/luxi/{topic}:={manager_prefix}/{topic}"])
        self.processes.append(subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT))
        status_pub = self.node.create_publisher(String, manager_prefix + "/base/status", 1)
        client = self.node.create_client(Command, manager_prefix + "/control")
        self.until(client.service_is_ready, "manager service ready", 6)
        initial_request = Command.Request()
        initial_request.command = "start"
        initial_future = client.call_async(initial_request)
        self.until(initial_future.done, "start without any base status is rejected")
        self.assertFalse(initial_future.result().success)
        for _ in range(30):
            message = String()
            message.data = json.dumps(self.status)
            status_pub.publish(message)
            self.step()
        request = Command.Request()
        request.command = "start"
        future = client.call_async(request)
        self.until(future.done, "manager start rejection")
        self.assertFalse(future.result().success)
        self.assertIn("web_base_control_active", future.result().message)
        self.assertTrue(self.status["enabled"])

    def tearDown(self):
        for process in reversed(self.processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        self.node.destroy_node()
        self.log.close()
        self.directory.cleanup()


if __name__ == "__main__":
    rclpy.init()
    try:
        unittest.main(verbosity=2)
    finally:
        rclpy.shutdown()
