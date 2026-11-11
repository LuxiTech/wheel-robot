#!/usr/bin/env python3
"""Run with the SDK dashboard PTY fixture on PYTHONPATH; no hardware ports."""

import json
import time
import unittest

import rclpy
from std_msgs.msg import String
from test_lift_dashboard import DashboardPosition


class LiftAuthority(DashboardPosition):
    def setUp(self):
        self.source = "console"
        super().setUp()

    def extra_arguments(self):
        self.authority = self.node.create_publisher(String, self.prefix + "/coordinated_control", 1)
        return super().extra_arguments()

    def extra_step(self, now):
        super().extra_step(now)
        message = String()
        message.data = json.dumps(
            {
                "epoch": "test-authority",
                "phase": "halt",
                "control_source": self.source,
                "sent_mono_ns": str(time.monotonic_ns()),
            }
        )
        self.authority.publish(message)

    def test_quest_connection_does_not_prevent_console_position_control(self):
        self.run_for(0.3)
        self.assertTrue(self.status["enabled"])
        self.assertTrue(self.status["dashboard_position"])
        self.assertEqual(self.moves, [])
        self.assertTrue(self.call("position_stop").success)
        self.lease = False
        self.wait(lambda: not self.status["enabled"], 2, "console released")
        self.source = "teleop"
        self.run_for(0.2)
        response = self.call("position_arm")
        self.assertFalse(response.success)
        self.assertIn("控制台权限", response.message)
        self.source = "console"
        self.run_for(0.2)
        self.owner = "console-restored-" + str(time.monotonic_ns())
        self.seq = 0
        response = self.call("position_arm")
        self.assertTrue(response.success, response.message)
        self.lease = True
        self.assertEqual(self.moves, [])


if __name__ == "__main__":
    rclpy.init()
    try:
        suite = unittest.TestSuite(
            [LiftAuthority("test_quest_connection_does_not_prevent_console_position_control")]
        )
        result = unittest.TextTestRunner(verbosity=2).run(suite)
    finally:
        rclpy.shutdown()
    raise SystemExit(0 if result.wasSuccessful() else 1)
