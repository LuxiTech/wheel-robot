#!/usr/bin/env python3
"""Run the deployed carrier against an enforcing HTTP SDK stub, never hardware."""

import importlib.util
import json
import os
import time
import unittest

spec = importlib.util.spec_from_file_location("legacy", os.environ["CARRIER_LEGACY_TEST"])
legacy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(legacy)


class ScopedCarrierTests(unittest.TestCase):
    setUp = legacy.CarrierTests.setUp
    tearDown = legacy.CarrierTests.tearDown
    send = legacy.CarrierTests.send

    def configure(self):
        self.shared = True
        self.protocol = True
        self.source = "console"
        self.available = True
        self.external = False
        self.feedback = True
        self.sdk_owner = ""
        self.sequence = 0
        self.reject_start = False
        self.speed_limit = 0.05
        owner = self

        def do_get(handler):
            handler.reply(
                {
                    "base": {"valid": owner.feedback, "flag_stop": 0},
                    "base_status": {
                        "fresh": True,
                        "feedback_fresh": owner.feedback,
                        "enabled": owner.enabled,
                        "shared_enable": owner.shared,
                        "external_control": owner.external,
                        "web_keyboard_control": True,
                        "keyboard_takeover_available": owner.available,
                        "web_keyboard_linear_limit": owner.speed_limit,
                        "control_source": owner.source,
                    },
                    "system": {"services": {"base": True}},
                    "scoped_web_keyboard": owner.protocol,
                    "navigation_cmd_vel": False,
                }
            )

        def do_post(handler):
            value = json.loads(handler.rfile.read(int(handler.headers["Content-Length"])))
            owner.events.append(value)
            command = value["command"]
            detail = value.get("detail", "")
            success = False
            if command == "keyboard_start":
                success = bool(detail) and not owner.sdk_owner and not owner.reject_start
                if success:
                    owner.sdk_owner = detail
                    owner.sequence = 0
                    owner.enabled = True
            elif command == "keyboard":
                detail = json.loads(detail)
                success = (
                    detail["owner"] == owner.sdk_owner
                    and isinstance(detail["sequence"], int)
                    and detail["sequence"] > owner.sequence
                    and len(detail["velocity"]) == 3
                    and all(abs(value) <= 1 for value in detail["velocity"])
                )
                if success:
                    owner.sequence = detail["sequence"]
            elif command == "stop":
                success = bool(detail)
                if detail == owner.sdk_owner:
                    owner.sdk_owner = ""
                    owner.enabled = False
            handler.reply({"success": success, "message": "mock scoped SDK"})

        self.server.RequestHandlerClass.do_GET = do_get
        self.server.RequestHandlerClass.do_POST = do_post

    def test_scoped_session_limits_sequences_and_release(self):
        self.configure()
        self.assertEqual(self.send("acquire", client_id="browser")[0], 200)
        first_owner = self.sdk_owner
        self.assertTrue(first_owner.startswith("atlas:"))
        self.assertEqual(self.send("status")[1]["linear_limit"], 0.05)
        self.assertEqual(
            self.send("move", client_id="browser", linear_x=0.051, command_seq=1)[0], 400
        )
        self.assertEqual(
            self.send("move", client_id="browser", linear_x=0.05, command_seq=2)[0], 200
        )
        time.sleep(0.13)
        commands = [
            json.loads(event["detail"]) for event in self.events if event["command"] == "keyboard"
        ]
        self.assertTrue(any(command["velocity"] == [1, 0, 0] for command in commands))
        self.assertTrue(all(command["owner"] == first_owner for command in commands))
        sequences = [command["sequence"] for command in commands]
        self.assertEqual(sequences, sorted(set(sequences)))
        self.assertEqual(self.send("release", client_id="browser")[0], 200)
        self.assertEqual(self.events[-1], {"command": "stop", "detail": first_owner})
        self.assertFalse(self.enabled)
        self.assertEqual(self.send("acquire", client_id="browser")[0], 200)
        self.assertNotEqual(self.sdk_owner, first_owner)

    def test_permissions_and_old_relay_still_block_acquisition(self):
        self.configure()
        for attribute, value in [
            ("source", "teleop"),
            ("external", True),
            ("available", False),
            ("protocol", False),
            ("feedback", False),
        ]:
            original = getattr(self, attribute)
            setattr(self, attribute, value)
            self.assertEqual(self.send("acquire", client_id="browser")[0], 503)
            self.assertFalse(self.events)
            setattr(self, attribute, original)

    def test_console_speed_selection_uses_sdk_manual_range(self):
        self.configure()
        self.speed_limit = 0.5
        self.assertEqual(self.send("acquire", client_id="browser")[0], 200)
        self.assertEqual(self.send("status")[1]["linear_limit"], 0.5)
        for sequence, speed in enumerate([0.05, 0.1, 0.3, 0.5], 1):
            self.assertEqual(
                self.send("move", client_id="browser", linear_x=speed, command_seq=sequence)[0],
                200,
            )
            time.sleep(0.08)
            commands = [
                json.loads(event["detail"])
                for event in self.events
                if event["command"] == "keyboard"
            ]
            self.assertAlmostEqual(commands[-1]["velocity"][0] * self.speed_limit, speed)
        self.assertEqual(
            self.send("move", client_id="browser", linear_x=0.501, command_seq=5)[0], 400
        )
        self.assertEqual(self.send("release", client_id="browser")[0], 200)

    def test_timeout_zeros_velocity_then_releases_and_never_rearms(self):
        self.configure()
        self.send("acquire", client_id="browser")
        self.send("move", client_id="browser", linear_y=0.05, command_seq=1)
        for _ in range(5):
            time.sleep(0.07)
            self.send("heartbeat", client_id="browser")
        commands = [
            json.loads(event["detail"]) for event in self.events if event["command"] == "keyboard"
        ]
        self.assertEqual(commands[-1]["velocity"], [0, 0, 0])
        time.sleep(1.1)
        self.assertFalse(self.send("status")[1]["armed"])
        self.assertFalse(self.enabled)
        self.assertEqual(self.send("heartbeat", client_id="browser")[0], 409)
        self.assertEqual(sum(event["command"] == "keyboard_start" for event in self.events), 1)

    def test_rejected_start_uses_scoped_cleanup(self):
        self.configure()
        self.reject_start = True
        self.assertEqual(self.send("acquire", client_id="browser")[0], 503)
        self.assertTrue(self.events[-1]["detail"].startswith("atlas:"))
        self.assertEqual(self.events[-1]["command"], "stop")
        self.assertFalse(self.send("status")[1]["armed"])


if __name__ == "__main__":
    unittest.main()
