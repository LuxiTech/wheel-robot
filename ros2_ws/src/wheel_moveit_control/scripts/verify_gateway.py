#!/usr/bin/env python3
"""ROS-message/lifecycle regressions in an isolated domain; never activate hardware."""

import argparse
import json
import os
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

import rclpy
from luxi_interfaces.msg import ArmState
from planning_gateway import Gateway


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config-dir", type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "223":
        raise RuntimeError("Use isolated ROS_DOMAIN_ID=223")
    if (
        json.loads((args.config_dir / "context.json").read_text())["hardware"]
        != "mock_components/GenericSystem"
    ):
        raise RuntimeError("Mock configuration required")
    rclpy.init()
    node = Gateway(args.config_dir)
    try:
        # Exercise real SDK conversion using actual ROS array.array setters.
        node.mock = False
        messages = []
        node.publisher = SimpleNamespace(publish=messages.append)
        state = ArmState()
        state.position = [0.0] * 16
        state.velocity = [0.0] * 16
        state.effort = [0.0] * 16
        state.valid = [True] * 16
        state.receive_mono_ns = time.monotonic_ns()
        node.sdk_feedback(state)
        node.publish_passive()
        assert len(messages[-1].position) == 14 + len(node.passive)
        assert set(node.names).issubset(messages[-1].name)
        original_stamp = node.latest_at
        state.receive_mono_ns -= 1_000_000_000
        node.sdk_feedback(state)
        assert node.latest_at == original_stamp, "Stale SDK data renewed freshness"

        # Cancellation during hardware activation must never submit a trajectory.
        node.mock = True
        calls = []

        def hardware(active):
            calls.append(active)
            if active:
                node.cancel_requested.set()

        node.hardware = hardware
        node.check_lift = lambda height: None
        node.action = SimpleNamespace(wait_for_server=lambda **kwargs: True)
        trajectory = SimpleNamespace(
            joint_names=node.names, points=[SimpleNamespace(positions=[0.0] * 14)]
        )
        with tempfile.TemporaryDirectory() as directory:
            node.plans["cancel-test"] = (
                trajectory,
                node.state(),
                Path(directory),
                {"height_um": 0},
            )
            try:
                node.execute({"plan_id": "cancel-test"})
            except RuntimeError as error:
                assert "cancelled before" in str(error), str(error)
            else:
                raise AssertionError("Cancelled plan was executed")
            assert calls == [True, False]
            assert node.executing_plan is None and node.active is None
        print(
            json.dumps(
                dict(
                    passed=True,
                    checks=[
                        "typed_sdk_state_conversion",
                        "stale_feedback_not_renewed",
                        "cancel_during_activation_no_goal",
                    ],
                    real_motor_commands_sent=False,
                )
            )
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
