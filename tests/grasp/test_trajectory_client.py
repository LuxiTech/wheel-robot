import copy
import json
import subprocess
from pathlib import Path

import pytest
from test_joint_client import Transport, client
from wheel_sdk_bridge.http_client import SDKError
from wheel_sdk_bridge.trajectory_client import JointTrajectoryClient

ROOT = Path(__file__).resolve().parents[2]


def connected():
    transport = Transport()
    transport.s["arm_startup"]["joint_control"]["trajectory_protocol"] = 1
    transport.s["arm_startup"]["nominal_target"] = [0.0] * 16
    return transport, JointTrajectoryClient(client(transport))


def test_complete_phase_is_one_request_and_preserves_gripper_hold():
    transport, trajectory = connected()
    transport.s["arm_startup"]["nominal_target"][7] = -0.4
    path = [[index * 0.001] + [0.0] * 15 for index in range(121)]
    original = copy.deepcopy(path)
    result = trajectory.submit(path)
    assert result["accepted"] and not result["completed"]
    assert len(transport.posts) == 1
    endpoint, payload = transport.posts[0]
    assert endpoint == "/api/arm_startup"
    assert payload["command"] == "joint_trajectory"
    detail = json.loads(payload["detail"])
    assert len(detail["waypoints"]) == 121
    assert all(point[7] == -0.4 for point in detail["waypoints"])
    assert detail["speed_rad_s"] == 0.2
    assert path == original


def test_legacy_sdk_is_rejected_before_any_post():
    transport, trajectory = connected()
    del transport.s["arm_startup"]["joint_control"]["trajectory_protocol"]
    with pytest.raises(SDKError, match="not installed"):
        trajectory.submit([[0.0] * 16, [0.1] * 16])
    assert not transport.posts


@pytest.mark.parametrize("failure", ["limits", "nan", "start", "disabled", "moving", "stale"])
def test_invalid_or_changed_phase_never_submits(failure):
    transport, trajectory = connected()
    path = [[0.0] * 16, [0.1] * 16]
    if failure == "limits":
        path[-1][2] = 2.01
    elif failure == "nan":
        path[-1][3] = float("nan")
    elif failure == "start":
        path[0][1] = 0.02
    elif failure == "disabled":
        transport.s["arms"]["enabled"] = False
    elif failure == "moving":
        transport.s["arms"]["velocity"][2] = 0.09
    elif failure == "stale":
        transport.s["arms"]["age_ns"][1] = 500000000
    with pytest.raises((SDKError, ValueError)):
        trajectory.submit(path)
    assert not transport.posts


def test_uncertain_post_is_not_retried_and_keeps_pending_identity():
    transport, trajectory = connected()
    transport.failure = SDKError("lost acknowledgement", uncertain=True)
    with pytest.raises(SDKError):
        trajectory.submit([[0.0] * 16, [0.1] * 16])
    assert trajectory.joints.pending_id
    with pytest.raises(SDKError, match="reconciled"):
        trajectory.submit([[0.0] * 16, [0.1] * 16])
    assert len(transport.posts) == 1


def test_unreached_endpoint_is_not_completion_and_stop_is_request_scoped():
    transport, trajectory = connected()
    request = trajectory.submit([[0.0] * 16, [0.1] * 16])["request_id"]
    joint = transport.s["arm_startup"]["joint_control"]
    joint.update(request_id=request, phase="moving")
    trajectory.stop(request)
    assert transport.posts[-1][1]["command"] == "joint_stop"
    assert json.loads(transport.posts[-1][1]["detail"])["request_id"] == request
    joint["phase"] = "needs_correction"
    result = trajectory.result(request)
    assert result["terminal"] and not result["completed"]
    assert trajectory.joints.pending_id is None
    count = len(transport.posts)
    with pytest.raises(SDKError, match="unowned"):
        trajectory.stop(request)
    assert len(transport.posts) == count


def test_cpp_path_actually_passes_waypoints_without_stopping_and_respects_bounds(tmp_path):
    directory = ROOT / "deployment/robot/joint_trajectory"
    binary = tmp_path / "path_test"
    subprocess.run(
        [
            "g++",
            "-std=c++17",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            str(directory / "test_joint_path.cpp"),
            "-o",
            str(binary),
        ],
        check=True,
    )
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
    assert "velocity/acceleration" in result.stdout


def test_trajectory_stop_does_not_require_valid_feedback():
    transport, trajectory = connected()
    identity = trajectory.submit([[0.0] * 16, [0.1] * 16])["request_id"]
    transport.s["arms"]["position"][13] = -2.01
    with pytest.raises(SDKError, match="outside SDK limits"):
        trajectory.joints.snapshot()
    trajectory.stop(identity)
    assert json.loads(transport.posts[-1][1]["detail"]) == {"request_id": identity}
    with pytest.raises(SDKError, match="unowned"):
        trajectory.stop("")
