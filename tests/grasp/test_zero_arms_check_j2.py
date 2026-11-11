import copy
import importlib.util
import json
from pathlib import Path

import pytest
from wheel_sdk_bridge.http_client import SDKError

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "zero_arms_check_j2", ROOT / "tools/grasp/zero_arms_check_j2.py"
)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


def joint_state(angle=0.0, request_id="zero-request", phase="ready"):
    return {
        "arms": {"position": [angle] * 16, "velocity": [0.0] * 16, "enabled": True},
        "arm_startup": {
            "manual_pose": {"fault_latched": False},
            "joint_control": {
                "request_id": request_id,
                "phase": phase,
                "target": [0.0] * 16,
                "reason": "test",
            },
        },
    }


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Joint:
    def __init__(self, state=None):
        self.state = state or joint_state()
        self.pending_id = "zero-request"
        self.moves = []
        self.stops = []

    def snapshot(self):
        return copy.deepcopy(self.state)

    def prepare(self, target, speed):
        return {"positions": target, "speed_rad_s": speed}

    def move(self, target, speed, *, enable_if_needed):
        self.moves.append((target, speed, enable_if_needed))
        return {"request_id": "zero-request", "accepted": True}

    def stop(self, request_id):
        self.stops.append(request_id)
        return {"success": True}


def wait(client, tmp_path, timeout=2.0):
    clock = Clock()
    return tool.wait_for_zero(
        client,
        "zero-request",
        output=tmp_path,
        timeout=timeout,
        clock=clock,
        sleep=clock.sleep,
    )


def test_preview_never_sends_a_command(monkeypatch, tmp_path):
    client = Joint(joint_state(0.5))
    client.pending_id = None
    monkeypatch.setattr(tool, "JointClient", lambda *_: client)
    output = tmp_path / "preview"
    monkeypatch.setattr(tool.sys, "argv", ["zero", "--output", str(output)])
    assert tool.main() == 0
    assert not client.moves and not client.stops
    report = json.loads((output / "report.json").read_text())
    assert report["commands_sent"] is False
    assert report["lift_commands_sent"] is False


def test_execute_sends_exact_zero_once_and_explicit_enable(monkeypatch, tmp_path):
    client = Joint()
    monkeypatch.setattr(tool, "JointClient", lambda *_: client)
    monkeypatch.setattr(tool, "wait_for_zero", lambda *_args, **_kwargs: joint_state())
    output = tmp_path / "execute"
    monkeypatch.setattr(tool.sys, "argv", ["zero", "--execute", "--output", str(output)])
    assert tool.main() == 0
    assert client.moves == [([0.0] * 16, 0.04, True)]
    assert not client.stops
    report = json.loads((output / "report.json").read_text())
    assert report["status"] == "zero_target_reached"
    assert report["lift_commands_sent"] is False
    assert report["encoder_zero_changed"] is False
    assert all(path.suffix in {".json", ".jsonl"} for path in output.iterdir())


def test_sdk_ready_with_nonzero_measurements_is_not_arrival(tmp_path):
    with pytest.raises(TimeoutError):
        wait(Joint(joint_state(0.15)), tmp_path)
    assert not (tmp_path / "arrived_joint_state.json").exists()


def test_measured_zero_must_remain_settled(tmp_path):
    client = Joint(joint_state(0.003))
    state = wait(client, tmp_path)
    assert state["arms"]["position"] == [0.003] * 16
    assert client.pending_id is None
    assert (tmp_path / "arrived_joint_state.json").exists()


@pytest.mark.parametrize("phase", ["fault", "stopped"])
def test_sdk_failure_is_not_retried(tmp_path, phase):
    client = Joint(joint_state(phase=phase))
    with pytest.raises(RuntimeError, match="SDK " + phase):
        wait(client, tmp_path)
    assert not client.moves


def test_lost_owner_is_not_stopped_or_overwritten(tmp_path):
    client = Joint(joint_state(request_id="someone-else"))
    with pytest.raises(RuntimeError, match="superseded"):
        wait(client, tmp_path)
    tool.stop_owned_request(client, tmp_path)
    assert not client.stops and not client.moves


def test_uncertain_ack_is_reconciled_without_retry(monkeypatch, tmp_path):
    client = Joint()

    def lost_ack(target, speed, *, enable_if_needed):
        client.moves.append((target, speed, enable_if_needed))
        raise SDKError("ack lost", uncertain=True)

    client.move = lost_ack
    monkeypatch.setattr(tool, "JointClient", lambda *_: client)
    output = tmp_path / "uncertain"
    monkeypatch.setattr(tool.sys, "argv", ["zero", "--execute", "--output", str(output)])
    assert tool.main() == 1
    assert len(client.moves) == 1
    assert client.stops == ["zero-request"]
    report = json.loads((output / "report.json").read_text())
    assert report["commands_sent"] is None
    assert report["submission_uncertain"] is True


def test_changed_sdk_target_is_not_silently_accepted(tmp_path):
    state = joint_state()
    state["arm_startup"]["joint_control"]["target"][9] = 0.1
    with pytest.raises(RuntimeError, match="target changed"):
        wait(Joint(state), tmp_path)
