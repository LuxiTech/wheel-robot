import copy
import importlib.util
import json
import struct
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "set_arm_encoder_zero", ROOT / "tools/grasp/set_arm_encoder_zero.py"
)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


def state():
    return {
        "system": {"mock": False},
        "server_mono_ns": 1000000000,
        "arms": {
            "fresh": True,
            "enabled": False,
            "joint_order": "right_then_left",
            "position": [0.0] * 16,
            "velocity": [0.0] * 16,
            "valid": [True] * 16,
            "age_ns": [10000] * 16,
            "receive_mono_ns": 999000000,
        },
    }


def test_single_joint_does_not_select_default_all_arm_ids():
    command = tool.zero_command("luxi_left", [2])
    assert command == [tool.CLI, "-i", "luxi_left", "set_zero", "--no-arm", "--id", "2"]


@pytest.mark.parametrize("interface,ids", [("can0", [2]), ("luxi_left", [0]), ("luxi_right", [9])])
def test_invalid_bus_or_motor_rejected(interface, ids):
    with pytest.raises(ValueError):
        tool.zero_command(interface, ids)


def test_disabled_and_stationary_required():
    tool.check_disabled(state())
    for key, value in [
        ("enabled", True),
        ("fresh", False),
        ("velocity", [0.1] * 16),
        ("valid", [False] * 16),
    ]:
        sample = state()
        sample["arms"][key] = value
        with pytest.raises(RuntimeError):
            tool.check_disabled(sample)


def test_stale_feedback_cannot_authorize_zero():
    sample = state()
    sample["arms"]["age_ns"][9] = 500000000
    with pytest.raises(RuntimeError, match="stale"):
        tool.check_disabled(sample)


def test_cli_success_does_not_replace_readback_check():
    tool.check_new_zero({2: {"status_nibble": 0, "position_code": 32767}})
    with pytest.raises(RuntimeError, match="not near zero"):
        tool.check_new_zero({2: {"status_nibble": 0, "position_code": 33000}})
    with pytest.raises(RuntimeError, match="enabled/faulted"):
        tool.check_new_zero({2: {"status_nibble": 1, "position_code": 32767}})


@pytest.mark.parametrize("frame_size", [16, 72])
def test_read_can_accepts_classic_and_fd_feedback(monkeypatch, frame_size):
    payload = bytes.fromhex("037fff7ff7ff211f")
    # This mirrors the live FD header (BRS flag set) and its 8-byte motor payload.
    frame = struct.pack("=IBB2x", 0x13, 8, int(frame_size == 72))
    frame += payload + bytes(frame_size - 16)
    calls = []

    class Bus:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            pass

        def setsockopt(self, *args):
            calls.append(("option", args))

        def bind(self, address):
            assert address == ("luxi_left",)

        def settimeout(self, _timeout):
            pass

        def send(self, message):
            calls.append(("send", message))

        def recv(self, length):
            assert length == 72
            return frame

    monkeypatch.setattr(tool.socket, "socket", lambda *_args: Bus())
    result = tool.read_can("luxi_left", [3])
    assert result[3]["position_code"] == 32767
    assert result[3]["status_nibble"] == 0
    assert calls[0] == ("option", (tool.socket.SOL_CAN_RAW, tool.socket.CAN_RAW_FD_FRAMES, 1))
    assert calls[1] == (
        "send",
        struct.pack("=IB3x8s", 0x7FF, 8, bytes([3, 0, 0xCC, 0, 0, 0, 0, 0])),
    )


def test_execute_needs_physical_support_confirmation(monkeypatch):
    monkeypatch.setattr(tool.sys, "argv", ["zero", "--execute"])
    monkeypatch.setattr(
        tool.subprocess, "run", lambda *_a, **_kw: pytest.fail("No SSH or motor command allowed")
    )
    with pytest.raises(SystemExit) as error:
        tool.main()
    assert error.value.code == 2


def test_local_preview_sends_only_preview_source_to_ssh(monkeypatch):
    calls = []

    def run(command, **kwargs):
        calls.append((copy.deepcopy(command), kwargs))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(tool.subprocess, "run", run)
    monkeypatch.setattr(tool.sys, "argv", ["zero", "--side", "left", "--joints", "2"])
    assert tool.main() == 0
    assert len(calls) == 1
    command, options = calls[0]
    assert "--side left --joints 2" in command[-1]
    assert "--execute" not in command[-1]
    assert "--on-robot" in command[-1]
    assert options["input"] == Path(tool.__file__).read_text()


def robot_fixture(monkeypatch, tmp_path, enabled=False):
    root = tmp_path / "sdk"
    folder = root / "Project/robot_bringup/config"
    folder.mkdir(parents=True)
    config = {side: {"offsets": [0] * 8, "interface": "luxi_" + side} for side in ["left", "right"]}
    for name in ["arms.json", "arms-kh.json", "arms-teleop.json"]:
        (folder / name).write_text(json.dumps(config))
    source = root / "device/openarm/openarm_can/setup/cli/commands/zero_position_commands.cpp"
    source.parent.mkdir(parents=True)
    source.write_text("test source")
    cli = root / "cli"
    cli.write_bytes(b"test binary")
    sample = state()
    sample["arms"]["enabled"] = enabled
    monkeypatch.setattr(tool, "SDK_ROOT", root)
    monkeypatch.setattr(tool, "CLI", str(cli))
    monkeypatch.setattr(tool, "sdk_state", lambda: copy.deepcopy(sample))
    monkeypatch.setattr(tool.os, "geteuid", lambda: 0)
    return root


def test_enabled_arms_refuse_hardware_write_or_service_stop(monkeypatch, tmp_path):
    root = robot_fixture(monkeypatch, tmp_path, enabled=True)
    monkeypatch.setattr(
        tool, "run", lambda *_a, **_k: pytest.fail("No service or motor command allowed")
    )
    assert tool.robot_main(SimpleNamespace(side="left", joints=[2], execute=True)) == 1
    report = json.loads(next(root.glob("work/arm-encoder-zero/*/report.json")).read_text())
    assert report["zero_writes_attempted"] == []
    assert report["monitor_left_stopped"] is False


def test_partial_write_failure_is_not_retried_or_reenabled(monkeypatch, tmp_path):
    root = robot_fixture(monkeypatch, tmp_path)
    calls = []

    def run(command):
        calls.append(command)
        if command[:2] == ["systemctl", "is-active"]:
            code = 3
        elif "set_zero" in command:
            code = 1
        else:
            code = 0
        return SimpleNamespace(returncode=code, stdout="test", stderr="test")

    monkeypatch.setattr(tool, "run", run)
    monkeypatch.setattr(
        tool, "read_can", lambda *_: {2: {"status_nibble": 0, "position_code": 31000}}
    )
    assert tool.robot_main(SimpleNamespace(side="both", joints=[2], execute=True)) == 1
    report = json.loads(next(root.glob("work/arm-encoder-zero/*/report.json")).read_text())
    assert report["zero_writes_attempted"] == ["right"]
    assert report["monitor_left_stopped"] is True
    assert len([command for command in calls if "set_zero" in command]) == 1
    assert not any("enable" in command or "start" in command for command in calls)
