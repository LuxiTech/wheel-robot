"""The composed CLI preserves failures and never converts a replay into motion."""

import json
from types import SimpleNamespace

import pytest
from grasp.visual_grasp_flow import parser, run_flow


def test_reused_same_capture_flows_through_scene_plan_and_retains_rejection(tmp_path):
    output = tmp_path / "flow"
    args = parser().parse_args(
        [
            "--output",
            str(output),
            "--calibration",
            str(tmp_path / "camera.json"),
            "--capture",
            str(tmp_path / "capture"),
            "--preview",
            str(tmp_path / "preview"),
        ]
    )
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        plan = output / "plan"
        plan.mkdir()
        (plan / "summary.json").write_text(
            json.dumps(
                dict(
                    status="scene_collision_rejected",
                    execution_ready=False,
                    execution_blockers=["collision_path_rejected"],
                )
            )
        )
        return SimpleNamespace(returncode=0)

    result = run_flow(args, runner=run)
    assert len(calls) == 1
    assert "plan_visual_grasp.py" in calls[0][1]
    assert result["completed"] and result["commands_sent"] is False
    assert result["execution_ready"] is False
    assert result["execution_blockers"] == ["collision_path_rejected"]
    assert result["mode"] == "read_only"


def test_capture_failure_stops_before_perception_or_motion(tmp_path):
    output = tmp_path / "flow"
    args = parser().parse_args(
        ["--output", str(output), "--calibration", str(tmp_path / "camera.json")]
    )
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=4)

    with pytest.raises(RuntimeError, match="capture.log"):
        run_flow(args, runner=run)
    report = json.loads((output / "flow.json").read_text())
    assert len(calls) == 1
    assert report["stages"][0]["status"] == "failed"
    assert report["commands_sent"] is False
    assert report["completed"] is False


def test_vision_python_keeps_virtualenv_symlink_in_subprocess(tmp_path):
    environment = tmp_path / "vision-env/bin"
    environment.mkdir(parents=True)
    system_python = tmp_path / "python-system"
    system_python.write_text("fixture")
    interpreter = environment / "python"
    interpreter.symlink_to(system_python)
    output = tmp_path / "flow"
    args = parser().parse_args(
        [
            "--output",
            str(output),
            "--calibration",
            str(tmp_path / "camera.json"),
            "--capture",
            str(tmp_path / "capture"),
            "--vision-python",
            str(interpreter),
        ]
    )
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        if "plan_visual_grasp.py" in command[1]:
            plan = output / "plan"
            plan.mkdir()
            (plan / "summary.json").write_text(
                json.dumps(
                    dict(
                        status="scene_collision_rejected",
                        execution_ready=False,
                        execution_blockers=["collision_path_rejected"],
                    )
                )
            )
        return SimpleNamespace(returncode=0)

    run_flow(args, runner=run)
    assert calls[0][0] == str(interpreter.absolute())
    assert calls[0][0] != str(interpreter.resolve())
    assert "--auto-target" in calls[0]


def test_process_launch_failure_does_not_leave_stage_running(tmp_path):
    output = tmp_path / "flow"
    args = parser().parse_args(
        ["--output", str(output), "--calibration", str(tmp_path / "camera.json")]
    )

    def run(command, **kwargs):
        raise FileNotFoundError("interpreter missing")

    with pytest.raises(FileNotFoundError):
        run_flow(args, runner=run)
    report = json.loads((output / "flow.json").read_text())
    assert report["stages"][0]["status"] == "failed"
    assert report["completed"] is False
