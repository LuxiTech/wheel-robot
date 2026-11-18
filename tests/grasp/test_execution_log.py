import importlib.util
import json
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "execution_log", ROOT / "tools/grasp/execution_log.py"
)
LOG = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LOG)


def test_failure_log_names_the_stage_and_retains_partial_trace(tmp_path):
    run = tmp_path / "run"
    trace = run / "execution_trace"
    trace.mkdir(parents=True)

    def event(kind, seconds, **detail):
        return dict(run_id="test", kind=kind, local_mono_ns=int(seconds * 1e9), detail=detail)

    rows = [
        event("run_started", 1),
        event("task_state", 2, phase="ALIGN_VISUAL_SERVO"),
        event("phase_start", 3, phase="align"),
        event(
            "reference_loop_failed",
            5,
            error="deadline missed",
            metrics=dict(generated=0),
            failed_tick=dict(stage="joint_reference_generation", dt_s=0.04),
        ),
        event("phase_failed", 5.1, phase="align", error="deadline missed"),
    ]
    path = trace / "events.jsonl"
    original = "\n".join(json.dumps(row) for row in rows) + '\n{"kind":'
    path.write_text(original)
    output = LOG.write_execution_log(run, exit_code=1)
    content = output.read_text()
    assert "停止位置：视觉对准" in content
    assert "触发原因：deadline missed" in content
    assert "耗时2.100s" in content
    assert "dt=0.04s" in content
    assert "损坏行：[6]" in content
    assert path.read_text() == original


def test_check_success_is_not_reported_as_a_successful_grasp(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    (run / "summary.json").write_text(json.dumps(dict(completed=True, complete_grasp=False)))
    result = LOG.write_execution_log(run).read_text()
    assert "限定流程完成（非完整抓取）" in result
    assert "停止位置：无" in result


@pytest.mark.parametrize("exit_code,gateway_failure", [(7, False), (130, False), (9, True)])
def test_wrapper_always_generates_a_log_after_child_failure(tmp_path, exit_code, gateway_failure):
    tools = tmp_path / "tools/grasp"
    tools.mkdir(parents=True)
    for name in ("execution_log.py", "run_current_visual_grasp.sh"):
        shutil.copy(ROOT / "tools/grasp" / name, tools / name)
    # Recorder lifecycle is exercised separately without a real SSH connection.
    (tools / "record_rosbag.py").write_text(
        "import subprocess, sys\n"
        "raise SystemExit(subprocess.call(sys.argv[sys.argv.index('--') + 1:]))\n"
    )
    resources = [
        next(
            line.split("=", 1)[1]
            for line in (tools / "run_current_visual_grasp.sh").read_text().splitlines()
            if line.startswith("calibration=")
        ),
        "outputs/grasp/joint-stream-deploy-20261115/closer-box/vision/detection.json",
    ]
    for name in resources:
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("{}")
    configuration = tmp_path / "config/robots/grasp_execution.json"
    configuration.parent.mkdir(parents=True)
    shutil.copy(ROOT / "config/robots/grasp_execution.json", configuration)
    (tools / "ensure_moveit_gateway.sh").write_text(
        f"echo 'RuntimeError: test startup failure'; exit {exit_code}\n"
        if gateway_failure
        else "exit 0\n"
    )
    interpreter = tmp_path / ".runtime/vision/venv/bin/python"
    interpreter.parent.mkdir(parents=True)
    interpreter.write_text(
        f"#!/bin/sh\necho 'RuntimeError: test startup failure'\nexit {exit_code}\n"
    )
    interpreter.chmod(0o755)
    result = subprocess.run(
        ["bash", str(tools / "run_current_visual_grasp.sh"), "check"],
        capture_output=True,
        text=True,
    )
    assert result.returncode == exit_code
    assert "Execution backend: moveit_trajectory" in result.stdout
    folders = [
        p.parent for p in (tmp_path / "outputs/grasp/visual-grasp-runs").glob("*/execution.log")
    ]
    assert len(folders) == 1
    log = folders[0] / "execution.log"
    assert log.is_symlink()
    content = log.read_text()
    assert "RuntimeError: test startup failure" in content
    assert f"退出码：{exit_code}" in content
    assert "未进入执行日志" in content
    assert (folders[0] / "console.log").exists()
