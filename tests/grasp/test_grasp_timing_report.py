import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "grasp_timing_report", ROOT / "tools/grasp/summarize_execution_timing.py"
)
REPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPORT)


def test_send_intervals_do_not_include_pause_between_sessions():
    events = [
        dict(local_mono_ns=stamp, detail=dict(request_id=request))
        for request, stamp in [
            ("a", 0),
            ("a", 10_000_000),
            ("b", 9_000_000_000),
            ("b", 9_020_000_000),
        ]
    ]
    assert REPORT.intervals(events) == [10.0, 20.0]


def test_historical_processing_is_not_mislabeled_as_pure_inference(tmp_path):
    trace = tmp_path / "execution_trace"
    trace.mkdir()
    events = [
        dict(
            kind="rgbd_response",
            local_mono_ns=1_000_000_000,
            detail=dict(headers={"X-Pair-Id": "frame"}),
        ),
        dict(
            kind="vision_quality",
            local_mono_ns=1_080_000_000,
            detail=dict(pair_id="frame", age_s=0.3),
        ),
        # The camera may return this same frame again after inference finishes.
        dict(
            kind="rgbd_response",
            local_mono_ns=1_200_000_000,
            detail=dict(headers={"x-pair-id": "frame"}),
        ),
    ]
    (trace / "events.jsonl").write_text("\n".join(json.dumps(event) for event in events))
    result = REPORT.analyze(tmp_path)
    assert result["vision"]["body_to_quality_event_ms"]["mean"] == 80.0
    assert result["vision"]["image_age_ms"]["mean"] == 300.0
    assert result["vision"]["processing_ms"] is None
    assert result["references"]["send_interval_ms"] is None
    assert result["endpoint_ik_solve_ms"] is None
