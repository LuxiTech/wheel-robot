"""Continuous multi-height recording: isolate stable levels without changing raw data."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/grasp"))
from split_recorded_lift_levels import split_levels


def test_keeps_distinct_plateaus_and_excludes_moving_transition():
    rows = []
    for index in range(160):
        moving = 50 <= index < 80
        height = 0 if index < 50 else (index - 50) * 3000 if moving else 90000
        rows.append(
            dict(
                receive_mono_ns=index * 100_000_000,
                current_um=height,
                valid=True,
                mock=False,
                fault=0,
                speed_rpm=10 if moving else 0,
            )
        )
    result = split_levels(rows)
    assert result["stable_height_levels_um"] == [0, 90000]
    assert len(result["stable_segments"]) == 2
    first, second = result["stable_segments"]
    assert first["end_receive_mono_ns"] < 5_000_000_000
    assert second["start_receive_mono_ns"] >= 8_000_000_000
    assert rows[60]["speed_rpm"] == 10


def test_does_not_bridge_a_feedback_outage_or_accept_mock():
    rows = [
        dict(
            receive_mono_ns=index * 100_000_000,
            current_um=-90000,
            valid=not 50 <= index < 90,
            mock=False,
            fault=0,
            speed_rpm=0,
        )
        for index in range(150)
    ]
    result = split_levels(rows)
    assert result["stable_height_levels_um"] == [-90000]
    assert len(result["stable_segments"]) == 2
    assert split_levels([dict(row, mock=True) for row in rows])["stable_segments"] == []
