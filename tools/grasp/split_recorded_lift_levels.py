#!/usr/bin/env python3
"""Identify stable lift intervals in one continuous bag; never publishes ROS data."""

import argparse
import bisect
import json
import statistics
from pathlib import Path


def split_levels(samples, span_um=500, window_ns=600_000_000, minimum_seconds=2.0):
    valid = sorted(
        [
            row
            for row in samples
            if row.get("valid")
            and not row.get("mock", True)
            and not row.get("fault")
            and row.get("speed_rpm") == 0
        ],
        key=lambda row: int(row["receive_mono_ns"]),
    )
    times = [int(row["receive_mono_ns"]) for row in valid]
    stable = []
    for index, time_ns in enumerate(times):
        lower = bisect.bisect_left(times, time_ns - window_ns)
        upper = bisect.bisect_right(times, time_ns + window_ns)
        window = valid[lower:upper]
        heights = [row["current_um"] for row in window]
        timestamps = times[lower:upper]
        if len(window) < 5:
            continue
        if timestamps[0] > time_ns - 400_000_000 or timestamps[-1] < time_ns + 400_000_000:
            continue
        if max(heights) - min(heights) > span_um:
            continue
        if max(b - a for a, b in zip(timestamps, timestamps[1:])) > 300_000_000:
            continue
        stable.append(
            dict(
                t=time_ns,
                height_um=int(statistics.median(heights)),
                bag_timestamp_ns=valid[index].get("bag_timestamp_ns"),
            )
        )
    chunks = []
    for row in stable:
        if (
            not chunks
            or row["t"] - chunks[-1][-1]["t"] > 400_000_000
            or abs(row["height_um"] - chunks[-1][0]["height_um"]) > span_um
        ):
            chunks.append([])
        chunks[-1].append(row)
    segments = []
    levels = []
    for chunk in chunks:
        duration = (chunk[-1]["t"] - chunk[0]["t"]) / 1e9
        if duration < minimum_seconds:
            continue
        height = int(statistics.median([row["height_um"] for row in chunk]))
        group = next(
            (i for i, old in enumerate(levels) if abs(old - height) <= span_um), len(levels)
        )
        if group == len(levels):
            levels.append(height)
        segments.append(
            dict(
                height_group=group,
                height_um=height,
                height_mm=height / 1000,
                start_receive_mono_ns=chunk[0]["t"],
                end_receive_mono_ns=chunk[-1]["t"],
                duration_seconds=duration,
                valid_sample_count=len(chunk),
                height_span_um=max(row["height_um"] for row in chunk)
                - min(row["height_um"] for row in chunk),
            )
        )
    return dict(
        schema="wheel_lift_recording_segments_v1",
        time_basis="robot_receive_monotonic",
        input_lift_messages=len(samples),
        valid_stationary_lift_messages=len(valid),
        stable_height_levels_um=levels,
        stable_segments=segments,
        rules=dict(
            window_ns=window_ns,
            maximum_height_span_um=span_um,
            maximum_valid_feedback_gap_ns=300_000_000,
            minimum_interval_seconds=minimum_seconds,
        ),
        transition_policy="Keep raw data; use stable segments for calibration, omit moving/invalid intervals",
        scope="Lift stability only; arm stationarity and checkerboard quality checked separately",
    )


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/luxi/lift/state"]))
    rows = []
    while reader.has_next():
        _, data, timestamp = reader.read_next()
        row = json.loads(deserialize_message(data, String).data)
        row["bag_timestamp_ns"] = int(timestamp)
        rows.append(row)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    report = split_levels(read_bag(args.bag))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        stream.write(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
