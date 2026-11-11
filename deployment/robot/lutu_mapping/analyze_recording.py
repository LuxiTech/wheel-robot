#!/usr/bin/env python3
"""Read recorded sensor timing and numerical validity without running a ROS node."""

import argparse
import json
import sqlite3
import struct
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("database", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    connection = sqlite3.connect(f"file:{args.database}?mode=ro", uri=True)
    topics = {
        row[0]: (row[1], row[2]) for row in connection.execute("SELECT id,name,type FROM topics")
    }
    samples = {}
    lidar_examples = []
    for topic_id, received, payload in connection.execute(
        "SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp"
    ):
        name, type_name = topics[topic_id]
        sec, nsec = struct.unpack_from("<iI", payload, 4)
        stamp = sec + nsec * 1e-9
        row = {"received": received * 1e-9, "stamp": stamp}
        rows = samples.setdefault(name, [])
        if name == "/livox/imu":
            message = deserialize_message(payload, get_message(type_name))
            row["acc"] = [
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ]
            row["gyro"] = [
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ]
        elif name == "/Odometry":
            message = deserialize_message(payload, get_message(type_name))
            point = message.pose.pose.position
            row["position"] = [point.x, point.y, point.z]
        elif len(rows) in {0, 50, 100}:
            message = deserialize_message(payload, get_message(type_name))
            points = np.array([[point.x, point.y, point.z] for point in message.points])
            finite = points[np.isfinite(points).all(axis=1)]
            lidar_examples.append(
                {
                    "index": len(rows),
                    "points": message.point_num,
                    "timebase_minus_header_ns": int(message.timebase) - (sec * 1000000000 + nsec),
                    "offset_range_ns": [
                        min(point.offset_time for point in message.points),
                        max(point.offset_time for point in message.points),
                    ],
                    "finite_points": len(finite),
                    "xyz_min": finite.min(axis=0).tolist(),
                    "xyz_max": finite.max(axis=0).tolist(),
                }
            )
        rows.append(row)
    result = {"database": str(args.database), "topics": {}, "lidar_examples": lidar_examples}
    for name, rows in samples.items():
        stamps = np.array([row["stamp"] for row in rows])
        deltas = np.diff(stamps)
        receive_deltas = np.diff([row["received"] for row in rows])
        result["topics"][name] = {
            "count": len(rows),
            "first": rows[0],
            "last": rows[-1],
            "header_interval_sec": {
                "min": float(deltas.min()),
                "median": float(np.median(deltas)),
                "p99": float(np.quantile(deltas, 0.99)),
                "max": float(deltas.max()),
            },
            "receive_interval_max_sec": float(receive_deltas.max()),
            "backward_stamps": int(np.count_nonzero(deltas < 0)),
        }
        if name == "/livox/imu":
            acceleration = np.array([row["acc"] for row in rows])
            gyro = np.array([row["gyro"] for row in rows])
            result["topics"][name].update(
                acceleration_mean=acceleration.mean(axis=0).tolist(),
                acceleration_norm_range=[
                    float(np.linalg.norm(acceleration, axis=1).min()),
                    float(np.linalg.norm(acceleration, axis=1).max()),
                ],
                gyro_abs_max=np.abs(gyro).max(axis=0).tolist(),
            )
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
