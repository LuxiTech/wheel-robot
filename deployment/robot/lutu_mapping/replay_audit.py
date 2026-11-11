#!/usr/bin/env python3
"""Compare fresh mapping and map-publisher stalls using recorded inputs in ROS domain 98."""

import argparse
import json
import os
import re
import signal
import subprocess
import time
from pathlib import Path


def stop_owned(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=4)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=4)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("work", type=Path)
    args = parser.parse_args()
    environment = dict(
        os.environ,
        ROS_DOMAIN_ID="98",
        ROS_LOCALHOST_ONLY="0",
        RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
    )
    environment["CYCLONEDDS_URI"] = (
        '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" /></Interfaces><DontRoute>true</DontRoute></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>64</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1" /></Peers></Discovery></Domain></CycloneDDS>'
    )
    configuration = args.root / "config/.generated/current/fast_lio_mid360.yaml"
    results = []
    for stall_ms in [0, 250]:
        tag = f"stall-{stall_ms}"
        mapper_log = args.work / f"replay-{tag}.log"
        with (
            mapper_log.open("w") as log,
            (args.work / f"bag-play-{tag}.log").open("w") as playback_log,
        ):
            mapper = subprocess.Popen(
                [
                    str(args.work / "audit_mapper"),
                    "--ros-args",
                    "--params-file",
                    str(configuration),
                    "-p",
                    "publish.map_en:=false",
                    "-p",
                    "pcd_save.pcd_save_en:=false",
                    "-p",
                    "runtime_pos_log_enable:=false",
                ],
                env=dict(environment, LUTU_AUDIT_MAP_STALL_MS=str(stall_ms)),
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            player = None
            try:
                time.sleep(1)
                if mapper.poll() is not None:
                    raise RuntimeError("Isolated mapper exited during startup")
                player = subprocess.Popen(
                    [
                        "ros2",
                        "bag",
                        "play",
                        str(args.work / "raw-inputs"),
                        "--topics",
                        "/livox/lidar",
                        "/livox/imu",
                    ],
                    env=environment,
                    stdout=playback_log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                if player.wait(timeout=35) != 0:
                    raise RuntimeError("Isolated bag playback failed")
                time.sleep(1)
            finally:
                if player is not None:
                    stop_owned(player)
                stop_owned(mapper)
        source = mapper_log.read_text()
        states = []
        for match in re.finditer(
            r"AUDIT_STATE t=([\d.e+-]+) imu=(\d+) features=(\d+) matched=(\d+) pos=([\d.e+-]+) speed=([\d.e+-]+)",
            source,
        ):
            t, imu, features, matched, position, speed = match.groups()
            states.append(
                {
                    "time": float(t),
                    "imu": int(imu),
                    "features": int(features),
                    "matched": int(matched),
                    "position_norm": float(position),
                    "speed": float(speed),
                }
            )
        result = {
            "stall_ms_per_second": stall_ms,
            "states": len(states),
            "empty_imu_frames": source.count("AUDIT_EMPTY_IMU"),
            "empty_imu_states_reusing_features": sum(
                state["imu"] == 0 and state["features"] > 0 for state in states
            ),
            "last": states[-1] if states else None,
            "max_speed": max((state["speed"] for state in states), default=None),
            "max_position_norm": max((state["position_norm"] for state in states), default=None),
            "trace": states,
        }
        results.append(result)
        print(
            json.dumps({key: value for key, value in result.items() if key != "trace"}), flush=True
        )
    (args.work / "replay-results.json").write_text(
        json.dumps({"ros_domain": 98, "production_touched": False, "runs": results}, indent=2)
        + "\n"
    )


if __name__ == "__main__":
    main()
