#!/usr/bin/env python3
"""Read-only joint interface check / payload preview. No live execute option."""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure

configure(ROOT)
from wheel_sdk_bridge.joint_client import JointClient


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", type=Path, default=ROOT / "config/robots/joint_control.json")
    p.add_argument("--positions", type=Path, help="Preview JSON q[16] request only; never submit")
    p.add_argument("--speed", type=float, default=0.02)
    p.add_argument("--output", type=Path)
    a = p.parse_args()
    policy = json.loads(a.config.read_text())
    client = JointClient(policy["base_url"], policy)
    result = client.preflight()
    if a.positions:
        result["unsent_request"] = client.prepare(json.loads(a.positions.read_text()), a.speed)
    text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if a.output:
        a.output.parent.mkdir(parents=True, exist_ok=True)
        with a.output.open("x") as f:
            f.write(text)
    print(text, end="")


if __name__ == "__main__":
    main()
