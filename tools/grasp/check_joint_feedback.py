#!/usr/bin/env python3
"""Compare old/new joint feedback using GET only; save raw rejected samples too."""

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure

configure(ROOT)
from wheel_sdk_bridge.joint_client import JointClient


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", type=Path, default=ROOT / "config/robots/joint_control.json")
    p.add_argument("--feedback-url", default="http://192.168.0.245:18032")
    p.add_argument("--samples", type=int, default=100)
    p.add_argument("--interval", type=float, default=0.1)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    if a.samples < 1 or a.interval < 0:
        p.error("samples must be positive; interval cannot be negative")
    policy = json.loads(a.config.read_text())
    a.output.mkdir(parents=True, exist_ok=False)
    clients = {
        "original": JointClient(
            policy["base_url"],
            dict(feedback_diagnostics=str((a.output / "original.jsonl").resolve())),
        ),
        "direct_ros": JointClient(
            policy["base_url"],
            dict(
                feedback_url=a.feedback_url,
                feedback_diagnostics=str((a.output / "direct_ros.jsonl").resolve()),
            ),
        ),
    }
    rows = {name: [] for name in clients}
    for i in range(a.samples):
        # Alternate order to avoid consistently favouring one endpoint.
        for name in list(clients)[:: 1 if i % 2 else -1]:
            c = clients[name]
            try:
                c.snapshot()
            except Exception:
                pass  # Error and raw response retained by JointClient.
            rows[name].append(c.last_diagnostic)
        time.sleep(a.interval)
    summary = {}
    for name, records in rows.items():
        metrics = {}
        for key in (
            "request_ms",
            "arms_cache_ms",
            "arms_effective_ms",
            "arm_startup_effective_ms",
            "max_joint_effective_ms",
            "pair_skew_ms",
        ):
            values = sorted(r["timing"][key] for r in records if key in r["timing"])
            if values:
                metrics[key] = dict(
                    median=statistics.median(values),
                    p95=values[max(0, int(0.95 * len(values) + 0.999999) - 1)],
                    max=max(values),
                )
        summary[name] = dict(
            samples=len(records),
            rejected=sum(not r["accepted"] for r in records),
            errors=sorted(set(r["error"] for r in records if r["error"])),
            metrics=metrics,
        )
    (a.output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 1 if summary["direct_ros"]["rejected"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
