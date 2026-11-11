#!/usr/bin/env python3
"""Run the SDK regression with its readiness fixture aligned to the live API."""

import argparse
import difflib
import os
import subprocess
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("work", type=Path)
    args = parser.parse_args()
    test_directory = args.root / "Project/device_adapters/test"
    original = (test_directory / "test_openarm_joint_target.py").read_text()
    before = "h.control.get('quest_window_samples',0)>=50"
    after = "h.control.get('quest_window_samples',0)>=3 and h.control.get('quest_age_ms',999)<150"
    if original.count(before) != 2:
        raise RuntimeError("SDK test fixture changed; inspect before adapting")
    # The unchanged installed SDK also fails the old 50-sample wait with fresh
    # 32 Hz input. Its actual command gate requires three samples and 100 ms.
    # Only test discovery changes; every actuator and safety assertion remains.
    updated = original.replace(before, after, 2)
    target = args.work / "test_joint_target_current_readiness.py"
    target.write_text(updated)
    (args.work / "joint-test-fixture.patch").write_text(
        "".join(
            difflib.unified_diff(
                original.splitlines(True),
                updated.splitlines(True),
                fromfile="SDK-test",
                tofile="isolated-test-fixture",
            )
        )
    )
    environment = dict(os.environ)
    environment["PYTHONPATH"] = str(test_directory) + ":" + environment.get("PYTHONPATH", "")
    command = [
        "python3",
        str(target),
        str(args.work / "bin/openarm_adapter"),
        str(args.work / "bin/control_manager"),
        str(args.work / "bin/web_ctrl"),
        str(args.root / "Project/robot_bringup/config/arms-kh.json"),
    ]
    result = subprocess.run(
        command, env=environment, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
    )
    (args.work / "joint-regression.log").write_text(result.stdout)
    print(result.stdout, end="")
    raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
