#!/usr/bin/env python3
"""One read-only entry: real capture -> visual targets -> IK -> scene collisions.

Motion execution remains a separate commissioned composition. This command
never changes authority, enables, moves, closes a gripper, or opens the lift.
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def run_flow(args, *, runner=subprocess.run):
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = dict(commands_sent=False, stages=[], completed=False, mode="read_only")

    def save():
        (output / "flow.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")

    def stage(name, command):
        row = dict(stage=name, command=command, status="running")
        report["stages"].append(row)
        save()
        print(name, flush=True)
        try:
            with (output / (name + ".log")).open("w") as stream:
                result = runner(
                    command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, text=True
                )
        except BaseException as exc:
            row.update(status="failed", error=str(exc))
            save()
            raise
        row.update(
            returncode=result.returncode, status="completed" if result.returncode == 0 else "failed"
        )
        save()
        if result.returncode:
            raise RuntimeError("Stage failed; inspect " + str(output / (name + ".log")))

    try:
        capture = args.capture.resolve() if args.capture else output / "capture"
        if args.capture is None:
            stage(
                "capture",
                [
                    sys.executable,
                    str(ROOT / "tools/grasp/current_gripper_pose.py"),
                    "--output",
                    str(capture),
                    "--capture-url",
                    args.url,
                    "--calibration",
                    str(args.calibration.resolve()),
                    "--image-max-age-ms",
                    "500",
                ],
            )
        preview = args.preview.resolve() if args.preview else output / "vision"
        if args.preview is None:
            command = [
                str(args.vision_python.absolute()),
                str(ROOT / "tools/grasp/preview_camera_grasps.py"),
                str(capture / "observation.npz"),
                "--lift-context",
                str(capture / "lift_context.json"),
                "--calibration",
                str(args.calibration.resolve()),
                "--output",
                str(preview),
            ]
            if args.box:
                command.extend(["--box", *map(str, args.box)])
            elif args.target_pixel:
                command.extend(["--target-pixel", *map(str, args.target_pixel)])
            else:
                command.append("--auto-target")
            if args.clean_depth:
                command.append("--clean-depth")
            stage("perception", command)
        stage(
            "scene_plan",
            [
                sys.executable,
                str(ROOT / "tools/grasp/plan_visual_grasp.py"),
                "--preview",
                str(preview),
                "--capture",
                str(capture),
                "--calibration",
                str(args.calibration.resolve()),
                "--output",
                str(output / "plan"),
                "--steps",
                "40",
            ],
        )
        summary = json.loads((output / "plan/summary.json").read_text())
        report.update(
            completed=True,
            planning_status=summary["status"],
            execution_ready=summary["execution_ready"],
            execution_blockers=summary["execution_blockers"],
            report=str(output / "plan/report.html"),
        )
    except BaseException as exc:
        report["error"] = str(exc)
        save()
        raise
    save()
    return report


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--output", type=Path, required=True)
    result.add_argument("--calibration", type=Path, required=True)
    result.add_argument("--url", default="http://192.168.2.199:18030")
    result.add_argument(
        "--vision-python", type=Path, default=ROOT / ".runtime/vision/venv/bin/python"
    )
    result.add_argument("--capture", type=Path, help="Explicit saved capture; remains replay-only")
    result.add_argument("--preview", type=Path, help="Reuse inference from the same capture pair")
    selection = result.add_mutually_exclusive_group()
    selection.add_argument("--box", type=float, nargs=4)
    selection.add_argument("--target-pixel", type=float, nargs=2)
    result.add_argument("--clean-depth", action="store_true")
    return result


if __name__ == "__main__":
    print(json.dumps(run_flow(parser().parse_args()), ensure_ascii=False, indent=2))
