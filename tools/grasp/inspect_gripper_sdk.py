#!/usr/bin/env python3
"""Read selected gripper definitions on the robot over SSH; never changes files."""

import argparse
import datetime
import json
import subprocess
from pathlib import Path

REMOTE = r"""
import hashlib,json,re
from pathlib import Path
root=Path(SDK_ROOT)
paths=set()
for directory in [root/'device/openarm/openarm_can/include', root/'device/openarm/openarm_can/src']:
    if directory.is_dir():
        paths.update(p for p in directory.rglob('*') if p.suffix in ('.hpp','.h','.cpp','.cc'))
for directory in [root/'device/openarm/openarm_description',root/'Project/robot_bringup/config']:
    if directory.is_dir():
        paths.update(p for p in directory.rglob('*') if p.name in
            ('kinematics.yaml','kinematics_link.yaml','openarm_parallel_gripper.xacro',
             'gripper_open_calibration.json','gripper_zero_calibration.json','arms-kh.json'))
paths.add(root/'Project/device_adapters/src/openarm_adapter.cpp')
records=[]
pattern=re.compile(r'set_position|gripper|finger|parallel|linkage|crank|radius|ratio|sin\(|cos\(',re.I)
for path in sorted(paths):
    if not path.is_file() or path.stat().st_size>2_000_000:continue
    data=path.read_bytes();lines=data.decode('utf-8',errors='replace').splitlines()
    matches=[i for i,line in enumerate(lines) if pattern.search(line)]
    if not matches and path.suffix not in ('.yaml','.json','.xacro'):continue
    selected=set(range(len(lines))) if path.suffix in ('.yaml','.json','.xacro') else set()
    for i in matches:
        selected.update(range(max(0,i-4),min(len(lines),i+9)))
    selected=sorted(selected)
    records.append(dict(path=str(path),sha256=hashlib.sha256(data).hexdigest(),
        excerpts=[dict(line=i+1,text=lines[i]) for i in selected[:500]],truncated=len(selected)>500))
print(json.dumps(dict(root=str(root),root_exists=root.is_dir(),files=records,commands_sent=False)))
"""


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--host", default="sunrise@192.168.0.245", help="SSH destination or configured alias"
    )
    p.add_argument(
        "--sdk-root", default="/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2"
    )
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    if not a.host or a.host.startswith("-") or any(c.isspace() for c in a.host):
        p.error("Expected an SSH destination or alias")
    a.output.parent.mkdir(parents=True, exist_ok=True)
    # Reserve before contacting the robot, so prior evidence cannot be overwritten.
    with a.output.open("x") as stream:
        report = dict(
            created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            host=a.host,
            sdk_root=a.sdk_root,
            commands_sent=False,
            scope="read_only_source_inspection",
        )
        try:
            result = subprocess.run(
                ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", a.host, "python3", "-"],
                input=REMOTE.replace("SDK_ROOT", repr(a.sdk_root), 1),
                text=True,
                capture_output=True,
                timeout=30,
            )
            report.update(returncode=result.returncode, stderr=result.stderr)
            if result.returncode:
                report.update(status="blocked_ssh", error=result.stderr.strip())
            else:
                report["inspection"] = json.loads(result.stdout)
                report["status"] = (
                    "inspected" if report["inspection"]["files"] else "no_matching_sources"
                )
        except (OSError, subprocess.TimeoutExpired, ValueError) as exc:
            report.update(status="blocked", error=str(exc))
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(
        json.dumps(
            dict(status=report["status"], output=str(a.output), error=report.get("error")),
            ensure_ascii=False,
        )
    )
    return 0 if report["status"] == "inspected" else 1


if __name__ == "__main__":
    raise SystemExit(main())
