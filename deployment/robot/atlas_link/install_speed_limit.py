#!/usr/bin/env python3
"""Install a tested speed-cap candidate while the base and teleoperation are idle."""

import argparse
import hashlib
import json
import shutil
import subprocess
import time
import urllib.request
from pathlib import Path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def state():
    with urllib.request.urlopen("http://127.0.0.1:8000/api/state", timeout=6) as response:
        return json.load(response)


def require_idle(snapshot):
    base = snapshot["base_status"]
    control = snapshot["control"]
    if not base["feedback_fresh"] or base["enabled"] or not base["stationary"]:
        raise RuntimeError("Base must be freshly measured, stopped and disabled")
    if base["keyboard_owner"] or base["external_control"]:
        raise RuntimeError("A base controller still owns the session")
    if not control["fresh"] or control["system_phase"] not in {"disabled", "stopped"}:
        raise RuntimeError("Stop teleoperation before replacing the base driver")
    if snapshot["recorder"]["state"] != "idle":
        raise RuntimeError("Recording must be idle")


def atomic_copy(source, target):
    target = target.resolve()
    temporary = target.with_name(target.name + ".speed-next")
    shutil.copy2(source, temporary)
    temporary.replace(target)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("work", type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.work / "manifest.json").read_text())
    verified = json.loads((args.work / "tests-passed.json").read_text())
    for name, checksum in verified.items():
        assert digest(args.work / name) == checksum, f"Tested artifact changed: {name}"
    for relative, metadata in manifest.items():
        assert digest(args.root / relative) == metadata["before_sha256"], relative
        assert digest(args.work / "candidate" / relative) == metadata["candidate_sha256"], relative
    service = Path("/etc/systemd/system/luxi-base.service.d/quest-teleop.conf")
    assert service.read_bytes() == (args.work / "service.before").read_bytes()
    snapshot = state()
    require_idle(snapshot)
    (args.work / "before-install.json").write_text(json.dumps(snapshot, indent=2) + "\n")
    protected = {
        name: digest(args.root / name)
        for name in [
            "Project/device_adapters/src/lift_adapter.cpp",
            "Project/device_adapters/src/openarm_adapter.cpp",
            "Project/control_manager/src/control_manager.cpp",
            "Project/luxi_web_ctrl/scripts/program.launch.xml",
            "Project/robot_bringup/config/arms-teleop.json",
        ]
    }
    binaries = {
        args.root
        / "build/install/luxi_device_adapters/lib/luxi_device_adapters/r680_adapter": args.work
        / "r680_adapter",
        args.root / "build/packages/luxi_device_adapters/r680_adapter": args.work / "r680_adapter",
    }
    targets = {args.root / name: args.work / "candidate" / name for name in manifest}
    targets.update(binaries)
    for name in ["app.js", "index.html"]:
        targets[args.root / "build/install/luxi_web_ctrl/share/luxi_web_ctrl/web" / name] = (
            args.work / "candidate/Project/luxi_web_ctrl/web" / name
        )
    backup = args.work / "installed-backup"
    backup.mkdir()
    receipt = []
    for index, target in enumerate(targets):
        shutil.copy2(target, backup / str(index))
        receipt.append({"target": str(target), "backup": str(backup / str(index))})
    shutil.copy2(service, backup / "service")
    require_idle(state())
    for target, candidate in targets.items():
        atomic_copy(candidate, target)
    subprocess.run(
        ["sudo", "-n", "install", "-m", "644", str(args.work / "service.candidate"), str(service)],
        check=True,
    )
    subprocess.run(["sudo", "-n", "systemctl", "daemon-reload"], check=True)
    require_idle(state())
    subprocess.run(["sudo", "-n", "systemctl", "restart", "luxi-base.service"], check=True)
    deadline = time.monotonic() + 25
    while True:
        snapshot = state()
        base = snapshot["base_status"]
        if base["fresh"] and base["feedback_fresh"] and base["web_keyboard_linear_limit"] == 0.5:
            assert not base["enabled"] and not base["keyboard_owner"]
            break
        if time.monotonic() > deadline:
            raise RuntimeError("Updated base speed capability has not become fresh")
        time.sleep(0.2)
    assert protected == {name: digest(args.root / name) for name in protected}
    (args.work / "after-install.json").write_text(json.dumps(snapshot, indent=2) + "\n")
    (args.work / "install-receipt.json").write_text(
        json.dumps(
            {
                "files": receipt,
                "restarted": ["luxi-base.service"],
                "protected": protected,
                "motion_commands_sent": False,
            },
            indent=2,
        )
        + "\n"
    )
    print("Installed configurable web speed cap: 0.50 m/s; base remains disabled")


if __name__ == "__main__":
    main()
