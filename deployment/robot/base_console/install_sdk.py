#!/usr/bin/env python3
"""Install a tested candidate only while the robot is idle; never enable motion."""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import time
import urllib.request
from pathlib import Path


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def state() -> dict:
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open("http://127.0.0.1:8000/api/state", timeout=5) as response:
        return json.load(response)


def require_idle(snapshot: dict, base_only: bool = False) -> None:
    if not base_only and (
        not snapshot.get("arms", {}).get("fresh") or snapshot["arms"].get("enabled")
    ):
        raise RuntimeError("Keep the current arm hold intact: deployment requires disabled arms")
    base = snapshot.get("base_status", {})
    if not base.get("feedback_fresh") or base.get("enabled"):
        raise RuntimeError("Deployment requires fresh, disabled base feedback")
    if any(abs(value) > 0.005 for value in snapshot.get("base", {}).get("velocity", [])):
        raise RuntimeError("Base is moving")
    if snapshot.get("lift_status", {}).get("enabled"):
        raise RuntimeError("Lift control is active")
    if snapshot.get("recorder", {}).get("state") != "idle":
        raise RuntimeError("Recording is active or unknown")
    if not snapshot.get("control", {}).get("fresh") or snapshot["control"].get(
        "system_phase"
    ) not in ("disabled", "stopped"):
        raise RuntimeError("Teleoperation is active")
    if snapshot.get("quest", {}).get("fresh"):
        raise RuntimeError("Quest is currently sending; coordinate the update before restarting")


def atomic_copy(source: Path, target: Path) -> None:
    if target.is_symlink():
        target = target.resolve()
    temporary = target.with_name(target.name + ".web-base-new")
    shutil.copy2(source, temporary)
    if target.exists():
        temporary.chmod(target.stat().st_mode)
    temporary.replace(target)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--sudo-password-env", default="WHEEL_ROBOT_SUDO_PASSWORD")
    parser.add_argument("--base-only", action="store_true")
    args = parser.parse_args()
    root = args.root.resolve()
    work = args.candidate.resolve()
    manifest = json.loads((work / "manifest.json").read_text())
    verified = json.loads((work / "tests-passed.json").read_text())
    for name, digest in verified.items():
        if sha256(work / name) != digest:
            raise RuntimeError("Candidate changed after its tests: " + name)
    for name, metadata in manifest.items():
        if sha256(root / name) != metadata["before_sha256"]:
            raise RuntimeError("Live source changed; preserve and rebase it first: " + name)
        if sha256(work / "candidate" / name) != metadata["candidate_sha256"]:
            raise RuntimeError("Candidate does not match its manifest: " + name)
    snapshot = state()
    require_idle(snapshot, args.base_only)
    (work / "before-install-state.json").write_text(json.dumps(snapshot, indent=2))

    credential = subprocess.run(["sudo", "-n", "true"], capture_output=True)
    if credential.returncode:
        password = os.environ.get(args.sudo_password_env)
        if not password:
            raise RuntimeError("A sudo credential is required to restart the idle services")
        subprocess.run(
            ["sudo", "-S", "-p", "", "-v"],
            input=password + "\n",
            text=True,
            check=True,
        )

    binary_paths = {
        "r680_adapter": "build/install/luxi_device_adapters/lib/luxi_device_adapters/r680_adapter",
        "control_manager": "build/install/luxi_control_manager/lib/luxi_control_manager/control_manager",
    }
    if args.base_only:
        del binary_paths["control_manager"]
    assets = {
        "Project/luxi_web_ctrl/web/app.js": "build/install/luxi_web_ctrl/share/luxi_web_ctrl/web/app.js",
        "Project/luxi_web_ctrl/web/index.html": "build/install/luxi_web_ctrl/share/luxi_web_ctrl/web/index.html",
    }
    backup = work / "installed-backup"
    backup.mkdir(exist_ok=False)
    for name in [*binary_paths.values(), *assets.values()]:
        source = root / name
        destination = backup / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
    # Recheck after backup and credential setup; no active device is restarted.
    require_idle(state(), args.base_only)
    for name in manifest:
        atomic_copy(work / "candidate" / name, root / name)
    for name, destination in binary_paths.items():
        atomic_copy(work / name, root / destination)
    services = ["luxi-base.service"]
    if not args.base_only:
        services.append("luxi-program.service")
    for service in services:
        subprocess.run(["sudo", "-n", "systemctl", "restart", service], check=True)
        print("Restarted idle " + service, flush=True)
    deadline = time.monotonic() + 20
    while True:
        snapshot = state()
        base = snapshot.get("base_status", {})
        if (
            base.get("web_keyboard_control")
            and base.get("feedback_fresh")
            and (not args.base_only or base.get("web_keyboard_manual_release"))
        ):
            if base.get("enabled") or base.get("keyboard_owner"):
                raise RuntimeError("Unexpected active base after install; inspect state")
            break
        if time.monotonic() >= deadline:
            raise RuntimeError("Updated base status did not become fresh")
        time.sleep(0.2)
    for source, destination in assets.items():
        atomic_copy(work / "candidate" / source, root / destination)
    (work / "after-install-state.json").write_text(json.dumps(snapshot, indent=2))
    receipt = {
        "source": manifest,
        "installed_binaries": {name: sha256(root / path) for name, path in binary_paths.items()},
        "restarted_services": services,
        "motion_commands_sent": False,
        "web_control_default": "off",
    }
    (work / "install-receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, indent=2))


if __name__ == "__main__":
    main()
