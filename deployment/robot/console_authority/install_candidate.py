#!/usr/bin/env python3
"""Install verified ownership candidates while measured devices are idle."""

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


def fetch(path):
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open("http://127.0.0.1:8000" + path, timeout=5) as response:
        return json.load(response)


def require_idle(state):
    arms = state.get("arms") or {}
    base = state.get("base_status") or {}
    lift = state.get("lift_status") or {}
    startup = state.get("arm_startup") or {}
    recorder = state.get("recorder") or {}
    control = state.get("control") or {}
    if (
        not arms.get("fresh")
        or arms.get("enabled")
        or len(arms.get("valid", [])) != 16
        or not all(arms["valid"])
    ):
        raise RuntimeError("Keep arm hold intact: install requires fresh disabled arms")
    if any(abs(value) > 0.08 for value in arms.get("velocity", [])):
        raise RuntimeError("Arms are not stationary")
    if startup.get("joint_control", {}).get("phase") in ["preparing", "moving"]:
        raise RuntimeError("A joint request is active")
    if (
        not base.get("fresh")
        or not base.get("feedback_fresh")
        or base.get("enabled")
        or not base.get("stationary")
    ):
        raise RuntimeError("Base must be disabled and measured stationary")
    if (
        not lift.get("fresh")
        or not lift.get("feedback_healthy")
        or lift.get("enabled")
        or lift.get("session_present")
        or not lift.get("stationary")
    ):
        raise RuntimeError("Lift must be stationary with no active control session")
    if recorder.get("state") not in [None, "idle"]:
        raise RuntimeError("A recording or recorder transition is active")
    if control.get("fresh") and control.get("system_phase") not in ["disabled", "stopped"]:
        raise RuntimeError("Teleoperation is active")


def atomic_copy(source, target):
    target = target.resolve()
    temporary = target.with_name(target.name + ".authority-new")
    shutil.copy2(source, temporary)
    if target.exists():
        temporary.chmod(target.stat().st_mode)
    temporary.replace(target)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("work", type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    work = args.work.resolve()
    manifest = json.loads((work / "manifest.json").read_text())
    verified = json.loads((work / "tests-passed.json").read_text())
    for name, metadata in manifest.items():
        if digest(root / name) != metadata["before_sha256"]:
            raise RuntimeError("Live source changed; preserve and rebase: " + name)
        if digest(work / "candidate" / name) != metadata["candidate_sha256"]:
            raise RuntimeError("Candidate source changed: " + name)
    for name, expected in verified["binaries"].items():
        if digest(work / "bin" / name) != expected:
            raise RuntimeError("Binary changed after tests: " + name)
    protected = json.loads((work / "protected-before.json").read_text())
    for name, expected in protected.items():
        if digest(root / name) != expected:
            raise RuntimeError("Protected SDK model/config changed during preparation: " + name)
    before = fetch("/api/state")
    require_idle(before)
    (work / "before-install-state.json").write_text(json.dumps(before, indent=2) + "\n")
    subprocess.run(["sudo", "-v"], check=True)
    binaries = {
        "control_manager": "luxi_control_manager",
        "r680_adapter": "luxi_device_adapters",
        "lift_adapter": "luxi_device_adapters",
        "openarm_adapter": "luxi_device_adapters",
        "web_ctrl": "luxi_web_ctrl",
    }
    replacements = {name: work / "candidate" / name for name in manifest}
    for target, package in binaries.items():
        replacements[f"build/install/{package}/lib/{package}/{target}"] = work / "bin" / target
    for asset in ["app.js", "joint-control.js", "index.html"]:
        source = "Project/luxi_web_ctrl/web/" + asset
        installed = "build/install/luxi_web_ctrl/share/luxi_web_ctrl/web/" + asset
        if digest(root / installed) != manifest[source]["before_sha256"]:
            raise RuntimeError("Installed asset has separate edits: " + installed)
        replacements[installed] = work / "candidate" / source
    header = "Project/robot_core/include/luxi/coordinated_control.hpp"
    installed_header = "build/install/luxi_core/include/luxi/coordinated_control.hpp"
    if digest(root / installed_header) != manifest[header]["before_sha256"]:
        raise RuntimeError("Installed coordination header has separate edits")
    replacements[installed_header] = work / "candidate" / header
    backup = work / "installed-backup"
    backup.mkdir(exist_ok=False)
    for name in replacements:
        saved = backup / name
        saved.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(root / name, saved)
    devices_before = fetch("/api/devices")
    (work / "before-devices.json").write_text(json.dumps(devices_before, indent=2) + "\n")
    boot_state = subprocess.run(
        ["systemctl", "is-enabled", "luxi-program.service"], capture_output=True, text=True
    ).stdout.strip()
    (work / "before-program-boot-state.txt").write_text(boot_state + "\n")
    require_idle(fetch("/api/state"))
    for name, source in replacements.items():
        atomic_copy(source, root / name)
    # No Quest or camera service is restarted. The lift is session-free, so its
    # shutdown path cannot close another owner's session or issue global STOP.
    restart = [
        "luxi-openarm-monitor.service",
        "luxi-base.service",
        "luxi-lift.service",
        "luxi-web.service",
        "luxi-rgbd-gateway.service",
    ]
    for service in restart:
        subprocess.run(["sudo", "-n", "systemctl", "restart", service], check=True)
        print("Restarted idle " + service, flush=True)
    # This program now supplies console permission heartbeats as well as teleop.
    # Boot always selects console and cannot issue an enable or a motion goal.
    subprocess.run(["sudo", "-n", "systemctl", "enable", "luxi-program.service"], check=True)
    program_action = (
        "restart"
        if devices_before.get("devices", {}).get("program", {}).get("ActiveState") == "active"
        else "start"
    )
    subprocess.run(["sudo", "-n", "systemctl", program_action, "luxi-program.service"], check=True)
    print("Started coordination program with console selected; no enable command", flush=True)
    deadline = time.monotonic() + 50
    while time.monotonic() < deadline:
        try:
            state = fetch("/api/state")
            if (
                state.get("control", {}).get("control_source") == "console"
                and state.get("control", {}).get("fresh")
                and state.get("base_status", {}).get("keyboard_takeover_available")
            ):
                break
        except (OSError, AttributeError):
            pass
        time.sleep(0.3)
    else:
        raise RuntimeError("Authority did not become ready; inspect installed services")
    (work / "after-install-state.json").write_text(json.dumps(state, indent=2) + "\n")
    require_idle(state)
    if any(digest(root / name) != expected for name, expected in protected.items()):
        raise RuntimeError("Protected SDK model/config changed during install")
    receipt = {
        "installed": {name: digest(root / name) for name in replacements},
        "restarted_services": restart,
        "started_service": "luxi-program.service",
        "program_action": program_action,
        "program_boot_state_before": boot_state,
        "program_boot_state_after": "enabled",
        "protected_model_and_config_unchanged": True,
        "control_source": state["control"]["control_source"],
        "motion_commands_sent": False,
        "arms_enabled_after": state["arms"]["enabled"],
        "lift_session_after": state["lift_status"]["session_present"],
        "arm_output_enable_count_after": state["arm_startup"]["output_enable_count"],
        "arm_output_disable_count_after": state["arm_startup"]["output_disable_count"],
    }
    (work / "install-receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, indent=2))


if __name__ == "__main__":
    main()
