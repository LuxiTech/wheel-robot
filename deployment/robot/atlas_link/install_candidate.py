#!/usr/bin/env python3
"""Install verified Atlas link candidates with backups; never acquires robot control."""

import argparse
import hashlib
import json
import shutil
import subprocess
import time
import urllib.request
from pathlib import Path


def request(url, body=None):
    payload = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=payload, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=8) as response:
        return json.load(response)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["atlas", "relay"])
    parser.add_argument("root", type=Path)
    parser.add_argument("work", type=Path)
    args = parser.parse_args()
    backup = args.work / "installed-backup"
    backup.mkdir()
    source = args.root / "project/luxi_lift_robot" if args.mode == "atlas" else args.root
    manifest = json.loads((args.work / "manifest.json").read_text())
    replacements = []
    for relative, fingerprint in manifest.items():
        target = source / relative
        candidate = args.work / "candidate" / relative
        assert digest(target) == fingerprint["original_sha256"], f"Live source changed: {target}"
        assert digest(candidate) == fingerprint["candidate_sha256"], (
            f"Candidate changed: {candidate}"
        )
        replacements.append((target, candidate))
    modules = []
    if args.mode == "relay":
        idle = request("http://127.0.0.1:18031/api/relay/status")
        base = request("http://127.0.0.1:18031/api/state")
        sdk = request("http://127.0.0.1:8000/api/state")
        assert not idle["armed"] and not idle["holder_client_id"]
        assert not base["base_status"]["enabled"]
        assert not base["base_status"]["external_control"]
        assert base["base_status"]["stationary"]
        (args.work / "sdk-before-install.json").write_text(json.dumps(sdk, indent=2) + "\n")
        replacements.append((args.root / "carrier_relay", args.work / "build/carrier_relay"))
    else:
        idle = request("http://127.0.0.1:8082/api/v1/robot")
        assert not idle["armed"] and not idle["holder_client_id"] and not idle["navigation_active"]
        modules = request("http://127.0.0.1:8082/api/v1/modules")["modules"]
        for module in modules:
            if module["module_id"] not in {"mapping", "lidar"}:
                assert module["process_state"] == "stopped", module
        state = request("http://127.0.0.1:8082/api/status")
        assert not state["pipeline"]["mapping_active"]
        (args.work / "modules-before-install.json").write_text(json.dumps(modules, indent=2) + "\n")
        replacements += [
            (args.root / "build/luxi_web_control/web_api_node", args.work / "web_api_node"),
            (
                args.root / "build/luxi_lift_robot/libluxi_carrier.so",
                args.work / "libluxi_carrier.so",
            ),
            (
                args.root / "install/luxi_lift_robot/lib/libluxi_carrier.so",
                args.work / "libluxi_carrier.so",
            ),
        ]
        web_source = args.root / "project/luxi-web-control/web-console"
        for relative in [
            "src/stores/manualControl.ts",
            "src/views/DeviceControlView.vue",
            "tests/manual_control.test.mjs",
        ]:
            replacements.append((web_source / relative, args.work / "web-candidate" / relative))
        for destination in [
            args.root / "project/luxi-web-control/web",
            args.root / "install/luxi_web_control/share/luxi_web_control/web",
        ]:
            for candidate in sorted((args.work / "web").rglob("*")):
                if candidate.is_file():
                    replacements.append(
                        (destination / candidate.relative_to(args.work / "web"), candidate)
                    )
    receipt = []
    for index, (target, candidate) in enumerate(replacements):
        assert candidate.is_file(), candidate
        original_hash = None
        if target.exists():
            original_hash = digest(target)
            shutil.copy2(target, backup / str(index))
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = target.with_name(target.name + ".atlas-link-next")
        shutil.copy2(candidate, temporary)
        temporary.replace(target)
        receipt.append(
            {
                "file": str(target),
                "backup": str(backup / str(index)),
                "before_sha256": original_hash,
                "after_sha256": digest(target),
            }
        )
    (args.work / "install-receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    if args.mode == "relay":
        subprocess.run(
            ["sudo", "-n", "systemctl", "restart", "luxi-atlas-carrier.service"], check=True
        )
    else:
        subprocess.run(["systemctl", "--user", "restart", "luxi-web-control.service"], check=True)
        deadline = time.monotonic() + 40
        while True:
            try:
                request("http://127.0.0.1:8082/api/v1/robot")
                break
            except (OSError, ValueError):
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.25)
        running = {item["module_id"] for item in modules if item["process_state"] == "running"}
        restore = "mapping" if "mapping" in running else "lidar" if "lidar" in running else None
        if restore:
            result = request(f"http://127.0.0.1:8082/api/v1/modules/{restore}/start", {})
            (args.work / "restore-sensors.json").write_text(json.dumps(result, indent=2) + "\n")
    print("Installed:", args.mode, "receipt:", args.work / "install-receipt.json", flush=True)


if __name__ == "__main__":
    main()
