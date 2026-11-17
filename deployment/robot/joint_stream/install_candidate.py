#!/usr/bin/env python3
"""Install the verified candidate only from fresh, disabled real-arm feedback."""

import argparse
import datetime
import hashlib
import ipaddress
import json
import os
import shlex
import shutil
import subprocess
from pathlib import Path
from urllib.request import ProxyHandler, build_opener


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def real_disabled_state(url):
    with build_opener(ProxyHandler({})).open(url + "/api/state", timeout=3) as response:
        state = json.load(response)
    arms = state.get("arms", {})
    startup = state.get("arm_startup", {})
    now = int(state["server_mono_ns"])
    if state.get("system", {}).get("mock") is not False:
        raise RuntimeError("Real robot feedback required")
    if arms.get("enabled") is not False or arms.get("fresh") is not True:
        raise RuntimeError("Fresh disabled arms required; an enabled hold cannot be restarted")
    if len(arms.get("valid", [])) != 16 or not all(value is True for value in arms["valid"]):
        raise RuntimeError("All sixteen joints must have valid feedback")
    if not 0 <= now - int(arms["receive_mono_ns"]) <= 150_000_000:
        raise RuntimeError("Real arm feedback expired")
    if startup.get("fresh") is not True or startup["joint_control"]["phase"] not in (
        "idle",
        "ready",
        "stopped",
    ):
        raise RuntimeError("No active request or latched fault permitted")
    if state.get("control", {}).get("control_source") != "console":
        raise RuntimeError("Console authority required")
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-root", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    parser.add_argument("--state-url", default="http://192.168.2.199:18030")
    parser.add_argument("--bind", default="192.168.2.199")
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    args.bind = str(ipaddress.IPv4Address(args.bind))
    root, candidate = args.sdk_root.resolve(), args.candidate.resolve()
    evidence = json.loads(args.evidence.read_text())
    for value in evidence["resources"].values():
        if digest(Path(value["path"])) != value["sha256"]:
            raise RuntimeError(
                "Resource changed since successful mock verification: " + value["path"]
            )
    mock = json.loads((candidate.parent / "mock-verification-timed/result.json").read_text())
    if mock.get("passed") is not True or mock.get("allow_hardware") is not False:
        raise RuntimeError("Successful isolated mock verification required")
    state = real_disabled_state(args.state_url)
    source = root / "Project/device_adapters/src/openarm_adapter.cpp"
    header = source.with_name("joint_stream.hpp")
    binary = (
        root / "build/install/luxi_device_adapters/lib/luxi_device_adapters/openarm_adapter"
    ).resolve()
    wrapper = root / "Project/luxi_web_ctrl/scripts/run_joint_stream_arms.sh"
    drop = Path("/etc/systemd/system/luxi-openarm-monitor.service.d/joint-stream.conf")
    if any(path.exists() for path in (header, wrapper, drop)):
        raise RuntimeError("New deployment paths already exist; inspect before replacing")
    if not args.apply:
        print(
            json.dumps(
                dict(prepared=True, arms_disabled=True, candidate=str(candidate), applied=False)
            )
        )
        return
    if os.geteuid() != 0:
        raise RuntimeError("Apply requires root for atomic installation and service configuration")
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    backup = candidate.parent / ("deployment-" + stamp)
    backup.mkdir(exist_ok=False)
    uid, gid = source.stat().st_uid, source.stat().st_gid
    originals = {"adapter.cpp": source, "adapter_binary": binary}
    for name, path in originals.items():
        shutil.copy2(path, backup / name)
    (backup / "before.json").write_text(json.dumps(state, indent=2) + "\n")
    (backup / "service-before.txt").write_text(
        subprocess.check_output(["systemctl", "cat", "luxi-openarm-monitor.service"], text=True)
    )
    report = dict(backup=str(backup), installed=False, restarted=False, motion_commands_sent=False)

    def replace(path, data, mode, owner_uid, owner_gid):
        temporary = path.with_name(path.name + ".joint-stream-new")
        if temporary.exists():
            raise RuntimeError("Unexpected staging file: " + str(temporary))
        temporary.write_bytes(data)
        temporary.chmod(mode)
        os.chown(temporary, owner_uid, owner_gid)
        os.replace(temporary, path)

    # Recheck immediately before stopping; no holding torque is removed.
    real_disabled_state(args.state_url)
    subprocess.run(["systemctl", "stop", "luxi-openarm-monitor.service"], check=True)
    try:
        replace(source, (candidate / "openarm_adapter.cpp").read_bytes(), 0o644, uid, gid)
        replace(header, (candidate / "joint_stream.hpp").read_bytes(), 0o644, uid, gid)
        replace(
            binary,
            (candidate / "build/openarm_adapter_stream_candidate").read_bytes(),
            0o755,
            uid,
            gid,
        )
        launcher = f"""#!/usr/bin/env bash
set -eo pipefail
SDK_ROOT={shlex.quote(str(root))}
source /opt/ros/humble/setup.bash
source "$SDK_ROOT/build/install/setup.bash"
export ROS_LOG_DIR="$SDK_ROOT/log/ros"
exec "$SDK_ROOT/build/install/luxi_device_adapters/lib/luxi_device_adapters/openarm_adapter" --ros-args \\
  -p "config:=$SDK_ROOT/Project/robot_bringup/config/arms-kh.json" \\
  -p mock:=false -p allow_hardware:=true -p "monitor_only:=${{LUXI_ACQUISITION_ONLY:-true}}" \\
  -p "joint_stream_bind:={args.bind}" -p joint_stream_port:=18034
"""
        replace(wrapper, launcher.encode(), 0o755, uid, gid)
        drop.parent.mkdir(exist_ok=True)
        replace(
            drop, ("[Service]\nExecStart=\nExecStart=" + str(wrapper) + "\n").encode(), 0o644, 0, 0
        )
        subprocess.run(["systemctl", "daemon-reload"], check=True)
        subprocess.run(["systemctl", "start", "luxi-openarm-monitor.service"], check=True)
        report.update(installed=True, restarted=True, installed_binary_sha256=digest(binary))
    except BaseException as error:
        # A failed start must not leave a partially switched service retrying.
        subprocess.run(["systemctl", "stop", "luxi-openarm-monitor.service"], check=False)
        replace(source, (backup / "adapter.cpp").read_bytes(), 0o644, uid, gid)
        replace(binary, (backup / "adapter_binary").read_bytes(), 0o755, uid, gid)
        for path in (header, wrapper, drop):
            path.unlink(missing_ok=True)
        subprocess.run(["systemctl", "daemon-reload"], check=True)
        report.update(rolled_back=True, error=str(error), service_left_stopped=True)
        raise
    finally:
        (backup / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()
