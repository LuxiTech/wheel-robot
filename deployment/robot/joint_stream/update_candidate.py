#!/usr/bin/env python3
"""Upgrade an existing stream adapter after mock verification and disabled-state checks."""

import argparse
import datetime
import hashlib
import json
import os
import shutil
import subprocess
import time
from pathlib import Path

from install_candidate import real_disabled_state


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-root", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--verification", type=Path, required=True)
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    root = args.sdk_root.resolve()
    candidate = args.candidate.resolve()
    manifest = json.loads((candidate / "manifest.json").read_text())
    verification = json.loads(args.verification.read_text())
    if (
        verification.get("passed") is not True
        or verification.get("mock") is not True
        or verification.get("allow_hardware") is not False
        or not {
            "mock_enable_only_no_hardware",
            "100hz_udp_references_with_30hz_synthetic_images",
            "invalid_duplicate_old_session_packets_and_unowned_stop_rejected",
            "command_loss_holds_without_automatic_resume",
            "new_commands_do_not_refresh_old_image_time",
            "authority_loss_stops_and_prevents_restart",
        }.issubset(verification.get("checks", []))
    ):
        raise RuntimeError("Successful isolated SDK verification required")
    relatives = (
        "openarm_adapter.cpp",
        "joint_stream.hpp",
        "build/openarm_adapter_stream_candidate",
    )
    for relative in relatives:
        if digest(candidate / relative) != verification["verified_sha256"][relative]:
            raise RuntimeError("Candidate changed after verification: " + relative)
    if '"joint_stream_fixed_target_protocol"' in (candidate / "openarm_adapter.cpp").read_text():
        if verification.get("fixed_target_three_seconds_passed") is not True:
            raise RuntimeError("Fixed-target isolated verification required")
    planned = (
        '"joint_stream_planned_trajectory_protocol"'
        in (candidate / "openarm_adapter.cpp").read_text()
    )
    if (
        planned
        and "planned_trajectory_session_survives_three_seconds" not in verification["checks"]
    ):
        raise RuntimeError("Planned trajectory isolated verification required")
    source = root / "Project/device_adapters/src/openarm_adapter.cpp"
    header = source.with_name("joint_stream.hpp")
    binary = (
        root / "build/install/luxi_device_adapters/lib/luxi_device_adapters/openarm_adapter"
    ).resolve()
    if digest(source) != manifest["source_sha256"]:
        raise RuntimeError("Running SDK source changed since candidate preparation")
    known_versions = {
        # Live fixed-target build inspected on 2026-11-16 before MoveIt adaptation.
        "bfac0cd0c33af88810682b83ef7f919194461ed2cf500d61d8af428ee4b9fcfa": 2000000000,
        # Deployed consecutive-reference receiver, inspected before fixed-target update.
        "21f533f966611804d741f762117600482b07f18d76931ad5ab69bff046ce0d0c": 2000000000,
        # First deployed stream binary, before the image TTL update.
        "3d233a755e932baae35fe7597e311a8cdc67f4dbe3565bf9b13740b5459fc047": 500000000,
        # Verified 2s deployment used for the recorded live alignment runs.
        "96f42388d7bae0154765336dc3b77aad781146781d5bc0db0636e7a1bee2e78b": 2000000000,
    }
    running_binary = digest(binary)
    if running_binary not in known_versions:
        raise RuntimeError("Unknown running SDK binary; inspect before upgrading")
    expected_ttl = known_versions[running_binary]
    if f"stream_image_age_ns = {expected_ttl};" not in header.read_text():
        raise RuntimeError("Running header does not match the known binary version")
    expected_header = (
        "e967a7647792686466a28ae695fa0faf0d41cbbc344e25cb6bbfc9d0e5c5a39b"
        if running_binary == "21f533f966611804d741f762117600482b07f18d76931ad5ab69bff046ce0d0c"
        else "1adfa87a984aafb8d081de0a84dd5a519c09208b33f123d00663c260f51aef30"
    )
    if running_binary == "bfac0cd0c33af88810682b83ef7f919194461ed2cf500d61d8af428ee4b9fcfa":
        expected_header = "ce3f781de2feb131e257d6ce5c099e65c3f94fb7b007127504929b43bc25b4b2"
    if expected_ttl == 2000000000 and digest(header) != expected_header:
        raise RuntimeError("Installed receiver changed since the live fault analysis")
    url = "http://127.0.0.1:18030"
    before = real_disabled_state(url)
    if not args.apply:
        print(json.dumps(dict(prepared=True, arms_disabled=True, applied=False)))
        return
    if os.geteuid() != 0:
        raise RuntimeError("Apply requires root for atomic installation")
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    backup = candidate.parent / ("deployment-stream-update-" + stamp)
    backup.mkdir(exist_ok=False)
    paths = dict(source=source, header=header, binary=binary)
    for name, path in paths.items():
        shutil.copy2(path, backup / name)
    (backup / "before.json").write_text(json.dumps(before, indent=2) + "\n")
    report = dict(
        installed=False,
        restarted=False,
        motion_commands_sent=False,
        backup=str(backup),
        verified_sha256=verification["verified_sha256"],
    )

    def replace(path, content):
        attributes = path.stat()
        temporary = path.with_name(path.name + ".stream-update-new")
        with temporary.open("xb") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.chmod(attributes.st_mode & 0o777)
        os.chown(temporary, attributes.st_uid, attributes.st_gid)
        os.replace(temporary, path)

    # 必须再次确认双臂失能。禁止重启正在使能保持的适配器，避免意外失去支撑。
    real_disabled_state(url)
    subprocess.run(["systemctl", "stop", "luxi-openarm-monitor.service"], check=True)
    try:
        for path, relative in zip(paths.values(), relatives):
            replace(path, (candidate / relative).read_bytes())
        report["installed"] = True
        subprocess.run(["systemctl", "start", "luxi-openarm-monitor.service"], check=True)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            try:
                state = real_disabled_state(url)
                if state["arm_startup"]["joint_control"].get(
                    "joint_stream_image_max_age_ms"
                ) == 2000 and (
                    not planned
                    or state["arm_startup"]["joint_control"].get(
                        "joint_stream_planned_trajectory_protocol"
                    )
                    == 1
                ):
                    report.update(restarted=True, after=state)
                    break
            except Exception:
                pass
            time.sleep(0.2)
        else:
            raise RuntimeError("New 2000ms SDK did not become fresh and disabled")
    except BaseException as error:
        report["error"] = str(error)
        # 失败后保留停机，恢复原文件但不盲目启动或发使能命令。
        subprocess.run(["systemctl", "stop", "luxi-openarm-monitor.service"], check=False)
        for name, path in paths.items():
            replace(path, (backup / name).read_bytes())
        report["rolled_back_files_service_stopped"] = True
        raise
    finally:
        (backup / "result.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report))


if __name__ == "__main__":
    main()
