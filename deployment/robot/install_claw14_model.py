"""Install the verified claw1.4 model beside old SDK models, without starting devices."""

import argparse
import hashlib
import json
import os
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

LEGACY_HASHES = {
    "openarm_bimanual.xml": "cb4aad7ef1ce36137ccc07894186f06a60fa45251f3bb94da91aa4be6bc23ae5",
    "cell/cell.xml": "64af269bf30d4da64df773ac0956a1190b6ae090496d87d7a7c124c8912b98fc",
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-root", required=True, type=Path)
    parser.add_argument("--work", required=True, type=Path)
    args = parser.parse_args()
    root, work = args.sdk_root.resolve(), args.work.resolve()
    staged = work / "staged"
    result_path = work / "deployment.json"
    if result_path.exists():
        raise ValueError("Deployment record already exists; inspect before another installation")
    probe = json.loads((work / "native_validation.json").read_text())
    if probe["commands_sent"] is not False or max(probe["ik_error_m"]) > 0.02:
        raise ValueError("Native SDK verification did not pass")
    source_model = staged / "openarm_claw14.xml"
    manifest = json.loads((staged / "openarm_claw14.manifest.json").read_text())
    if digest(source_model) != manifest["xml_sha256"]:
        raise ValueError("Staged XML fingerprint mismatch")
    destinations = [
        root / "device/openarm/openarm_mujoco/v2",
        root / "build/install/luxi_description/share/luxi_description/model",
    ]
    old_files = {}
    for folder in destinations:
        for relative, expected in LEGACY_HASHES.items():
            path = folder / relative
            if digest(path) != expected:
                raise ValueError("Legacy model changed; inspect " + str(path))
            old_files[str(path)] = expected
        for name in ("openarm_claw14.xml", "claw14_meshes", "openarm_claw14.manifest.json"):
            if (folder / name).exists():
                raise ValueError("New model destination already exists: " + str(folder / name))
    launches = [
        root / "Project/robot_bringup/launch/teleop.launch.xml",
        root / "Project/luxi_web_ctrl/scripts/program.launch.xml",
        root / "build/install/luxi_bringup/share/luxi_bringup/launch/teleop.launch.xml",
    ]
    launches += list((root / "build/install/luxi_web_ctrl").rglob("program.launch.xml"))
    old_value = "$(find-pkg-share luxi_description)/model/cell/cell.xml"
    new_value = "$(find-pkg-share luxi_description)/model/openarm_claw14.xml"
    replacements = {}
    for path in launches:
        text = path.read_text()
        if old_value not in text or '<arg name="model"' in text:
            raise ValueError("Unexpected launch model selection; inspect " + str(path))
        revised = text.replace(old_value, "$(var model)").replace(
            "<launch>", '<launch>\n <arg name="model" default="' + new_value + '"/>', 1
        )
        ET.fromstring(revised)
        backup = path.with_name(path.name + ".before-claw14-20261104")
        if backup.exists():
            raise ValueError("Launch backup already exists; inspect " + str(backup))
        replacements[path] = (revised, backup, digest(path))
    backups = {}
    for path, (_, backup, previous_hash) in replacements.items():
        shutil.copy2(path, backup)
        backups[str(path)] = dict(backup=str(backup), previous_sha256=previous_hash)
    installed = {}
    for folder in destinations:
        shutil.copytree(staged / "claw14_meshes", folder / "claw14_meshes")
        shutil.copy2(source_model, folder / "openarm_claw14.xml")
        shutil.copy2(
            staged / "openarm_claw14.manifest.json", folder / "openarm_claw14.manifest.json"
        )
        if digest(folder / "openarm_claw14.xml") != manifest["xml_sha256"]:
            raise ValueError("Installed model fingerprint mismatch")
        for source in (staged / "claw14_meshes").iterdir():
            if digest(source) != digest(folder / "claw14_meshes" / source.name):
                raise ValueError("Installed mesh fingerprint mismatch")
        installed[str(folder / "openarm_claw14.xml")] = manifest["xml_sha256"]
    for path, (revised, _, previous_hash) in replacements.items():
        if digest(path) != previous_hash:
            raise ValueError("Launch changed during installation; inspect " + str(path))
        temporary = path.with_name(path.name + ".claw14-tmp")
        with temporary.open("x") as stream:
            stream.write(revised)
        shutil.copymode(path, temporary)
        os.replace(temporary, path)
        backups[str(path)]["selected_sha256"] = digest(path)
    for path, expected in old_files.items():
        if digest(Path(path)) != expected:
            raise ValueError("Legacy model was altered: " + path)
    report = dict(
        installed_models=installed,
        legacy_models_unchanged=old_files,
        launches=backups,
        robot_motion_commands_sent=False,
        services_started_or_restarted=False,
        selection="claw1.4 for subsequent SDK teleop/program starts; no active motion node restarted",
        native_sdk_validation=probe["ik_error_m"],
    )
    with result_path.open("x") as stream:
        stream.write(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
