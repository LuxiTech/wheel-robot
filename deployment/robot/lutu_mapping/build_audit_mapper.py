#!/usr/bin/env python3
"""Build an isolated diagnostic mapper; never replace the production executable."""

import argparse
import hashlib
import json
import shlex
import subprocess
from pathlib import Path


def replace_once(text, before, after):
    if text.count(before) != 1:
        raise ValueError(f"Expected one source anchor: {before[:100]!r}")
    return text.replace(before, after, 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("work", type=Path)
    args = parser.parse_args()
    source = args.root / "project/luxi-atlas-lio/src/laserMapping.cpp"
    original = source.read_text()
    candidate = replace_once(original, "#include <thread>", "#include <thread>\n#include <cstdlib>")
    candidate = replace_once(
        candidate,
        "            p_imu->Process(Measures, kf, feats_undistort);",
        """            if (Measures.imu.empty()) {
                std::cerr << "AUDIT_EMPTY_IMU t=" << Measures.lidar_beg_time - first_lidar_time
                          << " reused_points=" << feats_undistort->size() << std::endl;
            }
            p_imu->Process(Measures, kf, feats_undistort);""",
    )
    candidate = replace_once(
        candidate,
        "            /******* Publish odometry *******/",
        """            std::cerr << "AUDIT_STATE t=" << Measures.lidar_beg_time - first_lidar_time
                      << " imu=" << Measures.imu.size() << " features=" << feats_down_size
                      << " matched=" << effct_feat_num << " pos=" << state_point.pos.norm()
                      << " speed=" << state_point.vel.norm() << std::endl;
            /******* Publish odometry *******/""",
    )
    candidate = replace_once(
        candidate,
        "    void map_publish_callback()\n    {\n        if (map_pub_en)",
        """    void map_publish_callback()
    {
        const char* stall_ms = std::getenv("LUTU_AUDIT_MAP_STALL_MS");
        if (stall_ms) std::this_thread::sleep_for(std::chrono::milliseconds(std::atoi(stall_ms)));
        if (map_pub_en)""",
    )
    args.work.mkdir(parents=True, exist_ok=True)
    (args.work / "audit").mkdir(exist_ok=True)
    isolated_source = args.work / "audit/laserMapping.cpp"
    isolated_source.write_text(candidate)
    diagnostics_root = args.work / "audit-root"
    for name in ["Log", "PCD"]:
        (diagnostics_root / name).mkdir(parents=True, exist_ok=True)
    build = args.root / "build/fast_lio"
    cmake = build / "CMakeFiles/fastlio_mapping.dir"
    flags = {}
    for line in (cmake / "flags.make").read_text().splitlines():
        if line.startswith("CXX_") and " = " in line:
            key, value = line.split(" = ", 1)
            flags[key] = shlex.split(value)
    defines = [flag for flag in flags["CXX_DEFINES"] if not flag.startswith("-DROOT_DIR=")]
    defines.append("-DROOT_DIR=" + json.dumps(str(diagnostics_root) + "/"))
    compile_command = [
        "/usr/bin/c++",
        "-I" + str(source.parent),
        *defines,
        *flags["CXX_INCLUDES"],
        *flags["CXX_FLAGS"],
        "-c",
        str(isolated_source),
        "-o",
        str(args.work / "audit_mapper.o"),
    ]
    with (args.work / "build-audit.log").open("w") as log:
        print("Building isolated diagnostic mapper with production flags", flush=True)
        subprocess.run(compile_command, cwd=build, stdout=log, stderr=subprocess.STDOUT, check=True)
        link = shlex.split((cmake / "link.txt").read_text())
        object_name = "CMakeFiles/fastlio_mapping.dir/src/laserMapping.cpp.o"
        link[link.index(object_name)] = str(args.work / "audit_mapper.o")
        link[link.index("-o") + 1] = str(args.work / "audit_mapper")
        subprocess.run(link, cwd=build, stdout=log, stderr=subprocess.STDOUT, check=True)
    receipt = {
        "production_source_sha256": hashlib.sha256(original.encode()).hexdigest(),
        "audit_source_sha256": hashlib.sha256(candidate.encode()).hexdigest(),
        "audit_binary_sha256": hashlib.sha256(
            (args.work / "audit_mapper").read_bytes()
        ).hexdigest(),
        "compile_command": compile_command,
        "production_changed": False,
    }
    (args.work / "audit-build.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print("Diagnostic mapper ready", flush=True)


if __name__ == "__main__":
    main()
