#!/usr/bin/env python3
"""Compile isolated SDK candidates with the live build's flags and link inputs."""

import argparse
import concurrent.futures
import hashlib
import json
import shlex
import subprocess
from pathlib import Path

TARGETS = {
    "control_manager": ("luxi_control_manager", "control_manager"),
    "r680_adapter": ("luxi_device_adapters", "device_adapters"),
    "lift_adapter": ("luxi_device_adapters", "device_adapters"),
    "openarm_adapter": ("luxi_device_adapters", "device_adapters"),
    "web_ctrl": ("luxi_web_ctrl", "luxi_web_ctrl"),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("work", type=Path)
    parser.add_argument("--skip-existing", action="store_true")
    parser.add_argument("--jobs", type=int, choices=[1, 2], default=1)
    args = parser.parse_args()
    binary_dir = args.work / "bin"
    binary_dir.mkdir(exist_ok=True)

    def build_target(item):
        target, (package, source_dir) = item
        build = args.root / "build/packages" / package
        cmake = build / "CMakeFiles" / (target + ".dir")
        flags = {}
        for line in (cmake / "flags.make").read_text().splitlines():
            if line.startswith("CXX_") and " = " in line:
                key, value = line.split(" = ", 1)
                flags[key] = shlex.split(value)
        original = args.root / "Project" / source_dir / "src" / (target + ".cpp")
        source = args.work / "candidate" / original.relative_to(args.root)
        obj = binary_dir / (target + ".o")
        binary = binary_dir / target
        if args.skip_existing and binary.is_file():
            print("Keeping completed candidate " + target, flush=True)
            return target, hashlib.sha256(binary.read_bytes()).hexdigest()
        compile_command = [
            "/usr/bin/c++",
            "-I" + str(args.work / "candidate/Project/robot_core/include"),
            "-I" + str(original.parent),
            *flags["CXX_DEFINES"],
            *flags["CXX_INCLUDES"],
            *flags["CXX_FLAGS"],
            "-c",
            str(source),
            "-o",
            str(obj),
        ]
        print("Compiling " + target, flush=True)
        with (args.work / ("build-" + target + ".log")).open("w") as log:
            subprocess.run(
                compile_command, cwd=build, stdout=log, stderr=subprocess.STDOUT, check=True
            )
            link = shlex.split((cmake / "link.txt").read_text())
            object_name = "CMakeFiles/" + target + ".dir/src/" + target + ".cpp.o"
            if link.count(object_name) != 1:
                raise RuntimeError("Unexpected link input: " + target)
            link[link.index(object_name)] = str(obj)
            link[link.index("-o") + 1] = str(binary)
            subprocess.run(link, cwd=build, stdout=log, stderr=subprocess.STDOUT, check=True)
        print("Built " + target, flush=True)
        return target, hashlib.sha256(binary.read_bytes()).hexdigest()

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        receipt = dict(pool.map(build_target, TARGETS.items()))
    (args.work / "build-receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")


if __name__ == "__main__":
    main()
