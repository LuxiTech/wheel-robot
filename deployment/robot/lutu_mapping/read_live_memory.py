#!/usr/bin/env python3
"""Sample known FAST-LIO globals with process_vm_readv; never stop or write the process."""

import argparse
import ctypes
import json
import os
import struct
import subprocess
import time
from pathlib import Path


class IOVec(ctypes.Structure):
    _fields_ = [("base", ctypes.c_void_p), ("length", ctypes.c_size_t)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pid", type=int)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    executable = Path(f"/proc/{args.pid}/exe").resolve()
    symbols = {}
    for line in subprocess.check_output(["nm", "-C", str(executable)], text=True).splitlines():
        columns = line.split()
        if len(columns) == 3 and columns[2] in {"Measures", "feats_undistort", "effct_feat_num"}:
            symbols[columns[2]] = int(columns[0], 16)
    base = None
    for line in Path(f"/proc/{args.pid}/maps").read_text().splitlines():
        columns = line.split()
        if len(columns) >= 6 and columns[-1] == str(executable) and int(columns[2], 16) == 0:
            base = int(columns[0].split("-")[0], 16)
            break
    if base is None:
        raise RuntimeError("Executable mapping not found")
    libc = ctypes.CDLL(None, use_errno=True)
    function = libc.process_vm_readv
    function.argtypes = [
        ctypes.c_int,
        ctypes.POINTER(IOVec),
        ctypes.c_ulong,
        ctypes.POINTER(IOVec),
        ctypes.c_ulong,
        ctypes.c_ulong,
    ]
    function.restype = ctypes.c_ssize_t

    def read(address, length):
        buffer = ctypes.create_string_buffer(length)
        local = IOVec(ctypes.cast(buffer, ctypes.c_void_p), length)
        remote = IOVec(address, length)
        returned = function(args.pid, ctypes.byref(local), 1, ctypes.byref(remote), 1, 0)
        if returned != length:
            raise OSError(ctypes.get_errno(), os.strerror(ctypes.get_errno()))
        return buffer.raw

    samples = []
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        measurement = read(base + symbols["Measures"], 112)
        begin, end = struct.unpack_from("dd", measurement)
        # libstdc++ deque<shared_ptr<const Imu>>: map/map_size, start[4], finish[4].
        deque = struct.unpack_from("10Q", measurement, 32)
        start_cur, start_first, start_last, start_node = deque[2:6]
        finish_cur, finish_first, finish_last, finish_node = deque[6:10]
        count = (
            (finish_node - start_node) // 8 * 32
            + (finish_cur - finish_first) // 16
            - (start_cur - start_first) // 16
        )
        if (
            0 <= count < 10000
            and start_first <= start_cur <= start_last
            and finish_first <= finish_cur <= finish_last
        ):
            point_cloud = struct.unpack("Q", read(base + symbols["feats_undistort"], 8))[0]
            # PCLHeader is 48 bytes on the deployed aarch64 libstdc++ ABI.
            vector_begin, vector_end = struct.unpack("QQ", read(point_cloud + 48, 16))
            samples.append(
                {
                    "monotonic": time.monotonic(),
                    "scan_begin": begin,
                    "scan_end": end,
                    "imu_count": count,
                    "feature_storage_bytes": vector_end - vector_begin,
                }
            )
        time.sleep(0.01)
    empty_frames = {
        sample["scan_begin"]
        for sample in samples
        if sample["imu_count"] == 0 and sample["feature_storage_bytes"] > 0
    }
    result = {
        "pid": args.pid,
        "read_method": "process_vm_readv",
        "process_stopped": False,
        "process_written": False,
        "sample_count": len(samples),
        "unique_empty_imu_frames_with_existing_features": len(empty_frames),
        "empty_frame_stamps": sorted(empty_frames),
        "samples": samples,
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key: value for key, value in result.items() if key != "samples"}))


if __name__ == "__main__":
    main()
