"""Bounded LZ4 blocks using the system library, without changing depth values."""

import ctypes
import ctypes.util
from functools import lru_cache


@lru_cache(maxsize=1)
def library():
    path = ctypes.util.find_library("lz4")
    if not path:
        raise RuntimeError("System liblz4 is required for lossless RGB-D transport")
    lib = ctypes.CDLL(path)
    lib.LZ4_compressBound.argtypes = [ctypes.c_int]
    lib.LZ4_compressBound.restype = ctypes.c_int
    for name in ("LZ4_compress_default", "LZ4_decompress_safe"):
        function = getattr(lib, name)
        function.argtypes = [ctypes.c_char_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
        function.restype = ctypes.c_int
    return lib


def compress(body):
    if not 0 < len(body) <= 24 * 1024 * 1024:
        raise ValueError("Invalid source size for lossless RGB-D")
    lib = library()
    bound = lib.LZ4_compressBound(len(body))
    output = ctypes.create_string_buffer(bound)
    size = lib.LZ4_compress_default(body, output, len(body), bound)
    if size <= 0:
        raise ValueError("Lossless RGB-D compression failed")
    return output.raw[:size]


def decompress(body, expected_size, maximum):
    if not 0 < expected_size <= maximum or not 0 < len(body) <= maximum:
        raise ValueError("Invalid or oversized lossless RGB-D block")
    output = ctypes.create_string_buffer(expected_size)
    size = library().LZ4_decompress_safe(body, output, len(body), expected_size)
    if size != expected_size:
        raise ValueError("Corrupt or truncated lossless RGB-D block")
    return output.raw[:size]
