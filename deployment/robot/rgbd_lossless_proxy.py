#!/usr/bin/env python3
"""Read-only, lossless HTTP compression for the existing SDK RGB-D endpoint."""

import argparse
import gzip
import re
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import ProxyHandler, build_opener

try:
    from wheel_perception_core.lossless import compress
except ModuleNotFoundError:
    from rgbd_lossless_codec import compress

MAX_BODY = 16 * 1024 * 1024
RGBD = re.compile(r"/api/cameras/(head_rgb|left_wrist_rgb|right_wrist_rgb)/rgbd\Z")


def handler(upstream):
    class ReadOnlyProxy(BaseHTTPRequestHandler):
        disable_nagle_algorithm = True

        def do_GET(self):
            rgbd = RGBD.fullmatch(self.path)
            if not rgbd and self.path != "/api/state":
                self.send_error(404)
                return
            try:
                started = time.monotonic()
                with build_opener(ProxyHandler({})).open(
                    upstream + self.path, timeout=0.5
                ) as response:
                    body = response.read(MAX_BODY + 1)
                    if len(body) > MAX_BODY:
                        raise ValueError("SDK response exceeds size limit")
                    headers = dict(response.headers)
                received = time.monotonic()
                source_size = len(body)
                accepted = self.headers.get("Accept-Encoding", "")
                encoding = ""
                checksum = zlib.crc32(body)
                if rgbd and "x-lz4-block" in accepted:
                    body = compress(body)
                    encoding = "x-lz4-block"
                elif rgbd and "gzip" in accepted:
                    body = gzip.compress(body, compresslevel=1, mtime=0)
                    encoding = "gzip"
                compressed_at = time.monotonic()
                self.send_response(200)
                for key, value in headers.items():
                    if key.lower() not in (
                        "content-length",
                        "connection",
                        "transfer-encoding",
                        "server",
                        "date",
                    ):
                        self.send_header(key, value)
                if encoding:
                    self.send_header("Content-Encoding", encoding)
                    self.send_header("X-Uncompressed-CRC32", str(checksum))
                self.send_header("Content-Length", str(len(body)))
                self.send_header("X-Proxy-Upstream-Ms", str((received - started) * 1000))
                self.send_header("X-Proxy-Compress-Ms", str((compressed_at - received) * 1000))
                self.send_header("X-Uncompressed-Bytes", str(source_size))
                self.send_header("Connection", "close")
                self.end_headers()
                self.wfile.write(body)
            except Exception:
                self.send_error(502, "SDK read unavailable")

        def do_POST(self):
            self.send_error(405, "Read only")

        do_PUT = do_POST
        do_DELETE = do_POST
        do_PATCH = do_POST

        def log_message(self, *args):
            pass

    return ReadOnlyProxy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.127.10")
    parser.add_argument("--port", type=int, default=18035)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), handler("http://127.0.0.1:18030"))
    server.daemon_threads = True
    server.serve_forever()


if __name__ == "__main__":
    main()
