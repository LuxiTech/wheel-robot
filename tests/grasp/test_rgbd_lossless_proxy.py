import gzip
import importlib.util
import threading
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

import pytest
from wheel_perception_core import rgbd


def test_lossless_proxy_preserves_pair_body_and_never_forwards_writes():
    path = Path(__file__).resolve().parents[2] / "deployment/robot/rgbd_lossless_proxy.py"
    spec = importlib.util.spec_from_file_location("lossless_proxy", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    body = b"original-metadata-and-IEEE-depth-bytes\0" * 1000
    reads = []

    class Upstream(BaseHTTPRequestHandler):
        def do_GET(self):
            reads.append(self.path)
            self.send_response(200)
            self.send_header("Content-Type", "multipart/mixed; boundary=original")
            self.send_header("X-Pair-Id", "original-source-pair")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    upstream = ThreadingHTTPServer(("127.0.0.1", 0), Upstream)
    proxy = ThreadingHTTPServer(
        ("127.0.0.1", 0), module.handler("http://127.0.0.1:" + str(upstream.server_port))
    )
    workers = [
        threading.Thread(target=server.serve_forever, daemon=True) for server in (upstream, proxy)
    ]
    for worker in workers:
        worker.start()
    op = build_opener(ProxyHandler({}))
    base = "http://127.0.0.1:" + str(proxy.server_port)
    try:
        with op.open(
            Request(base + "/api/cameras/head_rgb/rgbd", headers={"Accept-Encoding": "gzip"})
        ) as response:
            compressed = response.read()
            assert response.headers["X-Pair-Id"] == "original-source-pair"
            assert rgbd.decompress_body(compressed, response.headers["Content-Encoding"]) == body
            assert len(compressed) < len(body)
        for request in (
            Request(base + "/api/arm_startup", data=b"{}", method="POST"),
            Request(base + "/api/arm_startup"),
        ):
            with pytest.raises(HTTPError):
                op.open(request)
        assert reads == ["/api/cameras/head_rgb/rgbd"]
    finally:
        for server in (proxy, upstream):
            server.shutdown()
            server.server_close()
        for worker in workers:
            worker.join(timeout=1)


def test_gzip_limit_truncation_and_concatenation_are_rejected(monkeypatch):
    monkeypatch.setattr(rgbd, "MAX_BODY", 64)
    with pytest.raises(ValueError):
        rgbd.decompress_body(gzip.compress(b"x" * 65), "gzip")
    encoded = gzip.compress(b"original")
    with pytest.raises(ValueError):
        rgbd.decompress_body(encoded[:-2], "gzip")
    with pytest.raises(ValueError):
        rgbd.decompress_body(encoded + encoded, "gzip")
    with pytest.raises(ValueError):
        rgbd.decompress_body(encoded, "unknown")


def test_lz4_preserves_arbitrary_float_bytes_and_checks_size_crc(monkeypatch):
    from wheel_perception_core.lossless import compress

    body = bytes(range(256)) * 100
    encoded = compress(body)
    headers = {
        "X-Uncompressed-Bytes": str(len(body)),
        "X-Uncompressed-CRC32": str(zlib.crc32(body)),
    }
    assert rgbd.decompress_body(encoded, "x-lz4-block", headers) == body
    bad = dict(headers, **{"X-Uncompressed-CRC32": "0"})
    with pytest.raises(ValueError, match="checksum"):
        rgbd.decompress_body(encoded, "x-lz4-block", bad)
    with pytest.raises(ValueError):
        rgbd.decompress_body(encoded[:-1], "x-lz4-block", headers)
    monkeypatch.setattr(rgbd, "MAX_BODY", 64)
    with pytest.raises(ValueError, match="oversized"):
        rgbd.decompress_body(encoded, "x-lz4-block", headers)
