"""Model transfer resumes safely even when a server ignores Range."""
import importlib.util
from pathlib import Path
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading
import pytest


@pytest.mark.parametrize('supports_range', [True, False])
def test_resume_or_restart(tmp_path, supports_range):
    path = Path(__file__).resolve().parents[2]/'tools/grasp/download_models.py'
    spec = importlib.util.spec_from_file_location('download_models', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    data = bytes(range(256))*64
    received = []
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            received.append(self.headers.get('Range'))
            payload = data[1234:] if supports_range else data
            self.send_response(206 if supports_range else 200)
            if supports_range:
                self.send_header('Content-Range', f'bytes 1234-{len(data)-1}/{len(data)}')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
        def log_message(self, *args):
            pass
    server = HTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        partial = tmp_path/'model.partial'
        partial.write_bytes(data[:1234])
        module.download_url(f'http://127.0.0.1:{server.server_port}/model', partial)
        assert received == ['bytes=1234-']
        assert partial.read_bytes() == data
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
