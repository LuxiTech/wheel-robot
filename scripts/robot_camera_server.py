"""Loopback robot dashboard; commands are executed by the simulation thread."""
import json
import time
import threading
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit
from navigation_console_ipc import atomic_json
from robot_console_ipc import validate_command

ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / 'outputs/robot_cameras'
CAMERAS = ('Robot_A', 'Robot_B')


def read_status():
    now=time.time()
    try:
        status=json.loads((FOLDER/'status.json').read_text())
    except (OSError,ValueError):
        status=dict(updated_at=0.,cameras=[])
    display_connected=0<=now-status.get('updated_at',0)<6
    try:
        physics=json.loads((FOLDER/'operator_status.json').read_text())
        if physics.get('decoupled') and (physics['session']==status.get('session') or physics['updated_at']>=status.get('updated_at',0)):
            if physics['session']!=status.get('session'):
                status=dict(cameras=[])
            status.update({k:physics[k] for k in ('session','updated_at','operator')})
    except (OSError,ValueError,KeyError,TypeError):
        pass
    try:
        status['connected']=0<=now-status['updated_at']<6
        for camera in status['cameras']:
            camera['online']=status['connected'] and display_connected and camera['updated_at']>0 and not camera['error']
        return status
    except (KeyError,TypeError):
        return dict(connected=False,cameras=[])


def event_key(status):
    """Clock/heartbeat alone must never generate browser updates."""
    return (status.get('session'), status['connected'], tuple(
        (c['id'], c.get('version', c['updated_at']), c.get('state'), c['online'], c.get('error'))
        for c in status['cameras']), json.dumps(status.get('operator'), sort_keys=True))


class EventHub:
    def __init__(self):
        self.condition = threading.Condition()
        self.revision = 0
        self.viewers = 0
        self.status = dict(connected=False, cameras=[])
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self.watch, daemon=True)
        self.thread.start()

    def watch(self):
        previous = None
        lease_written = 0.
        while not self.stopped.is_set():
            if time.monotonic()-lease_written >= 1.:
                with self.condition:
                    viewers = self.viewers
                atomic_json(FOLDER / "viewers.json", dict(count=viewers, updated_at=time.time()))
                lease_written=time.monotonic()
            status = read_status()
            key = event_key(status)
            if key != previous:
                with self.condition:
                    self.status = status
                    self.revision += 1
                    self.condition.notify_all()
                previous = key
            self.stopped.wait(.25)


class Handler(BaseHTTPRequestHandler):
    def reply(self, code, value):
        body = json.dumps(value, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path != '/api/command':
            return self.reply(404, dict(error='未知接口'))
        origin = self.headers.get('Origin')
        if origin and urlsplit(origin).netloc != self.headers.get('Host'):
            return self.reply(403, dict(error='拒绝跨站控制请求'))
        if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
            return self.reply(415, dict(error='需要 JSON 指令'))
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 2048:
                raise ValueError('指令大小无效')
            cmd = json.loads(self.rfile.read(length))
            if not isinstance(cmd, dict):
                raise ValueError('指令必须是对象')
            status = read_status()
            if not status['connected'] or not status.get('operator'):
                return self.reply(409, dict(error='仿真控制未连接'))
            if cmd.get('session') != status.get('session'):
                return self.reply(409, dict(error='会话已改变，请等待重新连接'))
            validate_command(cmd, status['operator'])
            identity = f'{time.time_ns():020d}-{uuid.uuid4().hex}'
            queue = FOLDER / 'commands'
            queue.mkdir(exist_ok=True)
            atomic_json(queue / (identity + '.json'), dict(cmd, id=identity, created_at=time.time()))
            self.reply(202, dict(id=identity, message='已发送，等待仿真执行'))
        except (ValueError, TypeError, KeyError) as exc:
            self.reply(400, dict(error=str(exc)))

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == '/api/events':
            return self.events()
        code, mime = 200, 'application/json; charset=utf-8'
        if path == '/api/status':
            body = json.dumps(read_status(), ensure_ascii=False).encode()
        elif path in ('/', '/index.html'):
            body = (ROOT / 'scripts/web/robot_cameras.html').read_bytes()
            mime = 'text/html; charset=utf-8'
        elif path in tuple('/frames/' + name + '.jpg' for name in CAMERAS):
            try:
                body = (FOLDER / path.rsplit('/', 1)[1]).read_bytes()
                mime = 'image/jpeg'
            except OSError:
                code, body = 503, b'{"error":"Camera warming up"}'
        else:
            code, body = 404, b'{"error":"Not found"}'
        self.send_response(code)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def events(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('X-Accel-Buffering', 'no')
        self.end_headers()
        hub = self.server.event_hub
        with hub.condition:
            hub.viewers += 1
        revision = -1
        self.connection.settimeout(5.)
        try:
            self.wfile.write(b'retry: 3000\n\n')
            self.wfile.flush()
            while not hub.stopped.is_set():
                with hub.condition:
                    hub.condition.wait_for(lambda: hub.revision != revision or hub.stopped.is_set(), timeout=2.)
                    changed = hub.revision != revision
                    revision = hub.revision
                    status = hub.status
                payload = ('data: ' + json.dumps(status, ensure_ascii=False) + '\n\n').encode() if changed else b': keepalive\n\n'
                self.wfile.write(payload)
                self.wfile.flush()
        except (OSError, TimeoutError):
            pass
        finally:
            with hub.condition:
                hub.viewers -= 1

    def log_message(self, *args):
        pass


if __name__ == '__main__':
    server = ThreadingHTTPServer(('127.0.0.1', 8766), Handler)
    server.event_hub = EventHub()
    print('ROBOT_CAMERA_CONSOLE http://127.0.0.1:8766', flush=True)
    server.serve_forever()
