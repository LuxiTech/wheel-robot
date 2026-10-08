"""Local navigation operator page. Never invokes shells or touches Isaac APIs."""
import json
import errno
import time
import uuid
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse
from navigation_console_ipc import ACTIONS,atomic_json

ROOT=Path(__file__).resolve().parents[1]
FOLDER=ROOT/'outputs/navigation_console'


def read_status():
    try:
        data=json.loads((FOLDER/'status.json').read_text())
        data['connected']=0<=time.time()-data['updated_at']<3
        return data
    except (OSError,ValueError):return dict(connected=False)


class Handler(BaseHTTPRequestHandler):
    def send(self,code,data,mime='application/json; charset=utf-8'):
        body=json.dumps(data,ensure_ascii=False).encode() if not isinstance(data,bytes) else data
        self.send_response(code);self.send_header('Content-Type',mime)
        self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store')
        self.send_header('X-Content-Type-Options','nosniff');self.end_headers();self.wfile.write(body)

    def do_GET(self):
        if self.path=='/api/status':self.send(200,read_status())
        elif self.path in ('/','/index.html'):
            self.send(200,(ROOT/'scripts/web/navigation.html').read_bytes(),'text/html; charset=utf-8')
        else:self.send(404,{'error':'不存在'})

    def do_POST(self):
        if self.path!='/api/command':return self.send(404,{'error':'不存在'})
        # JSON-only, same-origin mutations; no CORS or arbitrary shell API.
        origin=self.headers.get('Origin')
        if origin and urlparse(origin).netloc!=self.headers.get('Host'):
            return self.send(403,{'error':'拒绝跨站控制请求'})
        if self.headers.get('Content-Type','').split(';')[0]!='application/json':
            return self.send(415,{'error':'需要 JSON 指令'})
        try:
            length=int(self.headers.get('Content-Length','0'))
            if not 0<length<=2048:raise ValueError('指令大小无效')
            cmd=json.loads(self.rfile.read(length))
            if not isinstance(cmd,dict):raise ValueError('指令必须为对象')
            status=read_status()
            if not status['connected']:return self.send(409,{'error':'仿真未连接'})
            if cmd.get('session')!=status['session']:return self.send(409,{'error':'仿真会话已改变，请刷新'})
            action=cmd.get('action')
            if action not in ACTIONS:raise ValueError('未知指令')
            if action=='goal' and cmd.get('goal') not in [p['id'] for p in status['config']['points']]:raise ValueError('未知取物点')
            if status['active'] and action in ('goal','tour'):return self.send(409,{'error':'任务执行中，请先停止'})
            identity=f'{time.time_ns():020d}-{uuid.uuid4().hex}'
            queue=FOLDER/'commands';queue.mkdir(parents=True,exist_ok=True)
            atomic_json(queue/f'{identity}.json',dict(id=identity,session=status['session'],action=action,
                         goal=cmd.get('goal'),created_at=time.time()))
            self.send(202,dict(id=identity,message='已发送，等待仿真确认'))
        except (ValueError,TypeError,KeyError) as exc:self.send(400,{'error':str(exc)})

    def log_message(self,*args):pass


if __name__=='__main__':
    for attempt in range(30):
        try:
            server=ThreadingHTTPServer(('127.0.0.1',8765),Handler)
            break
        except OSError as exc:
            if exc.errno!=errno.EADDRINUSE or attempt==29:raise
            time.sleep(.1)
    print('NAVIGATION_CONSOLE http://127.0.0.1:8765',flush=True)
    server.serve_forever()
