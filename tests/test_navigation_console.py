import json
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from urllib.request import Request,urlopen
from urllib.error import HTTPError
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import navigation_console_server as server
from navigation_console_ipc import NavigationConsole,atomic_json

class ConsoleTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.old_folder=server.FOLDER
        server.FOLDER=Path(self.tmp.name)
        self.status=dict(session='live',updated_at=time.time(),active=False,config={'points':[{'id':'home'}]})
        atomic_json(server.FOLDER/'status.json',self.status)
        self.http=server.ThreadingHTTPServer(('127.0.0.1',0),server.Handler)
        self.thread=threading.Thread(target=self.http.serve_forever,daemon=True);self.thread.start()
        self.url=f'http://127.0.0.1:{self.http.server_port}'
    def tearDown(self):
        self.http.shutdown();self.http.server_close();self.thread.join();server.FOLDER=self.old_folder;self.tmp.cleanup()
    def post(self,body,origin=None):
        headers={'Content-Type':'application/json'}
        if origin:headers['Origin']=origin
        try:
            with urlopen(Request(self.url+'/api/command',json.dumps(body).encode(),headers)) as r:return r.status,json.load(r)
        except HTTPError as e:
            with e:return e.code,json.load(e)
    def test_command_requires_live_session_and_valid_goal(self):
        self.assertEqual(self.post(dict(action='goal',goal='home',session='old'))[0],409)
        self.assertEqual(self.post(dict(action='goal',goal='missing',session='live'))[0],400)
        code,data=self.post(dict(action='goal',goal='home',session='live'));self.assertEqual(code,202)
        packet=json.loads((server.FOLDER/'commands'/f'{data["id"]}.json').read_text())
        self.assertEqual(packet['goal'],'home');self.assertEqual(packet['session'],'live')
    def test_stale_heartbeat_and_cross_origin_rejected(self):
        cmd=dict(action='stop',session='live')
        self.assertEqual(self.post(cmd,'https://unrelated.example')[0],403)
        self.status['updated_at']-=10;atomic_json(server.FOLDER/'status.json',self.status)
        self.assertEqual(self.post(cmd)[0],409)
    def test_active_task_cannot_be_replaced(self):
        self.status['active']=True;atomic_json(server.FOLDER/'status.json',self.status)
        self.assertEqual(self.post(dict(action='tour',session='live'))[0],409)
        self.assertEqual(self.post(dict(action='stop',session='live'))[0],202)
    def test_expired_queued_command_never_executes(self):
        console=NavigationConsole.__new__(NavigationConsole);console.session='live'
        console.commands=server.FOLDER/'commands';console.commands.mkdir()
        console.publish=lambda:None;console.rt=None
        atomic_json(console.commands/'expired.json',dict(session='live',created_at=time.time()-10,action='tour'))
        console.poll();self.assertFalse(console.last_command['ok']);self.assertFalse(list(console.commands.iterdir()))

if __name__=='__main__':unittest.main()
