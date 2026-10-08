import json
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import urlopen, Request

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import robot_camera_server as console


class CameraConsoleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.folder = Path(self.temp.name)
        self.patch = patch.object(console, 'FOLDER', self.folder)
        self.patch.start()

    def tearDown(self):
        self.patch.stop()
        self.temp.cleanup()

    def status(self, age=0, frame_age=0, error=None):
        now = time.time()
        (self.folder / 'status.json').write_text(json.dumps(dict(
            updated_at=now-age, cameras=[dict(id='Robot_A', updated_at=now-frame_age, error=error)])))

    def test_missing_and_stale_simulation(self):
        self.assertFalse(console.read_status()['connected'])
        self.status(age=10)
        result = console.read_status()
        self.assertFalse(result['connected'])
        self.assertFalse(result['cameras'][0]['online'])

    def test_static_frame_remains_online_but_camera_failure_does_not(self):
        self.status(frame_age=10)
        self.assertTrue(console.read_status()['connected'])
        self.assertTrue(console.read_status()['cameras'][0]['online'])
        self.status(error='render failed')
        self.assertFalse(console.read_status()['cameras'][0]['online'])
        self.status()
        self.assertTrue(console.read_status()['cameras'][0]['online'])

    def test_heartbeat_does_not_trigger_push_but_frame_and_state_do(self):
        self.status()
        first = console.read_status()
        first['cameras'][0]['version'] = 1
        import copy
        heartbeat = copy.deepcopy(first)
        heartbeat['updated_at'] += 2
        heartbeat['simulation_seconds'] = 99
        self.assertEqual(console.event_key(first), console.event_key(heartbeat))
        heartbeat['cameras'][0]['version'] = 2
        self.assertNotEqual(console.event_key(first), console.event_key(heartbeat))
        heartbeat['cameras'][0]['version'] = 1
        heartbeat['cameras'][0]['state'] = '抓取中'
        self.assertNotEqual(console.event_key(first), console.event_key(heartbeat))

    def test_physics_controls_survive_viewer_loss_but_not_physics_loss(self):
        now=time.time()
        (self.folder/'status.json').write_text(json.dumps(dict(session='physics',updated_at=now-10,
            cameras=[dict(id='Robot_A',updated_at=now-10,error=None)],operator=dict(mode='manual'))))
        physics=dict(session='physics',updated_at=now,decoupled=True,operator=dict(mode='relay',active=True))
        (self.folder/'operator_status.json').write_text(json.dumps(physics))
        result=console.read_status()
        self.assertTrue(result['connected'])
        self.assertTrue(result['operator']['active'])
        self.assertFalse(result['cameras'][0]['online'])
        # A renderer that is still alive must not mask the loss of its producer.
        status=json.loads((self.folder/'status.json').read_text());status['updated_at']=now
        (self.folder/'status.json').write_text(json.dumps(status))
        physics['updated_at']=now-10
        (self.folder/'operator_status.json').write_text(json.dumps(physics))
        self.assertFalse(console.read_status()['connected'])

    def test_new_legacy_session_supersedes_stale_decoupled_session(self):
        now=time.time()
        (self.folder/'operator_status.json').write_text(json.dumps(dict(session='old',updated_at=now-20,
            decoupled=True,operator=dict(mode='relay'))))
        (self.folder/'status.json').write_text(json.dumps(dict(session='new',updated_at=now,
            cameras=[],operator=dict(mode='manual'))))
        result=console.read_status()
        self.assertTrue(result['connected'])
        self.assertEqual(result['session'],'new')
        self.assertEqual(result['operator']['mode'],'manual')

    def test_only_allowed_frames_are_served_and_cache_is_disabled(self):
        (self.folder / 'Robot_A.jpg').write_bytes(b'jpeg fixture')
        server = console.ThreadingHTTPServer(('127.0.0.1', 0), console.Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = 'http://127.0.0.1:' + str(server.server_port)
        try:
            with urlopen(base + '/frames/Robot_A.jpg?v=3') as response:
                self.assertEqual(response.read(), b'jpeg fixture')
                self.assertEqual(response.headers['Cache-Control'], 'no-store')
                self.assertEqual(response.headers['Content-Type'], 'image/jpeg')
            for path, code in [('/frames/Robot_B.jpg', 503), ('/frames/../status.json', 404)]:
                with self.assertRaises(HTTPError) as error:
                    urlopen(base + path)
                self.assertEqual(error.exception.code, code)
                error.exception.close()
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

    def test_commands_require_live_session_and_valid_origin(self):
        self.status()
        data=console.read_status()
        data.update(session='current',operator=dict(mode='manual'))
        (self.folder/'status.json').write_text(json.dumps(data))
        server=console.ThreadingHTTPServer(('127.0.0.1',0),console.Handler)
        thread=threading.Thread(target=server.serve_forever,daemon=True)
        thread.start()
        url=f'http://127.0.0.1:{server.server_port}/api/command'
        def send(session,origin=None):
            headers={'Content-Type':'application/json'}
            if origin:headers['Origin']=origin
            return urlopen(Request(url,data=json.dumps(dict(session=session,action='reset')).encode(),headers=headers))
        try:
            for session,origin,code in [('old',None,409),('current','http://other.example',403)]:
                with self.assertRaises(HTTPError) as error:send(session,origin)
                self.assertEqual(error.exception.code,code)
                error.exception.close()
            with send('current') as response:self.assertEqual(response.status,202)
            self.assertEqual(len(list((self.folder/'commands').glob('*.json'))),1)
        finally:
            server.shutdown();server.server_close();thread.join()


if __name__ == '__main__':
    unittest.main()
