"""Loopback fake SDK contract tests. Never contact or control real hardware."""
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import shutil
import socket
import threading
import time
import uuid

import pytest
from wheel_navigation.atlas_client import AtlasClient
from wheel_navigation.service import NavigationService, NavigationError
from wheel_sdk_bridge.lift_client import LiftClient


def identifier(): return str(uuid.uuid4())


class FakeSDK:
    def __init__(self):
        self.posts = []
        self.lock = threading.RLock()
        self.online = True
        self.lift_online = True
        self.lost_ack = False
        self.keep_old_goal = False
        self.ready = True
        self.active = False
        self.age_ms = 10.
        self.stop_sequence = 0
        self.revision = 'revision-1'
        self.confirmation = {}
        self.points = [dict(name='A',id=1,x=.4,y=-.7,yaw=.02), dict(name='B',id=2,x=-1.5,y=-.9,yaw=-1.56)]
        self.nav = dict(state='stopped',goal_sequence=7,goal_reached=False,goal_x=.4,goal_y=-.7,
                        goal_yaw_rad=.02,mission_remaining=0,map_pose_age_sec=.01,odom_age_sec=.01)

    def arrive(self):
        self.nav.update(state='goal_reached', goal_reached=True, mission_remaining=0)

    def confirm(self):
        self.confirmation.update(state='stopped',stopped=True)

    def route(self, method, path, body):
        with self.lock:
            if method == 'POST': self.posts.append((path,body))
            if path.startswith('/api/maps/points?'):
                return 200,dict(ok=True,map_name='unstacking',map_revision=self.revision,points=self.points,recording_ready=self.ready)
            if path == '/api/status':
                if not self.online: return 503,dict(ok=False,error='offline')
                return 200,dict(ok=True,localization_status='localized: fake test map',nav_status=json.dumps(self.nav),
                    pipeline=dict(components=dict(navigation=dict(online=True,age_ms=self.age_ms))),stop_confirmation=self.confirmation)
            if path == '/api/map/status': return 200,dict(ok=True,nav_drive_active=self.active)
            if path == '/api/v1/robot':
                return 200,dict(ok=True,robot_type='lift_robot',connected=True,ready=True,estop=False,holder_client_id='')
            if path == '/api/state':
                if not self.lift_online: return 503,dict(ok=False,error='lift offline')
                return 200,dict(system=dict(mock=False,services=dict(base=True)),base=dict(valid=True),
                    base_status=dict(fresh=True,feedback_fresh=True,navigation_cmd_vel=True,external_control=False,
                                     stationary=True,reason='fake test feedback'),arms=dict(fresh=True),lift_status=dict(fresh=True))
            if path == '/api/maps/points/navigate':
                assert set(body)=={'map_name','map_revision','point_names'}
                assert body['map_name']=='unstacking' and body['map_revision']=='revision-1'
                assert body['point_names'] in (['A'],['B'])
                self.active=True
                if not self.keep_old_goal:
                    point=next(p for p in self.points if p['name']==body['point_names'][0])
                    self.nav.update(state='following',goal_sequence=self.nav['goal_sequence']+1,goal_reached=False,
                                    goal_x=point['x'],goal_y=point['y'],goal_yaw_rad=point['yaw'],mission_remaining=1)
                if self.lost_ack: return 0,{}
                return 200,dict(ok=True,nav_drive_active=True,goal_published=True,mission_size=1)
            if path == '/api/nav/stop':
                self.active=False;self.stop_sequence+=1
                self.confirmation=dict(stop_id='stop-'+str(self.stop_sequence),state='pending',stopped=False)
                self.nav.update(state='stopped',goal_reached=False)
                return 200,dict(ok=True,nav_drive_active=False,stop_confirmation=self.confirmation)
            return 404,dict(ok=False,error='unknown fake endpoint')


@contextmanager
def server(fake):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args): pass
        def handle_request(self):
            size=int(self.headers.get('Content-Length',0))
            body=json.loads(self.rfile.read(size)) if size else None
            code,data=fake.route(self.command,self.path,body)
            if code==0:
                self.connection.shutdown(socket.SHUT_RDWR);self.connection.close();return
            payload=json.dumps(data).encode()
            self.send_response(code);self.send_header('Content-Type','application/json')
            self.send_header('Content-Length',str(len(payload)));self.end_headers();self.wfile.write(payload)
        do_GET=handle_request
        do_POST=handle_request
    instance=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    thread=threading.Thread(target=instance.serve_forever,daemon=True);thread.start()
    try: yield 'http://127.0.0.1:'+str(instance.server_port)
    finally: instance.shutdown();instance.server_close();thread.join()


@pytest.fixture
def rig(tmp_path):
    fake=FakeSDK()
    with server(fake) as url:
        service=NavigationService(AtlasClient(url),LiftClient(url),tmp_path/'commands.json',enabled=True,poll_interval=3600.)
        yield fake,service
        service.close()


def test_disabled_control_has_two_real_protocol_projections_but_no_posts(rig):
    fake,service=rig;service.enabled=False
    snapshot=service.snapshot()
    assert snapshot['devices']['atlas']['connected'] and snapshot['devices']['lift']['connected']
    assert not snapshot['can_go']
    with pytest.raises(NavigationError) as error: service.navigate('A',identifier())
    assert error.value.status==423 and fake.posts==[]


def test_named_point_payload_idempotency_and_confirmed_a_then_b(rig):
    fake,service=rig
    request_id=identifier()
    assert service.navigate('A',request_id)['state']=='MOVING'
    assert service.navigate('A',request_id)['state']=='MOVING'
    assert len(fake.posts)==1
    with pytest.raises(NavigationError):service.navigate('B',identifier())
    fake.arrive()
    assert service.snapshot()['command']['state']=='ARRIVED'
    assert len(fake.posts)==1  # GET does not command a stop.
    assert service.snapshot(advance=True)['command']['state']=='STOPPING'
    assert fake.posts[-1]==('/api/nav/stop',{})
    assert service.snapshot()['command']['state']=='STOPPING'
    fake.confirm()
    done=service.snapshot()
    assert done['command']['state']=='SUCCEEDED' and done['can_go']
    assert service.navigate('B',identifier())['state']=='MOVING'
    assert fake.posts[-1][1]['point_names']==['B']


def test_old_reached_flag_and_wrong_goal_cannot_complete_command(rig):
    fake,service=rig
    fake.keep_old_goal=True;fake.arrive()
    service.navigate('A',identifier())
    assert service.snapshot()['command']['state']=='MOVING'
    fake.nav.update(goal_sequence=8,goal_x=20.)
    result=service.snapshot()
    assert result['command']['state']=='UNKNOWN' and not result['can_go']


def test_lost_movement_ack_is_not_retried_and_restart_does_not_resume(rig,tmp_path):
    fake,service=rig;fake.lost_ack=True
    request_id=identifier()
    result=service.navigate('B',request_id)
    assert result['state']=='UNKNOWN'
    service.navigate('B',request_id)
    assert len(fake.posts)==1
    restarted=NavigationService(service.atlas,service.lift,tmp_path/'commands.json',enabled=True)
    assert restarted.snapshot()['command']['state']=='UNKNOWN'
    with pytest.raises(NavigationError):restarted.navigate('A',identifier())
    assert len(fake.posts)==1
    stop_id=identifier()
    assert restarted.stop(stop_id)['state']=='STOPPING'
    fake.confirm()
    assert restarted.snapshot()['command']['state']=='STOPPED'


def test_old_stop_confirmation_is_not_the_new_stop_receipt(rig):
    fake,service=rig
    service.navigate('A',identifier())
    service.stop(identifier())
    fake.confirmation=dict(stop_id='previous-stop',state='stopped',stopped=True)
    assert service.snapshot()['command']['state']=='STOPPING'
    fake.confirmation=dict(stop_id='stop-1',state='stopped',stopped=True)
    assert service.snapshot()['command']['state']=='STOPPED'


@pytest.mark.parametrize('failure', ['localization','stale','lift','atlas'])
def test_unready_or_disconnected_sdk_never_receives_movement(rig,failure):
    fake,service=rig
    if failure=='localization':fake.ready=False
    elif failure=='stale':fake.age_ms=5000
    elif failure=='lift':fake.lift_online=False
    else:fake.online=False
    snapshot=service.snapshot()
    if failure=='lift':
        assert snapshot['devices']['atlas']['connected'] and not snapshot['devices']['lift']['connected']
    with pytest.raises(NavigationError):service.navigate('A',identifier())
    assert fake.posts==[]


def test_concurrent_go_requests_have_one_navigation_owner(rig):
    fake,service=rig
    def go(point):
        try:return service.navigate(point,identifier())['state']
        except NavigationError:return 'BUSY'
    with ThreadPoolExecutor(max_workers=2) as pool:states=list(pool.map(go,['A','B']))
    assert sorted(states)==['BUSY','MOVING'] and len(fake.posts)==1


def test_stale_running_feedback_stops_once_and_cannot_be_success(rig):
    fake,service=rig
    service.navigate('A',identifier());fake.age_ms=5000
    assert service.snapshot(advance=True)['command']['state']=='STOPPING'
    service.snapshot(advance=True)
    assert len([p for p in fake.posts if p[0]=='/api/nav/stop'])==1
    fake.confirm()
    assert service.snapshot()['command']['state']=='FAULT'


def test_map_revision_change_during_navigation_requests_stop(rig):
    fake,service=rig
    service.navigate('A',identifier());fake.revision='revision-2';fake.arrive()
    result=service.snapshot(advance=True)['command']
    assert result['state']=='STOPPING' and result['completion']=='FAULT'


def test_worker_finishes_an_accepted_job_without_browser_polling(rig):
    fake,service=rig
    service.poll_interval=.02
    service.navigate('A',identifier())
    fake.arrive();service.start_worker()
    deadline=time.monotonic()+3
    while time.monotonic()<deadline and not fake.stop_sequence: time.sleep(.02)
    assert fake.stop_sequence==1
    fake.confirm()
    while time.monotonic()<deadline:
        with service.journal.transaction() as data: result=deepcopy(service._current(data))
        if result['state']=='SUCCEEDED': break
        time.sleep(.02)
    assert result['state']=='SUCCEEDED'


def test_stop_timeout_does_not_release_navigation_ownership(rig):
    fake,service=rig
    now=[100.]
    service.clock=lambda:now[0]
    service.stop(identifier());now[0]=110.
    snapshot=service.snapshot()
    assert snapshot['command']['state']=='UNKNOWN' and not snapshot['can_go']
    with pytest.raises(NavigationError):service.navigate('A',identifier())


def test_http_validation_origin_control_gate_and_command_lookup(rig,tmp_path):
    pytest.importorskip('fastapi');pytest.importorskip('httpx')
    from fastapi.testclient import TestClient
    from management_api.app import create_app
    fake,service=rig
    root=Path(__file__).resolve().parents[1]
    shutil.copytree(root/'config',tmp_path/'config',dirs_exist_ok=True)
    shutil.copytree(root/'apps/management_web',tmp_path/'apps/management_web',dirs_exist_ok=True)
    with TestClient(create_app(tmp_path,navigation_service=service)) as client:
        request_id=identifier()
        assert client.get('/api/v1/devices').json()['atlas']['connected']
        assert client.post('/api/v1/navigation/go',json=dict(point='C',command_id=request_id)).status_code==422
        assert client.post('/api/v1/navigation/go',json=dict(point='A',command_id=request_id),headers={'Origin':'http://evil.example'}).status_code==403
        assert fake.posts==[]
        response=client.post('/api/v1/navigation/go',json=dict(point='A',command_id=request_id))
        assert response.status_code==202 and response.json()['state']=='MOVING'
        assert client.get('/api/v1/navigation/commands/'+request_id).json()['state']=='MOVING'
        assert client.post('/api/v1/navigation/go',json=dict(point='A',command_id=request_id)).status_code==202
        assert len(fake.posts)==1
        assert client.get('/navigation.js').status_code==200
        assert client.post('/api/v1/navigation/go',json=dict(point='A',command_id=identifier(),base_url='http://untrusted')).status_code==422
