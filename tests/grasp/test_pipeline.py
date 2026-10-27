"""Synthetic geometry verifies integration, explicitly not model accuracy."""
import json
from pathlib import Path
import threading
from http.server import HTTPServer,BaseHTTPRequestHandler
import time
import numpy as np
import pytest
from wheel_perception_core.pipeline import GraspPipeline
from wheel_perception_core.rgbd import RGBDClient,Observation
from wheel_motion_core.kinematics import RobotModel
from test_core import packet,ROOT


def test_http_atomic_transport_and_duplicate():
    kind,body=packet()
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            assert self.path=='/api/cameras/head_rgb/rgbd'
            self.send_response(200);self.send_header('Content-Type',kind)
            self.send_header('X-Pair-Id','123:1');self.end_headers();self.wfile.write(body)
        def log_message(self,*args):pass
    server=HTTPServer(('127.0.0.1',0),Handler);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    try:
        client=RGBDClient('http://127.0.0.1:'+str(server.server_port))
        assert client.receive('head_rgb').key==('head_rgb','123:1')
        with pytest.raises(ValueError,match='Duplicate'):client.receive('head_rgb')
    finally:server.shutdown();server.server_close();thread.join()


def test_known_visible_surface_pipeline(tmp_path):
    # A thin test box front at Z=0.8; fixture backend returns known transform.
    urdf=tmp_path/'robot.urdf'
    urdf.write_text('''<robot name="fixture"><link name="base_footprint"/><link name="head_d455_color_optical_frame"/>
    <joint name="camera" type="fixed"><parent link="base_footprint"/><child link="head_d455_color_optical_frame"/>
    <origin xyz="0.1 0 0"/></joint></robot>''')
    h,w=60,80;k=np.array([[200,0,40],[0,200,30],[0,0,1.]])
    mask=np.zeros((h,w),bool);mask[10:50,10:70]=True
    depth=np.zeros((h,w),np.float32);depth[mask]=.8
    m=dict(schema='luxi_rgbd_v1',camera_id='head_rgb',pair_id='fixture:1',frame_id='head_rgb_color_optical',
           intrinsics=dict(K=k.ravel().tolist(),D=[0]*5,distortion_model='plumb_bob'),
           depth=dict(value_semantics='color_optical_z'),mock=True,replay=True)
    obs=Observation(m,np.zeros((h,w,3),np.uint8),depth,time.monotonic())
    t=np.eye(4);t[2,3]=.81
    class Segmenter:
        def detect(self,rgb):return [dict(box=[10,10,70,50],score=1)]
        def segment(self,rgb,box):return mask
    class Estimator:
        def register(self,*args):return t
    rule=dict(width=.4,height=.22,inset=.007,below_top=.006)
    from test_live_trace import Trace
    trace=Trace(tmp_path)
    p=GraspPipeline(RobotModel(urdf),Segmenter(),Estimator(),rule,
                    [dict(position=[0,0,0],size=[.4,.3,.02])],trace=trace)
    out=p.process(obs,{},target_id='fixture_box')
    assert out['quality']['surface_rms_m']<1e-6
    assert out['pose'][0,3]==pytest.approx(.1)
    assert not out['executable'] and out['diagnostic_only']
    assert (tmp_path/'06_pose_unvalidated.png').is_file()
    assert (tmp_path/'07_grasp_tcp.png').is_file()
    stages=[json.loads(line) for line in (tmp_path/'vision.events.jsonl').read_text().splitlines()]
    assert [e['stage'] for e in stages if e['status']=='completed']==[
        'preprocess','detection','segmentation','pose','pose_quality','transform','keypoints']
    with pytest.raises(ValueError,match='Duplicate'):p.process(obs,{},target_id='fixture_box')


def test_live_capture_can_replay_if_available():
    from wheel_perception_core.rgbd import load_observation
    from wheel_perception_core.geometry import prepare_input
    file=ROOT/'outputs/grasp/head_rgb_20261016.npz'
    if not file.exists():pytest.skip('Local read-only hardware capture not present')
    obs=load_observation(file);inp=prepare_input(obs)
    assert inp.depth.shape==inp.rgb.shape[:2]
    assert inp.diagnostic_only
    assert np.count_nonzero(inp.depth)>0
