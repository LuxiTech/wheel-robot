import json
from pathlib import Path
from email.mime.multipart import MIMEMultipart
from email.mime.application import MIMEApplication
from email import policy
import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from wheel_perception_core.rgbd import decode,save_observation,load_observation
from wheel_perception_core.geometry import prepare_input,rim_targets,mask_quality
from wheel_motion_core.kinematics import RobotModel,PairIK
from wheel_motion_core.planning import plan_pair,sample_trajectory
from wheel_sdk_bridge.sdk import LiftMapping,arm_feedback,LocalLease

ROOT=Path(__file__).resolve().parents[2]
URDF=ROOT/'assets/robots/r680_lift_openarm/r680_lift_openarm_sim.urdf'


def packet(*,duplicate=False,age=0,mock=False,semantic='color_optical_z'):
    rgb=np.full((24,32,3),[9,60,200],np.uint8)
    ok,jpg=cv2.imencode('.jpg',rgb);assert ok
    depth=np.full((24,32),.8,dtype='<f4');depth[0,0]=np.nan
    m=dict(schema='luxi_rgbd_v1',camera_id='head_rgb',pair_id='123:1',sequence='1',
           aligned_to_color=True,frame_id='head_rgb_color_optical',width=32,height=24,mock=mock,age_ms=age,
           calibration_available=True,intrinsics=dict(K=[100,0,16,0,100,12,0,0,1],D=[0]*5,distortion_model='plumb_bob'),
           color=dict(encoding='jpeg',bytes=len(jpg)),depth=dict(encoding='32FC1',unit='metre',byte_order='little-endian',
            step=32*4,bytes=depth.nbytes,value_semantics=semantic))
    boundary='wheel-test-boundary'
    body=b''
    for name,data,kind in [('metadata',json.dumps(m).encode(),'application/json'),('color',jpg.tobytes(),'image/jpeg'),('depth',depth.tobytes(),'application/octet-stream')]:
        part=(f'--{boundary}\r\nContent-Type: {kind}\r\nContent-Disposition: inline; name="{name}"\r\n\r\n').encode()+data+b'\r\n'
        body+=part
        if name=='depth' and duplicate: body+=part
    body+=f'--{boundary}--\r\n'.encode()
    return 'multipart/mixed; boundary='+boundary,body



def test_atomic_decode_and_replay(tmp_path):
    obs=decode(*packet())
    assert np.isnan(obs.depth[0,0])
    assert obs.rgb[5,5,0]>190 # JPEG BGR converted to RGB.
    path=tmp_path/'obs.npz';save_observation(path,obs)
    with pytest.raises(FileExistsError):save_observation(path,obs)
    replay=load_observation(path)
    assert replay.metadata['replay'] and np.isnan(replay.depth[0,0])
    assert prepare_input(replay).diagnostic_only


def test_corrupt_payloads_rejected():
    with pytest.raises(ValueError):decode(*packet(duplicate=True))
    with pytest.raises(ValueError):decode(*packet(mock=True))
    with pytest.raises(ValueError):decode('application/json',b'{}')
    kind,body=packet()
    with pytest.raises((ValueError,KeyError)):decode(kind,body[:-100])


def test_depth_semantics_not_silently_certified():
    obs=decode(*packet(semantic='source_depth_z_resampled_to_color'))
    assert prepare_input(obs).diagnostic_only
    assert not mask_quality(np.ones((24,32),bool),np.zeros((24,32)))['accepted']


def test_rim_points_follow_object_transform():
    t=np.eye(4);t[:3,:3]=Rotation.from_euler('z',.6).as_matrix();t[:3,3]=[.5,.2,.3]
    g,pre=rim_targets(t,width=.4,height=.22,inset=.007,below_top=.006)
    local=[(np.linalg.inv(t)@v)[:3,3] for v in g.values()]
    assert sorted(round(x[0],3) for x in local)==[-.193,.193]
    assert all(np.allclose(x[1:],[0,.214]) for x in local)
    assert all(np.isclose(np.linalg.norm(pre[s][:3,3]-g[s][:3,3]),.06) for s in g)


def test_lift_mapping_and_reject_not_clip():
    mapping=LiftMapping(1,.2,-180000,180000)
    assert mapping.to_joint(-100000)==pytest.approx(.1)
    assert mapping.to_device(.1)==-100000
    with pytest.raises(ValueError):mapping.to_device(.4)
    with pytest.raises(ValueError):mapping.to_joint(float('nan'))


def test_sdk_right_left_and_feedback_validity():
    st=dict(position=[i*.01 for i in range(16)],valid=[True]*16,age_ns=[0]*16)
    q=arm_feedback(st)
    assert q['openarm_right_joint1']==0
    assert q['openarm_left_joint1']==.08
    st['valid'][1]=False
    with pytest.raises(ValueError):arm_feedback(st)


def test_local_lease_does_not_keep_disconnected_client_alive():
    now=[1_000_000_000];lease=LocalLease(clock=lambda:now[0])
    lease.remote_input(1);first=lease.begin('base')
    lease.command('velocity',velocity=[0,0,0])
    now[0]+=151_000_000
    with pytest.raises(ValueError):lease.command('velocity',velocity=[0,0,0])
    lease.remote_input(2)
    with pytest.raises(ValueError):lease.command('velocity',velocity=[0,0,0])
    second=lease.begin('base');assert first['owner']!=second['owner']
    with pytest.raises(ValueError):lease.command('heartbeat')


def test_lift_task_not_replayed():
    lease=LocalLease(clock=lambda:1_000_000_000);lease.remote_input(1);lease.begin('lift')
    lease.command('move',task_id=1,target_um=0)
    with pytest.raises(ValueError):lease.command('move',task_id=1,target_um=0)


@pytest.fixture
def actual_robot():
    if not URDF.exists(): pytest.skip('External robot asset required')
    robot=RobotModel(URDF)
    q={'lift_joint':.15}
    q.update({f'openarm_{s}_joint{i}':0. for s in ('left','right') for i in range(1,8)})
    for s in ('left','right'):q[f'openarm_{s}_joint2']=-.3 if s=='left' else .3;q[f'openarm_{s}_joint4']=1.3
    return robot,q


def test_urdf_fixed_and_moving_mounts(actual_robot):
    r,q=actual_robot
    head=r.transform('head_d455_color_optical_frame',{})
    assert np.isfinite(head).all()
    a=r.transform('left_wrist_eb435i_color_optical_frame',q)
    changed=dict(q,lift_joint=.25);b=r.transform('left_wrist_eb435i_color_optical_frame',changed)
    assert np.linalg.norm(b[:3,3]-a[:3,3])==pytest.approx(.1)
    with pytest.raises(ValueError):r.transform('left_wrist_eb435i_color_optical_frame',{})


def test_pair_ik_and_trajectory(actual_robot):
    r,q=actual_robot;tcp={s:np.eye(4) for s in ('left','right')}
    for t in tcp.values():t[2,3]=-.202
    ik=PairIK(r,tcp)
    targetq=q.copy();targetq['openarm_left_joint3']=.06;targetq['openarm_right_joint3']=-.05
    goals=ik.fk(targetq);sol=ik.solve(goals,q,fixed_lift=.15)
    assert sol.success and sol.joints['lift_joint']==pytest.approx(.15)
    plan=plan_pair(ik,goals,q,fixed_lift=.15,steps=4)
    assert not plan.executable and not plan.collision_checked
    assert all(a['time']<b['time'] for a,b in zip(plan.points,plan.points[1:]))
    assert sample_trajectory(plan,0)==plan.points[0]['joints']
    with pytest.raises(ValueError):plan_pair(ik,goals,q,collision_check=lambda x:False)


def test_ik_unreachable_rejected(actual_robot):
    r,q=actual_robot;ik=PairIK(r,{s:np.eye(4) for s in ('left','right')})
    targets=ik.fk(q)
    for t in targets.values():t[0,3]+=10
    assert not ik.solve(targets,q,max_nfev=20).success


def test_inverse_brown_real_input_shape():
    obs=decode(*packet())
    obs.metadata['intrinsics']['distortion_model']='realsense_2'
    obs.metadata['intrinsics']['D']=[.02,-.01,.001,-.002,.001]
    inp=prepare_input(obs)
    assert inp.rgb.shape==obs.rgb.shape and inp.depth.shape==obs.depth.shape
    assert 'inverse_brown_requires_device_reference_validation' in inp.reasons


def test_joint_history_no_latest_feedback_substitution():
    from wheel_sdk_bridge.feedback import JointHistory
    history=JointHistory();history.append(1.,{'j':0});history.append(1.05,{'j':1})
    assert history.at(1.025)['j']==pytest.approx(.5)
    with pytest.raises(ValueError):history.at(1.1)
    with pytest.raises(ValueError):history.append(.9,{'j':0})


def test_unverified_source_age_stays_diagnostic():
    obs=decode(*packet())
    obs.metadata['source_age_unverified']=True
    inp=prepare_input(obs)
    assert inp.diagnostic_only
    assert 'source_age_unverified' in inp.reasons


def test_collision_separating_axes():
    from wheel_motion_core.collision import Box,overlaps
    a=Box('a',np.eye(4),np.array([1,.1,.1]))
    t=np.eye(4);t[:3,3]=[0,1,0];b=Box('b',t,np.array([1,.1,.1]))
    assert not overlaps(a,b)
    t[:3,3]=[0,.1,0];assert overlaps(a,Box('c',t,[1,.1,.1]))
    t[:3,:3]=Rotation.from_euler('z',np.pi/2).as_matrix();t[:3,3]=[0,0,0]
    assert overlaps(a,Box('rotated',t,[1,.1,.1]))


def test_task_rejects_stale_plan_and_cancel_does_not_release():
    from wheel_task_core.state import TaskState
    state=TaskState();state.begin('box1');state.observed('box1','pair1',quality_valid=True)
    with pytest.raises(ValueError):state.planned('pair0',collision_checked=True,hardware_ready=True)
    state.planned('pair1',collision_checked=False,hardware_ready=True)
    assert state.phase=='PLAN'
    state.holding=True;state.cancel('lost feedback')
    assert state.phase=='HOLD' and state.holding
