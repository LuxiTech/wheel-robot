"""Live isolation/render soak check. Requires an idle manual simulation."""
import argparse
import json
import statistics
import sys
import subprocess
import threading
import time
import urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs'
BASE='http://127.0.0.1:8766'
parser=argparse.ArgumentParser()
parser.add_argument('--seconds',type=int,default=180)
args=parser.parse_args()
def status():return json.load(urllib.request.urlopen(BASE+'/api/status',timeout=5))
def physics():return json.loads((OUT/'robot_cameras/operator_status.json').read_text())
s=status()
if not s['connected'] or s['operator']['mode']!='manual' or s['operator']['active']:
    raise SystemExit('Requires live manual mode; will not interrupt an active task')
session=s['session']
results=dict(session=session,started_at=time.time(),passed=False)
def command(action,**kw):
    payload=json.dumps(dict(session=session,action=action,**kw)).encode()
    response=json.load(urllib.request.urlopen(urllib.request.Request(BASE+'/api/command',data=payload,headers={'Content-Type':'application/json'}),timeout=5))
    until=time.monotonic()+6
    while time.monotonic()<until:
        for result in status()['operator']['results']:
            if result['id']==response['id']:
                if not result['ok']:raise RuntimeError(result['message'])
                return
        time.sleep(.1)
    raise RuntimeError('No command acknowledgement')
stream=urllib.request.urlopen(BASE+'/api/events',timeout=5)
def drain():
    try:
        while stream.readline():pass
    except (OSError,ValueError):pass
threading.Thread(target=drain,daemon=True).start()
try:
    a=physics()
    subprocess.run(['docker','pause','wheel-robot-relay-viewer'],check=True)
    try:time.sleep(8)
    finally:subprocess.run(['docker','unpause','wheel-robot-relay-viewer'],check=True)
    b=physics()
    results['paused_viewer_simulation_delta']=b['simulation_seconds']-a['simulation_seconds']
    assert results['paused_viewer_simulation_delta']>0 and b['session']==session
    subprocess.run(['docker','stop','wheel-robot-relay-viewer'],check=True)
    a=physics();time.sleep(8);b=physics()
    results['closed_viewer_simulation_delta']=b['simulation_seconds']-a['simulation_seconds']
    assert results['closed_viewer_simulation_delta']>0 and b['session']==session
    command('lift',robot='Robot_A',height=.35)
    results['control_with_viewer_closed']=True
    subprocess.run([str(ROOT/'scripts/simctl'),'relay-viewer'],check=True,cwd=ROOT)
    until=time.monotonic()+90
    while time.monotonic()<until:
        p=OUT/'relay_viewer_performance.json'
        if p.exists() and time.time()-json.loads(p.read_text())['updated_at']<5:break
        time.sleep(.5)
    else:raise RuntimeError('Viewer failed to reconnect')
    try:command('lift',robot='Robot_A',height=.15)
    except RuntimeError as error:
        assert '地面' in str(error)
        results['unsafe_manual_lowering_rejected']=True
    else:raise AssertionError('Unsafe manual lowering was accepted')
    samples=[];camera_versions=[]
    began=time.monotonic();next_motion=began;cycle=0
    while time.monotonic()-began<args.seconds:
        if time.monotonic()>=next_motion:
            for robot in ('Robot_A','Robot_B'):
                for side in ('left','right'):
                    command('gripper',robot=robot,side=side,closed=bool(cycle%2))
            next_motion=time.monotonic()+8;cycle+=1
        viewer=json.loads((OUT/'relay_viewer_performance.json').read_text())
        assert viewer['session']==session and time.time()-viewer['updated_at']<6,'Viewer lost during soak'
        assert physics()['session']==session,'Physics reset during viewer test'
        samples.append(viewer)
        st=status()
        camera_versions.append([c.get('version',0) for c in st['cameras']])
        time.sleep(2)
    results.update(stress_seconds=time.monotonic()-began,gripper_cycles=cycle,
        viewer_fps_median=statistics.median(x['display_fps'] for x in samples),
        viewer_fps_min=min(x['display_fps'] for x in samples),
        state_age_max_seconds=max(x['state_age_seconds'] for x in samples),
        camera_versions_start=camera_versions[0],camera_versions_end=camera_versions[-1],
        physics_session_preserved=True)
    assert len(camera_versions[-1])==2 and min(camera_versions[-1])>0
    assert all(end>start for start,end in zip(camera_versions[0],camera_versions[-1])), 'Camera frames did not update during motion'
    results['passed']=True
finally:
    if sys.exc_info()[1] is not None:
        results['passed']=False
        results['error']=str(sys.exc_info()[1])
    try:command('reset')
    except Exception as error:results['cleanup_error']=str(error)
    stream.close()
    (OUT/'relay_decoupled_final_acceptance.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results),flush=True)
