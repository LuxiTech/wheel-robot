"""Launch isolated physics and GUI containers; GUI exit never stops physics."""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--viewer-only',action='store_true')
parser.add_argument('--camera-only',action='store_true')
args,physics_args=parser.parse_known_args()
os.chdir(ROOT)
IPC=Path('/dev/shm')/f'wheel-robot-relay-{os.getuid()}'
IPC.mkdir(mode=0o700,exist_ok=True)

def launch(role,extra_args=()):
    name='wheel-robot-relay-'+role
    inspection=subprocess.run(['docker','inspect',name],capture_output=True,text=True)
    if inspection.returncode==0:
        if json.loads(inspection.stdout)[0]['State']['Running']:
            if role in ('viewer','camera'):return name
            raise RuntimeError('Physics is already running')
        subprocess.run(['docker','rm',name],check=True,stdout=subprocess.DEVNULL)
    runtime=ROOT/'.runtime'/('isaac-'+role)
    new_runtime=not runtime.exists()
    for part in ('kit-cache','kit-data','kit-config','user-config','cache','compute','omniverse','logs','config','ov','data','pkg'):
        (runtime/part).mkdir(parents=True,exist_ok=True)
    if new_runtime:
        # Seed a private cache once. Never let processes write the same cache.
        for part in ('shadercache','nv_shadercache','DerivedDataCache'):
            source=ROOT/'.runtime/isaac/kit-cache'/part
            if source.exists():shutil.copytree(source,runtime/'kit-cache'/part,dirs_exist_ok=True)
    env=os.environ.copy()
    env.update(GPU_ID='0' if role=='physics' else '1',RELAY_IPC_DIR=str(IPC),
        RELAY_RUNTIME_ROOT=str(runtime),RELAY_CPU_LIMIT='8' if role=='physics' else ('2' if role=='camera' else '6'))
    command=['docker','compose','-f','compose.yaml','-f','compose.split.yaml']
    if role=='viewer':command+=['-f','compose.gui.yaml']
    command+=['run','-d','--name',name,'isaac-sim']
    if role=='physics':command+=['/workspace/scripts/run_relay_scene.py','--physics-only',*physics_args]
    elif role=='camera':command+=['/workspace/scripts/view_relay_state.py','--camera-worker','--fps','5']
    else:command+=['/workspace/scripts/view_relay_state.py','--no-camera-images',*extra_args]
    subprocess.run(command,env=env,check=True)
    return name

if not args.camera_only:
    subprocess.run([sys.executable,'scripts/prepare_display.py'],check=True)
if args.viewer_only:
    launch('viewer',physics_args)
elif args.camera_only:
    launch('camera')
else:
    server_log=(ROOT/'outputs/decoupled_console.log').open('a')
    server=subprocess.Popen([sys.executable,'scripts/robot_camera_server.py'],stdout=server_log,stderr=subprocess.STDOUT)
    time.sleep(.4)
    if server.poll() is not None:raise RuntimeError('Web console failed to start; inspect outputs/decoupled_console.log')
    physics=None
    try:
        physics=launch('physics')
        launch('viewer')
        launch('camera')
        print('DECOUPLED_READY: physics GPU 0, viewer GPU 1, http://127.0.0.1:8766',flush=True)
        subprocess.run(['docker','wait',physics],check=True)
    finally:
        server.terminate()
        server.wait(timeout=5)
