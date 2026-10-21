"""Original relay task and scene, with an isolated persistent model-perception worker."""
import argparse,datetime,hashlib,json,os,subprocess,sys,time,uuid
from pathlib import Path
from runlog import write_json
ROOT=Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser();p.add_argument('--gui',action='store_true');p.add_argument('--boxes',type=int,default=18)
p.add_argument('--skill-test',choices=['grasp','direct','handoff','retreat']);p.add_argument('--steps',type=int,default=0)
p.add_argument('--task-profile',type=Path,help='Relay settings overrides, relative to the project or an absolute project path')
a=p.parse_args();os.chdir(ROOT)
profile=None
if a.task_profile:
 profile=a.task_profile.resolve()
 if not profile.is_relative_to(ROOT):p.error('--task-profile must be inside the project')
 overrides=json.loads(profile.read_text())
 if not isinstance(overrides,dict):p.error('--task-profile must contain a JSON object')
if a.gui:subprocess.run([sys.executable,'scripts/prepare_display.py'],check=True)
run=ROOT/'outputs/grasp/relay_runs'/(datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ-')+uuid.uuid4().hex[:8]);run.mkdir(parents=True)
container='/workspace/'+str(run.relative_to(ROOT));cfg=json.loads((ROOT/'config/grasp/runtime.sim.json').read_text());write_json(run/'runtime.json',cfg)
manifest=dict(run_id=run.name,status='starting',scene='assets/generated/relay/main.usda',task='original RelayRuntime',perception='rgbd',boxes=a.boxes,skill_test=a.skill_test,hardware_access=False)
write_json(run/'manifest.json',manifest)
if profile:
 write_json(run/'task_profile.json',overrides)
 manifest['task_profile']=str(profile.relative_to(ROOT))
 write_json(run/'manifest.json',manifest)
paths=[ROOT/'scripts/relay_runtime.py',ROOT/'scripts/relay_rgbd_perception.py',ROOT/'scripts/run_relay_scene.py',*list((ROOT/'tools/grasp').glob('*.py')),*list((ROOT/'ros2_ws/src/wheel_grasp_core/wheel_grasp').glob('*.py'))]
write_json(run/'source_hashes.json',{str(x.relative_to(ROOT)):hashlib.sha256(x.read_bytes()).hexdigest() for x in paths})
for filename in ('dependencies.lock.json','models.lock.json','validated-environment.json'):
 (run/filename).write_bytes((ROOT/'config/grasp'/filename).read_bytes())
assets=['assets/generated/relay/main.usda','assets/generated/relay/initial_state.json',
        'assets/robots/relay/relay_robot.urdf','assets/robots/generated/relay/robot_metadata.json',
        'config/relay_config.json','config/relay_demo_config.json',cfg['model']['mesh']]
write_json(run/'asset_hashes.json',{name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in assets})
compose=['docker','compose','-p','wheel-robot-grasp','-f','compose.grasp.yaml','--profile','sim','run','--rm','--no-deps','-T','-e','PYTHONUNBUFFERED=1']
worker_name='relay-model-'+run.name.lower();sim_name='relay-sim-'+run.name.lower();processes=[]
print('RUN_DIRECTORY='+str(run),flush=True)
try:
 with (run/'model_worker.log').open('w') as log:
  worker=subprocess.Popen(compose+['--name',worker_name,'perception','/workspace/tools/grasp/relay_model_worker.py',container],stdout=log,stderr=subprocess.STDOUT);processes.append(worker)
  deadline=time.monotonic()+180
  while not (run/'model.ready.json').exists():
   if worker.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Model worker startup failed; see model_worker.log')
   time.sleep(.2)
  command=compose+['--name',sim_name,'-e','RELAY_MODEL_RUN='+container,'-e','PYTHONPATH=/workspace/grasp_core:/workspace/tools/grasp']
  if a.gui:command+=['-e','DISPLAY='+os.environ['DISPLAY'],'-e','XAUTHORITY=/tmp/wheel-robot.Xauthority','-v','/tmp/.X11-unix:/tmp/.X11-unix:ro','-v',str(ROOT/'.runtime/gui/Xauthority')+':/tmp/wheel-robot.Xauthority:ro']
  command+=['sim','/workspace/scripts/run_relay_scene.py','--auto','--start-immediately','--perception','rgbd','--boxes',str(a.boxes),'--steps',str(a.steps)]
  if a.gui:command+=['--gui']
  if a.skill_test:command+=['--skill-test',a.skill_test]
  if profile:command+=['--task-profile',container+'/task_profile.json']
  write_json(run/'command.json',command);manifest['status']='running';write_json(run/'manifest.json',manifest)
  with (run/'simulation.log').open('w') as out:
   sim=subprocess.Popen(command,stdout=out,stderr=subprocess.STDOUT);processes.append(sim)
   while sim.poll() is None:
    report=run/'relay_report.json'
    if report.exists():
     result=json.loads(report.read_text())
     state='task_completed_viewer_open' if (result.get('passed') or result.get('skill_passed')) else 'task_failed_viewer_open'
     if manifest['status']!=state:
      manifest.update(status=state,error=result.get('error'));write_json(run/'manifest.json',manifest)
    time.sleep(.5)
   code=sim.returncode
  report=run/'relay_report.json'
  if report.exists():
   result=json.loads(report.read_text());write_json(run/'relay_report.json',result)
   manifest['status']='completed' if code==0 and (result.get('passed') or result.get('skill_passed')) else 'failed'
  else:manifest['status']='failed'
except BaseException as exc:
 manifest.update(status='failed',error=str(exc));raise
finally:
 (run/'stop').touch()
 for name in (sim_name,worker_name):subprocess.run(['docker','stop','-t','3',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
 for process in processes:
  try:process.wait(timeout=10)
  except subprocess.TimeoutExpired:process.terminate()
 write_json(run/'manifest.json',manifest)
