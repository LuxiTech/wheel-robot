"""Host orchestrator. Every worker is isolated; SDK access is read-only."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import uuid
from runlog import RunLog, write_json

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', choices=['sim','sdk'], default='sim', help='Default sim never contacts hardware; sdk is an explicit future hardware input')
    parser.add_argument('--config', help='Default: runtime.sim.json for sim; runtime.container.yaml for sdk')
    parser.add_argument('--box', nargs=4, type=float, help='Explicit RGB bounding box x1 y1 x2 y2')
    parser.add_argument('--cycles', type=int, default=1, help='Sequential fresh snapshot -> simulation runs')
    parser.add_argument('--gui', action='store_true', help='Show the actual execution in the local Isaac window')
    parser.add_argument('--replay', type=Path, help='Replay a run with a completed capture; no SDK access')
    parser.add_argument('--perception-only', action='store_true', help='Diagnose head RGB-D without requiring arm/lift feedback; no IK or simulation')
    parser.add_argument('--timeout', type=float, default=600, help='Maximum seconds per container stage')
    args = parser.parse_args()
    if args.gui:
        subprocess.run([sys.executable,'scripts/prepare_display.py'],cwd=ROOT,check=True)
    args.config = args.config or ('config/grasp/runtime.sim.json' if args.source=='sim' else 'config/grasp/runtime.container.yaml')
    if args.cycles < 1 or args.timeout <= 0:
        parser.error('cycles and timeout must be positive')
    for _ in range(args.cycles):
        run_id = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ-') + uuid.uuid4().hex[:8]
        directory = ROOT / 'outputs/grasp/runs' / run_id
        directory.mkdir(parents=True)
        log = RunLog(directory, 'orchestrator')
        container_dir = '/workspace/' + str(directory.relative_to(ROOT))
        manifest = dict(run_id=run_id, mode='replay_to_sim' if args.replay else ('simulation_pipeline' if args.source=='sim' else 'real_sdk_snapshot_to_sim'), source=args.source, hardware_commands=False,
                        box=args.box, target_id=run_id, status='running', perception_only=args.perception_only)
        write_json(directory/'manifest.json', manifest)
        print('RUN_DIRECTORY=' + str(directory), flush=True)
        compose = ['docker', 'compose', '-p', 'wheel-robot-grasp', '-f', 'compose.grasp.yaml']
        try:
            with log.stage('snapshot_configuration'):
                config = ((args.replay/'runtime.yaml') if args.replay else (ROOT / args.config)).read_bytes()
                (directory/'runtime.yaml').write_bytes(config)
                if args.source == 'sim' and not args.replay:
                    write_json(directory/'runtime.json',json.loads(config))
                manifest['config_sha256'] = hashlib.sha256(config).hexdigest()
                for filename in ('dependencies.lock.json', 'models.lock.json', 'validated-environment.json'):
                    (directory/filename).write_bytes((ROOT/'config/grasp'/filename).read_bytes())
                sources = [* (ROOT/'tools/grasp').glob('*.py'),
                           * (ROOT/'ros2_ws/src/wheel_grasp_core/wheel_grasp').glob('*.py'),
                           ROOT/'scripts/run_grasp_snapshot.py', ROOT/'scripts/capture_grasp_sim.py',
                           ROOT/'scripts/grasp_sim_scene.py', ROOT/'compose.grasp.yaml']
                sources += [ROOT/'scripts/grasp_carry.py']
                write_json(directory/'source_hashes.json', {
                    str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
                with (directory/'container-images.json').open('w') as out:
                    images = ['wheel-robot-grasp-perception:fp-a1b694b']
                    if args.source == 'sdk' and not args.replay:
                        images.append('wheel-robot-sdk-bridge:ba0c732')
                    subprocess.run(['docker', 'image', 'inspect', *images], cwd=ROOT, stdout=out, check=True)
                with (directory/'git-state.txt').open('w') as out:
                    subprocess.run(['git', 'status', '--short'], cwd=ROOT, stdout=out, check=True)
                    subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, stdout=out, check=True)
            if args.replay:
                with log.stage('replay_inputs'):
                    receipt = json.loads((args.replay/'capture.result.json').read_text())
                    if receipt.get('status') != 'completed' or (not args.perception_only and not receipt.get('simulation_initialization_valid')):
                        raise ValueError('Replay requires a validated capture')
                    if args.box is None:
                        manifest['box'] = json.loads((args.replay/'manifest.json').read_text()).get('box')
                    manifest['source'] = json.loads((args.replay/'manifest.json').read_text()).get('source','sdk')
                    for filename in ('observation.npz', 'joints.json', 'capture_context.json', 'runtime.json',
                                     'asset_hashes.json'):
                        shutil.copyfile(args.replay/filename, directory/filename)
                    hashes = json.loads((directory/'asset_hashes.json').read_text())
                    for name, expected in hashes.items():
                        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected:
                            raise ValueError('Replay asset changed: '+name)
                    manifest['replay_source'] = str(args.replay.resolve())
            for stage, service in [('capture', 'sim' if args.source=='sim' else 'sdk_bridge'), ('perception_plan', 'perception'), ('simulate', 'sim')]:
                if args.replay and stage == 'capture':
                    continue
                if args.perception_only and stage == 'simulate':
                    continue
                manifest['stage'] = stage
                write_json(directory/'manifest.json', manifest)
                name = 'grasp-' + run_id.lower() + '-' + stage.replace('_', '-')
                worker = ['/isaac-sim/python.sh', '/workspace/scripts/'+('capture_grasp_sim.py' if stage=='capture' else 'run_grasp_snapshot.py'), container_dir] if service == 'sim' else []
                command = compose + ['--profile', 'sim', 'run', '--rm', '--no-deps', '--name', name,
                                     '-T', '-e', 'PYTHONUNBUFFERED=1']
                if service == 'sdk_bridge':
                    command += ['--entrypoint', 'python3']
                if args.gui and stage == 'simulate':
                    command += ['-e','GRASP_GUI=1','-e','DISPLAY='+os.environ['DISPLAY'],
                                '-e','XAUTHORITY=/tmp/wheel-robot.Xauthority',
                                '-v','/tmp/.X11-unix:/tmp/.X11-unix:ro',
                                '-v',str(ROOT/'.runtime/gui/Xauthority')+':/tmp/wheel-robot.Xauthority:ro']
                if service == 'sim':
                    command += [service] + worker[1:]
                else:
                    command += [service, '/workspace/tools/grasp/pipeline_worker.py', stage, container_dir]
                with log.stage(stage, command=command):
                    with (directory/(stage+'.log')).open('w') as out:
                        process = subprocess.Popen(command, cwd=ROOT, stdout=out, stderr=subprocess.STDOUT)
                        try:
                            code = process.wait(timeout=args.timeout)
                            if code:
                                raise RuntimeError('%s exited %s; see %s.log and worker error JSON' % (stage, code, stage))
                        finally:
                            if process.poll() is None:
                                subprocess.run(['docker', 'stop', '-t', '5', name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)
                                process.terminate()
                                process.wait(timeout=15)
                    receipt = json.loads((directory/(stage+'.result.json')).read_text())
                    if receipt.get('run_id') != run_id or receipt.get('status') != 'completed':
                        raise ValueError('Missing or mismatched worker completion receipt')
                    manifest.setdefault('results', {})[stage] = receipt
            manifest['status'] = 'completed'
        except BaseException as exc:
            manifest.update(status='failed', error=str(exc))
            for component in ('capture', 'perception_plan', 'simulate', 'orchestrator'):
                error_path = directory/(component+'.error.json')
                if error_path.exists():
                    manifest['failure'] = json.loads(error_path.read_text())
                    print('FAILED [%s] %s' % (manifest['failure']['stage'],manifest['failure']['error']), flush=True)
                    break
            raise
        finally:
            write_json(directory/'manifest.json', manifest)


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda *_: sys.exit('terminated'))
    main()
