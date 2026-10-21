"""Run an isolated physical grasp check and retain contact/trajectory evidence.

Example (inside the Isaac container):
  python.sh scripts/validate_relay_grasp.py --case baseline --source-box 12
Overrides are local to this diagnostic process; production settings stay intact.
"""
import argparse
import hashlib
import json
import os
import runpy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common

parser = argparse.ArgumentParser()
parser.add_argument('--case', required=True)
parser.add_argument('--source-box', type=int, default=12)
parser.add_argument('--skill', choices=['grasp', 'retreat', 'direct', 'handoff'], default='grasp')
parser.add_argument('--overrides', default='{}', help='JSON settings overrides for this run only')
parser.add_argument('--overrides-file', help='JSON settings profile for this run only')
parser.add_argument('--capture', action='store_true')
args = parser.parse_args()
if Path(args.case).name != args.case:
    parser.error('--case must be a directory name')
out = common.OUTPUTS / 'pinch_grasp_validation' / args.case
out.mkdir(parents=True, exist_ok=False)
common.OUTPUTS = out
(out / 'robot_cameras').mkdir(exist_ok=True)
os.environ['RELAY_STATE_PATH'] = str(out / 'state.bin')
settings = json.loads(Path(args.overrides_file).read_text()) if args.overrides_file else {}
settings.update(json.loads(args.overrides))
(out / 'case.json').write_text(json.dumps(dict(vars(args), overrides_applied=settings, sha256={
    str(p.relative_to(common.ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
    for p in (common.ROOT/'assets/robots/relay/relay_robot.urdf',
              common.ROOT/'config/relay_demo_config.json',
              common.ROOT/'scripts/relay_runtime.py')}), indent=2))
import numpy as np
import relay_runtime

original_init = relay_runtime.RelayRuntime.__init__
original_validate = relay_runtime.RelayRuntime.validate_state
original_hands = relay_runtime.RelayRuntime.hands
commands = (out / 'motion_commands.jsonl').open('w', buffering=1)
trace = (out / 'contacts.jsonl').open('w', buffering=1)
contact_points = (out / 'contact_points.jsonl').open('w', buffering=1)
last_sample = -1.

def initialize(self, *a, **kw):
    original_init(self, *a, settings_overrides=settings, **kw)
    (out / 'effective_settings.json').write_text(json.dumps(self.settings, indent=2))
    (out / 'finger_collision_overrides.json').write_text(json.dumps(self.finger_collision_overrides, indent=2))
    import omni.physx
    from pxr import PhysicsSchemaTools

    def scalar_or_vector(value):
        try:
            return list(value)
        except TypeError:
            return float(value)

    def record_contact_points(headers, data):
        target = f'/World/Crates/Crate_{args.source_box:02d}'
        for header in headers:
            actors = [str(PhysicsSchemaTools.intToSdfPath(a))
                      for a in (header.actor0, header.actor1)]
            if target not in actors or not any('/Robots/' in a for a in actors):
                continue
            points = []
            for i in range(header.num_contact_data):
                point = data[header.contact_data_offset + i]
                points.append({name: scalar_or_vector(getattr(point, name))
                               for name in ('position', 'normal', 'impulse', 'separation')
                               if hasattr(point, name)})
            if points:
                contact_points.write(json.dumps(dict(t=self.time, actors=actors, points=points)) + '\n')

    self.validation_contact_subscription = (
        omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(record_contact_points))

def record(self):
    crates = []
    for i, (crate, spec) in enumerate(zip(self.crates, self.initial)):
        p, q = crate.get_world_pose()
        crates.append(dict(id=i, position=p.tolist(), quaternion=q.tolist(),
                           displacement=float(np.linalg.norm(p - np.array(spec['position']))),
                           contacts=sorted(self.box_contacts(i))))
    return dict(t=self.time, states=dict(self.states), working=dict(self.working_boxes),
                joints={w:{n:float(v) for n,v in self.joints(w).items()} for w in ('A','B')},
                targets={w:dict(self.controls['Robot_'+w].targets) for w in ('A','B')},
                hands={w:{s:t.tolist() for s,t in self.actual_hands(w).items()} for w in ('A','B')},
                bases={w:self.base(w).tolist() for w in ('A','B')}, crates=crates)

def validate(self):
    global last_sample
    if self.time - last_sample >= .05:
        trace.write(json.dumps(record(self)) + '\n')
        last_sample = self.time
    try:
        original_validate(self)
    except Exception:
        (out / 'failure_snapshot.json').write_text(json.dumps(record(self), indent=2))
        raise

def move_hands(self, who, targets, **kw):
    commands.write(json.dumps(dict(t=self.time, who=who, states=dict(self.states),
        targets={s:t.tolist() for s,t in targets.items()},
        cartesian=kw.get('cartesian',True))) + '\n')
    yield from original_hands(self, who, targets, **kw)

relay_runtime.RelayRuntime.hands = move_hands
relay_runtime.RelayRuntime.__init__ = initialize
relay_runtime.RelayRuntime.validate_state = validate
sys.argv = [str(common.ROOT / 'scripts/run_relay_scene.py'), '--auto', '--boxes', '1',
            '--skill-test', args.skill, '--steps', '30000']
if args.skill != 'handoff':
    sys.argv += ['--source-box', str(args.source_box)]
if args.capture:
    sys.argv.append('--capture')
else:
    sys.argv += ['--physics-only', '--exit-on-finish']
try:
    runpy.run_path(sys.argv[0], run_name='__main__')
finally:
    trace.close()
    commands.close()
    contact_points.close()
