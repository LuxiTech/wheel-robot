"""Isolated, repeatable loop ablations; never a manipulation acceptance test.

Uses the normal scene bootstrap, with an isolated output directory and IPC file.
The production runner is read, not modified. Run only with other physics paused.
"""
import collections
import json
import math
import os
from pathlib import Path
import sys
import time


def profile(env):
    import carb
    import omni.physx
    import omni.kit.app
    from pxr import PhysicsSchemaTools
    out = env['OUTPUTS']
    callback = env['on_contacts']
    publisher = env['state_publisher']
    optimized_publish = publisher.publish
    callback_seconds = 0.
    callback_calls = 0
    pairs = collections.Counter()

    def category(path):
        if '/Crates/' in path:
            return 'crate'
        if '/Robots/' in path:
            who = 'A' if '/Robot_A/' in path else 'B'
            if 'gripper' in path:
                return who + '_gripper'
            if 'wheel' in path or 'roller' in path:
                return who + '_wheel'
            return who + '_body_arm'
        if 'Pallet' in path:
            return 'pallet'
        if 'Handoff' in path:
            return 'handoff'
        return 'environment'

    def measured(headers, data):
        nonlocal callback_seconds, callback_calls
        started = time.perf_counter()
        callback(headers, data)
        callback_seconds += time.perf_counter() - started
        callback_calls += 1

    def legacy_contacts(headers, data):
        from omni.physx.bindings._physx import ContactEventType
        for h in headers:
            a = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
            b = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            lost = h.type == ContactEventType.CONTACT_LOST
            if lost or h.num_contact_data:
                env['contact_tracker'].update(a,b,int(h.collider0),int(h.collider1),lost=lost)
            if h.num_contact_data:
                a = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
                b = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
                if '/Crates/' in a or '/Crates/' in b:
                    env['contacts'].add(tuple(sorted((a,b))))

    def legacy_publish(simulation_seconds, operator):
        from pxr import UsdGeom
        now=time.monotonic()
        if now-publisher.last<.05:return
        publisher.last=now
        started=time.perf_counter()
        cache=UsdGeom.XformCache()
        poses=[]
        for path in publisher.physics_poses.paths:
            m=cache.GetLocalToWorldTransform(env['stage'].GetPrimAtPath(path))
            q=m.ExtractRotationQuat()
            poses.append([path,[*m.ExtractTranslation(),q.GetReal(),*q.GetImaginary()]])
        snapshot=dict(session=publisher.session,monotonic=now,updated_at=time.time(),
            simulation_seconds=simulation_seconds,poses=poses,
            joints={n:r.get_joint_positions().tolist() for n,r in env['robots'].items()},operator=operator)
        if publisher.channel.publish(snapshot):publisher.frames+=1
        publisher.seconds+=time.perf_counter()-started

    def census(headers, data):
        for h in headers:
            a = category(str(PhysicsSchemaTools.intToSdfPath(h.actor0)))
            b = category(str(PhysicsSchemaTools.intToSdfPath(h.actor1)))
            pairs[' / '.join(sorted((a, b)))] += 1

    interface = omni.physx.get_physx_simulation_interface()
    manager = omni.kit.app.get_app().get_extension_manager()
    results = []
    # Each run resets to the same dense 18-box scene and repeats the same inputs.
    default_modes = ['baseline', 'no_export', 'no_python_contacts', 'baseline_repeat',
                     'no_ui_listeners', 'no_ui_no_export', 'no_ui_no_export_no_contacts']
    modes = os.environ.get('RELAY_PROFILE_MODES', ','.join(default_modes)).split(',')
    settings = carb.settings.get_settings()
    original_usd = settings.get('/physics/updateToUsd')
    for mode in modes:
        legacy=mode.startswith('legacy')
        callback=legacy_contacts if legacy else env['on_contacts']
        publisher.publish=legacy_publish if legacy else optimized_publish
        settings.set('/physics/updateToUsd', True if legacy else (False if 'no_usd' in mode or mode.startswith('optimized') else original_usd))
        if mode == 'no_ui_listeners':
            extensions = ['omni.kit.window.stage', 'omni.kit.window.property',
                          'omni.kit.window.viewport', 'omni.behavior.scripting.core']
            changes = []
            for name in extensions:
                before = manager.is_extension_enabled(name)
                manager.set_extension_enabled_immediate(name, False)
                changes.append(dict(name=name, before=before,
                                    after=manager.is_extension_enabled(name)))
            (out / 'extension-changes.json').write_text(json.dumps(changes, indent=2))
        env['subscription'] = None
        env['state_publisher'] = None if 'no_export' in mode else publisher
        if mode not in ('no_python_contacts', 'no_ui_no_export_no_contacts', 'no_usd_no_export_no_contacts'):
            env['subscription'] = interface.subscribe_contact_report_events(measured)
        env['reset']()
        env['step'](120)
        callback_seconds = 0.
        callback_calls = 0
        exported_before = publisher.seconds
        durations = []
        # One full period returns the velocity command to its initial phase.
        for tick in range(480):
            for who, ctl in env['controls'].items():
                direction = 1 if who.endswith('A') else -1
                ctl.set_twist(direction * .12 * math.sin(tick * math.tau / 480), 0., .08)
            started = time.perf_counter()
            env['step']()
            durations.append(time.perf_counter() - started)
        row = dict(mode=mode, steps=len(durations), wall_seconds=sum(durations),
                   mean_ms=sum(durations) / len(durations) * 1000,
                   callback_seconds=callback_seconds, callback_calls=callback_calls,
                   export_seconds=publisher.seconds-exported_before,
                   realtime_factor=len(durations)*env['c']['physics']['dt']/sum(durations))
        if hasattr(publisher,'physics_poses'):
            import numpy as np
            reader=publisher.physics_poses
            snapshot=reader.read()
            errors=[]
            for obj in [*env['robots'].values(),*env['crates'],env['slider']]:
                p,q=obj.get_world_pose()
                actual=snapshot[reader.indices[obj.prim_path]]
                errors.append(float(np.linalg.norm(actual[:3]-p)))
                if abs(float(np.dot(actual[3:],q)))<.99999:
                    raise RuntimeError('Physical quaternion mismatch')
            row['max_pose_error_m']=max(errors)
            if max(errors)>1e-5:
                raise RuntimeError('Physical pose mismatch')
            if legacy:
                from pxr import UsdGeom
                cache=UsdGeom.XformCache()
                usd_errors=[]
                for path, actual in zip(reader.body_paths,snapshot):
                    m=cache.GetLocalToWorldTransform(env['stage'].GetPrimAtPath(path))
                    usd_errors.append(float(np.linalg.norm(np.asarray(m.ExtractTranslation())-actual[:3])))
                    q=m.ExtractRotationQuat()
                    if abs(float(np.dot([q.GetReal(),*q.GetImaginary()],actual[3:])))<.99999:
                        raise RuntimeError('USD orientation mismatch')
                row['max_usd_pose_error_m']=max(usd_errors)
                if max(usd_errors)>1e-5:
                    raise RuntimeError('USD position mismatch')
        results.append(row)
        (out / 'ablation-results.json').write_text(json.dumps(results, indent=2))
        print('PROFILE_RESULT', json.dumps(row), flush=True)
    # Separate census, excluded from timing to avoid charging its own overhead.
    env['subscription'] = None
    env['subscription'] = interface.subscribe_contact_report_events(census)
    env['reset']()
    env['step'](120)
    pairs.clear()
    env['step'](120)
    (out / 'contact-census.json').write_text(json.dumps(dict(pairs.most_common()), indent=2))
    env['subscription'] = None
    env['state_publisher'] = None


if __name__ == '__main__':
    import common
    common.OUTPUTS = common.ROOT / 'outputs/performance-diagnosis' / os.environ.get('RELAY_PROFILE_RUN', 'isolated-ablation')
    common.OUTPUTS.mkdir(parents=True, exist_ok=True)
    (common.OUTPUTS / 'robot_cameras').mkdir(exist_ok=True)
    os.environ['RELAY_STATE_PATH'] = str(common.OUTPUTS / 'state.bin')
    runner = Path(__file__).with_name('run_relay_scene.py')
    source = runner.read_text()
    marker = '    if not args.test:\n        count=0\n'
    if source.count(marker) != 1:
        raise RuntimeError('Runner bootstrap changed; update diagnostic hook')
    source = source.replace(marker, '    profile_hook(globals())\n' + marker)
    sys.argv = [str(runner), '--physics-only', '--steps', '1']
    exec(compile(source, str(runner), 'exec'),
         dict(__name__='__main__', __file__=str(runner), profile_hook=profile))
