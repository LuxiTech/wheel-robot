"""Isolated snapshot workers. No SDK write/control API is used."""
import argparse
import hashlib
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time
from urllib.request import urlopen

sys.path.insert(0, '/workspace/grasp_core')
from runlog import RunLog, write_json


def measured_state(state, mapping, request_seconds):
    from wheel_grasp.sdk import arm_feedback, LiftMapping
    if state['system'].get('mock') is not False:
        raise ValueError('Real SDK state required')
    arms = state['arms']
    if not isinstance(arms, dict):
        raise ValueError('SDK arms feedback unavailable: /api/state arms=null; restore measured arm feedback before simulation initialization')
    age = (int(state['server_mono_ns'])-int(arms['receive_mono_ns']))/1e9 + request_seconds
    if not 0 <= age <= .15:
        raise ValueError('Arm snapshot stale: %.3f seconds' % age)
    joints = arm_feedback(arms, max_age_ns=max(0, int((.15-age)*1e9)))
    if mapping is None:
        raise ValueError('lift_mapping missing: configure measured offset_m and direction')
    lift = state['lift']
    if not lift or lift.get('valid') is not True:
        raise ValueError('SDK lift feedback unavailable/invalid')
    age = (int(state['server_mono_ns'])-int(lift['receive_mono_ns']))/1e9 + request_seconds
    if not 0 <= age <= .15 or not 0 <= float(lift['effective_age_ms'])+age*1000 <= 150:
        raise ValueError('SDK lift feedback stale')
    joints['lift_joint'] = LiftMapping(**mapping).to_joint(lift['current_um'])
    return joints


def capture(cfg, directory, log, *, perception_only=False):
    import numpy as np
    import cv2
    from wheel_grasp.rgbd import RGBDClient, save_observation
    def state(label):
        start = time.monotonic()
        with urlopen(cfg['base_url'].rstrip('/')+'/api/state', timeout=3) as response:
            raw = json.load(response)
        duration = time.monotonic()-start
        write_json(directory/(label+'.json'), raw)
        return raw, duration
    with log.stage('sdk_state_before'):
        before, before_time = state('state_before')
    with log.stage('sdk_rgbd', camera=cfg['camera']):
        def raw_response(headers, body):
            write_json(directory/'rgbd_headers.json', headers)
            (directory/'rgbd_response.mime').write_bytes(body)
        observation = RGBDClient(cfg['base_url'], on_response=raw_response).receive(cfg['camera'])
        save_observation(directory/'observation.npz', observation)
        cv2.imwrite(str(directory/'raw_rgb.png'), cv2.cvtColor(observation.rgb, cv2.COLOR_RGB2BGR))
        log.event('sdk_rgbd', 'saved', camera_id=observation.key[0], pair_id=observation.key[1])
    with log.stage('sdk_state_after'):
        after, after_time = state('state_after')
    if perception_only:
        if cfg['camera'] != 'head_rgb':
            raise ValueError('Perception-only without measured joints requires fixed head_rgb camera')
        write_json(directory/'joints.json', {})
        write_json(directory/'capture_context.json', dict(pair_id=observation.key[1], camera_id=observation.key[0],
                   synchronization='fixed_head_perception_only', simulation_initialization_valid=False))
        log.event('feedback_validation', 'skipped', reason='explicit perception-only mode; no IK or simulation')
        return
    with log.stage('feedback_validation'):
        first = measured_state(before, cfg['lift_mapping'], before_time)
        last = measured_state(after, cfg['lift_mapping'], after_time)
        # HTTP snapshots cannot establish exposure-time wrist transforms.
        if cfg['camera'] != 'head_rgb':
            raise ValueError('Wrist camera requires timestamped joint interpolation; HTTP snapshot runner supports head_rgb only')
        span = (int(after['server_mono_ns'])-int(before['server_mono_ns']))/1e9
        if not 0 < span <= .5:
            raise ValueError('Snapshot state bracket exceeds 500 ms')
        for name in first:
            if abs(first[name]-last[name]) > (.001 if name == 'lift_joint' else .01):
                raise ValueError('Robot moved during snapshot: '+name)
        velocities = after['arms']['velocity']
        if not np.isfinite(velocities).all() or max(abs(float(v)) for i,v in enumerate(velocities) if i not in (7,15)) > .03:
            raise ValueError('Snapshot requires stationary physical arms')
        write_json(directory/'joints.json', last)
        write_json(directory/'capture_context.json', dict(pair_id=observation.key[1], camera_id=observation.key[0],
                   state_bracket_s=span, frame_id=cfg['base_frame'], synchronization='stationary_head_snapshot'))


def perceive_plan(cfg, directory, log, manifest):
    import numpy as np
    import cv2
    from wheel_grasp.rgbd import load_observation
    from wheel_grasp.kinematics import RobotModel, PairIK
    from wheel_grasp.backends import NanoDetectorSegmenter, FoundationPoseBackend
    from wheel_grasp.pipeline import GraspPipeline
    from wheel_grasp.planning import plan_pair
    from wheel_grasp.cli import json_value
    def save(name, value):
        write_json(directory/name, json.loads(json.dumps(value, default=json_value)))
    joints = json.loads((directory/'joints.json').read_text())
    simulation = manifest.get('source') == 'sim'
    observation = load_observation(directory/'observation.npz', allow_mock=simulation)
    if simulation and (observation.metadata.get('source') != 'isaac_sim' or observation.metadata.get('mock') is not True):
        raise ValueError('Simulation mode requires explicitly labelled Isaac observations')
    with log.stage('load_models', pair_id=observation.key[1]):
        m = cfg['model']
        robot = RobotModel(cfg['urdf'])
        segmenter = NanoDetectorSegmenter(m['owl_engine'],m['sam_encoder'],m['sam_decoder'],m['prompt'])
        estimator = FoundationPoseBackend(m['foundationpose_repository'], m['mesh'], debug_dir=str(directory/'foundationpose'))
    # Save intermediate outputs even when later quality checks reject the frame.
    class TracedSegmenter:
        def detect(self, rgb):
            with log.stage('detection', pair_id=observation.key[1]):
                cv2.imwrite(str(directory/'rgb.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                result = segmenter.detect(rgb)
                save('detections.json', result)
                return result
        def segment(self, rgb, box):
            with log.stage('segmentation', pair_id=observation.key[1]):
                result = segmenter.segment(rgb, box)
                cv2.imwrite(str(directory/'mask.png'), result.astype(np.uint8)*255)
                cv2.imwrite(str(directory/'rgb.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                return result
    class TracedEstimator:
        def register(self, inp, mask, identity):
            with log.stage('pose_registration', pair_id=observation.key[1]):
                result = estimator.register(inp, mask, identity)
                save('camera_pose.json', result)
                return result
    with log.stage('perception_and_keypoints', pair_id=observation.key[1]):
        pipe = GraspPipeline(robot, TracedSegmenter(), TracedEstimator(), cfg['rules'],
                             json.loads(Path(m['colliders']).read_text()), base=cfg['base_frame'])
        result = pipe.process(observation, joints, target_id=manifest['target_id'], box=manifest['box'])
        result.pop('mask')
        save('perception.json', result)
    if manifest.get('perception_only'):
        return
    with log.stage('joint_ik', pair_id=observation.key[1]):
        tcp = {side: np.eye(4) for side in ('left','right')}
        for side in tcp:
            tcp[side][:3,3] = cfg['tcp'][side]
        ik = PairIK(robot, tcp, base=cfg['base_frame'])
        plans = []
        current = joints
        clearance_lift = cfg.get('planning_clearance_lift_m')
        if clearance_lift is not None and clearance_lift > current['lift_joint']:
            with log.stage('plan_clearance_lift'):
                if not ik.lower[0] <= clearance_lift <= ik.upper[0]:
                    raise ValueError('Clearance lift outside URDF travel')
                raised = dict(current,lift_joint=float(clearance_lift))
                duration = 1.875*(clearance_lift-current['lift_joint'])/min(.03,robot.joints['lift_joint']['velocity'])
                phase = dict(phase='clearance',points=[dict(time=0.,joints=current),dict(time=duration,joints=raised)],
                             collision_checked=False,executable=False,reason='simulation_clearance_lift')
                plans.append(phase);save('plan_clearance.json',phase)
                result['clearance'] = ik.fk(raised)
                save('perception.json',result)
                current = raised
        for phase in ('pregrasp', 'grasp'):
            with log.stage('ik_'+phase):
                plan = plan_pair(ik, result[phase], current,
                                 steps=cfg.get('planning_steps',20),
                                 speed_limits={n: .03 if n=='lift_joint' else .2 for n in ik.names})
                current = plan.points[-1]['joints']
                plans.append(dict(phase=phase, **asdict(plan)))
                save('plan_'+phase+'.json', plans[-1])
        save('trajectory.json', dict(run_id=directory.name, pair_id=observation.key[1],
             mode='simulation_only', phases=plans, base_fixed=True,
             limitations=['No physical gripper closure or lift success claim',
                          'Environment contains only perceived target and floor; collision checks occur in simulation']))


def main():
    import yaml
    parser = argparse.ArgumentParser()
    parser.add_argument('stage', choices=['capture','perception_plan'])
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    directory = args.directory
    log = RunLog(directory, args.stage)
    with log.stage(args.stage):
        cfg = yaml.safe_load((directory/'runtime.yaml').read_text())
        write_json(directory/'runtime.json', cfg)
        if args.stage == 'capture':
            files = [cfg['urdf'], cfg['model']['mesh'], cfg['model']['colliders'],
                     'assets/robots/generated/robot.usdc', 'assets/robots/generated/robot_metadata.json']
            write_json(directory/'asset_hashes.json', {p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in files})
        manifest = json.loads((directory/'manifest.json').read_text())
        if args.stage == 'capture':
            capture(cfg, directory, log, perception_only=manifest.get('perception_only',False))
        else:
            perceive_plan(cfg, directory, log, manifest)
        write_json(directory/(args.stage+'.result.json'), dict(run_id=directory.name, status='completed',
                   simulation_initialization_valid=not manifest.get('perception_only',False)))


if __name__ == '__main__':
    main()
