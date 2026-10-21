"""Read-only capture, diagnostics, calibrated FK and offline model planning."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import numpy as np
import yaml
from .rgbd import RGBDClient,save_observation,load_observation
from .kinematics import RobotModel,PairIK
from .pipeline import CAMERA_FRAMES,GraspPipeline
from .geometry import prepare_input,mask_quality
from .planning import plan_pair


def json_value(x):
    if isinstance(x,np.ndarray): return x.tolist()
    if isinstance(x,np.generic): return x.item()
    raise TypeError(type(x).__name__)


def build_parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default='config/grasp/runtime.yaml')
    sub=p.add_subparsers(dest='command',required=True)
    sub.add_parser('doctor')
    c=sub.add_parser('capture');c.add_argument('--url');c.add_argument('--camera');c.add_argument('--output',required=True)
    d=sub.add_parser('inspect');d.add_argument('observation')
    f=sub.add_parser('frames');f.add_argument('--joints',required=True,help='JSON named measured joints')
    k=sub.add_parser('ik');k.add_argument('--joints',required=True);k.add_argument('--targets',required=True)
    k.add_argument('--fixed-lift',type=float);k.add_argument('--output',required=True)
    g=sub.add_parser('perceive');g.add_argument('observation');g.add_argument('--joints',required=True)
    g.add_argument('--target-id',required=True);g.add_argument('--box',type=float,nargs=4);g.add_argument('--output',required=True)
    return p


def main():
    args=build_parser().parse_args()
    cfg=yaml.safe_load(Path(args.config).read_text())
    if args.command=='doctor':
        import importlib.util
        m=cfg['model']
        result=dict(sdk_commit=cfg['sdk_commit'],urdf_present=Path(cfg['urdf']).is_file(),
            model_assets={key:bool(m.get(key) and Path(m[key]).is_file()) for key in ('mesh','colliders','owl_engine','sam_encoder','sam_decoder')},
            modules={key:importlib.util.find_spec(key) is not None for key in ('rclpy','torch','tensorrt','nanoowl','nanosam','nvdiffrast')},
            lift_mapping_configured=cfg['lift_mapping'] is not None,tcp_hardware_validated=cfg['tcp_hardware_validated'],
            hardware_execution_implemented=False)
        print(json.dumps(result,ensure_ascii=False,indent=2));return
    if args.command=='capture':
        obs=RGBDClient(args.url or cfg['base_url']).receive(args.camera or cfg['camera'])
        save_observation(args.output,obs)
        print(json.dumps(dict(camera=obs.key[0],pair_id=obs.key[1],output=args.output),ensure_ascii=False));return
    if args.command=='inspect':
        obs=load_observation(args.observation);inp=prepare_input(obs)
        print(json.dumps(dict(metadata=obs.metadata,valid_depth=mask_quality(np.ones(inp.depth.shape,bool),inp.depth),
                              diagnostic_only=inp.diagnostic_only,reasons=inp.reasons),ensure_ascii=False,default=json_value,indent=2));return
    joints=json.loads(Path(args.joints).read_text());robot=RobotModel(cfg['urdf'])
    if args.command=='frames':
        result={c:robot.transform(f,joints,cfg['base_frame']) for c,f in CAMERA_FRAMES.items()}
    elif args.command=='ik':
        tcp={s:np.eye(4) for s in ('left','right')}
        for s in tcp: tcp[s][:3,3]=cfg['tcp'][s]
        ik=PairIK(robot,tcp,base=cfg['base_frame'])
        targets=json.loads(Path(args.targets).read_text())
        result=asdict(plan_pair(ik,targets,joints,fixed_lift=args.fixed_lift))
    else:
        from .backends import NanoDetectorSegmenter,FoundationPoseBackend
        m=cfg['model'];segmenter=NanoDetectorSegmenter(m['owl_engine'],m['sam_encoder'],m['sam_decoder'],m['prompt'])
        estimator=FoundationPoseBackend(m['foundationpose_repository'],m['mesh'],debug_dir='outputs/foundationpose')
        pipe=GraspPipeline(robot,segmenter,estimator,cfg['rules'],json.loads(Path(m['colliders']).read_text()),base=cfg['base_frame'])
        result=pipe.process(load_observation(args.observation),joints,target_id=args.target_id,box=args.box)
        result.pop('mask')
    text=json.dumps(result,ensure_ascii=False,default=json_value,indent=2)
    if hasattr(args,'output'):
        with Path(args.output).open('x') as out: out.write(text+'\n')
    else: print(text)


if __name__=='__main__': main()
