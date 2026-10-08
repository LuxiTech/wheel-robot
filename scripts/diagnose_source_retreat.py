"""Replay saved retreat IK without moving the simulator."""
import json
import argparse
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from common import ROOT,OUTPUTS
from whole_body_control import WholeBodyIK


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--report',default='performance-validation/continuous-18-report.json')
    parser.add_argument('--output',default='source-retreat-fix')
    parser.add_argument('--skip-candidates',action='store_true')
    args=parser.parse_args()
    report=json.loads((OUTPUTS/args.report).read_text())
    gpu=WholeBodyIK(ROOT/'assets/robots/relay/relay_robot.urdf',backend='cuda')
    cpu=WholeBodyIK(ROOT/'assets/robots/relay/relay_robot.urdf',backend='cpu')
    actual=report['joints']['A'];command=report['joint_targets']['A']
    arm=gpu.arms['left']
    root=np.array(report['hands']['A']['left']) @ np.linalg.inv(arm.fk([actual[n] for n in arm.names],actual['lift_joint']))
    base=np.r_[root[:3,3],Rotation.from_matrix(root[:3,:3]).as_euler('xyz')[2]]
    starts={s:root @ a.fk([command[n] for n in a.names],command['lift_joint']) for s,a in gpu.arms.items()}
    goals={s:np.array(t) for s,t in report['hand_goals']['A'].items()}
    rows=[]
    for name,solver,attempts,window in [('original',gpu,1,.3),('gpu_multiseed',gpu,8,.3),('cpu_multiseed',cpu,4,.3)]:
        joints=command.copy();segments=[]
        for u in np.linspace(0,1,24)[1:]:
            targets={}
            for side,t in goals.items():
                p=np.eye(4);p[:3,3]=(1-u)*starts[side][:3,3]+u*t[:3,3]
                p[:3,:3]=Slerp([0,1],Rotation.from_matrix([starts[side][:3,:3],t[:3,:3]]))([u]).as_matrix()[0]
                targets[side]=p
            ans=solver.solve(targets,base,joints,attempts=attempts,joint_window=window)
            segments.append(dict(fraction=float(u),success=bool(ans.success),position_error=ans.position_error,
                angle_error=ans.angle_error,lift=ans.joints['lift_joint'],
                near_limits={n:float(min(ans.joints[n]-solver.lower[i],solver.upper[i]-ans.joints[n]))
                    for i,n in enumerate(solver.names) if min(ans.joints[n]-solver.lower[i],solver.upper[i]-ans.joints[n])<.02}))
            if not ans.success:break
            joints=ans.joints
        row=dict(case=name,success=segments[-1]['success'] and len(segments)==23,segments=segments)
        rows.append(row);print(json.dumps(row),flush=True)
    # Split translations from roll; try retract, rise, then rotate, and vice versa.
    for name,order in [('rise_retract_roll',['rise','retract','roll']),('roll_retract_rise',['roll','retract','rise']),('retract_rise_roll',['retract','rise','roll'])]:
        joints=command.copy();current={s:t.copy() for s,t in starts.items()};segments=[];ok=True
        for phase in order:
            end={s:t.copy() for s,t in current.items()}
            for s in end:
                if phase=='rise':end[s][2,3]=goals[s][2,3]
                elif phase=='retract':end[s][:2,3]=goals[s][:2,3]
                else:end[s][:3,:3]=goals[s][:3,:3]
            begin={s:t.copy() for s,t in current.items()}
            for u in np.linspace(0,1,24)[1:]:
                targets={}
                for s,t in end.items():
                    p=np.eye(4);p[:3,3]=(1-u)*begin[s][:3,3]+u*t[:3,3]
                    p[:3,:3]=Slerp([0,1],Rotation.from_matrix([begin[s][:3,:3],t[:3,:3]]))([u]).as_matrix()[0];targets[s]=p
                ans=gpu.solve(targets,base,joints,attempts=4,joint_window=.3)
                if not ans.success:
                    segments.append(dict(phase=phase,fraction=float(u),success=False,position_error=ans.position_error,angle_error=ans.angle_error));ok=False;break
                joints=ans.joints
            if not ok:break
            current=end;segments.append(dict(phase=phase,success=True,lift=joints['lift_joint']))
        row=dict(case=name,success=ok,segments=segments);rows.append(row);print(json.dumps(row),flush=True)
    out=OUTPUTS/args.output;out.mkdir(parents=True,exist_ok=True)
    (out/'ik-replay.json').write_text(json.dumps(dict(base=base.tolist(),start_joints=command,goals={s:t.tolist() for s,t in goals.items()},cases=rows),indent=2))
    if args.skip_candidates:return
    candidates=[]
    # Modify the object displacement relative to the saved final hand goals.
    box=report['boxes'][6]
    origin=np.asarray(box['position'])+np.array([.1,0,.02])
    axis=np.array([np.cos(base[3]),np.sin(base[3]),0.])
    for retract in (.03,.06,.10):
        for dz in (0.,-.04,-.08):
            for roll in (0.,8.,-8.):
                rotate=Rotation.from_rotvec(axis*np.deg2rad(roll+8.)).as_matrix()
                end={}
                for s,t in goals.items():
                    p=t.copy();p[:3,:3]=rotate @ t[:3,:3]
                    p[:3,3]=origin+rotate@(t[:3,3]-origin)+np.array([retract-.1,0,dz-.02]);end[s]=p
                joints=command.copy();ok=True;maximum=0.
                for u in np.linspace(0,1,24)[1:]:
                    targets={}
                    for s,t in end.items():
                        p=np.eye(4);p[:3,3]=(1-u)*starts[s][:3,3]+u*t[:3,3]
                        p[:3,:3]=Slerp([0,1],Rotation.from_matrix([starts[s][:3,:3],t[:3,:3]]))([u]).as_matrix()[0];targets[s]=p
                    ans=gpu.solve(targets,base,joints,attempts=4,joint_window=.3)
                    maximum=max(maximum,ans.position_error)
                    if not ans.success:ok=False;break
                    joints=ans.joints
                row=dict(retract=retract,dz=dz,roll=roll,success=ok,max_position_error=maximum,lift=joints['lift_joint'])
                candidates.append(row)
                if ok:print('CANDIDATE',json.dumps(row),flush=True)
    (out/'retreat-candidates.json').write_text(json.dumps(candidates,indent=2))


if __name__=='__main__':main()
