"""Offline replay of an instrumented handoff-carry failure; no robot commands."""
import json, argparse
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from common import ROOT, OUTPUTS
from whole_body_control import WholeBodyIK


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--report', default='relay_demo_report.json')
    parser.add_argument('--output', default='handoff-top-diagnosis.json')
    args=parser.parse_args()
    report=json.loads((OUTPUTS/args.report).read_text())
    e=next(e for e in reversed(report['events']) if e['state']=='HANDOFF_CARRY_PATH_REJECTED')
    base=np.array(e['base']); grasp={s:np.array(t) for s,t in e['grasp_transforms'].items()}
    observed=np.array(e['observed_pose']); stage=np.array(e['staging_pose']); final=np.array(e['final_pose'])
    initial=report['joint_targets']['A']
    gpu=WholeBodyIK(ROOT/'assets/robots/relay/relay_robot.urdf',backend='cuda')
    cpu=WholeBodyIK(ROOT/'assets/robots/relay/relay_robot.urdf',backend='cpu')
    root=np.eye(4);root[:3,:3]=Rotation.from_euler('z',base[3]).as_matrix();root[:3,3]=base[:3]
    def segment(solver,joints,box,attempts=4,window=.3,seconds=3.):
        starts={s:root@a.fk([joints[n] for n in a.names],joints['lift_joint']) for s,a in solver.arms.items()}
        goals={s:box@t for s,t in grasp.items()}; maximum=0.
        for u in np.linspace(0,1,max(3,int(seconds*8)))[1:]:
            targets={}
            for s,g in goals.items():
                t=np.eye(4);t[:3,3]=(1-u)*starts[s][:3,3]+u*g[:3,3]
                t[:3,:3]=Slerp([0,1],Rotation.from_matrix([starts[s][:3,:3],g[:3,:3]]))([u]).as_matrix()[0]
                targets[s]=t
            ans=solver.solve(targets,base,joints,attempts=attempts,joint_window=window)
            delta=max(abs(ans.joints[n]-joints[n]) for n in solver.names[1:]); maximum=max(maximum,ans.position_error)
            joints=ans.joints
            if not ans.success or delta>.45:break
        margins={n:round(float(min(joints[n]-solver.lower[i],solver.upper[i]-joints[n])),6) for i,n in enumerate(solver.names)}
        return joints,dict(success=bool(ans.success and delta<=.45 and u==1),fraction=float(u),position_error=ans.position_error,
                          angle_error=ans.angle_error,max_position_error=maximum,lift=joints['lift_joint'],max_joint_delta=delta,
                          near_limits={n:m for n,m in margins.items() if m<.025})
    results=[]
    for name,solver,attempts in [('gpu_original',gpu,1),('gpu_8seeds',gpu,8),('cpu_4seeds',cpu,4)]:
        _,r=segment(solver,initial.copy(),stage,attempts);r['case']=name;results.append(r);print(json.dumps(r),flush=True)
    for order in [('rise','retract','rotate'),('retract','rise','rotate'),('rotate','retract','rise')]:
        pose=observed.copy();pose[:3,:3]=Rotation.from_euler('z',Rotation.from_matrix(pose[:3,:3]).as_euler('xyz')[2]).as_matrix()
        joints=initial.copy();legs=[]
        for phase in (*order,'final'):
            if phase=='rise':pose[2,3]=stage[2,3]
            elif phase=='retract':pose[:2,3]=stage[:2,3]
            elif phase=='rotate':pose[:3,:3]=stage[:3,:3]
            else:pose=final.copy()
            joints,r=segment(gpu,joints,pose,seconds=4. if phase=='final' else 3.);r['phase']=phase;legs.append(r)
            if not r['success']:break
        row=dict(case='_'.join(order),legs=legs);results.append(row);print(json.dumps(row),flush=True)
    # Examine direct final target without path-local joint bounds.
    for name,solver in [('gpu_endpoint',gpu),('cpu_endpoint',cpu)]:
        ans=solver.solve({s:stage@t for s,t in grasp.items()},base,initial,attempts=8)
        row=dict(case=name,max_joint_change=max(abs(ans.joints[n]-initial[n]) for n in solver.names[1:]),success=bool(ans.success),position_error=ans.position_error,angle_error=ans.angle_error,
                 near_limits={n:float(min(ans.joints[n]-solver.lower[i],solver.upper[i]-ans.joints[n])) for i,n in enumerate(solver.names)
                              if min(ans.joints[n]-solver.lower[i],solver.upper[i]-ans.joints[n])<.025})
        results.append(row);print(json.dumps(row),flush=True)
    # Preserve orientation and grasp, vary body-relative forward and lateral position.
    candidates=[]
    axes=root[:3,:3]
    for forward in (-.10,-.05,0.,.05,.10):
        for lateral in (-.10,-.05,0.,.05,.10):
            p=stage.copy();p[:3,3]+=axes@np.array([forward,lateral,0.])
            joints,r=segment(gpu,initial.copy(),p)
            if r['success']:
                finish=final.copy();finish[:3,3]+=axes@np.array([forward,lateral,0.])
                _,last=segment(gpu,joints,finish,seconds=4.)
            else:last=None
            row=dict(forward_offset=forward,lateral_offset=lateral,staging=r,final=last)
            candidates.append(row)
            if last and last['success']:print('CANDIDATE',json.dumps(row),flush=True)
    out=dict(snapshot=e,base_relative_observed=(np.linalg.inv(root)@observed).tolist(),base_relative_staging=(np.linalg.inv(root)@stage).tolist(),cases=results,candidates=candidates)
    (OUTPUTS/args.output).write_text(json.dumps(out,indent=2))

if __name__=='__main__':main()
