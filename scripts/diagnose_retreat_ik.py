"""Offline IK ablations from a retained relay failure; never commands robots."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from whole_body_control import WholeBodyIK
from common import ROOT

p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args()
report=json.loads((args.run/'relay_report.json').read_text())
target_record=json.loads((args.run/'frames/000101/rule_targets_latest.json').read_text())
targets={s:np.array(t) for s,t in target_record['targets'].items()}
settings=report['task_settings']
solvers={kind:WholeBodyIK(ROOT/'assets/robots/relay/relay_robot.urdf',settings['tcp'],backend=kind) for kind in ('cuda','cpu')}
ik=solvers['cuda'];q=report['joints']['A']
arm=ik.arms['left'];root=np.array(report['hands']['A']['left'])@np.linalg.inv(arm.fk([q[n] for n in arm.names],q['lift_joint']))
base=np.r_[root[:3,3],Rotation.from_matrix(root[:3,:3]).as_euler('xyz')[2]]
starts={s:root@a.fk([q[n] for n in a.names],q['lift_joint']) for s,a in ik.arms.items()}
results=dict(base=base.tolist(),seed_source='Measured failure feedback; pre-fault drive commands were overwritten by the stop handler.',cases=[])

def path(label,solver,goal,attempts=1,window=.3,seed=None):
    joints=q.copy() if seed is None else seed.copy()
    origin={s:root@a.fk([joints[n] for n in a.names],joints['lift_joint']) for s,a in ik.arms.items()}
    rows=[]
    for u in np.linspace(0,1,24)[1:]:
        step={}
        for s,t in goal.items():
            v=np.eye(4);v[:3,3]=(1-u)*origin[s][:3,3]+u*t[:3,3]
            v[:3,:3]=Slerp([0,1],Rotation.from_matrix([origin[s][:3,:3],t[:3,:3]]))([u]).as_matrix()[0];step[s]=v
        answer=solver.solve(step,base,joints,attempts=attempts,joint_window=window)
        jump=max(abs(answer.joints[n]-joints[n]) for n in solver.names[1:])
        rows.append(dict(u=float(u),success=answer.success,position_error=answer.position_error,angle_error=answer.angle_error,max_joint_step=jump,joints=answer.joints))
        if not answer.success or jump>.45:break
        joints=answer.joints
    row=dict(label=label,passed=len(rows)==23 and rows[-1]['success'] and rows[-1]['max_joint_step']<=.45,steps=rows)
    results['cases'].append(row)
    print(label,row['passed'],'u',rows[-1]['u'],'pe',rows[-1]['position_error'],'ae',rows[-1]['angle_error'],flush=True)
    (args.run/'ik_diagnosis.json').write_text(json.dumps(results,indent=2,default=lambda v:v.item()))
    return row

baseline=path('original_gpu_1',ik,targets)
path('original_gpu_32',ik,targets,attempts=32)
path('original_cpu_4',solvers['cpu'],targets,attempts=4)
path('original_gpu_wider_window',ik,targets,attempts=32,window=.6)
for retract,raise_m in ((0.,0.),(.03,0.),(.03,.02),(.06,0.),(.1,0.)):
    alternative={s:t.copy() for s,t in targets.items()}
    # Original goal is 0.10 m outward in world +X and 0.02 m up.
    for t in alternative.values():t[0,3]+=retract-.1;t[2,3]+=raise_m-.02
    path(f'alternative_retract_{retract}_raise_{raise_m}',ik,alternative,attempts=4)

for name,goal in [('failure_waypoint',None),('endpoint',targets)]:
    if goal is None:
        u=baseline['steps'][-1]['u'];goal={}
        for s,t in targets.items():
            v=np.eye(4);v[:3,3]=(1-u)*starts[s][:3,3]+u*t[:3,3]
            v[:3,:3]=Slerp([0,1],Rotation.from_matrix([starts[s][:3,:3],t[:3,:3]]))([u]).as_matrix()[0];goal[s]=v
    answer=ik.solve(goal,base,q,attempts=128)
    margins={n:min(answer.joints[n]-ik.lower[i],ik.upper[i]-answer.joints[n]) for i,n in enumerate(ik.names)}
    results[name]=dict(success=answer.success,position_error=answer.position_error,angle_error=answer.angle_error,joints=answer.joints,limit_margins=margins)
    print(name,results[name],flush=True)
(args.run/'ik_diagnosis.json').write_text(json.dumps(results,indent=2,default=lambda v:v.item()))

# Counterfactual limits are diagnostic only: no URDF edits or drive commands.
low,high=ik.lower.copy(),ik.upper.copy()
for mode in ('lift_only','wrists_only','lift_and_wrists'):
    ik.lower=low.copy();ik.upper=high.copy()
    if 'lift' in mode:ik.upper[0]=.5
    if 'wrists' in mode:
        for i,n in enumerate(ik.names):
            if n.endswith(('joint5','joint6')):
                ik.lower[i]-=.35;ik.upper[i]+=.35
    path('counterfactual_'+mode,ik,targets,attempts=32)
ik.lower=low;ik.upper=high

import gpu_whole_body_ik as gpu
original_launch=gpu.wp.launch
def longer_solve(*a,**kw):
    if a and a[0] is gpu.solve_kernel:
        kw['inputs']=list(kw['inputs']);kw['inputs'][-3]=600
    return original_launch(*a,**kw)
gpu.wp.launch=longer_solve
ik.gpu.cache.clear()
try:
    path('original_gpu_600_iterations',ik,targets,attempts=32)
finally:
    gpu.wp.launch=original_launch
