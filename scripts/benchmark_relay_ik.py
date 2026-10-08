"""Reproducible CPU/CUDA comparison including failed live-task poses."""
import json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from whole_body_control import WholeBodyIK,transform
from common import ROOT,OUTPUTS

def main():
    cpu=WholeBodyIK(ROOT/'assets/robots/relay/relay_robot.urdf')
    gpu=WholeBodyIK(ROOT/'assets/robots/relay/relay_robot.urdf',backend='cuda')
    gpu.gpu.cache_capacity=0
    cfg=json.loads((ROOT/'config/relay_robot_config.json').read_text())
    q={n:cfg['home'].get(n,0.) for n in cpu.names}
    rng=np.random.default_rng(5)
    cases=[]
    for i in range(12):
        seed={n:float(np.clip(q[n]+rng.uniform(-.2,.2),cpu.lower[j]+.001,cpu.upper[j]-.001)) for j,n in enumerate(cpu.names)}
        target={s:a.fk([seed[n] for n in a.names],seed['lift_joint']) for s,a in cpu.arms.items()}
        cases.append((f'reachable_{i}',target,np.zeros(4),q,dict(attempts=3)))
    target=np.eye(4);target[:3,3]=[10,10,10]
    cases.append(('unreachable',{'left':target},np.zeros(4),q,dict(attempts=1)))
    for f in ['relay_center_retreat_before_report.json','relay_base_first_handoff_pass_b_grasp_failure_report.json','relay_demo_report.json']:
        p=OUTPUTS/f
        if not p.exists():continue
        report=json.loads(p.read_text())
        for who in ('A','B'):
            if who not in report.get('hand_goals',{}):continue
            arm=cpu.arms['left'];joints=report['joints'][who];fk=arm.fk([joints[n] for n in arm.names],joints['lift_joint'])
            root=np.array(report['hands'][who]['left'])@np.linalg.inv(fk)
            yaw=Rotation.from_matrix(root[:3,:3]).as_euler('xyz')[2]
            base=np.r_[root[:3,3],yaw]
            cases.append((f+':'+who,{s:np.array(t) for s,t in report['hand_goals'][who].items()},base,joints,dict(attempts=1,joint_window=.15)))
    gpu.solve(cases[0][1],cases[0][2],cases[0][3],**cases[0][4])
    rows=[]
    for name,target,base,seed,kw in cases:
        row={'case':name}
        for backend,solver in [('cpu',cpu),('cuda',gpu)]:
            t=time.perf_counter();r=solver.solve(target,base,seed,**kw)
            row[backend]=dict(milliseconds=(time.perf_counter()-t)*1000,success=bool(r.success),position_error=r.position_error,angle_error=r.angle_error)
        rows.append(row);print(json.dumps(row),flush=True)
    summary={b:dict(successes=sum(r[b]['success'] for r in rows),median_ms=float(np.median([r[b]['milliseconds'] for r in rows]))) for b in ('cpu','cuda')}
    (OUTPUTS/'relay_ik_benchmark.json').write_text(json.dumps(dict(summary=summary,cases=rows),indent=2))
    print('SUMMARY',summary)
if __name__=='__main__':main()
