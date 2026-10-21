"""Continuous pair trajectories. Collision validation is an explicit dependency."""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .kinematics import pose


@dataclass
class Trajectory:
    points: list
    collision_checked: bool
    executable: bool
    reason: str


def plan_pair(ik,targets,measured,*,lift_bounds=None,fixed_lift=None,steps=20,
              speed_limits=None,collision_check=None):
    if steps<2: raise ValueError('At least two trajectory segments required')
    starts=ik.fk(measured);targets={s:pose(targets[s]) for s in starts}
    limits={n:ik.model.joints[n]['velocity'] for n in ik.names}
    for n,v in (speed_limits or {}).items():
        if n not in limits or not np.isfinite(v) or v<=0: raise ValueError('Invalid speed limit')
        limits[n]=min(limits[n],v)
    if any(not np.isfinite(v) or v<=0 for v in limits.values()): raise ValueError('Missing velocity limits')
    interpolators={s:Slerp([0,1],Rotation.from_matrix([starts[s][:3,:3],targets[s][:3,:3]])) for s in starts}
    previous=dict(measured);elapsed=0.;points=[dict(time=0.,joints={n:float(measured[n]) for n in ik.names})]
    if collision_check is not None and not collision_check(previous): raise ValueError('Initial configuration in collision')
    for u in np.linspace(0,1,steps+1)[1:]:
        goal={}
        for side in starts:
            t=np.eye(4);t[:3,3]=(1-u)*starts[side][:3,3]+u*targets[side][:3,3]
            t[:3,:3]=interpolators[side]([u]).as_matrix()[0];goal[side]=t
        sol=ik.solve(goal,previous,lift_bounds=lift_bounds,fixed_lift=fixed_lift)
        if not sol.success:
            raise ValueError('Pair IK path infeasible at fraction %.4f: position_errors_m=%s rotation_errors_rad=%s' %
                             (u,sol.position_errors,sol.rotation_errors))
        delta={n:abs(sol.joints[n]-previous[n]) for n in ik.names}
        if any(delta[n]>.3 for n in ik.names if n!='lift_joint'):
            raise ValueError('IK branch discontinuity at fraction %.4f: %s' % (u,delta))
        # Check interior configurations as well as segment endpoints.
        if collision_check is not None:
            samples=max(2,int(max(delta.values())/.01)+1)
            for v in np.linspace(0,1,samples)[1:]:
                q={n:previous[n]+v*(sol.joints[n]-previous[n]) for n in ik.names}
                if not collision_check(q): raise ValueError('Trajectory collision')
        # Quintic segment scaling maximum derivative = 1.875.
        elapsed+=max(.05,max(1.875*delta[n]/limits[n] for n in ik.names))
        points.append(dict(time=elapsed,joints={n:float(sol.joints[n]) for n in ik.names}))
        previous=sol.joints
    checked=collision_check is not None
    # This is a kinematic plan, not authorization to send it to hardware.
    return Trajectory(points,checked,False,'hardware_trajectory_bridge_not_implemented' if checked else 'collision_check_required')


def sample_trajectory(trajectory,t):
    if not np.isfinite(t) or t<0: raise ValueError('Invalid trajectory time')
    pts=trajectory.points
    if t>=pts[-1]['time']: return pts[-1]['joints'].copy()
    for a,b in zip(pts,pts[1:]):
        if a['time']<=t<b['time']:
            u=(t-a['time'])/(b['time']-a['time']);u=u**3*(10-15*u+6*u*u)
            return {n:a['joints'][n]+u*(b['joints'][n]-a['joints'][n]) for n in a['joints']}
    raise ValueError('Malformed trajectory')
