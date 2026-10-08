"""Bounded joint optimization of a planar base, lift and two seven-axis arms.

This solves kinematics, not dynamics or force control. Callers must check
collisions and contacts and execute on a common time base with feedback.
"""
from dataclasses import dataclass
import time
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from arm_ik import ArmIK


def transform(position, quaternion):
    t = np.eye(4)
    t[:3, 3] = position
    q = np.asarray(quaternion)
    t[:3, :3] = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    return t


@dataclass
class WholeBodyResult:
    base: np.ndarray
    joints: dict
    position_error: float
    angle_error: float
    success: bool


class WholeBodyIK:
    def __init__(self, urdf, tcp=(0, 0, -.202), *, backend='cpu', device='cuda:0'):
        self.arms = {s: ArmIK(urdf, s, tcp=tcp) for s in ('left', 'right')}
        self.names = ['lift_joint'] + [n for arm in self.arms.values() for n in arm.names]
        self.lower = np.r_[0., self.arms['left'].lower, self.arms['right'].lower]
        self.upper = np.r_[.4, self.arms['left'].upper, self.arms['right'].upper]
        if backend not in ('cpu','cuda'):
            raise ValueError('Unknown IK backend: '+backend)
        self.backend=backend
        self.stats=dict(backend=backend,device=device if backend=='cuda' else 'cpu',calls=0,seconds=0.,failures=0)
        self.gpu=None
        if backend=='cuda':
            from gpu_whole_body_ik import CudaIK
            self.gpu=CudaIK(self.arms,device)

    def solve(self, targets, base, joints, *, base_window=0., lift=None, attempts=3, joint_window=None, fixed_joints=None):
        base = np.asarray(base, dtype=float)  # x,y,z,yaw
        seed = np.array([joints[n] for n in self.names])
        if (not targets or not set(targets)<=set(self.arms) or base.shape!=(4,) or
                not np.isfinite(base).all() or not np.isfinite(seed).all() or attempts<1 or
                any(np.shape(t)!=(4,4) or not np.isfinite(t).all() for t in targets.values())):
            raise ValueError('Invalid IK target, seed or base')
        x0 = np.r_[base[[0, 1, 3]], np.clip(seed, self.lower+1e-8, self.upper-1e-8)]
        window = max(float(base_window), 1e-9)
        low = np.r_[x0[:3]-[window, window, window], self.lower]
        high = np.r_[x0[:3]+[window, window, window], self.upper]
        if joint_window is not None:
            low[4:]=np.maximum(low[4:],x0[4:]-joint_window)
            high[4:]=np.minimum(high[4:],x0[4:]+joint_window)
        if lift is not None:
            if not 0 <= lift <= .4:
                raise ValueError('Lift target exceeds travel')
            low[3], high[3] = max(0, lift-1e-9), min(.4, lift+1e-9)
        for name,value in (fixed_joints or {}).items():
            i=3+self.names.index(name)
            low[i],high[i]=max(low[i],value-1e-9),min(high[i],value+1e-9)
        for i,side in enumerate(self.arms):
            if side not in targets:
                low[4+7*i:11+7*i]=x0[4+7*i:11+7*i]-1e-9
                high[4+7*i:11+7*i]=x0[4+7*i:11+7*i]+1e-9
        x0 = np.clip(x0, low+1e-12, high-1e-12)
        # Eliminate fixed coordinates rather than numerically differentiating
        # near-zero bounds. This is particularly important while one arm holds.
        free=np.flatnonzero(high-low>1e-7)

        def poses(x):
            root = np.eye(4)
            root[:3, 3] = [x[0], x[1], base[2]]
            root[:3, :3] = Rotation.from_euler('z', x[2]).as_matrix()
            return {s: root @ arm.fk(x[4+7*i:11+7*i], x[3])
                    for i, (s, arm) in enumerate(self.arms.items()) if s in targets}

        def residual(x):
            result = []
            for side, actual in poses(x).items():
                desired = targets[side]
                result.extend(actual[:3, 3]-desired[:3, 3])
                result.extend(.20*Rotation.from_matrix(desired[:3, :3] @ actual[:3, :3].T).as_rotvec())
            result.extend(.002*(x[4:]-x0[4:]))
            result.extend(.002*(x[:3]-x0[:3]))
            result.append(.002*(x[3]-x0[3]))
            return result

        if self.gpu is not None:
            started=time.perf_counter()
            optimum=self.gpu.solve(targets,base,x0,low,high,attempts)
            actual=poses(optimum)
            pe=max(np.linalg.norm(t[:3,3]-targets[s][:3,3]) for s,t in actual.items())
            ae=max(np.linalg.norm(Rotation.from_matrix(targets[s][:3,:3] @ t[:3,:3].T).as_rotvec()) for s,t in actual.items())
            # Independent CPU FK validation only; never CPU optimization fallback.
            valid=bool(np.isfinite(optimum).all() and np.all(optimum>=low-1e-6) and np.all(optimum<=high+1e-6))
            success=bool(valid and pe<.008 and ae<.08)
            self.stats['calls']+=1
            self.stats['seconds']+=time.perf_counter()-started
            self.stats['failures']+=int(not success)
            self.stats['cache_hits']=self.gpu.cache_hits
            self.stats['gpu_launches']=self.gpu.calls
            return WholeBodyResult(optimum[:3],dict(zip(self.names,map(float,optimum[3:]))),float(pe),float(ae),success)

        best = None
        rng = np.random.default_rng(42)
        for attempt in range(attempts):
            start = x0.copy()
            if attempt:
                start[4:] = rng.uniform(low[4:], high[4:])
            def unpack(values):
                x=x0.copy()
                x[free]=values
                return x
            if len(free):
                fit = least_squares(lambda values:residual(unpack(values)),start[free],
                                    bounds=(low[free],high[free]),max_nfev=150,
                                    ftol=1e-8,xtol=1e-8,gtol=1e-8)
                optimum=unpack(fit.x)
            else:
                optimum=x0.copy()
            actual = poses(optimum)
            pe = max(np.linalg.norm(t[:3, 3]-targets[s][:3, 3]) for s,t in actual.items())
            ae = max(np.linalg.norm(Rotation.from_matrix(targets[s][:3, :3] @ t[:3, :3].T).as_rotvec()) for s,t in actual.items())
            ans = WholeBodyResult(optimum[:3], dict(zip(self.names, map(float, optimum[3:]))),
                                  float(pe), float(ae), pe < .008 and ae < .08)
            if best is None or pe+.2*ae < best.position_error+.2*best.angle_error:
                best = ans
            if ans.success:
                return ans
        return best


def minimum_duration(start, end, limits, minimum=0.1):
    """Quintic time scaling has maximum derivative 1.875."""
    return max([minimum] + [1.875*abs(end[n]-start[n])/limits[n] for n in end])


def smoothstep(t):
    t = np.clip(t, 0., 1.)
    return float(t*t*t*(10+t*(-15+6*t)))
