"""Simulated calibrated depth, ideal instance detection and geometric 6D fitting.

Instance labels come from ray-hit labels, not a trained RGB detector. The pose
fitter receives measured points and the task's nominal pose, never a crate's
simulator transform. Sensor extrinsics use simulated robot localization.
"""
from dataclasses import dataclass
import json
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from common import ROOT
from relay_task import RelayFault


@dataclass
class PoseEstimate:
    pose: np.ndarray
    residual_m: float
    points: int
    bbox: tuple
    source: str = 'ideal_instance_depth_geometric_6d'


class CratePoseEstimator:
    def __init__(self, colliders):
        self.centers=np.array([p['position'] for p in colliders])
        self.half=np.array([p['size'] for p in colliders])/2

    def distances(self, points):
        d=np.abs(points[:,None,:]-self.centers[None,:,:])-self.half[None,:,:]
        signed=np.linalg.norm(np.maximum(d,0),axis=2)+np.minimum(np.max(d,axis=2),0)
        return np.min(np.abs(signed),axis=1)

    def fit(self, points, prior, bbox=()):
        points=np.asarray(points,dtype=float)
        if points.ndim!=2 or points.shape[1]!=3 or len(points)<40 or not np.isfinite(points).all():
            raise RelayFault('Insufficient or invalid depth points for crate pose')
        if len(points)>600:
            points=points[np.linspace(0,len(points)-1,600,dtype=int)]
        prior=np.asarray(prior,dtype=float)
        if prior.shape!=(4,4) or not np.isfinite(prior).all():
            raise RelayFault('Invalid pose prior')
        def residual(x):
            r=Rotation.from_rotvec(x[3:]).as_matrix() @ prior[:3,:3]
            local=(points-(prior[:3,3]+x[:3])) @ r
            # Small prior resolves ambiguous views without fabricating confidence.
            return np.r_[self.distances(local),.001*x]
        fit=least_squares(residual,np.zeros(6),bounds=([-0.12]*3+[-.4]*3,[.12]*3+[.4]*3),
                          loss='soft_l1',f_scale=.003,max_nfev=100)
        t=prior.copy()
        t[:3,3]+=fit.x[:3]
        t[:3,:3]=Rotation.from_rotvec(fit.x[3:]).as_matrix() @ prior[:3,:3]
        error=float(np.sqrt(np.mean(residual(fit.x)[:-6]**2)))
        # Degenerate views cannot constrain all six pose dimensions.
        singular=np.linalg.svd(fit.jac[:-6],compute_uv=False)
        if error>.006 or singular[-1]<.03 or np.any(np.abs(fit.x[:3])>.115):
            raise RelayFault(f'Unreliable 6D fit: residual={error:.4f}m, conditioning={singular[-1]:.4f}')
        return PoseEstimate(t,error,len(points),tuple(bbox))


class SimDepthPerception:
    def __init__(self, runtime):
        import omni.physx
        self.rt=runtime
        self.query=omni.physx.get_physx_scene_query_interface()
        self.estimator=CratePoseEstimator(json.loads((ROOT/'assets/generated/relay/crate_colliders.json').read_text()))
        self.priors={i:np.eye(4) for i in range(len(runtime.initial))}
        for i,spec in enumerate(runtime.initial):
            self.priors[i][:3,3]=spec['position']
            self.priors[i][:3,:3]=Rotation.from_euler('z',spec.get('yaw',0.)).as_matrix()
        self.last={}

    def set_prior(self,index,pose):
        self.priors[index]=np.asarray(pose).copy()

    def observe(self,index):
        prior=self.priors[index]
        who='B' if self.rt.inventory.owners.get(index) in ('R','B','TARGET') else 'A'
        base=self.rt.base(who)
        # A synthetic RGB-D camera mounted above the base, viewing a known task ROI.
        # Aiming the ROI is a simulated pan/tilt sensor, not the existing fixed D455.
        origin=base[:3]+[0,0,1.25]
        look=prior[:3,3]+[0,0,self.rt.c['crate']['size'][2]/2]
        forward=look-origin
        forward/=np.linalg.norm(forward)
        right=np.cross(forward,[0,0,1.])
        right/=np.linalg.norm(right)
        up=np.cross(right,forward)
        points=[]
        pixels=[]
        path=self.rt.initial[index]['path']+'/'
        for v in range(49):
            for u in range(65):
                direction=forward+right*((u-32)/64*.9)+up*((24-v)/48*.7)
                direction/=np.linalg.norm(direction)
                hit=self.query.raycast_closest(tuple(map(float,origin)),tuple(map(float,direction)),3.)
                if hit['hit'] and str(hit['collision']).startswith(path):
                    points.append(origin+direction*float(hit['distance']))
                    pixels.append((u,v))
        if not pixels:
            raise RelayFault('Target not visible in simulated depth camera')
        uv=np.array(pixels)
        estimate=self.estimator.fit(points,prior,(*uv.min(axis=0),*uv.max(axis=0)))
        self.priors[index]=estimate.pose.copy()
        self.last[index]=dict(residual_m=estimate.residual_m,points=estimate.points,
                              bbox=list(map(int,estimate.bbox)),source=estimate.source)
        return estimate.pose
