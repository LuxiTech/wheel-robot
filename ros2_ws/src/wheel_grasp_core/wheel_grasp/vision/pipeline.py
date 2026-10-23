"""Observation -> selected instance -> pose -> URDF transform -> pair targets."""
import time
from contextlib import nullcontext
import numpy as np
from scipy.spatial.transform import Rotation
from wheel_grasp.vision.geometry import prepare_input, mask_quality, rim_targets
from wheel_grasp.motion.kinematics import pose

CAMERA_FRAMES={
 'head_rgb':'head_d455_color_optical_frame',
 'left_wrist_rgb':'left_wrist_eb435i_color_optical_frame',
 'right_wrist_rgb':'right_wrist_eb435i_color_optical_frame',
}


def surface_quality(model_input,mask,camera_pose,colliders,*,max_rms=.006):
    """Measured visible points to known crate surfaces; never uses scene truth."""
    yy,xx=np.where(mask & np.isfinite(model_input.depth) & (model_input.depth>0))
    if len(xx)<80: return dict(accepted=False,reason='insufficient_depth')
    rays=np.stack([xx,yy,np.ones(len(xx))],axis=1)@np.linalg.inv(model_input.k).T
    points=rays*model_input.depth[yy,xx,None]
    local=(points-camera_pose[:3,3])@camera_pose[:3,:3]
    if not colliders: raise ValueError('Known geometry required for quality check')
    # Validate every mask point. Linear subsampling overweights the first/last
    # boundary pixels and can turn a single background pixel into a false fault.
    distances=np.full(len(points),np.inf)
    for item in colliders:
        d=np.abs(local-np.asarray(item['position']))-np.asarray(item['size'])/2
        signed=np.linalg.norm(np.maximum(d,0),axis=1)+np.minimum(np.max(d,axis=1),0)
        distances=np.minimum(distances,np.abs(signed))
    d=distances
    rms=float(np.sqrt(np.mean(d*d)))
    return dict(accepted=rms<=max_rms,surface_rms_m=rms,points=len(points),
                surface_p95_m=float(np.quantile(d,.95)),sampling='all_valid_mask_points')


class GraspPipeline:
    def __init__(self,robot,segmenter,estimator,rules,colliders,*,base='base_footprint',trace=None):
        self.trace=trace
        self.robot=robot;self.segmenter=segmenter;self.estimator=estimator
        self.rules=rules;self.colliders=colliders;self.base=base
        self.last=None;self.last_key=None;self.target=None;self.last_time=None

    def stage(self, name):
        return self.trace.stage(name) if self.trace else nullcontext()

    def emit(self, name, **data):
        if self.trace: self.trace.record(name, **data)

    def process(self,observation,joints,*,target_id,box=None,track=False):
        with self.stage('preprocess'):
            if observation.key==self.last_key: raise ValueError('Duplicate pair')
            inp=prepare_input(observation)
            if not target_id: raise ValueError('Explicit target identity required')
            self.emit('preprocess', inp=inp)
        with self.stage('detection'):
            candidates=self.segmenter.detect(inp.rgb) if box is None else []
            self.emit('detection', inp=inp, candidates=candidates, box=box)
            if box is None:
                if len(candidates)!=1:
                    raise ValueError('Select an explicit box when detection is empty or ambiguous; candidates='+str(len(candidates)))
                box=candidates[0]['box']
            box=np.asarray(box,float)
            h,w=inp.depth.shape
            if (box.shape!=(4,) or not np.isfinite(box).all() or
                    not (0<=box[0]<box[2]<=w and 0<=box[1]<box[3]<=h)):
                raise ValueError('Invalid target bounding box')
        with self.stage('segmentation'):
            mask=self.segmenter.segment(inp.rgb,box)
            quality=mask_quality(mask,inp.depth)
            self.emit('segmentation', inp=inp, mask=mask, quality=quality)
            if not quality['accepted']:
                self.target=None;raise ValueError('Target depth quality rejected: '+str(quality))
        identity=(observation.key[0],target_id)
        can_track=(track and identity==self.target and self.last is not None
                   and self.last_time is not None and time.monotonic()-self.last_time<.5)
        try:
            with self.stage('pose'):
                t=pose(self.estimator.track(inp,identity) if can_track else self.estimator.register(inp,mask,identity))
                self.emit('pose', inp=inp, pose=t, colliders=self.colliders, frame_id=inp.frame)
            with self.stage('pose_quality'):
                check=surface_quality(inp,mask,t,self.colliders)
                self.emit('pose_quality', quality=check)
                if not check['accepted']: raise ValueError('Estimated pose fails surface check: '+str(check))
                if can_track and (np.linalg.norm(t[:3,3]-self.last[:3,3])>.05 or
                        Rotation.from_matrix(t[:3,:3]@self.last[:3,:3].T).magnitude()>.25):
                    raise ValueError('Pose jump; re-register target')
        except Exception:
            self.target=None;self.last=None;raise
        with self.stage('transform'):
            camera=self.robot.transform(CAMERA_FRAMES[observation.key[0]],joints,self.base)
            box_base=camera@t
            self.emit('transform', camera=camera, frame_id=self.base)
        with self.stage('keypoints'):
            grasp,pre=rim_targets(box_base,**self.rules)
            self.emit('keypoints', inp=inp, grasp=grasp, pregrasp=pre, camera=camera, frame_id=self.base)
        self.last=t;self.target=identity;self.last_key=observation.key;self.last_time=time.monotonic()
        return dict(target_id=target_id,pair_id=observation.key[1],camera_id=observation.key[0],
                    frame_id=self.base,pose=box_base,grasp=grasp,pregrasp=pre,
                    mask=mask,quality={**quality,**check},diagnostic_only=inp.diagnostic_only,
                    reasons=inp.reasons,executable=False)


def refine_surface_pose(model_input,mask,initial,colliders):
    """Bounded local point-to-model refinement of a model prediction, not a task prior."""
    from scipy.optimize import least_squares
    initial=pose(initial)
    yy,xx=np.where(mask & np.isfinite(model_input.depth) & (model_input.depth>0))
    if len(xx)<80:raise ValueError('Insufficient measured depth for refinement')
    ix=np.linspace(0,len(xx)-1,min(600,len(xx)),dtype=int);xx=xx[ix];yy=yy[ix]
    points=(np.stack([xx,yy,np.ones(len(xx))],axis=1)@np.linalg.inv(model_input.k).T)*model_input.depth[yy,xx,None]
    centers=np.asarray([c['position'] for c in colliders]);half=np.asarray([c['size'] for c in colliders])/2
    def residual(x):
        r=Rotation.from_rotvec(x[3:]).as_matrix()@initial[:3,:3]
        local=(points-initial[:3,3]-x[:3])@r
        d=np.abs(local[:,None,:]-centers[None,:,:])-half[None,:,:]
        signed=np.linalg.norm(np.maximum(d,0),axis=2)+np.minimum(np.max(d,axis=2),0)
        return np.r_[np.min(np.abs(signed),axis=1),.001*x]
    bounds=np.array([.04]*3+[.2]*3)
    fit=least_squares(residual,np.zeros(6),bounds=(-bounds,bounds),loss='soft_l1',f_scale=.003,max_nfev=60)
    if np.any(np.abs(fit.x)>.98*bounds):raise ValueError('Local refinement exhausted correction bounds')
    t=initial.copy();t[:3,3]+=fit.x[:3];t[:3,:3]=Rotation.from_rotvec(fit.x[3:]).as_matrix()@initial[:3,:3]
    return t,dict(translation_correction_m=fit.x[:3].tolist(),rotation_correction_rad=fit.x[3:].tolist(),iterations=fit.nfev)
