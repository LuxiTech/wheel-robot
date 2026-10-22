"""Task-layout ROI association, never simulator instance labels or live object poses."""
import itertools
import numpy as np
from wheel_grasp.motion.kinematics import pose


def projected_box(prior_world,camera_world,k,size,image_shape):
    t=np.linalg.inv(pose(camera_world))@pose(prior_world)
    w,d,h=size
    corners=np.array(list(itertools.product([-w/2,w/2],[-d/2,d/2],[0,h])))
    xyz=corners@t[:3,:3].T+t[:3,3]
    if np.min(xyz[:,2])<=.03:raise ValueError('Task ROI behind/too close to camera')
    uv=xyz@np.asarray(k).T;uv=uv[:,:2]/uv[:,2,None]
    lo=uv.min(0);hi=uv.max(0);height,width=image_shape
    lo=np.maximum(lo,[0,0]);hi=np.minimum(hi,[width,height])
    if np.any(hi-lo<12):raise ValueError('Task ROI outside fixed URDF camera view')
    return np.r_[lo,hi]


def associate(detections,roi):
    roi=np.asarray(roi);area=np.prod(roi[2:]-roi[:2]);scores=[]
    for d in detections:
        b=np.asarray(d['box'],float)
        inter=np.prod(np.maximum(0,np.minimum(b[2:],roi[2:])-np.maximum(b[:2],roi[:2])))
        coverage=inter/area
        if coverage<.35:continue
        union=area+np.prod(b[2:]-b[:2])-inter
        scores.append((float(inter/max(union,1)),d,coverage))
    if not scores:raise ValueError('No RGB detection supports the requested task ROI')
    scores.sort(key=lambda x:x[0],reverse=True)
    candidate=np.asarray(scores[0][1]['box'],float)
    # An open-vocabulary box may cover the entire stack. Use the independently
    # taught slot ROI as a SAM prompt, never as the measured output pose.
    prompt=candidate if scores[0][0]>.55 and np.prod(candidate[2:]-candidate[:2])<area*1.5 else roi
    return prompt,dict(method='task_layout_roi_with_rgb_detection',roi=roi.tolist(),
                       detection=scores[0][1],iou=scores[0][0],prompt=prompt.tolist())
