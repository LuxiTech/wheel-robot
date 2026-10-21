"""Diagnostic overlays. All projections use the saved optical camera frame."""
import cv2
import numpy as np


def project_points(points, camera_world, k, shape):
    points=np.asarray(points,dtype=float).reshape(-1,3)
    cam=(np.linalg.inv(camera_world) @ np.c_[points,np.ones(len(points))].T).T[:,:3]
    uvh=(np.asarray(k).reshape(3,3) @ cam.T).T
    out=[]
    for p,q in zip(cam,uvh):
        if not np.all(np.isfinite(p)) or p[2]<=.001:
            out.append(dict(pixel=None,visible=False,reason='behind_camera'));continue
        uv=q[:2]/q[2]
        inside=bool(0<=uv[0]<shape[1] and 0<=uv[1]<shape[0])
        out.append(dict(pixel=uv.tolist(),visible=inside,reason='in_image' if inside else 'outside_image'))
    return out


def draw_targets(image, targets, camera_world, k):
    """Projected desired TCPs, not measured finger contacts or visibility tests."""
    out=image.copy();metadata={}
    for side,t in targets.items():
        t=np.asarray(t);p=project_points([t[:3,3]],camera_world,k,out.shape)[0];metadata[side]=p
        color=(255,180,0) if side=='left' else (0,170,255)
        if p['visible']:
            xy=tuple(np.rint(p['pixel']).astype(int));cv2.drawMarker(out,xy,color,cv2.MARKER_CROSS,18,2)
            cv2.putText(out,side+' TCP', (max(0,xy[0]-35),max(16,xy[1]-12)),cv2.FONT_HERSHEY_SIMPLEX,.45,color,1)
        else:
            cv2.putText(out,side+': '+p['reason'],(8,40+20*len(metadata)),cv2.FONT_HERSHEY_SIMPLEX,.45,color,1)
    cv2.putText(out,'Desired TCP projection (occlusion not tested)',(8,18),cv2.FONT_HERSHEY_SIMPLEX,.43,(255,255,255),1)
    return out,metadata


def draw_detections(image,detections,roi=None):
    out=image.copy()
    for d in detections:
        x1,y1,x2,y2=np.rint(d['box']).astype(int)
        cv2.rectangle(out,(x1,y1),(x2,y2),(0,220,255),2)
        cv2.putText(out,'box %.3f'%d.get('score',0),(max(0,x1),max(15,y1)),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,220,255),1)
    if roi is not None:
        x1,y1,x2,y2=np.rint(roi).astype(int);cv2.rectangle(out,(x1,y1),(x2,y2),(255,80,255),1)
    return out


def draw_mask(image,mask):
    out=image.copy();out[mask]=(out[mask]*.55+np.array([40,220,40])*.45).astype(np.uint8)
    return out


def draw_pose(image,pose,k,size):
    out=image.copy();sx,sy,sz=size
    corners=np.array([[x*sx/2,y*sy/2,z*sz] for z in (0,1) for y in (-1,1) for x in (-1,1)])
    points=(np.asarray(pose) @ np.c_[corners,np.ones(8)].T).T[:,:3]
    projected=project_points(points,np.eye(4),k,out.shape)
    for a,b in ((0,1),(0,2),(1,3),(2,3),(4,5),(4,6),(5,7),(6,7),(0,4),(1,5),(2,6),(3,7)):
        pa,pb=projected[a],projected[b]
        if pa['pixel'] is not None and pb['pixel'] is not None:
            p,q=[tuple(np.clip(np.rint(v['pixel']),-100000,100000).astype(int)) for v in (pa,pb)]
            ok,p,q=cv2.clipLine((0,0,out.shape[1],out.shape[0]),p,q)
            if ok:cv2.line(out,p,q,(255,200,50),2)
    return out
