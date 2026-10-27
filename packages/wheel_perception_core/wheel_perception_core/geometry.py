"""Model input geometry and deterministic, object-relative grasp rules."""
from dataclasses import dataclass
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from wheel_task_core.transforms import pose


@dataclass
class ModelInput:
    rgb: np.ndarray
    depth: np.ndarray
    k: np.ndarray
    frame: str
    diagnostic_only: bool
    reasons: list


def prepare_input(obs):
    m=obs.metadata;i=m['intrinsics'];k=np.asarray(i['K'],float).reshape(3,3)
    d=np.asarray(i['D'],float);kind=i['distortion_model']
    rgb=obs.rgb.copy();depth=obs.depth.copy()
    depth[~np.isfinite(depth)|(depth<=0)]=0 # FoundationPose invalid-depth convention.
    h,w=depth.shape
    if np.any(d!=0):
        if len(d)!=5: raise ValueError('Expected five Brown coefficients')
        if kind=='plumb_bob':
            mx,my=cv2.initUndistortRectifyMap(k,d,None,k,(w,h),cv2.CV_32FC1)
        elif kind=='realsense_2':
            # Inverse Brown: undistorted = Brown(distorted). Invert that
            # polynomial to sample distorted input at an undistorted output ray.
            yy,xx=np.indices((h,w),dtype=np.float32)
            pixels=np.stack([xx,yy],axis=-1).reshape(-1,1,2)
            normalized=(pixels.reshape(-1,2)-[k[0,2],k[1,2]])/[k[0,0],k[1,1]]
            x,y=normalized.T.copy();tx,ty=normalized.T
            k1,k2,p1,p2,k3=d
            for _ in range(30):
                r2=x*x+y*y;rad=1+k1*r2+k2*r2*r2+k3*r2*r2*r2
                fx=x*rad+2*p1*x*y+p2*(r2+2*x*x)-tx
                fy=y*rad+p1*(r2+2*y*y)+2*p2*x*y-ty
                if max(np.max(np.abs(fx)),np.max(np.abs(fy)))<1e-10: break
                slope=k1+2*k2*r2+3*k3*r2*r2
                a=rad+2*x*x*slope+2*p1*y+6*p2*x
                b=2*x*y*slope+2*p1*x+2*p2*y
                c=rad+2*y*y*slope+6*p1*y+2*p2*x
                det=a*c-b*b
                if np.any(np.abs(det)<1e-12): raise ValueError('Singular inverse Brown map')
                dx=(c*fx-b*fy)/det;dy=(-b*fx+a*fy)/det
                x-=dx;y-=dy
            else: raise ValueError('Inverse Brown map did not converge')
            mx=(x*k[0,0]+k[0,2]).reshape(h,w).astype(np.float32)
            my=(y*k[1,1]+k[1,2]).reshape(h,w).astype(np.float32)
        else: raise ValueError('Unsupported distortion: '+kind)
        rgb=cv2.remap(rgb,mx,my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
        depth=cv2.remap(depth,mx,my,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT)
    elif kind not in ('plumb_bob','realsense_2'):
        raise ValueError('Unsupported projection model')
    reasons=[]
    if m['depth'].get('value_semantics')!='color_optical_z':
        reasons.append('source_depth_z_requires_metric_validation')
    if kind=='realsense_2' and np.any(d):
        reasons.append('inverse_brown_requires_device_reference_validation')
    if m.get('replay') or m.get('mock'):
        reasons.append('replay_or_mock')
    if m.get('source_age_unverified'):
        reasons.append('source_age_unverified')
    return ModelInput(rgb,depth,k,m['frame_id'],bool(reasons),reasons)


def mask_quality(mask,depth,*,min_points=80,min_ratio=.15):
    mask=np.asarray(mask)
    if mask.dtype!=bool or mask.shape!=depth.shape: raise ValueError('Mask shape/type mismatch')
    valid=mask & np.isfinite(depth) & (depth>0)
    count=int(valid.sum());area=int(mask.sum());ratio=count/max(1,area)
    yy,xx=np.where(valid)
    spread=(int(np.ptp(xx)) if count else 0,int(np.ptp(yy)) if count else 0)
    accepted=count>=min_points and ratio>=min_ratio and min(spread)>=5
    return dict(accepted=bool(accepted),valid_points=count,valid_ratio=ratio,spread=spread)


def rim_targets(box_pose,*,width,height,inset,below_top,pitch_degrees=50,heading=0,clearance=.06,
                tool_convention='legacy'):
    box_pose=pose(box_pose)
    if not (0<inset<width/2 and 0<below_top<height and clearance>=0):
        raise ValueError('Invalid rim geometry')
    yaw=Rotation.from_matrix(box_pose[:3,:3]).as_euler('xyz')[2]
    facing=1 if np.sin(heading-yaw)>=0 else -1
    grasp={};pre={}
    if tool_convention not in ('legacy','pinch_y'):
        raise ValueError('Unsupported tool convention')
    for side,sign in [('left',1),('right',-1)]:
        t=np.eye(4)
        t[:3,3]=[-sign*facing*(width/2-inset),0,height-below_top]
        t[:3,:3]=(Rotation.from_euler('z',facing*np.pi/2).as_matrix()
            @Rotation.from_euler('z',-np.pi/2).as_matrix()
            @Rotation.from_euler('x',pitch_degrees,degrees=True).as_matrix()
            @Rotation.from_euler('z',0 if side=='left' else np.pi).as_matrix())
        if tool_convention == 'pinch_y':
            # The supplied pinch URDF opens/closes along tool Y. Align that
            # axis with the selected wall normal; approach in the wall plane.
            # Both wrists carry a camera on tool +X. Keep +X above the rim
            # on both hands: flipping Y on the right also flips X and sweeps
            # the camera housing downward. Pinch closure is symmetric in Y.
            y=np.array([-facing,0.,0.])
            toward_robot=box_pose[:3,:3].T@np.array([-np.cos(heading),-np.sin(heading),0.])
            direction=1 if toward_robot[1]>=0 else -1
            angle=np.deg2rad(pitch_degrees)
            z=np.array([0.,direction*np.sin(angle),np.cos(angle)])
            t[:3,:3]=np.column_stack((np.cross(y,z),y,z))
        grasp[side]=box_pose@t
        pre[side]=grasp[side].copy();pre[side][:3,3]+=pre[side][:3,2]*clearance
    return grasp,pre


def connected_depth_mask(mask,depth,k,*,max_gap=.025,min_fraction=.5):
    """Keep the dominant 3D-connected foreground, not background seen through gaps."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    valid=np.asarray(mask,bool)&np.isfinite(depth)&(depth>0)
    yy,xx=np.indices(depth.shape)
    xyz=(np.stack([xx,yy,np.ones_like(xx)],axis=-1)@np.linalg.inv(k).T)*depth[:,:,None]
    ids=np.full(depth.shape,-1,dtype=np.int32);ids[valid]=np.arange(int(valid.sum()))
    if valid.sum()<80:raise ValueError('Insufficient foreground for depth clustering')
    rows=[];cols=[]
    for a,b in ((np.s_[:,:-1],np.s_[:,1:]),(np.s_[:-1,:],np.s_[1:,:])):
        adjacent=valid[a]&valid[b]&(np.linalg.norm(xyz[a]-xyz[b],axis=-1)<max_gap)
        rows.append(ids[a][adjacent]);cols.append(ids[b][adjacent])
    row=np.concatenate(rows);col=np.concatenate(cols)
    graph=coo_matrix((np.ones(len(row),dtype=np.uint8),(row,col)),shape=(int(valid.sum()),)*2).tocsr()
    count,labels=connected_components(graph,directed=False)
    sizes=np.bincount(labels);largest=int(np.argmax(sizes));fraction=float(sizes[largest]/valid.sum())
    if sizes[largest]<80 or fraction<min_fraction:raise ValueError('RGB mask has no dominant connected depth object')
    clean=np.zeros_like(valid);clean[valid]=labels==largest
    return clean,dict(components=int(count),retained_fraction=fraction,max_neighbor_gap_m=max_gap,
                      input_points=int(valid.sum()),retained_points=int(clean.sum()))
