from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from wheel_grasp.vision.pipeline import refine_surface_pose,surface_quality


def test_refinement_reduces_depth_model_error_without_task_prior():
    k=np.array([[100.,0,40],[0,100.,40],[0,0,1]])
    yy,xx=np.indices((80,80));rays=np.stack([xx,yy,np.ones_like(xx)],axis=-1)@np.linalg.inv(k).T
    truth=np.eye(4);truth[:3,3]=[.02,.01,1.]
    truth[:3,:3]=Rotation.from_euler('xyz',[.3,.5,.2]).as_matrix()
    origin=-truth[:3,3]@truth[:3,:3];directions=rays@truth[:3,:3]
    half=np.array([.2,.15,.11])
    a=(-half-origin)/directions;b=(half-origin)/directions
    near=np.minimum(a,b).max(-1);far=np.maximum(a,b).min(-1)
    mask=(near>0)&(near<far);depth=np.where(mask,near,0)
    inp=SimpleNamespace(k=k,depth=depth)
    colliders=[dict(position=[0,0,0],size=(2*half).tolist())]
    guess=truth.copy();guess[:3,3]+=[.008,-.007,.006]
    guess[:3,:3]=Rotation.from_rotvec([.02,-.02,.015]).as_matrix()@guess[:3,:3]
    before=surface_quality(inp,mask,guess,colliders)['surface_rms_m']
    result,details=refine_surface_pose(inp,mask,guess,colliders)
    after=surface_quality(inp,mask,result,colliders)['surface_rms_m']
    assert after<before*.1 and after<.001
    assert np.linalg.norm(result[:3,3]-truth[:3,3])<.002


def test_quality_uses_whole_mask_instead_of_overweighting_boundary_sample():
    depth=np.ones((100,100));depth[0,0]=1.2
    inp=SimpleNamespace(k=np.array([[100.,0,50],[0,100.,50],[0,0,1]]),depth=depth)
    result=surface_quality(inp,np.ones_like(depth,dtype=bool),np.eye(4),
        [dict(position=[0,0,1.001],size=[20,20,.002])])
    assert result['points']==10000
    assert abs(result['surface_rms_m']-.198/100)<1e-9
    assert result['accepted']
