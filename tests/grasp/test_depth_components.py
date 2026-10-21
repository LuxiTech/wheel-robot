import numpy as np
import pytest
from wheel_grasp.geometry import connected_depth_mask


def test_depth_discontinuity_separates_background_without_moving_points():
    depth=np.ones((50,50));depth[:,40:]=2.
    mask=np.ones_like(depth,dtype=bool);k=np.diag([100.,100.,1.])
    clean,details=connected_depth_mask(mask,depth,k)
    assert clean[:,:40].all() and not clean[:,40:].any()
    assert details['retained_points']==2000 and details['components']==2
    assert np.all(depth[:,40:]==2.)


def test_no_dominant_depth_object_is_rejected():
    depth=np.repeat(np.array([1.,2.,3.]),20)[None,:].repeat(40,axis=0)
    with pytest.raises(ValueError,match='no dominant'):
        connected_depth_mask(np.ones_like(depth,dtype=bool),depth,np.diag([200.,200.,1.]))
