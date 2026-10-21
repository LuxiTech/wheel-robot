import numpy as np
import pytest
from wheel_grasp.association import projected_box,associate


def test_task_roi_requires_visible_geometry():
    camera=np.eye(4);p=np.eye(4);p[2,3]=2
    k=np.array([[320,0,320],[0,320,240],[0,0,1]])
    roi=projected_box(p,camera,k,[.4,.3,.22],(480,640))
    assert np.allclose(roi,[288,216,352,264])
    p[2,3]=-1
    with pytest.raises(ValueError,match='behind'):projected_box(p,camera,k,[.4,.3,.22],(480,640))


def test_detection_association_cannot_return_unobserved_layout_pose():
    roi=np.array([100,100,180,180])
    with pytest.raises(ValueError,match='No RGB detection'):associate([],roi)
    with pytest.raises(ValueError,match='No RGB detection'):associate([dict(box=[200,200,280,280],score=.9)],roi)
    prompt,details=associate([dict(box=[20,20,300,300],score=.9)],roi)
    assert np.array_equal(prompt,roi)
    assert details['method']=='task_layout_roi_with_rgb_detection'


def test_instance_detection_wins_over_whole_stack():
    roi=np.array([100,100,180,180])
    near=dict(box=[101,100,181,180],score=.6)
    prompt,details=associate([dict(box=[20,20,300,300],score=.9),near],roi)
    assert np.array_equal(prompt,near['box'])
