"""Shared validation for rigid transforms, independent of ROS and SDK."""
import numpy as np


def pose(value):
    t=np.asarray(value,dtype=float)
    if (t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1])
            or not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-6)
            or not np.isclose(np.linalg.det(t[:3,:3]),1,atol=1e-6)):
        raise ValueError('Expected finite rigid transform')
    return t

