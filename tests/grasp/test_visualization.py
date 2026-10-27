import json
import sys
from pathlib import Path
from types import SimpleNamespace
import cv2
import numpy as np
from wheel_perception_core.visualization import project_points,draw_targets


def test_projection_extrinsics_and_invalid_depth():
    camera=np.eye(4);camera[:3,3]=[1,2,3]
    k=np.array([[100,0,50],[0,100,40],[0,0,1]])
    p=project_points([[1,2,5],[2,2,5],[1,2,2]],camera,k,(80,100))
    assert p[0]['pixel']==[50,40] and p[0]['visible']
    assert p[1]['pixel']==[100,40] and not p[1]['visible']
    assert p[2]['pixel'] is None and p[2]['reason']=='behind_camera'
