import json
import sys
from pathlib import Path
from types import SimpleNamespace
import cv2
import numpy as np
from wheel_grasp.visualization import project_points,draw_targets


def test_projection_extrinsics_and_invalid_depth():
    camera=np.eye(4);camera[:3,3]=[1,2,3]
    k=np.array([[100,0,50],[0,100,40],[0,0,1]])
    p=project_points([[1,2,5],[2,2,5],[1,2,2]],camera,k,(80,100))
    assert p[0]['pixel']==[50,40] and p[0]['visible']
    assert p[1]['pixel']==[100,40] and not p[1]['visible']
    assert p[2]['pixel'] is None and p[2]['reason']=='behind_camera'


def test_trace_preserves_each_target_and_source_frame(tmp_path):
    root=Path(__file__).resolve().parents[2]
    sys.path.insert(0,str(root/'scripts'));sys.path.insert(0,str(root/'tools/grasp'))
    from relay_rgbd_perception import ModelRGBDPerception
    folder=tmp_path/'frames'/'000001';folder.mkdir(parents=True)
    cv2.imwrite(str(folder/'rgb.png'),np.zeros((80,100,3),dtype=np.uint8))
    adapter=ModelRGBDPerception.__new__(ModelRGBDPerception)
    adapter.root=tmp_path;adapter.target_sequence=0
    adapter.rt=SimpleNamespace(time=2.,states={'A':'ASSISTED_GRASP'})
    adapter.last_frames={'A':(folder,np.eye(4),[100,0,50,0,100,40,0,0,1],12,'run:1',1.9)}
    t=np.eye(4);t[2,3]=2.
    adapter.record_targets('A',{'left':t},kind='rule_targets',box_pose=t,spread=0.)
    t[0,3]=.1
    adapter.record_targets('A',{'left':t},kind='rule_targets',box_pose=t,spread=0.)
    records=[json.loads(x) for x in (folder/'grasp_targets.jsonl').read_text().splitlines()]
    assert len(records)==2
    assert records[0]['targets']['left'][0][3]==0
    assert records[1]['targets']['left'][0][3]==.1
    assert records[1]['pair_id']=='run:1' and records[1]['target_id']=='12'
    assert records[1]['projections']['left']['pixel']==[55,40]
    image=cv2.imread(str(folder/'rule_targets_overlay.png'))
    assert image is not None and image.any()
    assert len((tmp_path/'grasp_targets.jsonl').read_text().splitlines())==2
