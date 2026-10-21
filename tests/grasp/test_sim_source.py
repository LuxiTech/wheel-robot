"""Input isolation contract: default run must not start an SDK receiver."""
import json
from pathlib import Path
import shutil
import sys
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/grasp'))
import run_pipeline
from wheel_grasp.rgbd import load_observation,save_observation,decode
from wheel_grasp.geometry import rim_targets
from scipy.spatial.transform import Rotation
from test_core import packet


def test_default_orchestration_never_starts_sdk(tmp_path,monkeypatch):
    for filename in ['config/grasp/runtime.sim.json','config/grasp/dependencies.lock.json',
                     'config/grasp/models.lock.json','config/grasp/validated-environment.json',
                     'scripts/run_grasp_snapshot.py','scripts/capture_grasp_sim.py',
                     'scripts/grasp_sim_scene.py','scripts/grasp_carry.py','compose.grasp.yaml']:
        target=tmp_path/filename;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/filename,target)
    commands=[]
    class Worker:
        def __init__(self,command,**kwargs):
            commands.append(command)
            directory=tmp_path/Path(command[-1]).relative_to('/workspace')
            stage=json.loads((directory/'manifest.json').read_text())['stage']
            (directory/(stage+'.result.json')).write_text(json.dumps(dict(run_id=directory.name,status='completed')))
        def wait(self,**kwargs):return 0
        def poll(self):return 0
    monkeypatch.setattr(run_pipeline,'ROOT',tmp_path)
    monkeypatch.setattr(run_pipeline.subprocess,'run',lambda *a,**k:None)
    monkeypatch.setattr(run_pipeline.subprocess,'Popen',Worker)
    monkeypatch.setattr(sys,'argv',['run_pipeline.py'])
    run_pipeline.main()
    assert len(commands)==3
    assert 'sim' in commands[0] and commands[0][-2].endswith('capture_grasp_sim.py')
    assert all('sdk_bridge' not in c for c in commands)
    manifest=json.loads(next((tmp_path/'outputs/grasp/runs').glob('*/manifest.json')).read_text())
    assert manifest['source']=='sim' and manifest['status']=='completed'
    assert manifest['mode']=='simulation_pipeline'


def test_sim_frames_require_explicit_opt_in(tmp_path):
    observation=decode(*packet(mock=True),allow_mock=True)
    observation.metadata['source']='isaac_sim'
    save_observation(tmp_path/'sim.npz',observation)
    with pytest.raises(ValueError,match='Real observation required'):
        load_observation(tmp_path/'sim.npz')
    assert load_observation(tmp_path/'sim.npz',allow_mock=True).metadata['source']=='isaac_sim'


def test_sim_has_no_hardware_endpoint_and_uses_isolated_services():
    cfg=json.loads((ROOT/'config/grasp/runtime.sim.json').read_text())
    assert 'base_url' not in cfg
    compose=yaml.safe_load((ROOT/'compose.grasp.yaml').read_text())
    base=yaml.safe_load((ROOT/'compose.yaml').read_text())
    assert compose['services']['perception']['network_mode']=='none'
    assert base['services']['isaac-sim']['network_mode']=='none'
    assert cfg['lift_mapping'] is None  # Simulation does not require hardware zero calibration.


def test_pinch_closing_axis_matches_box_wall_normal():
    pose=np.eye(4);pose[:3,3]=[.55,0,.15]
    pose[:3,:3]=Rotation.from_euler('z',-np.pi/2).as_matrix()
    goals,pre=rim_targets(pose,width=.4,height=.22,inset=.007,below_top=.006,
                         tool_convention='pinch_y')
    for side in goals:
        assert abs(goals[side][:3,1]@pose[:3,0])==pytest.approx(1)
        assert goals[side][2,2]>0
        assert goals[side][2,0]>0  # Wrist camera on +X stays above the rim on both hands.
        assert pre[side][0,3]<goals[side][0,3]
