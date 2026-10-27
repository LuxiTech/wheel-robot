import json
from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'tools/grasp'))
from runlog import RunLog
from wheel_sdk_bridge.feedback import measured_state


def state():
    return dict(system={'mock':False}, server_mono_ns='1000000000',
                arms=dict(receive_mono_ns='990000000', fresh=True, position=[0.]*16,
                          valid=[True]*16, age_ns=[0]*16),
                lift=dict(valid=True, receive_mono_ns='990000000', effective_age_ms=10, current_um=0))


def test_nested_error_preserves_deepest_stage(tmp_path):
    log = RunLog(tmp_path,'worker')
    with pytest.raises(ValueError):
        with log.stage('pipeline'):
            with log.stage('pose_registration', pair_id='camera:42'):
                raise ValueError('bad pose')
    error = json.loads((tmp_path/'worker.error.json').read_text())
    assert error['stage']=='pose_registration'
    assert error['pair_id']=='camera:42'
    assert 'ValueError: bad pose' in error['traceback']
    events = [json.loads(line) for line in (tmp_path/'worker.events.jsonl').read_text().splitlines()]
    assert [e['status'] for e in events] == ['started','started','failed','failed']


def test_missing_arms_is_actionable():
    data=state();data['arms']=None
    with pytest.raises(ValueError, match='arms=null'):
        measured_state(data,None,.01)


def test_stale_snapshot_cannot_use_fresh_joint_flags():
    data=state();data['arms']['receive_mono_ns']='0'
    with pytest.raises(ValueError, match='snapshot stale'):
        measured_state(data,None,.01)


def test_lift_mapping_is_required():
    with pytest.raises(ValueError, match='lift_mapping missing'):
        measured_state(state(),None,.01)


def test_valid_measured_mapping():
    values=measured_state(state(),dict(direction=1,offset_m=.2,device_min_um=-180000,device_max_um=180000),.01)
    assert len(values)==15
    assert values['lift_joint']==.2


def test_mock_state_rejected():
    data=state();data['system']['mock']=True
    with pytest.raises(ValueError, match='Real SDK'):
        measured_state(data,None,0)
