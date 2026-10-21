"""The old task must consume new measurements without a simulator-pose fallback."""
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from relay_runtime import RelayRuntime


def runtime(sensor):
    rt=RelayRuntime.__new__(RelayRuntime)
    rt.perception='rgbd';rt.held={'A':None,'B':None};rt.pose_symmetry={}
    class ForbiddenTruth:
        def get_world_pose(self):raise AssertionError('Perception read simulator object pose')
    rt.crates=[ForbiddenTruth()];rt.depth_sensor=sensor
    return rt


def test_observe_consumes_model_measurement():
    measured=np.eye(4);measured[:3,3]=[1.,2.,3.]
    rt=runtime(SimpleNamespace(observe=lambda index:measured.copy()))
    assert np.array_equal(rt.observe(0),measured)


def test_model_failure_propagates_without_truth_fallback():
    def failed(index):raise ValueError('model rejected frame')
    rt=runtime(SimpleNamespace(observe=failed))
    with pytest.raises(ValueError,match='model rejected'):rt.observe(0)
