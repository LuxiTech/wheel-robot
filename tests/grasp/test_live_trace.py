"""Diagnostic artifacts and failure localization, independent of GPU models."""
import json
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools/grasp'))
from perceive_live import Trace, LazyEstimator, LazyRobot
from wheel_perception_core.geometry import ModelInput


def test_images_and_deep_error_survive_failure(tmp_path):
    trace=Trace(tmp_path)
    inp=ModelInput(np.zeros((32,40,3),np.uint8),np.ones((32,40),np.float32),np.eye(3),'camera',True,['replay'])
    with pytest.raises(ValueError):
        with trace.stage('preprocess'):trace.record('preprocess',inp=inp)
        with trace.stage('detection'):
            trace.record('detection',inp=inp,candidates=[],box=None)
            raise ValueError('No target <crate>')
    trace.report('failed')
    assert (tmp_path/'01_rectified_rgb.png').is_file()
    assert (tmp_path/'02_depth.png').is_file()
    assert (tmp_path/'03_detections.png').is_file()
    assert json.loads((tmp_path/'vision.error.json').read_text())['stage']=='detection'
    report=(tmp_path/'report.html').read_text()
    assert '&lt;crate&gt;' in report and '<crate>' not in report
    assert json.loads((tmp_path/'preprocess.json').read_text())['diagnostic_only']


def test_missing_real_geometry_and_calibration_fail_closed(tmp_path):
    trace=Trace(tmp_path)
    cfg={'model':{},'object_geometry_validated':False,'camera_extrinsics_validated':False}
    with pytest.raises(ValueError,match='measured mesh'):
        LazyEstimator(cfg,trace).register(None,None,None)
    with pytest.raises(ValueError,match='real URDF'):
        LazyRobot(cfg).transform('head',{},'base')
    assert json.loads((tmp_path/'vision.error.json').read_text())['stage']=='pose_init'
