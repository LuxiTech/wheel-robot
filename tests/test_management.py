"""Read-only northbound projection and optional HTTP adapter integration."""
import json
from pathlib import Path
import shutil

import pytest
from wheel_gateway.service import RuntimeGateway

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def project(tmp_path):
    shutil.copytree(ROOT/'config', tmp_path/'config')
    shutil.copytree(ROOT/'apps/management_web', tmp_path/'apps/management_web')
    return tmp_path


def test_gateway_reports_unconfigured_runtime_and_does_not_create_outputs(project):
    gateway = RuntimeGateway(project)
    status = gateway.status()
    assert status['read_only'] is True
    assert status['source'] == 'default_runtime_composition'
    assert status['navigation_ready'] is False
    assert status['vision_source_ready'] is False
    assert status['motion_executor_ready'] is False
    assert gateway.workflow()['hardware_commands_sent'] is False
    assert gateway.runs() == []
    assert not (project/'outputs').exists()
    (project/'.env').write_text('SECRET=do-not-read', encoding='utf-8')
    assert 'SECRET' not in json.dumps(gateway.configurations())


def test_gateway_reads_valid_results_and_ignores_bad_or_external_reports(project, tmp_path):
    folder = project/'outputs/robot'
    valid = folder/('a'*32)
    valid.mkdir(parents=True)
    (valid/'result.json').write_text(json.dumps(dict(success=False, phase='NOT_READY',
                                                   reason='not integrated', token='hidden')), encoding='utf-8')
    bad = folder/('b'*32)
    bad.mkdir()
    (bad/'result.json').write_text('{broken', encoding='utf-8')
    ignored = folder/'untrusted-name'
    ignored.mkdir()
    (ignored/'result.json').write_text('{}', encoding='utf-8')
    assert RuntimeGateway(project).runs() == [dict(run_id='a'*32, success=False,
        phase='NOT_READY', reason='not integrated', may_be_holding=None)]
    external = tmp_path/'outside.json'
    external.write_text('{"phase":"outside"}', encoding='utf-8')
    link = folder/('c'*32)
    link.mkdir()
    try:
        (link/'result.json').symlink_to(external)
    except OSError:
        pytest.skip('Symlink creation unavailable on this host')
    assert len(RuntimeGateway(project).runs()) == 1


def test_http_queries_and_frontend_reject_task_submission(project):
    pytest.importorskip('fastapi', reason='Management environment optional')
    pytest.importorskip('httpx', reason='HTTP test client optional')
    from fastapi.testclient import TestClient
    from management_api.app import create_app
    with TestClient(create_app(project)) as client:
        assert client.get('/api/v1/health').json()['read_only'] is True
        assert client.get('/api/v1/status').json()['motion_executor_ready'] is False
        assert len(client.get('/api/v1/workflow').json()['phases']) == 14
        assert 'robots' in client.get('/api/v1/configurations').json()
        assert client.get('/api/v1/runs').json() == []
        assert '机器人管理控制台' in client.get('/').text
        assert client.get('/app.js').status_code == 200
        assert client.post('/api/v1/tasks', json={'target_id':'crate'}).status_code == 405
        assert client.get('/.env').status_code == 404
