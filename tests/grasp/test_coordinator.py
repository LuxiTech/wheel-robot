"""Application integration with fake ports; no device commands or model inference."""
import asyncio
from dataclasses import replace
import json

import pytest

from wheel_grasp.contracts import GraspObservation, MotionPlan, TransferJob
from wheel_grasp.main import create_application, main
from wheel_grasp.task.coordinator import TaskCoordinator, WORKFLOW


JOB = TransferJob('box1', 'source', 'destination', 'slot1')


class Navigation:
    ready = True

    def __init__(self):
        self.calls = []

    async def go_to(self, station):
        self.calls.append(station)

    async def stop(self):
        self.calls.append('stop')


class Vision:
    ready = True

    def __init__(self):
        self.sequence = 0
        self.modify = lambda observation: observation

    async def observe(self, target_id):
        self.sequence += 1
        return self.modify(GraspObservation(
            target_id, str(self.sequence), 'base_footprint', 10., True, False, {}))


class Motion:
    ready = True

    def __init__(self):
        self.calls = []
        self.modify = lambda plan: plan

    async def prepare_transport(self, holding):
        self.calls.append(('transport', holding))

    async def plan_grasp(self, observation):
        return self.modify(MotionPlan(observation.target_id, observation.pair_id, True, True, []))

    async def pregrasp(self, plan):
        self.calls.append(('pregrasp', plan.pair_id))

    async def approach(self, plan):
        self.calls.append(('approach', plan.pair_id))

    async def close_and_verify(self):
        self.calls.append('close')

    async def test_lift(self):
        self.calls.append('lift')

    async def place(self, slot):
        self.calls.append(('place', slot))

    async def hold(self):
        self.calls.append('hold')


def application(**kwargs):
    return TaskCoordinator(Navigation(), Vision(), Motion(), clock=lambda: 10., **kwargs)


def test_transfer_reobserves_before_approach_and_finishes_only_after_release():
    app = application()
    result = asyncio.run(app.run(JOB))
    assert result.success and not result.may_be_holding
    assert [e['phase'] for e in app.events] == list(WORKFLOW)
    assert app.navigation.calls == ['source', 'destination']
    assert app.motion.calls == [('transport', False), ('pregrasp', '1'),
                                ('approach', '2'), 'close', 'lift',
                                ('transport', True), ('place', 'slot1')]
    with pytest.raises(RuntimeError):
        asyncio.run(app.run(JOB))


@pytest.mark.parametrize('change', [
    dict(target_id='other'), dict(pair_id=''), dict(frame_id='map'),
    dict(captured_at=0), dict(captured_at=11), dict(captured_at=float('nan')),
    dict(diagnostic_only=True), dict(quality_valid=False),
])
def test_bad_observation_stops_before_arm_motion(change):
    app = application()
    app.vision.modify = lambda observation: replace(observation, **change)
    result = asyncio.run(app.run(JOB))
    assert not result.success and result.phase == 'FAULT'
    assert app.motion.calls == [('transport', False), 'hold']
    assert app.navigation.calls == ['source', 'stop']


def test_reobservation_cannot_reuse_old_pair():
    app = application()
    app.vision.modify = lambda observation: replace(observation, pair_id='same')
    result = asyncio.run(app.run(JOB))
    assert result.phase == 'FAULT' and 'duplicate' in result.reason
    assert ('approach', 'same') not in app.motion.calls


@pytest.mark.parametrize('change', [dict(pair_id='old'), dict(target_id='other'),
                                  dict(collision_checked=False), dict(execution_ready=False)])
def test_unchecked_or_stale_plan_is_not_dispatched(change):
    app = application()
    app.motion.modify = lambda plan: replace(plan, **change)
    result = asyncio.run(app.run(JOB))
    assert not result.success
    assert app.motion.calls == [('transport', False), 'hold']


def test_slow_planning_invalidates_observation_before_dispatch():
    app = application()
    original = app.motion.plan_grasp

    async def delayed(observation):
        app.clock = lambda: 12.
        return await original(observation)

    app.motion.plan_grasp = delayed
    assert 'stale' in asyncio.run(app.run(JOB)).reason
    assert app.motion.calls == [('transport', False), 'hold']


def test_close_timeout_retains_possible_load_and_does_not_navigate():
    app = application(step_timeout=.02)

    async def stalled():
        await asyncio.sleep(10)

    app.motion.close_and_verify = stalled
    result = asyncio.run(app.run(JOB))
    assert result.phase == 'HOLD' and result.may_be_holding
    assert 'CLOSE' in result.reason
    assert app.navigation.calls == ['source', 'stop']
    assert app.motion.calls[-1] == 'hold'
    assert 'lift' not in app.motion.calls


def test_failed_stop_does_not_suppress_hold_or_hide_stop_error():
    app = application()

    async def fail():
        raise RuntimeError('feedback lost')

    app.motion.test_lift = fail
    app.navigation.stop = fail
    result = asyncio.run(app.run(JOB))
    assert result.phase == 'HOLD'
    assert app.motion.calls[-1] == 'hold'
    assert result.stop_errors and 'navigation.stop' in result.stop_errors[0]


def test_cancel_during_close_holds_without_release():
    async def scenario():
        app = application()
        closing = asyncio.Event()

        async def close():
            closing.set()
            await asyncio.sleep(10)

        app.motion.close_and_verify = close
        task = asyncio.create_task(app.run(JOB))
        await closing.wait()
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert app.phase == 'HOLD' and app.may_be_holding
        assert app.motion.calls[-1] == 'hold'
        assert ('place', 'slot1') not in app.motion.calls

    asyncio.run(scenario())


def test_unconfigured_application_refuses_before_any_command():
    app = create_application({})
    result = asyncio.run(app.run(JOB))
    assert not result.success and result.phase == 'NOT_READY'
    assert app.events[0]['phase'] == 'NOT_READY'


def test_main_logs_not_ready_and_returns_failure(tmp_path):
    config = tmp_path/'task.yaml'
    config.write_text(json.dumps(dict(
        job=dict(target_id='box', source_station='s', destination_station='d', destination_slot='slot'),
        output_directory=str(tmp_path/'runs'))))
    assert main(['--config', str(config), 'run']) == 2
    report = json.loads(next((tmp_path/'runs').glob('*/result.json')).read_text())
    assert report['phase'] == 'NOT_READY' and not report['success']
