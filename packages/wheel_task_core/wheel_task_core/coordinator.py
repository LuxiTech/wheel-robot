"""One transfer at a time; orchestration never owns a motor control loop."""
from __future__ import annotations

import asyncio
from dataclasses import dataclass
import math
import time

from wheel_task_core.contracts import Motion, Navigation, TransferJob, Vision


@dataclass(frozen=True)
class TaskResult:
    success: bool
    phase: str
    reason: str
    may_be_holding: bool
    stop_errors: tuple = ()


WORKFLOW = (
    'PREPARE_EMPTY', 'NAVIGATE_SOURCE', 'OBSERVE', 'PLAN_PREGRASP',
    'PREGRASP', 'REOBSERVE', 'PLAN_APPROACH', 'APPROACH', 'CLOSE',
    'TEST_LIFT', 'PREPARE_LOADED', 'NAVIGATE_DESTINATION', 'PLACE',
    'COMPLETE',
)


class TaskCoordinator:
    def __init__(self, navigation: Navigation, vision: Vision, motion: Motion,
                 *, base_frame='base_footprint', max_observation_age=1.0,
                 step_timeout=60.0, stop_timeout=5.0, clock=time.monotonic):
        for value in (max_observation_age, step_timeout, stop_timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Timeouts must be finite and positive')
        self.navigation, self.vision, self.motion = navigation, vision, motion
        self.base_frame = base_frame
        self.max_observation_age = max_observation_age
        self.step_timeout, self.stop_timeout = step_timeout, stop_timeout
        self.clock = clock
        self.phase = 'IDLE'
        self.may_be_holding = False
        self.events = []
        self.stop_errors = ()

    def transition(self, phase):
        self.phase = phase
        self.events.append(dict(phase=phase, monotonic=self.clock()))

    async def step(self, phase, operation, *args):
        self.transition(phase)
        return await asyncio.wait_for(operation(*args), timeout=self.step_timeout)

    def validate_observation(self, observation, job, previous_pair=None):
        age = self.clock() - observation.captured_at
        if (observation.target_id != job.target_id or not observation.pair_id
                or observation.pair_id == previous_pair):
            raise ValueError('Wrong target or duplicate observation')
        if observation.frame_id != self.base_frame:
            raise ValueError('Observation must use the configured robot base frame')
        if not math.isfinite(age) or not 0 <= age <= self.max_observation_age:
            raise ValueError('Observation is stale or from an invalid clock')
        if observation.diagnostic_only or not observation.quality_valid:
            raise ValueError('Observation is diagnostic-only or quality rejected')

    @staticmethod
    def validate_plan(plan, observation):
        if (plan.target_id != observation.target_id or plan.pair_id != observation.pair_id
                or not plan.collision_checked or not plan.execution_ready):
            raise ValueError('Plan is stale, unchecked or not executable')

    async def stop(self):
        # Attempt both independently. A failed base stop must not suppress arm hold.
        results = await asyncio.gather(
            asyncio.wait_for(self.navigation.stop(), self.stop_timeout),
            asyncio.wait_for(self.motion.hold(), self.stop_timeout),
            return_exceptions=True,
        )
        self.stop_errors = tuple(
            name + ': ' + type(result).__name__ + ': ' + str(result)
            for name, result in zip(('navigation.stop', 'motion.hold'), results)
            if isinstance(result, BaseException)
        )

    async def run(self, job: TransferJob):
        if self.phase != 'IDLE':
            raise RuntimeError('Create a new coordinator for a new task; automatic resume is disabled')
        missing = [name for name, adapter in (
            ('navigation', self.navigation), ('vision', self.vision), ('motion', self.motion)
        ) if adapter.ready is not True]
        if missing:
            self.transition('NOT_READY')
            return TaskResult(False, self.phase, 'Adapters not ready: ' + ', '.join(missing), False)
        try:
            await self.step('PREPARE_EMPTY', self.motion.prepare_transport, False)
            await self.step('NAVIGATE_SOURCE', self.navigation.go_to, job.source_station)
            observation = await self.step('OBSERVE', self.vision.observe, job.target_id)
            self.validate_observation(observation, job)
            plan = await self.step('PLAN_PREGRASP', self.motion.plan_grasp, observation)
            self.validate_observation(observation, job)
            self.validate_plan(plan, observation)
            await self.step('PREGRASP', self.motion.pregrasp, plan)
            previous_pair = observation.pair_id
            observation = await self.step('REOBSERVE', self.vision.observe, job.target_id)
            self.validate_observation(observation, job, previous_pair)
            plan = await self.step('PLAN_APPROACH', self.motion.plan_grasp, observation)
            self.validate_observation(observation, job)
            self.validate_plan(plan, observation)
            await self.step('APPROACH', self.motion.approach, plan)
            # A timed-out close may already have acquired the load.
            self.may_be_holding = True
            await self.step('CLOSE', self.motion.close_and_verify)
            await self.step('TEST_LIFT', self.motion.test_lift)
            await self.step('PREPARE_LOADED', self.motion.prepare_transport, True)
            await self.step('NAVIGATE_DESTINATION', self.navigation.go_to, job.destination_station)
            await self.step('PLACE', self.motion.place, job.destination_slot)
            self.may_be_holding = False
            self.transition('COMPLETE')
            return TaskResult(True, self.phase, '', False)
        except asyncio.CancelledError:
            await self.stop()
            self.transition('HOLD' if self.may_be_holding else 'CANCELLED')
            raise
        except Exception as exc:
            failed_phase = self.phase
            await self.stop()
            self.transition('HOLD' if self.may_be_holding else 'FAULT')
            return TaskResult(False, self.phase,
                              failed_phase + ': ' + type(exc).__name__ + ': ' + str(exc),
                              self.may_be_holding, self.stop_errors)
