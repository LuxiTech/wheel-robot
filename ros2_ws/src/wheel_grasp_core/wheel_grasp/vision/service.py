"""Adapt the retained RGB-D/keypoint pipeline to the task interface."""
from __future__ import annotations

import asyncio
from dataclasses import dataclass
from functools import partial
from typing import Protocol

from wheel_grasp.contracts import GraspObservation
from wheel_grasp.vision.rgbd import Observation


@dataclass(frozen=True)
class SynchronizedCapture:
    observation: Observation
    joints: dict
    captured_at: float
    box: object = None


class CaptureSource(Protocol):
    ready: bool

    async def capture(self, target_id: str) -> SynchronizedCapture:
        """Return calibrated RGB-D and measured joints at the exposure time.

        captured_at is translated to the task process monotonic clock. A source
        must reject unknown age, incomplete feedback or unavailable transforms.
        box is an optional selected image ROI; multiple detections require one.
        """
        ...


class PipelineVision:
    def __init__(self, source: CaptureSource, pipeline):
        self.source, self.pipeline = source, pipeline
        # Inference backends retain tracking state: serialize worker access even
        # if the awaiting task times out. Late results never execute a command.
        from concurrent.futures import ThreadPoolExecutor
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='vision')

    @property
    def ready(self):
        return self.source.ready is True

    async def observe(self, target_id):
        capture = await self.source.capture(target_id)
        result = await asyncio.get_running_loop().run_in_executor(
            self.worker, partial(self.pipeline.process, capture.observation,
                                 capture.joints, target_id=target_id, box=capture.box))
        return GraspObservation(
            result['target_id'], result['pair_id'], result['frame_id'],
            capture.captured_at, bool(result['quality']['accepted']),
            result['diagnostic_only'], result,
        )

    def close(self):
        self.worker.shutdown(wait=True)


class UnconfiguredVision:
    ready = False

    async def observe(self, target_id):
        raise NotImplementedError('Calibrated RGB-D / timestamped joint source is not connected')
