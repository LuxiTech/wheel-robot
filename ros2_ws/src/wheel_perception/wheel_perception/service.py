"""Adapt the retained RGB-D/keypoint pipeline to the task interface."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from functools import partial
from typing import Protocol

from wheel_perception_core.rgbd import Observation
from wheel_task_core.contracts import GraspObservation


@dataclass(frozen=True)
class SynchronizedCapture:
    observation: Observation
    joints: dict
    captured_at: float
    box: object = None
    transform_context: object = None


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
        self.tracking = False
        # Inference backends retain tracking state: serialize worker access even
        # if the awaiting task times out. Late results never execute a command.
        from concurrent.futures import ThreadPoolExecutor

        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vision")

    @property
    def ready(self):
        return self.source.ready is True

    async def observe(self, target_id):
        capture = await self.source.capture(target_id)
        options = dict(target_id=target_id, box=capture.box)
        if self.tracking:
            options["track"] = True
        if capture.transform_context is not None:
            options["transform_context"] = capture.transform_context
        result = await asyncio.get_running_loop().run_in_executor(
            self.worker,
            partial(self.pipeline.process, capture.observation, capture.joints, **options),
        )
        if capture.transform_context is not None:
            result = dict(result)
            result["capture_context"] = capture.transform_context
        return GraspObservation(
            result["target_id"],
            result["pair_id"],
            result["frame_id"],
            capture.captured_at,
            bool(result["quality"]["accepted"]),
            result["diagnostic_only"],
            result,
        )

    def close(self):
        self.worker.shutdown(wait=True)
        if hasattr(self.source, "close"):
            self.source.close()

    def enable_streaming(self):
        from wheel_sdk_bridge.calibrated_capture import ReadOnlyHeadCapture
        from wheel_sdk_bridge.streaming_capture import StreamingHeadCapture

        if isinstance(self.source, ReadOnlyHeadCapture) and not isinstance(
            self.source, StreamingHeadCapture
        ):
            self.source = StreamingHeadCapture(
                self.source.url,
                self.source.calibration,
                max_age_ms=self.source.max_age_ms,
                stationary_diagnostic=self.source.stationary_diagnostic,
                strict_lift_max_age_ms=self.source.strict_lift_max_age_ms,
                lift_max_sample_gap_ms=self.source.lift_max_sample_gap_ms,
                on_response=self.source.client.on_response,
            )
        if not isinstance(self.source, StreamingHeadCapture):
            raise ValueError("A time-paired streaming head capture source is required")
        self.tracking = True
        self.source.start()


class UnconfiguredVision:
    ready = False

    async def observe(self, target_id):
        raise NotImplementedError("Calibrated RGB-D / timestamped joint source is not connected")


class PlanningTargetSource:
    """Connect the existing live PipelineVision to stationary grasp execution.

    Source blockers come from loaded physical calibration/model/rule records.
    All pipeline diagnostic reasons survive conversion to a motion packet.
    No unvalidated calibration flag is silently promoted to acceptance.
    """

    def __init__(self, vision, rules, *, source_blockers=(), metric_depth_validated=False):
        self.vision, self.rules = vision, rules
        self.source_blockers = tuple(source_blockers)
        self.metric_depth_validated = metric_depth_validated

    def enable_streaming(self):
        self.vision.enable_streaming()

    def disable_streaming(self):
        if hasattr(self.vision.source, "close"):
            self.vision.source.close()

    def __call__(self, target_id):
        from wheel_perception_core.targets import visual_target

        observation = asyncio.run(self.vision.observe(target_id))
        result = observation.data
        if observation.frame_id != "arm_origin":
            raise ValueError("Live vision must output registered SDK arm_origin poses")
        blockers = list(self.source_blockers)
        if observation.diagnostic_only:
            blockers.extend(result.get("reasons") or ["vision_is_diagnostic_only"])
        if not observation.quality_valid:
            blockers.append("vision_quality_rejected")
        quality = dict(result["quality"])
        quality["metric_depth_semantics_validated"] = self.metric_depth_validated
        # Height comes from this very capture context, never an unrelated
        # latest lift sample fetched after slow inference.
        context = result.get("capture_context", {})
        if (
            context.get("schema") not in ("wheel_lift_capture_v1", "wheel_lift_stream_capture_v1")
            or context.get("pair_id") != observation.pair_id
        ):
            raise ValueError("Height must come from the matching visual capture context")
        height_um = context["height_um"]
        target = visual_target(
            target_id=observation.target_id,
            pair_id=observation.pair_id,
            object_pose_origin=result["pose"],
            rules=self.rules,
            height_um=height_um,
            quality=quality,
            source_blockers=blockers,
            observed_at_monotonic=observation.captured_at,
        )
        if "image_receive_mono_ns" in context:
            target["image_receive_mono_ns"] = int(context["image_receive_mono_ns"])
        return target
