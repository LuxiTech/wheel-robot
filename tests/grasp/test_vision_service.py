"""Task adapter preserves the retained pipeline's identity and diagnostic gate."""
import asyncio
from types import SimpleNamespace

import pytest

from wheel_perception.service import PipelineVision, SynchronizedCapture


@pytest.mark.parametrize('diagnostic', [True, False])
def test_service_preserves_capture_time_frame_and_diagnostic_status(diagnostic):
    observation = object()
    joints = {'lift_joint': .15}

    class Source:
        ready = True

        async def capture(self, target_id):
            assert target_id == 'box1'
            return SynchronizedCapture(observation, joints, 12., [10, 20, 30, 40])

    class Pipeline:
        def process(self, obs, measured, *, target_id, box):
            assert obs is observation and measured == joints
            assert box == [10, 20, 30, 40]
            return dict(target_id=target_id, pair_id='camera:1', frame_id='base_footprint',
                        quality={'accepted': True}, diagnostic_only=diagnostic,
                        executable=False, grasp={'left': 'fixture', 'right': 'fixture'})

    service = PipelineVision(Source(), Pipeline())
    try:
        result = asyncio.run(service.observe('box1'))
        assert service.ready
        assert result.captured_at == 12. and result.pair_id == 'camera:1'
        assert result.frame_id == 'base_footprint'
        assert result.diagnostic_only is diagnostic
        assert result.quality_valid and not result.data['executable']
    finally:
        service.close()


def test_model_failure_is_propagated_without_fabricating_targets():
    class Source:
        ready = True

        async def capture(self, target_id):
            return SynchronizedCapture(object(), {}, 0.)

    def reject(*args, **kwargs):
        raise ValueError('depth quality rejected')

    service = PipelineVision(Source(), SimpleNamespace(process=reject))
    try:
        with pytest.raises(ValueError, match='depth quality rejected'):
            asyncio.run(service.observe('box1'))
    finally:
        service.close()
