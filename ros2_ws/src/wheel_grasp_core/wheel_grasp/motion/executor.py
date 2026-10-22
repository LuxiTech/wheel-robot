"""Robot-local execution boundary, separate from the existing offline planner.

Implement against the SDK control manager, with exclusive ownership, measured
joint feedback and cancellation. Do not publish directly to internal commands.
"""


class SDKMotionExecutor:
    ready = False

    async def prepare_transport(self, holding):
        raise NotImplementedError('Measured transport posture is not connected')

    async def plan_grasp(self, observation):
        raise NotImplementedError('Hardware-calibrated scene and planner are not connected')

    async def pregrasp(self, plan):
        raise NotImplementedError('SDK autonomous dual-arm trajectory execution is not connected')

    async def approach(self, plan):
        raise NotImplementedError('SDK autonomous dual-arm trajectory execution is not connected')

    async def close_and_verify(self):
        raise NotImplementedError('Measured bilateral grasp verification is not connected')

    async def test_lift(self):
        raise NotImplementedError('Measured test-lift verification is not connected')

    async def place(self, slot):
        raise NotImplementedError('Support, release and withdrawal verification are not connected')

    async def hold(self):
        raise NotImplementedError('SDK hold acknowledgement is not connected')
