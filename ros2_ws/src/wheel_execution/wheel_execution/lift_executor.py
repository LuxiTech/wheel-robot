"""Async absolute lift targets; calibrated URDF conversion is explicit."""

import asyncio
from functools import partial


class SDKLiftExecutor:
    def __init__(self, client, mapping=None):
        self.client = client
        self.mapping = mapping

    async def _call(self, function, *args, **kwargs):
        return await asyncio.get_running_loop().run_in_executor(
            None, partial(function, *args, **kwargs)
        )

    async def _command(self, function, *args):
        try:
            return await self._call(function, *args)
        except asyncio.CancelledError:
            # The worker may already have written. stop() serializes after it
            # and confirms the real state before cancellation is propagated.
            await asyncio.shield(self.hold())
            raise

    async def preflight(self):
        return await self._call(self.client.preflight)

    async def open(self):
        return await self._command(self.client.open)

    async def move_lift_m(self, position_m):
        """Absolute SDK encoder metres (not height above the floor)."""
        return await self._command(self.client.move, position_m)

    async def move_joint_m(self, joint_m):
        """IK/URDF prismatic coordinate, requiring a measured LiftMapping."""
        if self.mapping is None:
            raise ValueError("Calibrated LiftMapping required for IK lift coordinates")
        return await self.move_lift_m(self.mapping.to_device(joint_m) / 1e6)

    async def status(self, task_id):
        return await self._call(self.client.result, task_id)

    async def wait(self, task_id, timeout=180.0):
        return await self._command(self.client.wait, task_id, timeout)

    async def hold(self):
        """Stop, confirm stationary and release the position-control lease."""
        return await self._call(self.client.stop)

    async def close(self):
        return await self.hold()
