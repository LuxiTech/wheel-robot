"""Single SDK joint-target execution boundary, separate from unready grasp planning."""

import asyncio
from functools import partial


class SDKJointExecutor:
    def __init__(self, client):
        self.client = client

    @property
    def ready(self):
        return self.client.motion_allowed

    async def preflight(self):
        return await asyncio.get_running_loop().run_in_executor(None, self.client.preflight)

    async def move_joints(self, positions, speed_rad_s=0.02, *, enable_if_needed=False):
        return await asyncio.get_running_loop().run_in_executor(
            None,
            partial(self.client.move, positions, speed_rad_s, enable_if_needed=enable_if_needed),
        )

    async def status(self, request_id):
        return await asyncio.get_running_loop().run_in_executor(
            None, partial(self.client.result, request_id)
        )

    async def hold(self, request_id):
        return await asyncio.get_running_loop().run_in_executor(
            None, partial(self.client.stop, request_id)
        )
