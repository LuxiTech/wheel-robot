"""Navigation boundary. Hardware integration is intentionally not implemented."""


class Nav2AprilTagNavigation:
    ready = False

    async def go_to(self, station):
        raise NotImplementedError('Nav2 + AprilTag docking adapter is not connected')

    async def stop(self):
        raise NotImplementedError('Navigation cancellation/stop acknowledgement is not connected')
