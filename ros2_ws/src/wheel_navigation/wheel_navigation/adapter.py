"""Task Navigation port backed by lucid-atlas A/B APIs and measured completion."""
import asyncio
import uuid


class AtlasNavigation:
    def __init__(self, service=None, station_points=None):
        self.service = service
        self.station_points = station_points or getattr(service, 'station_points', None) or {'source_pallet':'A', 'target_pallet':'B'}

    @property
    def ready(self):
        return self.service is not None and self.service.enabled

    async def go_to(self, station):
        if not self.ready:
            raise RuntimeError('Atlas navigation service is not configured/enabled')
        point = self.station_points.get(station, station)
        command_id = str(uuid.uuid4())
        result = await asyncio.to_thread(self.service.navigate, point, command_id)
        if result['state'] not in ('MOVING','SUBMITTING'):
            raise RuntimeError(result['reason'])
        self.service.start_worker()
        try:
            while True:
                snapshot = await asyncio.to_thread(self.service.snapshot)
                result = snapshot.get('command') or {}
                if result.get('command_id') != command_id:
                    raise RuntimeError('Navigation ownership changed')
                if result['state'] == 'SUCCEEDED': return
                if result['state'] in ('REJECTED','FAULT','CANCELLED','STOPPED'):
                    raise RuntimeError(result['reason'])
                if result['state'] == 'UNKNOWN' and result.get('stage') == 'stop':
                    raise RuntimeError(result['reason'])
                await asyncio.sleep(self.service.poll_interval)
        except asyncio.CancelledError:
            await asyncio.shield(self.stop())
            raise

    async def stop(self):
        if not self.ready:
            raise RuntimeError('Atlas navigation service is not configured/enabled')
        command_id = str(uuid.uuid4())
        await asyncio.to_thread(self.service.stop, command_id)
        for _ in range(int(self.service.stop_timeout / self.service.poll_interval) + 2):
            snapshot = await asyncio.to_thread(self.service.snapshot)
            result = snapshot.get('command') or {}
            if result.get('command_id') == command_id:
                if result['state'] == 'STOPPED': return
                if result['state'] == 'UNKNOWN': raise RuntimeError(result['reason'])
            await asyncio.sleep(self.service.poll_interval)
        raise RuntimeError('SDK physical stop was not confirmed')
