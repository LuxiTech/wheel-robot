"""lucid-atlas named-map-point APIs; never duplicates chassis control."""
from concurrent.futures import ThreadPoolExecutor
from urllib.parse import urlencode
from wheel_sdk_bridge.http_client import JSONClient, SDKError


class AtlasClient:
    def __init__(self, base_url, map_name='unstacking', timeout=3.0):
        self.http = JSONClient(base_url, timeout)
        self.map_name = map_name

    def snapshot(self):
        paths = {
            'status':'/api/status', 'map':'/api/map/status',
            'points':'/api/maps/points?' + urlencode({'map_name':self.map_name}),
            'robot':'/api/v1/robot',
        }
        result = {}
        with ThreadPoolExecutor(max_workers=4) as pool:
            pending = {key:pool.submit(self.http.request, 'GET', path) for key,path in paths.items()}
            for key, future in pending.items():
                try:
                    result[key] = future.result()
                except SDKError as exc:
                    result[key] = {'error':str(exc)}
        if result['status'].get('ok') is not True:
            raise SDKError(result['status'].get('error', 'Atlas status schema is incompatible'), 503)
        return result

    def navigate(self, point, revision):
        return self.http.request('POST', '/api/maps/points/navigate', {
            'map_name':self.map_name, 'map_revision':revision, 'point_names':[point]})

    def stop(self):
        return self.http.request('POST', '/api/nav/stop', {})
