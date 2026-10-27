"""Read-only lift_robot_ros2 connection and chassis feedback projection."""
from wheel_sdk_bridge.http_client import JSONClient, SDKError


class LiftClient:
    def __init__(self, base_url, timeout=3.0):
        self.http = JSONClient(base_url, timeout)

    def snapshot(self):
        data = self.http.request('GET', '/api/state')
        if not isinstance(data.get('system'), dict) or 'base_status' not in data:
            raise SDKError('lift_robot_ros2 /api/state schema is incompatible')
        if any(data.get(key) is not None and not isinstance(data[key], dict)
               for key in ('base','base_status','arms','lift_status')):
            raise SDKError('lift_robot_ros2 feedback fields must be objects or null')
        return data


def base_ready(data):
    system = data.get('system') or {}
    base = data.get('base') or {}
    status = data.get('base_status') or {}
    return (system.get('mock') is False and (system.get('services') or {}).get('base') is True
            and base.get('valid') is True and status.get('fresh') is True
            and status.get('feedback_fresh') is True and status.get('navigation_cmd_vel') is True
            and status.get('external_control') is not True)


def base_stationary(data):
    return base_ready(data) and (data.get('base_status') or {}).get('stationary') is True
