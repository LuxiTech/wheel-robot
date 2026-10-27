"""External SDK configuration assembly; no SDK source imports or launch commands."""
from pathlib import Path
import yaml
from wheel_navigation.atlas_client import AtlasClient
from wheel_navigation.service import NavigationService
from wheel_sdk_bridge.lift_client import LiftClient


def create_navigation_service(root, enabled=False):
    root = Path(root)
    config = yaml.safe_load((root/'config/deployments/devices.yaml').read_text(encoding='utf-8'))
    nav = config['navigation']
    if nav['allowed_points'] != ['A','B']:
        raise ValueError('This navigation adapter supports only the configured A/B contract')
    values = [nav[k] for k in ('request_timeout_s','mission_timeout_s','feedback_max_age_s','poll_interval_s','stop_timeout_s')]
    if any(type(v) not in (int,float) or not 0 < v < 10000 for v in values):
        raise ValueError('Navigation timeouts must be finite and positive')
    return NavigationService(
        AtlasClient(config['atlas']['base_url'], config['atlas']['map_name'], nav['request_timeout_s']),
        LiftClient(config['lift']['base_url'], nav['request_timeout_s']),
        root/'.runtime/navigation/commands.json', enabled=enabled,
        mission_timeout=nav['mission_timeout_s'], feedback_max_age=nav['feedback_max_age_s'],
        poll_interval=nav['poll_interval_s'], stop_timeout=nav['stop_timeout_s'],
        station_points=nav['station_points'])
