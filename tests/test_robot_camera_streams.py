"""Run with Isaac's Python, which supplies NumPy and Pillow."""
import sys
import json
import time
from unittest.mock import Mock
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from robot_camera_streams import RobotCameraStreams


class Camera:
    def __init__(self):
        self.calls = 0
        self.value = 10

    def get_data(self):
        self.calls += 1
        return np.full((360, 640, 4), self.value, dtype=np.uint8)


class CameraStreamTests(unittest.TestCase):
    def test_readback_follows_viewer_lease_without_toggling_hydra(self):
        with tempfile.TemporaryDirectory() as folder:
            stream=RobotCameraStreams.__new__(RobotCameraStreams)
            stream.folder=Path(folder)
            stream.last_render_check=0.
            stream.render_enabled=False
            stream.last_viewers=False
            stream.last_scene=None
            stream.performance=dict(prepare_seconds=0.,render_requests=0)
            product=Mock()
            stream.streams=[dict(product=product)]
            stream.scene_sample=lambda: np.zeros(3)
            self.assertFalse(stream.prepare_render())
            def lease(count,age=0):
                (stream.folder/'viewers.json').write_text(json.dumps(dict(count=count,updated_at=time.time()-age)))
                stream.last_render_check=0.
            lease(1)
            self.assertTrue(stream.prepare_render())
            product.hydra_texture.set_updates_enabled.assert_not_called()
            self.assertFalse(stream.prepare_render())
            product.hydra_texture.set_updates_enabled.assert_not_called()
            stream.last_scene=np.zeros(3)
            lease(1)
            self.assertFalse(stream.prepare_render())
            stream.scene_sample=lambda: np.ones(3)
            lease(1)
            self.assertTrue(stream.prepare_render())
            lease(1,age=10)
            self.assertFalse(stream.prepare_render())
            lease(0)
            self.assertFalse(stream.prepare_render())

    def test_static_scene_skips_readback_encoding_and_versions(self):
        camera = Camera()
        with tempfile.TemporaryDirectory() as folder:
            stream = RobotCameraStreams.__new__(RobotCameraStreams)
            stream.folder = Path(folder)
            stream.session = 'test'
            stream.last_publish = stream.last_heartbeat = stream.sequence = 0
            stream.last_scene = stream.last_states = stream.status = None
            position = np.zeros(3)
            stream.scene_sample = lambda: position.copy()
            stream.streams = [dict(id='Robot_A', annotator=camera, updated_at=0.,
                                  error=None, pixels=None, version=0)]
            stream.publish(0)
            frame = stream.folder / 'Robot_A.jpg'
            timestamp = frame.stat().st_mtime_ns
            for i in range(10):
                stream.last_publish = stream.last_heartbeat = 0
                stream.publish(i + 1)
            self.assertEqual(camera.calls, 1)
            self.assertEqual(frame.stat().st_mtime_ns, timestamp)
            self.assertEqual(stream.sequence, 1)
            position[0] = .1
            camera.value = 20
            stream.last_publish = 0
            stream.publish(12)
            self.assertEqual(camera.calls, 2)
            self.assertEqual(stream.streams[0]['version'], 2)


if __name__ == '__main__':
    unittest.main()
