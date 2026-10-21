"""Robot-mounted RGB cameras publishing atomic JPEGs for the local console."""
import io
import json
import os
import time
import uuid

import numpy as np
from PIL import Image

from common import OUTPUTS, ROOT
from navigation_console_ipc import atomic_json


class RobotCameraStreams:
    def __init__(self, stage, robots, metadata, bodies=()):
        import omni.replicator.core as rep
        from pxr import Gf, UsdGeom

        self.folder = OUTPUTS / 'robot_cameras'
        self.folder.mkdir(exist_ok=True)
        self.session = uuid.uuid4().hex
        self.last_publish = 0.
        self.last_render_check = 0.
        self.render_requested = False
        self.render_enabled = False
        self.last_viewers = False
        self.performance = dict(render_requests=0, readback_seconds=0., encode_write_seconds=0., prepare_seconds=0.)
        self.sequence = 0
        self.robots = robots
        self.bodies = list(bodies)
        self.last_scene = None
        self.last_states = None
        self.last_heartbeat = 0.
        self.status = None
        self.streams = []
        calibration = json.loads((ROOT/'config/grasp/runtime.sim.json').read_text())['simulation']['camera']
        width, height = calibration['width'], calibration['height']
        self.image_shape = (height, width)
        for name in robots:
            root = '/World/Robots/' + name + metadata['frames']['head_d455_color_optical_frame']
            camera = UsdGeom.Camera.Define(stage, root + '/ConsoleCamera')
            # Match the URDF RGB-D perception camera, including its field of view.
            camera.AddRotateXOp().Set(180.)
            camera.CreateFocalLengthAttr(24.)
            camera.CreateHorizontalApertureAttr(24*width/calibration['fx'])
            camera.CreateVerticalApertureAttr(24*height/calibration['fy'])
            camera.CreateClippingRangeAttr(Gf.Vec2f(.03, 30.))
            product = rep.create.render_product(str(camera.GetPath()), (width, height))
            annotator = rep.AnnotatorRegistry.get_annotator('rgb')
            annotator.attach([product])
            self.streams.append(dict(id=name, product=product, annotator=annotator,
                                     updated_at=0., error=None, pixels=None, version=0))

    def prepare_render(self):
        """Gate readback only while dynamic Hydra toggling is under investigation."""
        started=time.perf_counter()
        self.render_requested=False
        now=time.time()
        if now-self.last_render_check >= .2:
            self.last_render_check=now
            try:
                lease=json.loads((self.folder/'viewers.json').read_text())
                viewers=lease['count']>0 and 0<=now-lease['updated_at']<4.
            except (OSError, ValueError, KeyError, TypeError):
                viewers=False
            if viewers:
                sample=self.scene_sample()
                self.render_requested=(not self.last_viewers or self.last_scene is None or
                    not np.allclose(sample,self.last_scene,atol=1e-4,rtol=0))
            self.last_viewers=viewers
        # Do not toggle live Hydra products: repeated native RTX crashes were
        # observed after enabling that optimization. Keep their lifecycle stable.
        self.performance['prepare_seconds']+=time.perf_counter()-started
        self.performance['render_requests']+=int(self.render_requested)
        return self.render_requested

    def scene_sample(self):
        values = []
        for robot in self.robots.values():
            position, orientation = robot.get_world_pose()
            values.extend(position)
            values.extend(orientation)
            values.extend(robot.get_joint_positions())
        for body in self.bodies:
            position, orientation = body.get_world_pose()
            values.extend(position)
            values.extend(orientation)
        return np.asarray(values)

    def publish(self, simulation_seconds, runtime=None, operator=None, read_images=True):
        now = time.time()
        if not read_images and now - self.last_publish < .2:
            return
        self.last_publish = now
        sample = self.scene_sample() if read_images else self.last_scene
        states = dict(runtime.states) if runtime else {}
        # Ignore sub-0.1 mm / 0.0001 rad physics jitter. Compare against the
        # last published pose, so slow accumulated motion still triggers frames.
        changed = (self.status is None or states != self.last_states or
                   (read_images and (self.last_scene is None or
                    not np.allclose(sample, self.last_scene, atol=1e-4, rtol=0))))
        if not changed and self.status is not None:
            operator_changed = self.status.get('operator') != operator
            if now - self.last_heartbeat >= 2. or operator_changed:
                self.status['updated_at'] = now
                self.status['operator'] = operator
                atomic_json(self.folder / 'status.json', self.status)
                self.last_heartbeat = now
            return
        channels = []
        for stream in self.streams:
            try:
                if not read_images:
                    raise StopIteration
                started=time.perf_counter()
                pixels = np.asarray(stream['annotator'].get_data())
                if hasattr(self,'performance'):
                    self.performance['readback_seconds']+=time.perf_counter()-started
                started=time.perf_counter()
                if pixels.ndim != 3 or pixels.shape[:2] != self.image_shape:
                    raise ValueError('等待相机首帧')
                pixels = pixels[:, :, :3].astype(np.uint8)
                if stream['pixels'] is None or not np.array_equal(pixels, stream['pixels']):
                    buffer = io.BytesIO()
                    Image.fromarray(pixels).save(buffer, format='JPEG', quality=82)
                    target = self.folder / (stream['id'] + '.jpg')
                    temporary = target.with_suffix('.tmp')
                    temporary.write_bytes(buffer.getvalue())
                    os.replace(temporary, target)
                    stream['pixels'] = pixels.copy()
                    stream['version'] += 1
                    stream['updated_at'] = now
                stream['error'] = None
                if hasattr(self,'performance'):
                    self.performance['encode_write_seconds']+=time.perf_counter()-started
            except StopIteration:
                pass
            except Exception as exc:
                stream['error'] = str(exc)
            key = stream['id'].removeprefix('Robot_')
            channels.append(dict(id=stream['id'], updated_at=stream['updated_at'],
                version=stream['version'], error=stream['error'],
                state=states.get(key, '待机') if runtime else '手动控制',
                url='/frames/' + stream['id'] + '.jpg'))
        if read_images and not any(channel['error'] for channel in channels):
            self.last_scene = sample.copy()
            self.last_states = states
        self.last_states = states
        self.sequence += 1
        self.status = dict(session=self.session, updated_at=now, sequence=self.sequence,
                           simulation_seconds=simulation_seconds, cameras=channels, operator=operator)
        atomic_json(self.folder / 'status.json', self.status)
        self.last_heartbeat = now
