"""Strict reader for SDK ba0c732 atomic RGB-D, plus lossless replay storage."""
from dataclasses import dataclass
from email import policy
from email.parser import BytesParser
from pathlib import Path
from urllib.request import urlopen
import json
import time
import cv2
import numpy as np

MAX_BODY = 24 * 1024 * 1024
CAMERAS = {'head_rgb', 'left_wrist_rgb', 'right_wrist_rgb'}


@dataclass
class Observation:
    metadata: dict
    rgb: np.ndarray
    depth: np.ndarray
    received_at: float

    @property
    def key(self):
        return self.metadata['camera_id'], self.metadata['pair_id']


def validate(metadata, rgb, depth, *, allow_mock=False):
    m = metadata
    if m.get('schema') != 'luxi_rgbd_v1' or m.get('aligned_to_color') is not True:
        raise ValueError('Only atomic color-aligned luxi_rgbd_v1 is supported')
    if m.get('camera_id') not in CAMERAS or not isinstance(m.get('pair_id'), str) or not m['pair_id']:
        raise ValueError('Invalid camera/pair identity')
    if m.get('mock') is not False and not allow_mock:
        raise ValueError('Real observation required')
    w, h = m['width'], m['height']
    if type(w) is not int or type(h) is not int or not (0 < w <= 1920 and 0 < h <= 1080):
        raise ValueError('Invalid dimensions')
    if rgb.shape != (h,w,3) or rgb.dtype != np.uint8 or depth.shape != (h,w):
        raise ValueError('Pixel shape/type mismatch')
    d=m['depth']
    if (m['color']['encoding'] != 'jpeg' or d['encoding'] != '32FC1'
            or d['unit'] != 'metre' or d['byte_order'] != 'little-endian'
            or d['step'] != w*4 or d['bytes'] != w*h*4):
        raise ValueError('Invalid depth encoding/units/stride')
    if m.get('frame_id') != m['camera_id']+'_color_optical':
        raise ValueError('Unexpected optical frame')
    if m.get('calibration_available') is not True:
        raise ValueError('Calibration unavailable')
    k=np.asarray(m['intrinsics']['K'],dtype=float).reshape(3,3)
    coeff=np.asarray(m['intrinsics']['D'],dtype=float)
    if (not np.isfinite(k).all() or not np.isfinite(coeff).all()
            or k[0,0]<=0 or k[1,1]<=0 or not np.allclose(k[2],[0,0,1])):
        raise ValueError('Invalid camera intrinsics')
    if not np.isfinite(float(m['age_ms'])) or float(m['age_ms'])<0:
        raise ValueError('Invalid source age')


def decode(content_type, body, *, allow_mock=False):
    if len(body)>MAX_BODY:
        raise ValueError('RGBD response exceeds size limit')
    msg=BytesParser(policy=policy.default).parsebytes(
        ('Content-Type: '+content_type+'\r\nMIME-Version: 1.0\r\n\r\n').encode()+body)
    if msg.get_content_type()!='multipart/mixed' or not msg.is_multipart():
        raise ValueError('Expected multipart/mixed')
    parts={}
    types={'metadata':'application/json','color':'image/jpeg','depth':'application/octet-stream'}
    for part in msg.iter_parts():
        name=part.get_param('name',header='content-disposition')
        if name not in types or name in parts or part.get_content_type()!=types[name]:
            raise ValueError('Unexpected/duplicate MIME part')
        parts[name]=part.get_payload(decode=True)
    if set(parts)!=set(types):
        raise ValueError('Incomplete RGBD response')
    m=json.loads(parts['metadata'])
    w,h=m['width'],m['height']
    if type(w) is not int or type(h) is not int or not (0<w<=1920 and 0<h<=1080):
        raise ValueError('Invalid dimensions')
    if len(parts['depth'])!=w*h*4 or len(parts['color'])!=m['color']['bytes']:
        raise ValueError('Payload length mismatch')
    bgr=cv2.imdecode(np.frombuffer(parts['color'],np.uint8),cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError('Invalid JPEG')
    rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
    depth=np.frombuffer(parts['depth'],dtype='<f4').reshape(h,w).copy()
    validate(m,rgb,depth,allow_mock=allow_mock)
    return Observation(m,rgb,depth,time.monotonic())


class RGBDClient:
    """Read-only HTTP. Never probes or submits control endpoints."""
    def __init__(self, base_url, *, max_age_ms=250, timeout=3, on_response=None):
        self.base_url=base_url.rstrip('/')
        self.max_age_ms=max_age_ms
        self.timeout=timeout
        self.last={}
        self.on_response=on_response

    def receive(self,camera):
        if camera not in CAMERAS:
            raise ValueError('Unknown camera')
        start=time.monotonic()
        with urlopen(self.base_url+'/api/cameras/'+camera+'/rgbd',timeout=self.timeout) as response:
            body=response.read(MAX_BODY+1)
            if self.on_response is not None:
                self.on_response(dict(response.headers),body)
            obs=decode(response.headers.get('Content-Type',''),body)
            if response.headers.get('X-Pair-Id')!=obs.metadata['pair_id'] or obs.key[0]!=camera:
                raise ValueError('Response identity mismatch')
        # Conservatively include entire request time, not just server age.
        age=float(obs.metadata['age_ms'])+(time.monotonic()-start)*1000
        if age>self.max_age_ms:
            raise ValueError('RGBD observation is stale: pair=%s age_ms=%.1f max_age_ms=%.1f' %
                             (obs.key[1],age,self.max_age_ms))
        if self.last.get(camera)==obs.key[1]:
            raise ValueError('Duplicate RGBD pair')
        self.last[camera]=obs.key[1]
        obs.metadata=dict(obs.metadata,transport_age_ms=age)
        return obs


def save_observation(path, obs):
    """Exclusive create; preserve original metadata, RGB pixels and float depth."""
    with Path(path).open('xb') as f:
        np.savez_compressed(f,metadata=json.dumps(obs.metadata,ensure_ascii=False),
                            rgb=obs.rgb,depth=obs.depth)


def load_observation(path, *, allow_mock=False):
    with np.load(path,allow_pickle=False) as data:
        m=json.loads(str(data['metadata']))
        rgb,depth=data['rgb'].copy(),data['depth'].copy()
    validate(m,rgb,depth,allow_mock=allow_mock)
    # Replay is explicitly not a fresh observation that may command hardware.
    return Observation(dict(m,replay=True),rgb,depth,time.monotonic())
