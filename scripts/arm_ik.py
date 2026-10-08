"""URDF kinematics and bounded pose IK, independent of Isaac or world truth."""
from dataclasses import dataclass
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def fixed_frame_transform(urdf_path, frame, base='base_footprint'):
    """Nominal calibration from the fixed URDF chain; reject moving mounts."""
    root = ET.parse(urdf_path).getroot()
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    transform = np.eye(4)
    visited = set()
    while frame != base:
        if frame in visited:
            raise ValueError('Cyclic URDF frame chain')
        visited.add(frame)
        joint = joints[frame]
        if joint.get('type') != 'fixed':
            raise ValueError('Frame needs live joint feedback: '+frame)
        origin = joint.find('origin')
        local = np.eye(4)
        if origin is not None:
            local[:3, :3] = Rotation.from_euler('xyz', list(map(float, origin.get('rpy', '0 0 0').split()))).as_matrix()
            local[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
        transform = local @ transform
        frame = joint.find('parent').get('link')
    return transform


@dataclass
class IKResult:
    joints: dict
    position_error: float
    angle_error: float
    success: bool


class ArmIK:
    def __init__(self, urdf_path, side, tcp=None):
        root = ET.parse(urdf_path).getroot()
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain = []
        child = f'openarm_{side}_ee_base_link'
        while child != 'base_footprint':
            j = by_child[child]
            chain.append(j)
            child = j.find('parent').get('link')
        self.chain = []
        self.names = [f'openarm_{side}_joint{i}' for i in range(1, 8)]
        self.limits = []
        for j in reversed(chain):
            origin = j.find('origin')
            t = np.eye(4)
            if origin is not None:
                t[:3, :3] = Rotation.from_euler('xyz', list(map(float, origin.get('rpy', '0 0 0').split()))).as_matrix()
                t[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
            axis = j.find('axis')
            a = np.array(list(map(float, axis.get('xyz').split()))) if axis is not None else np.zeros(3)
            self.chain.append((j.get('name'), j.get('type'), t, a))
            if j.get('name') in self.names:
                lim = j.find('limit')
                self.limits.append((float(lim.get('lower')), float(lim.get('upper'))))
        self.lower, self.upper = np.array(self.limits).T
        # Finger pivot X differs between the original and normalized EE frames.
        pivot = root.find(f"joint[@name='openarm_{side}_finger_joint1']/origin")
        self.tcp = np.asarray(tcp if tcp is not None else
                              (float(pivot.get('xyz').split()[0]), 0.0, -0.17), dtype=float)

    def fk(self, q, lift=0.4, links=False):
        values = dict(zip(self.names, q))
        values['lift_joint'] = lift
        t = np.eye(4)
        points = []
        for name, kind, origin, axis in self.chain:
            t = t @ origin
            motion = np.eye(4)
            if kind == 'prismatic':
                motion[:3, 3] = axis * values.get(name, 0)
            elif kind in ('continuous', 'revolute'):
                motion[:3, :3] = Rotation.from_rotvec(axis * values.get(name, 0)).as_matrix()
            t = t @ motion
            points.append(t[:3, 3].copy())
        end = t.copy()
        end[:3, 3] += t[:3, :3] @ self.tcp
        return (end, np.array(points)) if links else end

    def solve(self, position, rotation, seed, lift=0.4, attempts=5):
        position = np.asarray(position, dtype=float)
        rotation = np.asarray(rotation, dtype=float)
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(rotation)):
            raise ValueError('Nonfinite IK target')
        seed = np.clip(np.asarray(seed, dtype=float), self.lower + 1e-7, self.upper - 1e-7)
        def residual(q):
            t = self.fk(q, lift)
            angle = Rotation.from_matrix(rotation @ t[:3, :3].T).as_rotvec()
            return np.r_[t[:3, 3] - position, 0.18 * angle, 0.0002 * (q - seed)]
        best = None
        rng = np.random.default_rng(42)
        for i in range(attempts):
            start = seed if i == 0 else rng.uniform(self.lower, self.upper)
            fit = least_squares(residual, start, bounds=(self.lower, self.upper), max_nfev=160,
                                ftol=1e-8, xtol=1e-8, gtol=1e-8)
            t = self.fk(fit.x, lift)
            pe = float(np.linalg.norm(t[:3, 3] - position))
            ae = float(np.linalg.norm(Rotation.from_matrix(rotation @ t[:3, :3].T).as_rotvec()))
            result = IKResult(dict(zip(self.names, map(float, fit.x))), pe, ae, pe < 0.008 and ae < 0.08)
            if best is None or pe + .18 * ae < best.position_error + .18 * best.angle_error:
                best = result
            if result.success:
                break
        return best


def squeeze_orientation(side):
    # Local -Z is the fingertip approach axis; X lies along the box face.
    sign = 1 if side == 'left' else -1
    return np.array([[0, sign, 0], [0, 0, sign], [1, 0, 0]], dtype=float)
