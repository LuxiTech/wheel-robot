"""Body-frame omni IK and rate-limited drive targets (no root pose/velocity forcing)."""

import json
import math
from common import ROOT


def load_robot_data():
    cfg = json.loads((ROOT / "config/robot_config.json").read_text())
    meta = json.loads(
        (ROOT / "assets/robots/generated/robot_metadata.json").read_text()
    )
    if meta["source_urdf"] != cfg["source_urdf"] or meta.get("home") != cfg["home"]:
        raise RuntimeError("Robot source/home changed: run ./scripts/simctl build-robot first")
    return cfg, meta


class OmniKinematics:
    def __init__(self, wheels, max_linear=1.0, max_angular=1.5):
        self.wheels = wheels
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.matrix = []
        for w in wheels:
            ax, ay, _ = w["axis"]
            x, y, _ = w["position"]
            r = w["radius"]
            # Ground velocity caused by axle rotation is opposite to a x z.
            tx, ty = ay, -ax
            self.matrix.append([tx / r, ty / r, (-tx * y + ty * x) / r])

    def inverse(self, twist):
        if not all(math.isfinite(x) for x in twist):
            raise ValueError("Nonfinite twist")
        vx, vy, w = twist
        speed = math.hypot(vx, vy)
        scale = min(1, self.max_linear / max(speed, 1e-12))
        v = [vx * scale, vy * scale, max(-self.max_angular, min(self.max_angular, w))]
        q = [sum(a * b for a, b in zip(row, v)) for row in self.matrix]
        scale = min(
            [1]
            + [
                wheel["max_velocity"] / max(abs(rate), 1e-12)
                for wheel, rate in zip(self.wheels, q)
            ]
        )
        return [rate * scale for rate in q]

    def forward(self, rates):
        # Small pivoted linear solve; usable in tests without NumPy/Isaac.
        a = [row[:] + [b] for row, b in zip(self.matrix, rates)]
        for i in range(3):
            pivot = max(range(i, 3), key=lambda k: abs(a[k][i]))
            a[i], a[pivot] = a[pivot], a[i]
            div = a[i][i]
            if abs(div) < 1e-10:
                raise ValueError("Singular wheel geometry")
            a[i] = [v / div for v in a[i]]
            for j in range(3):
                if j != i:
                    f = a[j][i]
                    a[j] = [x - f * y for x, y in zip(a[j], a[i])]
        return [row[3] for row in a]


class RobotControl:
    def __init__(self, stage, root="/World/Robot", *, cfg=None, meta=None):
        from pxr import UsdPhysics

        if (cfg is None) != (meta is None):
            raise ValueError("Supply both robot cfg and metadata, or neither")
        self.cfg, self.meta = load_robot_data() if cfg is None else (cfg, meta)
        self.ik = OmniKinematics(
            self.meta["wheels"],
            self.cfg["base"]["max_linear_speed"],
            self.cfg["base"]["max_angular_speed"],
        )
        self.specs = {s["name"]: s for s in self.meta["actuated_joints"]}
        self.drives = {
            name: UsdPhysics.DriveAPI(
                stage.GetPrimAtPath(root + s["path"]),
                "linear" if s["kind"] == "lift" else "angular",
            )
            for name, s in self.specs.items()
        }
        self.requested = [0.0, 0.0, 0.0]
        self.actual = [0.0, 0.0, 0.0]
        self.remaining = None
        self.targets = {
            name: self.cfg["home"].get(name, 0.0)
            for name, s in self.specs.items()
            if s["kind"] != "wheel"
        }
        self.sent = self.targets.copy()
        self.written_targets = {}

    def write_drive(self, name, value, velocity=False):
        # Reauthoring unchanged USD attributes invalidates the stage every
        # physics tick. The drive retains its target until it changes.
        key = (name, velocity)
        if self.written_targets.get(key) == value:
            return
        drive = self.drives[name]
        attr = drive.GetTargetVelocityAttr() if velocity else drive.GetTargetPositionAttr()
        attr.Set(value)
        self.written_targets[key] = value

    def set_twist(self, vx, vy, wz, duration=None):
        if not all(math.isfinite(v) for v in [vx, vy, wz]):
            raise ValueError("Invalid twist")
        self.requested = self.ik.forward(self.ik.inverse([vx, vy, wz]))
        self.remaining = duration

    def stop(self):
        self.requested = [0.0, 0.0, 0.0]
        self.actual = [0.0, 0.0, 0.0]
        self.remaining = None
        for w in self.meta["wheels"]:
            self.write_drive(w["joint"], 0.0, velocity=True)

    def set_joint(self, name, position):
        s = self.specs[name]
        if s["kind"] == "wheel" or not math.isfinite(position):
            raise ValueError("Invalid joint target")
        self.targets[name] = max(s["lower"], min(s["upper"], position))

    def gripper(self, side, opened):
        name = f"openarm_{side}_finger_joint1"
        self.set_joint(
            name, (math.pi / 4 if side == "left" else -math.pi / 4) if opened else 0.0
        )

    def home(self):
        self.stop()
        for name in self.targets:
            self.targets[name] = self.cfg["home"].get(name, 0.0)

    def update(self, dt):
        if dt <= 0 or not math.isfinite(dt):
            return
        if self.remaining is not None:
            self.remaining -= dt
            if self.remaining <= 0:
                self.requested = [0.0, 0.0, 0.0]
                self.remaining = None
        dx = self.requested[0] - self.actual[0]
        dy = self.requested[1] - self.actual[1]
        scale = min(
            1.0,
            self.cfg["base"]["max_linear_acceleration"]
            * dt
            / max(math.hypot(dx, dy), 1e-12),
        )
        self.actual[0] += dx * scale
        self.actual[1] += dy * scale
        dw = self.requested[2] - self.actual[2]
        limit = self.cfg["base"]["max_angular_acceleration"] * dt
        self.actual[2] += max(-limit, min(limit, dw))
        for w, rate in zip(self.meta["wheels"], self.ik.inverse(self.actual)):
            self.write_drive(w["joint"], math.degrees(rate), velocity=True)
        for name, target in self.targets.items():
            s = self.specs[name]
            kind = s["kind"]
            limit = min(s["velocity"], self.cfg[kind]["command_velocity_limit"]) * dt
            delta = max(-limit, min(limit, target - self.sent[name]))
            self.sent[name] += delta
            value = self.sent[name] if kind == "lift" else math.degrees(self.sent[name])
            self.write_drive(name, value)
