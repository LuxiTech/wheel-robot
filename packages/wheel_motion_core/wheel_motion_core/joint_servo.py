"""Laptop differential IK and bounded references using the verified SDK/URDF chain.

Feedback is never zeroed, re-signed or used as a substitute for the commanded
reference. Following error uses measured joints; IK integrates the nominal
reference so gravity deflection is not repeatedly integrated into the target.
"""

import time

import numpy as np
from scipy.spatial.transform import Rotation
from wheel_task_core.transforms import pose

ARM_AXES = np.array([*range(7), *range(8, 15)])


class JointReferenceGenerator:
    def __init__(
        self,
        model,
        nominal,
        *,
        speed_rad_s=0.2,
        acceleration_rad_s2=0.5,
        speed_limits=None,
        reference_margin_rad=0.02,
    ):
        if not np.isfinite([speed_rad_s, acceleration_rad_s2]).all():
            raise ValueError("Finite reference bounds required")
        if not 0.02 <= speed_rad_s <= 0.2 or not 0 < acceleration_rad_s2 <= 2:
            raise ValueError("Reference bounds outside commissioned SDK range")
        self.model = model
        self.position = self._vector(nominal)
        self.velocity = np.zeros(16)
        self.speed = speed_rad_s
        self.acceleration = acceleration_rad_s2
        self.lower, self.upper = np.full(16, -np.inf), np.full(16, np.inf)
        self.axis_speed = np.full(16, speed_rad_s)
        if speed_limits is not None:
            limits = self._vector(speed_limits)
            if np.any(limits <= 0):
                raise ValueError("Positive original SDK speed limits required")
            self.axis_speed = np.minimum(self.axis_speed, limits)
        self.chains = {}
        for side, start in (("right", 0), ("left", 8)):
            self.chains[side] = model.urdf.chain(
                f"openarm_{side}_link7", base=f"openarm_{side}_base_link"
            )
            for index in range(7):
                name = f"openarm_{side}_joint{index + 1}"
                self.lower[start + index], self.upper[start + index] = model.joints[name]["bounds"]
                self.axis_speed[start + index] = min(
                    self.axis_speed[start + index], model.joints[name]["velocity"]
                )
        self._in_range(self.position)
        if not np.isfinite(reference_margin_rad) or not 0 <= reference_margin_rad <= 0.2:
            raise ValueError("Invalid joint reference braking margin")
        self.reference_margin_rad = reference_margin_rad
        # 原SDK限位不变。在其内留出动态跟随余量；若起点已在余量区内，允许平滑回撤，
        # 不把实测角强制裁剪，也不允许参考继续向边界推进。
        self.reference_lower = np.minimum(self.lower + reference_margin_rad, self.position)
        self.reference_upper = np.maximum(self.upper - reference_margin_rad, self.position)
        self.measured_errors = {}

    @staticmethod
    def bounded_velocity(jacobian, task_velocity, lower, upper, preference):
        """Redistribute task velocity when an axis saturates; never just clip its component."""
        velocity = np.zeros(7)
        free = np.ones(7, dtype=bool)
        for _ in range(8):
            if not free.any():
                break
            reduced = jacobian[:, free]
            residual = task_velocity - jacobian[:, ~free] @ velocity[~free]
            preferred = preference[free]
            velocity[free] = preferred + reduced.T @ np.linalg.solve(
                reduced @ reduced.T + 0.02**2 * np.eye(6), residual - reduced @ preferred
            )
            clipped = np.clip(velocity, lower, upper)
            excess = np.where(free, np.abs(velocity - clipped), 0)
            if excess.max() <= 1e-12:
                return clipped
            saturated = int(excess.argmax())
            velocity[saturated] = clipped[saturated]
            free[saturated] = False
        return velocity

    @staticmethod
    def _vector(values):
        result = np.asarray(values, dtype=float)
        if result.shape != (16,) or not np.isfinite(result).all():
            raise ValueError("Sixteen finite SDK joint angles required")
        return result.copy()

    def _in_range(self, values):
        if np.any(values < self.lower) or np.any(values > self.upper):
            raise ValueError("Joint reference/feedback outside original limits")

    def geometry(self, side, q16, height_m):
        """Analytic spatial Jacobian, SDK right8+left8, metres and radians."""
        if not np.isfinite(height_m):
            raise ValueError("Finite measured lift required")
        transform = self.model.bases[side].copy()
        transform[2, 3] += height_m
        axes, origins = [], []
        start = 0 if side == "right" else 8
        index = 0
        for joint in self.chains[side]:
            transform = transform @ joint["t"]
            if joint["kind"] == "fixed":
                continue
            if joint["kind"] != "revolute" or joint["mimic"] is not None:
                raise ValueError("Expected seven independent SDK revolute joints")
            axes.append(transform[:3, :3] @ joint["axis"])
            origins.append(transform[:3, 3].copy())
            motion = np.eye(4)
            motion[:3, :3] = Rotation.from_rotvec(joint["axis"] * q16[start + index]).as_matrix()
            transform = transform @ motion
            index += 1
        if index != 7:
            raise ValueError("SDK arm chain must contain seven joints")
        axes = np.asarray(axes)
        jacobian = np.vstack((np.cross(axes, transform[:3, 3] - np.asarray(origins)).T, axes.T))
        return transform, jacobian

    def step(
        self,
        targets,
        measured,
        height_m,
        dt,
        *,
        joint_goal=None,
        measured_velocity=None,
        command_bias=None,
    ):
        if not np.isfinite(dt) or not 0 < dt <= 0.03:
            raise ValueError("Control deadline missed; no catch-up reference jump")
        measured = self._vector(measured)
        measured_velocity = (
            self._vector(measured_velocity) if measured_velocity is not None else np.zeros(16)
        )
        command_bias = self._vector(command_bias) if command_bias is not None else np.zeros(16)
        self._in_range(measured)
        if np.max(np.abs(measured[ARM_AXES] - self.position[ARM_AXES])) > 0.25:
            raise ValueError("Measured nominal following error exceeds 0.25 rad")
        if set(targets) != {"left", "right"}:
            raise ValueError("Bilateral wrist targets required")
        joint_goal = self._vector(joint_goal) if joint_goal is not None else None
        if joint_goal is not None:
            self._in_range(joint_goal)
        next_velocity = np.zeros(16)
        for side, start in (("right", 0), ("left", 8)):
            desired = pose(targets[side])
            actual, _ = self.geometry(side, measured, height_m)
            current, jacobian = self.geometry(side, self.position, height_m)
            # 实测角用于到位与跟随误差；名义参考用于积分，避免将重力下垂反复加进目标。
            self.measured_errors[side] = dict(
                position_m=float(np.linalg.norm(desired[:3, 3] - actual[:3, 3])),
                angle_rad=float(
                    Rotation.from_matrix(desired[:3, :3] @ actual[:3, :3].T).magnitude()
                ),
            )
            error = np.r_[
                desired[:3, 3] - current[:3, 3],
                0.2 * Rotation.from_matrix(desired[:3, :3] @ current[:3, :3].T).as_rotvec(),
            ]
            weighted = jacobian.copy()
            weighted[3:] *= 0.2
            section = slice(start, start + 7)
            distance_lower = np.maximum(0, self.position[section] - self.reference_lower[section])
            distance_upper = np.maximum(0, self.reference_upper[section] - self.position[section])
            # Leave one tick of braking reserve before a joint bound.
            stopping_lower = np.maximum(
                0, np.sqrt(2 * self.acceleration * distance_lower) - self.acceleration * dt
            )
            stopping_upper = np.maximum(
                0, np.sqrt(2 * self.acceleration * distance_upper) - self.acceleration * dt
            )
            lower_velocity = -np.minimum(self.axis_speed[section], stopping_lower)
            upper_velocity = np.minimum(self.axis_speed[section], stopping_upper)
            # 接近真实限位时先降速，避免电机滞后后又快速追上而越界；不改变SDK限位。
            hard_distance = np.minimum(
                self.position[section] - self.lower[section],
                self.upper[section] - self.position[section],
            )
            near_limit_speed = np.maximum(0.005, 0.5 * hard_distance)
            lower_velocity = np.maximum(lower_velocity, -near_limit_speed)
            upper_velocity = np.minimum(upper_velocity, near_limit_speed)
            # 用实测位置/速度提前收住参考：为网络反馈延迟和电机追赶留出时间。
            predicted_upper = np.maximum(
                0,
                self.upper[section]
                - measured[section]
                - np.maximum(0, measured_velocity[section]) * 0.2
                - 0.01,
            )
            predicted_lower = np.maximum(
                0,
                measured[section]
                - self.lower[section]
                - np.maximum(0, -measured_velocity[section]) * 0.2
                - 0.01,
            )
            upper_velocity = np.minimum(upper_velocity, 0.5 * predicted_upper)
            lower_velocity = np.maximum(lower_velocity, -0.5 * predicted_lower)
            # 参考不能在实物停滞时继续积分。把SDK已有补偿一起计入超前量，
            # 在原生0.25rad跟随保护之前减速/回撤，避免用更大的位置差持续顶推。
            lead = self.position[section] + command_bias[section] - measured[section]
            upper_headroom = np.maximum(0, 0.15 - lead)
            lower_headroom = np.maximum(0, 0.15 + lead)
            upper_velocity = np.minimum(
                upper_velocity,
                np.maximum(
                    0, np.sqrt(2 * self.acceleration * upper_headroom) - self.acceleration * dt
                ),
            )
            lower_velocity = np.maximum(
                lower_velocity,
                -np.maximum(
                    0, np.sqrt(2 * self.acceleration * lower_headroom) - self.acceleration * dt
                ),
            )
            upper_velocity = np.minimum(
                upper_velocity,
                np.where(lead > 0.15, -np.minimum(0.05, np.maximum(0, lead - 0.15) * 2), np.inf),
            )
            lower_velocity = np.maximum(
                lower_velocity,
                np.where(lead < -0.15, np.minimum(0.05, np.maximum(0, -lead - 0.15) * 2), -np.inf),
            )
            if np.any(lower_velocity > upper_velocity + 1e-8):
                raise ValueError("Measured tracking and original joint bounds require a hold")
            preference = np.zeros(7)
            if self.reference_margin_rad > 0 and np.linalg.norm(error) > 1e-9:
                band = 2 * self.reference_margin_rad
                preference = 0.04 * (
                    np.maximum(0, 1 - (self.position[section] - self.lower[section]) / band)
                    - np.maximum(0, 1 - (self.upper[section] - self.position[section]) / band)
                )
            # 某轴开始减速后，重新把误差分配给其余关节；避免只裁剪该轴后仍不断顶限位。
            if joint_goal is None:
                velocity = self.bounded_velocity(
                    weighted, 4.0 * error, lower_velocity, upper_velocity, preference
                )
            else:
                # 对准使用低频完整IK选择关节构型，高频只做连续关节跟踪；
                # 下插仍使用雅可比保持腕端路径，不把关节插值说成笛卡尔直线。
                velocity = np.clip(
                    4 * (joint_goal[section] - measured[section]),
                    lower_velocity,
                    upper_velocity,
                )
            next_velocity[section] = self.velocity[section] + np.clip(
                velocity - self.velocity[section],
                -self.acceleration * dt,
                self.acceleration * dt,
            )
        candidate = self.position + next_velocity * dt
        # 新视觉目标不会清零速度历史；先限速/限加速度，再积分成连续关节位置参考。
        self._in_range(candidate)
        self.position, self.velocity = candidate, next_velocity
        return self.position.tolist(), self.velocity.tolist()


class JointPostureResolver:
    """Low-rate endpoint IK outside the 100Hz thread; numerical seeds are not feedback."""

    def __init__(self, model, *, margin_rad=0.02):
        self.model = model
        self.ik = model.ik(position_tolerance_m=0.005)
        self.ik.lower[1:] += margin_rad
        self.ik.upper[1:] -= margin_rad
        self.previous = None
        self.rng = np.random.default_rng(114)

    def solve(self, targets, measured, height_m):
        solve_started = time.monotonic()
        candidates = [self.model.values(measured, height_m)]
        if self.previous is not None:
            candidates.insert(0, self.previous)
        center = (self.ik.lower + self.ik.upper) / 2
        center[0] = height_m
        candidates.append(dict(zip(self.ik.names, center)))
        best = None
        for attempt in range(8):
            seed = candidates[attempt] if attempt < len(candidates) else None
            if seed is None:
                values = self.rng.uniform(self.ik.lower, self.ik.upper)
                values[0] = height_m
                seed = dict(zip(self.ik.names, values))
            solution = self.ik.solve(targets, seed, fixed_lift=height_m, max_nfev=160)
            score = sum(solution.position_errors.values()) + 0.2 * sum(
                solution.rotation_errors.values()
            )
            if best is None or score < best[0]:
                best = (score, solution)
            if solution.success:
                break
        solution = best[1]
        self.previous = solution.joints
        return self.model.sdk_target(solution.joints, measured), dict(
            success=solution.success,
            position_errors=solution.position_errors,
            rotation_errors=solution.rotation_errors,
            solve_wall_s=time.monotonic() - solve_started,
            attempts=attempt + 1,
        )


_posture_worker = None


def initialize_posture_worker(model):
    global _posture_worker
    _posture_worker = JointPostureResolver(model)


def solve_posture_in_worker(targets, measured, height_m):
    return _posture_worker.solve(targets, measured, height_m)


def posture_worker_ready():
    return _posture_worker is not None
