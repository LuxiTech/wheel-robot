"""Controller-independent placement evaluator. Unknown release/contact fails closed."""

import math
from common import table_by_id


class PlacementTask:
    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.stable_seconds = 0.0
        self.success = False

    def update(
        self,
        *,
        position,
        half_extents,
        linear_velocity,
        angular_velocity,
        released,
        supported,
        dt,
    ):
        t = table_by_id(self.config, self.config["task"]["target"])
        rule = self.config["task"]
        numbers = [*position, *half_extents, *linear_velocity, *angular_velocity, dt]
        valid = all(math.isfinite(float(v)) for v in numbers) and dt > 0
        inside = all(
            abs(position[i] - t["xy"][i]) + half_extents[i] <= t["size"][i] / 2 - 0.02
            for i in [0, 1]
        )
        on_surface = (
            abs(position[2] - half_extents[2] - t["size"][2])
            <= rule["height_tolerance"]
        )
        slow = math.sqrt(sum(v * v for v in linear_velocity)) < rule["linear_speed_max"]
        slow &= (
            math.sqrt(sum(v * v for v in angular_velocity)) < rule["angular_speed_max"]
        )
        eligible = (
            valid
            and inside
            and on_surface
            and slow
            and released is True
            and supported is True
        )
        self.stable_seconds = self.stable_seconds + dt if eligible else 0.0
        self.success = self.stable_seconds >= rule["hold_seconds"]
        return self.success
