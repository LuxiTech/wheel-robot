"""Offline planning capability; never converts a plan into execution authority."""
from wheel_motion_core.planning import plan_pair


class OfflinePlanningService:
    def __init__(self, ik, collision_check=None):
        self.ik = ik
        self.collision_check = collision_check

    def plan(self, targets, measured, **options):
        return plan_pair(self.ik, targets, measured,
                         collision_check=self.collision_check, **options)
