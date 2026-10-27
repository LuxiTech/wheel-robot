"""Application contracts shared by navigation, vision and motion adapters.

Adapters return only after measured completion, not after accepting a request.
Observation ages use this process's monotonic clock; remote clocks must be
translated/validated by the receiving adapter before constructing observations.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol


@dataclass(frozen=True)
class TransferJob:
    target_id: str
    source_station: str
    destination_station: str
    destination_slot: str

    def __post_init__(self):
        for value in (self.target_id, self.source_station,
                      self.destination_station, self.destination_slot):
            if not isinstance(value, str) or not value.strip():
                raise ValueError('Job identifiers must be nonempty strings')


@dataclass(frozen=True)
class GraspObservation:
    target_id: str
    pair_id: str
    frame_id: str
    captured_at: float
    quality_valid: bool
    diagnostic_only: bool
    data: dict


@dataclass(frozen=True)
class MotionPlan:
    target_id: str
    pair_id: str
    collision_checked: bool
    execution_ready: bool
    trajectory: Any


class Navigation(Protocol):
    ready: bool

    async def go_to(self, station: str) -> None:
        """Navigate through the configured station provider and confirm stationary.

        The Atlas A/B provider verifies map-goal arrival and physical stop.
        Precision docking, when needed, is a separate commissioned capability.
        """
        ...

    async def stop(self) -> None: ...


class Vision(Protocol):
    ready: bool

    async def observe(self, target_id: str) -> GraspObservation: ...


class Motion(Protocol):
    ready: bool

    async def prepare_transport(self, holding: bool) -> None: ...
    async def plan_grasp(self, observation: GraspObservation) -> MotionPlan: ...
    async def pregrasp(self, plan: MotionPlan) -> None: ...
    async def approach(self, plan: MotionPlan) -> None: ...
    async def close_and_verify(self) -> None: ...
    async def test_lift(self) -> None: ...
    async def place(self, slot: str) -> None:
        """Confirm support, release, verify release, and withdraw the hands."""
        ...
    async def hold(self) -> None:
        """Stop trajectories and retain load; never implicitly release/park."""
        ...
