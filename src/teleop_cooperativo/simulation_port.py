"""Simulator boundary kept independent from ROS and Gazebo."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from .command_composer import Twist


@dataclass(frozen=True)
class ToolPose:
    """Tool pose in the fixed world frame."""

    position: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


class SimulationPort(Protocol):
    """Minimum contract required by the mother-laptop runner."""

    @property
    def ready(self) -> bool:
        """Whether commands can be applied."""

    @property
    def pose(self) -> ToolPose:
        """Current finite tool pose."""

    def apply_twist(self, twist: Twist, dt_sec: float) -> ToolPose:
        """Advance the simulation by one bounded time step."""

    def stop(self) -> None:
        """Force the simulator to its safe idle state."""
