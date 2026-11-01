"""Deterministic Cartesian harness used before the Gazebo digital twin."""

from __future__ import annotations

import math

from .command_composer import Twist
from .simulation_port import ToolPose


class SimulationFault(RuntimeError):
    """Raised when a simulated command cannot be applied safely."""


class DeterministicToolSimulator:
    """Integrate a world-frame twist with workspace and time-step guards."""

    def __init__(
        self,
        *,
        initial_pose: ToolPose | None = None,
        workspace_min: tuple[float, float, float] = (-0.80, -0.80, 0.05),
        workspace_max: tuple[float, float, float] = (0.80, 0.80, 1.20),
        maximum_step_sec: float = 0.10,
    ) -> None:
        self._workspace_min = _finite_vector(workspace_min, "workspace_min")
        self._workspace_max = _finite_vector(workspace_max, "workspace_max")
        if any(
            lower >= upper
            for lower, upper in zip(self._workspace_min, self._workspace_max)
        ):
            raise ValueError("workspace bounds must be strictly increasing")
        self.maximum_step_sec = _positive_finite(
            maximum_step_sec, "maximum_step_sec"
        )
        start = initial_pose or ToolPose(
            position=(0.0, 0.0, 0.60),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
        self._pose = _validated_pose(start)
        self._assert_inside_workspace(self._pose.position)
        self._ready = True
        self._last_twist = _zero_twist()

    @property
    def ready(self) -> bool:
        return self._ready

    @property
    def pose(self) -> ToolPose:
        return self._pose

    @property
    def last_twist(self) -> Twist:
        return self._last_twist

    def apply_twist(self, twist: Twist, dt_sec: float) -> ToolPose:
        """Apply one atomic integration step or preserve the previous pose."""
        if not self._ready:
            raise SimulationFault("simulator is not ready")
        dt = _positive_finite(dt_sec, "dt_sec")
        if dt > self.maximum_step_sec:
            raise SimulationFault("simulation step exceeds maximum_step_sec")
        linear = _finite_vector(twist.linear, "linear twist")
        angular = _finite_vector(twist.angular, "angular twist")
        target_position = tuple(
            position + velocity * dt
            for position, velocity in zip(self._pose.position, linear)
        )
        self._assert_inside_workspace(target_position)
        target_orientation = _integrate_world_angular_velocity(
            self._pose.quaternion_xyzw, angular, dt
        )
        self._pose = ToolPose(target_position, target_orientation)
        self._last_twist = Twist(linear=linear, angular=angular)
        return self._pose

    def stop(self) -> None:
        self._last_twist = _zero_twist()

    def set_ready(self, ready: bool) -> None:
        self._ready = bool(ready)
        if not self._ready:
            self.stop()

    def _assert_inside_workspace(
        self, position: tuple[float, float, float]
    ) -> None:
        if any(
            value < lower or value > upper
            for value, lower, upper in zip(
                position, self._workspace_min, self._workspace_max
            )
        ):
            raise SimulationFault("target pose leaves the configured workspace")


def _integrate_world_angular_velocity(
    quaternion: tuple[float, float, float, float],
    angular: tuple[float, float, float],
    dt_sec: float,
) -> tuple[float, float, float, float]:
    magnitude = math.sqrt(sum(value * value for value in angular))
    if magnitude <= 1.0e-15:
        return quaternion
    angle = magnitude * dt_sec
    half = angle * 0.5
    scale = math.sin(half) / magnitude
    delta = (
        angular[0] * scale,
        angular[1] * scale,
        angular[2] * scale,
        math.cos(half),
    )
    return _normalize_quaternion(_multiply_quaternion(delta, quaternion))


def _multiply_quaternion(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _normalize_quaternion(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm <= 1.0e-15:
        raise SimulationFault("orientation quaternion is invalid")
    return tuple(value / norm for value in quaternion)


def _validated_pose(pose: ToolPose) -> ToolPose:
    return ToolPose(
        _finite_vector(pose.position, "position"),
        _normalize_quaternion(
            _finite_quaternion(pose.quaternion_xyzw, "quaternion")
        ),
    )


def _finite_vector(
    values: tuple[float, float, float], label: str
) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError(f"{label} must contain three values")
    vector = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in vector):
        raise ValueError(f"{label} must be finite")
    return vector


def _finite_quaternion(
    values: tuple[float, float, float, float], label: str
) -> tuple[float, float, float, float]:
    if len(values) != 4:
        raise ValueError(f"{label} must contain four values")
    quaternion = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in quaternion):
        raise ValueError(f"{label} must be finite")
    return quaternion


def _positive_finite(value: object, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be positive and finite")
    try:
        numeric = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} must be positive and finite") from error
    if not math.isfinite(numeric) or numeric <= 0.0:
        raise ValueError(f"{label} must be positive and finite")
    return numeric


def _zero_twist() -> Twist:
    return Twist((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
