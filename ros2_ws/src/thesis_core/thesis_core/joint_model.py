"""Canonical JACO2 joint model and operational motion limits."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Final, Mapping, Sequence, Tuple, cast


JointVector = Tuple[float, ...]

JOINT_NAMES: Final[Tuple[str, ...]] = tuple(
    f'j2n6s300_joint_{index}' for index in range(1, 7)
)
JOINT_COUNT: Final[int] = len(JOINT_NAMES)

# Application-space position bounds.  The URDF retains the manufacturer's
# physical model; these values are the canonical limits used by thesis nodes.
JOINT_POSITION_LIMITS: Final[Mapping[str, Tuple[float, float]]] = {
    JOINT_NAMES[0]: (-2.0 * math.pi, 2.0 * math.pi),
    JOINT_NAMES[1]: (math.radians(47.0), math.radians(313.0)),
    JOINT_NAMES[2]: (math.radians(19.0), math.radians(341.0)),
    JOINT_NAMES[3]: (-2.0 * math.pi, 2.0 * math.pi),
    JOINT_NAMES[4]: (-2.0 * math.pi, 2.0 * math.pi),
    JOINT_NAMES[5]: (-2.0 * math.pi, 2.0 * math.pi),
}

CONTINUOUS_JOINT_INDEXES: Final[Tuple[int, ...]] = (0, 3, 4, 5)
CONTINUOUS_JOINTS: Final[frozenset[str]] = frozenset(
    JOINT_NAMES[index] for index in CONTINUOUS_JOINT_INDEXES
)

# Deliberately conservative operational limits for this thesis.  The JACO2
# URDF keeps the nominal physical limits (36/48 deg/s), while every command
# path in the application is bounded here to 18/24 deg/s.
JOINT_VELOCITY_LIMITS: Final[Mapping[str, float]] = {
    JOINT_NAMES[0]: math.radians(18.0),
    JOINT_NAMES[1]: math.radians(18.0),
    JOINT_NAMES[2]: math.radians(18.0),
    JOINT_NAMES[3]: math.radians(24.0),
    JOINT_NAMES[4]: math.radians(24.0),
    JOINT_NAMES[5]: math.radians(24.0),
}


@dataclass(frozen=True)
class SaturationResult:
    """Result of applying operational velocity and position bounds."""

    positions: JointVector
    requested_velocities: JointVector
    applied_velocities: JointVector
    limiting_joint: str
    requested_velocity: float
    limited_velocity: float
    was_limited: bool


def validate_joint_names(names: Sequence[str]) -> Tuple[str, ...]:
    """Return canonical names or raise for any order/size mismatch."""
    result = tuple(names)
    if result != JOINT_NAMES:
        raise ValueError('expected six arm joints in canonical order')
    return result


def validate_joint_positions(
    values: Sequence[float],
    name: str = 'positions',
) -> JointVector:
    """Return one finite six-joint vector."""
    if len(values) != JOINT_COUNT:
        raise ValueError(f'{name} must contain six joint positions')
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f'{name} contains a non-finite position')
    return cast(JointVector, result)


def validate_duration(duration: float) -> float:
    """Return a finite positive duration."""
    result = float(duration)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError('duration must be positive and finite')
    return result


def validate_position_limits(values: Sequence[float]) -> JointVector:
    """Return positions or raise for the first application-limit violation."""
    result = validate_joint_positions(values)
    for joint_name, position in zip(JOINT_NAMES, result):
        lower, upper = JOINT_POSITION_LIMITS[joint_name]
        if not lower <= position <= upper:
            raise ValueError(
                f'{joint_name}={position:.6f} rad outside '
                f'[{lower:.6f}, {upper:.6f}] rad'
            )
    return result


def shortest_joint_delta(
    joint_name: str,
    target_position: float,
    current_position: float,
    *,
    position_limits: Mapping[
        str, Tuple[float, float]
    ] = JOINT_POSITION_LIMITS,
    continuous_joints: Sequence[str] = CONTINUOUS_JOINTS,
) -> float:
    """Return target-current with the shortest continuous-joint turn."""
    if joint_name not in position_limits:
        raise ValueError(f'unknown joint: {joint_name}')
    if (
        not math.isfinite(target_position)
        or not math.isfinite(current_position)
    ):
        raise ValueError('joint positions must be finite')
    delta = float(target_position) - float(current_position)
    if joint_name in continuous_joints:
        return math.atan2(math.sin(delta), math.cos(delta))
    return delta


def normalize_target(
    start: Sequence[float],
    target: Sequence[float],
    *,
    joint_names: Sequence[str] = JOINT_NAMES,
    position_limits: Mapping[
        str, Tuple[float, float]
    ] = JOINT_POSITION_LIMITS,
    continuous_joints: Sequence[str] = CONTINUOUS_JOINTS,
) -> JointVector:
    """Represent a target using the shortest continuous-joint turns."""
    names = tuple(str(name) for name in joint_names)
    if not names or len(set(names)) != len(names):
        raise ValueError('joint_names must be non-empty and unique')
    if len(start) != len(names) or len(target) != len(names):
        raise ValueError('start and target must match the joint count')
    start_vector = tuple(float(value) for value in start)
    target_vector = tuple(float(value) for value in target)
    if not all(math.isfinite(value) for value in start_vector):
        raise ValueError('start contains a non-finite position')
    if not all(math.isfinite(value) for value in target_vector):
        raise ValueError('target contains a non-finite position')
    return cast(JointVector, tuple(
        current + shortest_joint_delta(
            joint_name,
            requested,
            current,
            position_limits=position_limits,
            continuous_joints=continuous_joints,
        )
        for joint_name, current, requested in zip(
            names, start_vector, target_vector
        )
    ))


def saturate_target_by_velocity(
    current: Sequence[float],
    requested: Sequence[float],
    duration: float,
    *,
    joint_names: Sequence[str] = JOINT_NAMES,
    position_limits: Mapping[
        str, Tuple[float, float]
    ] = JOINT_POSITION_LIMITS,
    velocity_limits: Mapping[str, float] = JOINT_VELOCITY_LIMITS,
    continuous_joints: Sequence[str] = CONTINUOUS_JOINTS,
) -> SaturationResult:
    """
    Bound a target by configured velocity and position limits.

    Omitting the model arguments preserves the canonical JACO2 behaviour.
    """
    names = tuple(str(name) for name in joint_names)
    if not names or len(set(names)) != len(names):
        raise ValueError('joint_names must be non-empty and unique')
    if len(current) != len(names) or len(requested) != len(names):
        raise ValueError(
            'current and requested must match the configured joint count'
        )
    current_vector = tuple(float(value) for value in current)
    requested_vector = tuple(float(value) for value in requested)
    if not all(math.isfinite(value) for value in current_vector):
        raise ValueError('current contains a non-finite position')
    if not all(math.isfinite(value) for value in requested_vector):
        raise ValueError('requested contains a non-finite position')
    duration = validate_duration(duration)

    continuous = frozenset(str(name) for name in continuous_joints)
    if not continuous.issubset(names):
        raise ValueError('continuous_joints must belong to joint_names')
    for name in names:
        if name not in position_limits or name not in velocity_limits:
            raise ValueError(f'missing limits for joint {name}')
        lower, upper = position_limits[name]
        velocity = float(velocity_limits[name])
        if (
            not math.isfinite(float(lower))
            or not math.isfinite(float(upper))
            or float(lower) >= float(upper)
            or not math.isfinite(velocity)
            or velocity <= 0.0
        ):
            raise ValueError(f'invalid limits for joint {name}')

    bounded = []
    requested_velocities = []
    applied_velocities = []
    ratios = []
    was_limited = False

    for joint_name, current_value, requested_value in zip(
        names, current_vector, requested_vector
    ):
        delta = shortest_joint_delta(
            joint_name,
            requested_value,
            current_value,
            position_limits=position_limits,
            continuous_joints=continuous,
        )
        requested_velocity = delta / duration
        velocity_limit = float(velocity_limits[joint_name])
        applied_velocity = min(
            max(requested_velocity, -velocity_limit), velocity_limit
        )
        bounded_value = current_value + applied_velocity * duration
        lower, upper = position_limits[joint_name]
        bounded_value = min(max(bounded_value, lower), upper)
        applied_velocity = (bounded_value - current_value) / duration

        limited = not math.isclose(
            applied_velocity, requested_velocity, rel_tol=0.0, abs_tol=1.0e-12
        )
        was_limited = was_limited or limited
        bounded.append(bounded_value)
        requested_velocities.append(requested_velocity)
        applied_velocities.append(applied_velocity)
        ratios.append(abs(requested_velocity) / velocity_limit)

    limiting_index = max(range(len(names)), key=ratios.__getitem__)
    return SaturationResult(
        positions=cast(JointVector, tuple(bounded)),
        requested_velocities=cast(
            JointVector, tuple(requested_velocities)
        ),
        applied_velocities=cast(JointVector, tuple(applied_velocities)),
        limiting_joint=names[limiting_index],
        requested_velocity=requested_velocities[limiting_index],
        limited_velocity=applied_velocities[limiting_index],
        was_limited=was_limited,
    )
