"""Deterministic short-horizon joint-space prediction."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence, Tuple

from thesis_core.joint_model import (
    CONTINUOUS_JOINTS,
    JOINT_NAMES,
    JOINT_POSITION_LIMITS,
    JOINT_VELOCITY_LIMITS,
    validate_duration,
)


JointVector = Tuple[float, ...]
JointSamples = Tuple[JointVector, ...]


@dataclass(frozen=True)
class JointPrediction:
    """Joint samples and their exact temporal discretization."""

    positions: JointSamples
    sample_times: Tuple[float, ...]
    sample_period: float
    horizon: float
    effective_duration: float


def _validate_positive_finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f'{name} must be positive and finite')
    return result


def _validate_sample_count(samples: int) -> int:
    if isinstance(samples, bool) or not isinstance(samples, int):
        raise ValueError('samples must be an integer')
    if samples < 2:
        raise ValueError('samples must be at least two')
    return samples


def _validate_joint_positions(
    values: Sequence[float], name: str, joint_count: int,
) -> JointVector:
    if len(values) != joint_count:
        raise ValueError(
            f'{name} must contain {joint_count} joint positions'
        )
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f'{name} contains a non-finite position')
    return result


def _validate_model(
    joint_names: Sequence[str],
    position_limits: Mapping[str, Tuple[float, float]],
    velocity_limits: Mapping[str, float],
    continuous_joints: Sequence[str],
) -> Tuple[Tuple[str, ...], frozenset[str]]:
    names = tuple(str(name) for name in joint_names)
    if not names or len(set(names)) != len(names):
        raise ValueError('joint_names must be non-empty and unique')
    continuous = frozenset(str(name) for name in continuous_joints)
    if not continuous.issubset(names):
        raise ValueError('continuous_joints must belong to joint_names')
    for joint_name in names:
        if (
            joint_name not in position_limits
            or joint_name not in velocity_limits
        ):
            raise ValueError(f'missing limits for joint {joint_name}')
        lower, upper = (
            float(value) for value in position_limits[joint_name]
        )
        velocity = float(velocity_limits[joint_name])
        if (
            not math.isfinite(lower)
            or not math.isfinite(upper)
            or lower >= upper
            or not math.isfinite(velocity)
            or velocity <= 0.0
        ):
            raise ValueError(f'invalid limits for joint {joint_name}')
    return names, continuous


def _validate_configuration_limits(
    values: JointVector,
    name: str,
    joint_names: Sequence[str],
    position_limits: Mapping[str, Tuple[float, float]],
) -> None:
    for joint_name, value in zip(joint_names, values):
        lower, upper = position_limits[joint_name]
        if not lower <= value <= upper:
            raise ValueError(
                f'{name}: {joint_name}={value:.6f} rad outside '
                f'[{lower:.6f}, {upper:.6f}] rad'
            )


def predict_joint_samples(
    current: Sequence[float],
    target: Sequence[float],
    duration: float,
    horizon: float,
    samples: int,
    *,
    joint_names: Sequence[str] = JOINT_NAMES,
    position_limits: Mapping[
        str, Tuple[float, float]
    ] = JOINT_POSITION_LIMITS,
    velocity_limits: Mapping[str, float] = JOINT_VELOCITY_LIMITS,
    continuous_joints: Sequence[str] = CONTINUOUS_JOINTS,
) -> JointPrediction:
    """
    Predict a velocity-bounded piecewise-linear joint trajectory.

    The historical JACO2 model remains the default.  Supplying the four
    model arguments applies the same prediction algorithm to another robot
    profile without importing a robot-specific package.
    """
    names, continuous = _validate_model(
        joint_names,
        position_limits,
        velocity_limits,
        continuous_joints,
    )
    current_vector = _validate_joint_positions(
        current, 'current', len(names)
    )
    target_vector = _validate_joint_positions(
        target, 'target', len(names)
    )
    duration = validate_duration(duration)
    horizon = _validate_positive_finite(horizon, 'horizon')
    sample_count = _validate_sample_count(samples)
    _validate_configuration_limits(
        current_vector, 'current', names, position_limits
    )
    _validate_configuration_limits(
        target_vector, 'target', names, position_limits
    )

    normalized_target = tuple(
        current_value + (
            math.atan2(
                math.sin(target_value - current_value),
                math.cos(target_value - current_value),
            )
            if joint_name in continuous
            else target_value - current_value
        )
        for joint_name, current_value, target_value in zip(
            names, current_vector, target_vector
        )
    )
    _validate_configuration_limits(
        normalized_target, 'normalized target', names, position_limits
    )
    deltas = tuple(
        target_value - current_value
        for current_value, target_value in zip(
            current_vector, normalized_target
        )
    )
    effective_duration = max(
        duration,
        *(
            abs(delta) / float(velocity_limits[joint_name])
            for joint_name, delta in zip(names, deltas)
        ),
    )
    sample_period = horizon / float(sample_count - 1)
    sample_times = tuple(
        index * sample_period for index in range(sample_count)
    )
    predicted = []
    for sample_time in sample_times:
        alpha = min(sample_time / effective_duration, 1.0)
        configuration = tuple(
            current_value + alpha * delta
            for current_value, delta in zip(current_vector, deltas)
        )
        _validate_configuration_limits(
            configuration, 'predicted sample', names, position_limits
        )
        predicted.append(configuration)

    return JointPrediction(
        positions=tuple(predicted),
        sample_times=sample_times,
        sample_period=sample_period,
        horizon=horizon,
        effective_duration=effective_duration,
    )


def is_state_fresh(
    received_monotonic: float,
    now_monotonic: float,
    max_state_age_sec: float,
) -> bool:
    """Return whether a received state is valid at the evaluation instant."""
    values = (received_monotonic, now_monotonic, max_state_age_sec)
    if not all(math.isfinite(value) for value in values):
        return False
    age = now_monotonic - received_monotonic
    return max_state_age_sec > 0.0 and 0.0 <= age <= max_state_age_sec
