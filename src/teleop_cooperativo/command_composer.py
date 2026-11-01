"""Safe composition of Cartesian velocity components."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

from .domain import ANGULAR_AXES, LINEAR_AXES, Axis, canonical_values


@dataclass(frozen=True)
class Twist:
    """One bounded Cartesian velocity vector in the fixed world frame."""

    linear: tuple[float, float, float]
    angular: tuple[float, float, float]


def compose_twist(
    values: Mapping[Axis | str, object],
    *,
    max_linear_speed_mps: float = 0.05,
    max_angular_speed_radps: float = 0.175,
) -> Twist:
    """Convert normalized axes to a bounded resultant Cartesian twist."""
    linear_limit = _positive_finite(max_linear_speed_mps, "linear speed limit")
    angular_limit = _positive_finite(
        max_angular_speed_radps, "angular speed limit"
    )
    normalized = canonical_values(values)
    linear = tuple(normalized.get(axis, 0.0) * linear_limit for axis in LINEAR_AXES)
    angular = tuple(
        normalized.get(axis, 0.0) * angular_limit for axis in ANGULAR_AXES
    )
    return Twist(
        linear=_limit_norm(linear, linear_limit),
        angular=_limit_norm(angular, angular_limit),
    )


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


def _limit_norm(
    values: tuple[float, float, float], limit: float
) -> tuple[float, float, float]:
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= limit:
        return values
    scale = limit / norm
    return tuple(value * scale for value in values)
