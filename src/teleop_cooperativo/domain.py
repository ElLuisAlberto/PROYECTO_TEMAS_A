"""Shared types and validation for cooperative Cartesian control."""

from __future__ import annotations

from enum import Enum
import math
from typing import Iterable, Mapping


class Axis(str, Enum):
    """One independently assignable Cartesian velocity component."""

    X = "X"
    Y = "Y"
    Z = "Z"
    ROLL = "ROLL"
    PITCH = "PITCH"
    YAW = "YAW"

    @classmethod
    def parse(cls, value: "Axis | str") -> "Axis":
        """Convert an axis label to its canonical enum value."""
        if isinstance(value, cls):
            return value
        if not isinstance(value, str):
            raise ValueError("axis must be a string")
        try:
            return cls(value.upper())
        except ValueError as error:
            raise ValueError(f"unknown axis: {value!r}") from error


AXES = tuple(Axis)
LINEAR_AXES = (Axis.X, Axis.Y, Axis.Z)
ANGULAR_AXES = (Axis.ROLL, Axis.PITCH, Axis.YAW)


def canonical_axes(values: Iterable[Axis | str]) -> tuple[Axis, ...]:
    """Validate and preserve a unique ordered axis selection."""
    axes = tuple(Axis.parse(value) for value in values)
    if len(set(axes)) != len(axes):
        raise ValueError("an axis may only appear once in a request")
    return axes


def normalized_value(value: object) -> float:
    """Return one finite normalized command in the inclusive range [-1, 1]."""
    if isinstance(value, bool):
        raise ValueError("control value must be numeric, not boolean")
    try:
        numeric = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError("control value must be numeric") from error
    if not math.isfinite(numeric):
        raise ValueError("control value must be finite")
    if not -1.0 <= numeric <= 1.0:
        raise ValueError("control value must be between -1 and 1")
    return numeric


def canonical_values(
    values: Mapping[Axis | str, object],
) -> dict[Axis, float]:
    """Validate a mapping of normalized inputs keyed by Cartesian axis."""
    if not isinstance(values, Mapping) or not values:
        raise ValueError("input values must be a non-empty mapping")
    normalized: dict[Axis, float] = {}
    for raw_axis, raw_value in values.items():
        axis = Axis.parse(raw_axis)
        if axis in normalized:
            raise ValueError(f"duplicated input axis: {axis.value}")
        normalized[axis] = normalized_value(raw_value)
    return normalized
