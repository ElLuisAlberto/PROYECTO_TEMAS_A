"""Robot-profile Cartesian helpers with an exact legacy JACO2 fallback."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from thesis_core.cartesian_kinematics import (
    pose_error,
    solve_inverse_kinematics_transform,
)
from thesis_core.jaco_kinematics import end_effector_transform


@dataclass(frozen=True)
class ProfileIkResult:
    """Common result view used by all historical Cartesian adapters."""

    success: bool
    positions: tuple[float, ...]
    position_error_m: float
    orientation_error_rad: float
    iterations: int
    detail: str = ''


def position_only_is_permitted(profile, orientation_motion_rad: float) -> bool:
    """
    Allow underactuated IK only for a request with no tool rotation.

    P3020 has five actuated axes.  Dropping an explicitly requested
    orientation would be unsafe and misleading, so position-only IK is
    permitted only when the requested orientation motion is numerically zero.
    """
    angle = float(orientation_motion_rad)
    if not math.isfinite(angle) or angle < 0.0:
        raise ValueError('orientation motion must be finite and non-negative')
    return profile.joint_count < 6 and angle <= 1.0e-6


class CartesianProfileModel:
    """Expose FK/IK while retaining the exact historical JACO2 solver."""

    def __init__(self, profile, legacy_jaco2: bool) -> None:
        """Store one validated profile and select its solver backend."""
        self.profile = profile
        self.legacy_jaco2 = bool(legacy_jaco2)

    def forward(self, positions: Sequence[float]) -> np.ndarray:
        """Return the configured tool transform in the model world frame."""
        if self.legacy_jaco2:
            return end_effector_transform(positions)
        return self.profile.forward(positions)[self.profile.tip_link]

    def inverse(
        self,
        seed: Sequence[float],
        target: np.ndarray,
        position_tolerance_m: float,
        orientation_tolerance_rad: float,
        position_only: bool = False,
    ) -> ProfileIkResult:
        """Solve IK and return the historical result contract."""
        if self.legacy_jaco2:
            result = solve_inverse_kinematics_transform(
                seed,
                target,
                position_tolerance_m=position_tolerance_m,
                orientation_tolerance_rad=orientation_tolerance_rad,
            )
            return ProfileIkResult(
                success=result.success,
                positions=tuple(result.positions),
                position_error_m=float(result.position_error_m),
                orientation_error_rad=float(result.orientation_error_rad),
                iterations=int(result.iterations),
                detail=str(result.detail),
            )

        positions = self.profile.inverse(
            target, seed, position_only=position_only
        )
        actual = self.forward(positions)
        position, orientation = pose_error(target, actual)
        position_error = math.sqrt(sum(value * value for value in position))
        orientation_error = math.sqrt(
            sum(value * value for value in orientation)
        )
        success = position_error <= float(position_tolerance_m)
        if not position_only:
            success = (
                success
                and orientation_error <= float(orientation_tolerance_rad)
            )
        return ProfileIkResult(
            success=success,
            positions=tuple(positions),
            position_error_m=position_error,
            orientation_error_rad=orientation_error,
            iterations=0,
            detail=(
                'converged'
                if success
                else 'profile IK did not meet the Cartesian tolerance'
            ),
        )
