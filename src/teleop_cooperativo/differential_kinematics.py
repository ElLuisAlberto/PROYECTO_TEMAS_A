"""Differential kinematics for the six-DOF Gazebo integration substitute.

This module deliberately has no ROS, Gazebo, or third-party numerical
dependency.  It is the checked mathematical boundary used by the future ROS
adapter; it must not be described as a Jaco2 model.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .command_composer import Twist
from .simulation_port import ToolPose


JOINT_COUNT = 6
JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, JOINT_COUNT + 1))
JOINT_LIMITS = (
    (-2.90, 2.90),
    (-2.00, 2.00),
    (-2.60, 2.60),
    (-2.90, 2.90),
    (-2.00, 2.00),
    (-2.90, 2.90),
)
JOINT_VELOCITY_LIMITS = (1.0, 1.0, 1.0, 1.2, 1.2, 1.5)

_JOINT_ORIGINS = (
    (0.0, 0.0, 0.15),
    (0.0, 0.0, 0.16),
    (0.0, 0.0, 0.36),
    (0.0, 0.0, 0.30),
    (0.0, 0.0, 0.14),
    (0.0, 0.0, 0.12),
)
_JOINT_AXES = (
    (0.0, 0.0, 1.0),
    (0.0, 1.0, 0.0),
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)
_TOOL_OFFSET = (0.0, 0.0, 0.10)
_IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

Vector = tuple[float, float, float]
Matrix3 = tuple[Vector, Vector, Vector]
Jacobian = tuple[tuple[float, ...], ...]


@dataclass(frozen=True)
class JointVelocityCommand:
    """A finite, bounded joint velocity and its short-horizon target."""

    velocities: tuple[float, ...]
    target_positions: tuple[float, ...]


def forward_kinematics(joints: Sequence[object]) -> ToolPose:
    """Return ``tool_frame`` pose in world for the substitute URDF chain."""
    positions, rotation, _, _ = _chain_state(joints)
    return ToolPose(_add(positions[-1], _mat_vec(rotation, _TOOL_OFFSET)), _quaternion(rotation))


def geometric_jacobian(joints: Sequence[object]) -> Jacobian:
    """Return the world-frame 6×6 geometric Jacobian at ``tool_frame``."""
    positions, rotation, axes, origins = _chain_state(joints)
    tip = _add(positions[-1], _mat_vec(rotation, _TOOL_OFFSET))
    columns = tuple(
        _cross(axis, _subtract(tip, origin)) + axis
        for axis, origin in zip(axes, origins)
    )
    return tuple(tuple(column[row] for column in columns) for row in range(6))


def twist_to_joint_command(
    joints: Sequence[object],
    twist: Twist,
    dt_sec: object,
    *,
    damping: object = 0.08,
) -> JointVelocityCommand:
    """Map a bounded world-frame twist to a safe short-horizon joint target.

    Damped least squares remains finite at singular configurations.  Per-joint
    velocity and position limits are enforced after the solve, so the returned
    target is always inside the URDF limits.
    """
    values = _validated_joints(joints)
    dt = _positive_finite(dt_sec, "dt_sec")
    damp = _positive_finite(damping, "damping")
    desired = _twist_vector(twist)
    jacobian = geometric_jacobian(values)
    jj_transpose = _mat_mul(jacobian, _transpose(jacobian))
    system = tuple(
        tuple(value + (damp * damp if row == column else 0.0)
              for column, value in enumerate(line))
        for row, line in enumerate(jj_transpose)
    )
    intermediate = _solve_linear(system, desired)
    unconstrained = tuple(
        sum(jacobian[row][column] * intermediate[row] for row in range(6))
        for column in range(JOINT_COUNT)
    )
    velocities: list[float] = []
    targets: list[float] = []
    for position, velocity, limit, speed_limit in zip(
        values, unconstrained, JOINT_LIMITS, JOINT_VELOCITY_LIMITS
    ):
        bounded = max(-speed_limit, min(speed_limit, velocity))
        bounded = max((limit[0] - position) / dt, bounded)
        bounded = min((limit[1] - position) / dt, bounded)
        target = position + bounded * dt
        velocities.append(bounded)
        targets.append(max(limit[0], min(limit[1], target)))
    return JointVelocityCommand(tuple(velocities), tuple(targets))


def _chain_state(
    joints: Sequence[object]) -> tuple[list[Vector], Matrix3, list[Vector], list[Vector]]:
    values = _validated_joints(joints)
    position: Vector = (0.0, 0.0, 0.0)
    rotation: Matrix3 = _IDENTITY
    joint_positions: list[Vector] = []
    world_axes: list[Vector] = []
    for value, origin, axis in zip(values, _JOINT_ORIGINS, _JOINT_AXES):
        position = _add(position, _mat_vec(rotation, origin))
        joint_positions.append(position)
        world_axis = _mat_vec(rotation, axis)
        world_axes.append(world_axis)
        rotation = _mat_mul3(rotation, _axis_rotation(axis, value))
    return joint_positions, rotation, world_axes, joint_positions


def _validated_joints(joints: Sequence[object]) -> tuple[float, ...]:
    if isinstance(joints, (str, bytes)) or len(joints) != JOINT_COUNT:
        raise ValueError("exactly six joint positions are required")
    values = tuple(_finite(value, "joint position") for value in joints)
    for index, (value, limits) in enumerate(zip(values, JOINT_LIMITS), start=1):
        if not limits[0] <= value <= limits[1]:
            raise ValueError(f"joint_{index} is outside its configured limits")
    return values


def _twist_vector(twist: Twist) -> tuple[float, ...]:
    if not isinstance(twist, Twist):
        raise ValueError("twist must be a Twist")
    values = tuple(twist.linear) + tuple(twist.angular)
    if len(values) != 6:
        raise ValueError("twist must have three linear and three angular values")
    return tuple(_finite(value, "twist value") for value in values)


def _finite(value: object, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be finite")
    try:
        numeric = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} must be finite") from error
    if not math.isfinite(numeric):
        raise ValueError(f"{label} must be finite")
    return numeric


def _positive_finite(value: object, label: str) -> float:
    numeric = _finite(value, label)
    if numeric <= 0.0:
        raise ValueError(f"{label} must be positive")
    return numeric


def _add(left: Vector, right: Vector) -> Vector:
    return tuple(a + b for a, b in zip(left, right))  # type: ignore[return-value]


def _subtract(left: Vector, right: Vector) -> Vector:
    return tuple(a - b for a, b in zip(left, right))  # type: ignore[return-value]


def _cross(left: Vector, right: Vector) -> Vector:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _mat_vec(matrix: Matrix3, vector: Vector) -> Vector:
    return tuple(sum(row[index] * vector[index] for index in range(3)) for row in matrix)  # type: ignore[return-value]


def _mat_mul3(left: Matrix3, right: Matrix3) -> Matrix3:
    return tuple(
        tuple(sum(left[row][index] * right[index][column] for index in range(3)) for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def _axis_rotation(axis: Vector, angle: float) -> Matrix3:
    x, y, z = axis
    cosine = math.cos(angle)
    sine = math.sin(angle)
    inverse = 1.0 - cosine
    return (
        (cosine + x * x * inverse, x * y * inverse - z * sine, x * z * inverse + y * sine),
        (y * x * inverse + z * sine, cosine + y * y * inverse, y * z * inverse - x * sine),
        (z * x * inverse - y * sine, z * y * inverse + x * sine, cosine + z * z * inverse),
    )


def _quaternion(rotation: Matrix3) -> tuple[float, float, float, float]:
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return ((rotation[2][1] - rotation[1][2]) / scale,
                (rotation[0][2] - rotation[2][0]) / scale,
                (rotation[1][0] - rotation[0][1]) / scale, 0.25 * scale)
    diagonal = (rotation[0][0], rotation[1][1], rotation[2][2])
    index = max(range(3), key=diagonal.__getitem__)
    if index == 0:
        scale = math.sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]) * 2.0
        return (0.25 * scale, (rotation[0][1] + rotation[1][0]) / scale,
                (rotation[0][2] + rotation[2][0]) / scale, (rotation[2][1] - rotation[1][2]) / scale)
    if index == 1:
        scale = math.sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]) * 2.0
        return ((rotation[0][1] + rotation[1][0]) / scale, 0.25 * scale,
                (rotation[1][2] + rotation[2][1]) / scale, (rotation[0][2] - rotation[2][0]) / scale)
    scale = math.sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]) * 2.0
    return ((rotation[0][2] + rotation[2][0]) / scale, (rotation[1][2] + rotation[2][1]) / scale,
            0.25 * scale, (rotation[1][0] - rotation[0][1]) / scale)


def _transpose(matrix: Jacobian) -> Jacobian:
    return tuple(tuple(matrix[row][column] for row in range(len(matrix))) for column in range(len(matrix[0])))


def _mat_mul(left: Jacobian, right: Jacobian) -> Jacobian:
    return tuple(
        tuple(sum(left[row][index] * right[index][column] for index in range(len(right))) for column in range(len(right[0])))
        for row in range(len(left))
    )


def _solve_linear(matrix: Jacobian, vector: Sequence[float]) -> tuple[float, ...]:
    """Solve a small positive-definite system by pivoted Gaussian elimination."""
    size = len(vector)
    augmented = [list(matrix[row]) + [float(vector[row])] for row in range(size)]
    for pivot in range(size):
        selected = max(range(pivot, size), key=lambda row: abs(augmented[row][pivot]))
        if abs(augmented[selected][pivot]) < 1e-12:
            raise ValueError("damped kinematics system is singular")
        augmented[pivot], augmented[selected] = augmented[selected], augmented[pivot]
        divisor = augmented[pivot][pivot]
        augmented[pivot] = [entry / divisor for entry in augmented[pivot]]
        for row in range(size):
            if row == pivot:
                continue
            factor = augmented[row][pivot]
            augmented[row] = [entry - factor * base for entry, base in zip(augmented[row], augmented[pivot])]
    return tuple(row[-1] for row in augmented)
