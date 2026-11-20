"""Pure differential kinematics for the Kinova Jaco2 J2N6S300.

The constants in this module are transcribed from the official J2N6S300
Xacro chain: six local-Z joints, its fixed inter-link transforms, and the
end-effector offset.  It deliberately has no ROS, Gazebo, or CORE-project
dependency, so it is the mathematical boundary for FASE_1's Jaco2 simulator.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .command_composer import Twist
from .simulation_port import ToolPose


JOINT_COUNT = 6
JOINT_NAMES = tuple(f"j2n6s300_joint_{index}" for index in range(1, JOINT_COUNT + 1))

# Joint 1 and wrist joints are continuous in the source Xacro, whose
# simulation description bounds them to +/- 2*pi.  Joints 2 and 3 have the
# physical stop ranges published by Kinova: 47..313 and 19..341 degrees.
JOINT_LIMITS = (
    (-2.0 * math.pi, 2.0 * math.pi),
    (math.radians(47.0), math.radians(313.0)),
    (math.radians(19.0), math.radians(341.0)),
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
)

# Deliberately below the Xacro limits (36 deg/s for J1--J3 and 48 deg/s for
# J4--J6), providing a conservative simulation default.
JOINT_VELOCITY_LIMITS = (
    math.radians(18.0),
    math.radians(18.0),
    math.radians(18.0),
    math.radians(24.0),
    math.radians(24.0),
    math.radians(24.0),
)

# (xyz, intrinsic RPY) origins, in the exact order of the official Xacro.
_JOINT_ORIGINS = (
    ((0.0, 0.0, 0.15675), (0.0, math.pi, 0.0)),
    ((0.0, 0.0016, -0.11875), (-math.pi / 2.0, 0.0, math.pi)),
    ((0.0, -0.410, 0.0), (0.0, math.pi, 0.0)),
    ((0.0, 0.2073, -0.0114), (-math.pi / 2.0, 0.0, math.pi)),
    ((0.0, -0.03703, -0.06414), (math.pi / 3.0, 0.0, math.pi)),
    ((0.0, -0.03703, -0.06414), (math.pi / 3.0, 0.0, math.pi)),
)
_END_EFFECTOR_ORIGIN = ((0.0, 0.0, -0.1600), (math.pi, 0.0, math.pi / 2.0))
_IDENTITY: "Matrix3" = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
_LOCAL_Z = (0.0, 0.0, 1.0)

Vector = tuple[float, float, float]
Matrix3 = tuple[Vector, Vector, Vector]
Jacobian = tuple[tuple[float, ...], ...]


@dataclass(frozen=True)
class JointVelocityCommand:
    """Finite bounded joint velocity plus its short-horizon target."""

    velocities: tuple[float, ...]
    target_positions: tuple[float, ...]


def forward_kinematics(joints: Sequence[object]) -> ToolPose:
    """Return the world pose of ``j2n6s300_end_effector``.

    The pose includes the fixed Jaco2 end-effector offset, not merely link 6.
    """
    _, rotation, tip, _ = _chain_state(joints)
    return ToolPose(tip, _quaternion(rotation))


def geometric_jacobian(joints: Sequence[object]) -> Jacobian:
    """Return the 6x6 world-frame geometric Jacobian at the end effector."""
    axes, _, tip, origins = _chain_state(joints)
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
    """Map a world-frame twist to a bounded Jaco2 joint command.

    A damped least-squares solve keeps the command finite at or near
    singularities.  The resulting velocity and target are independently
    constrained by the conservative velocity and official position limits.
    """
    values = _validated_joints(joints)
    dt = _positive_finite(dt_sec, "dt_sec")
    damp = _positive_finite(damping, "damping")
    jacobian = geometric_jacobian(values)
    desired = _twist_vector(twist)
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
    for position, velocity, limits, speed_limit in zip(
        values, unconstrained, JOINT_LIMITS, JOINT_VELOCITY_LIMITS
    ):
        bounded = max(-speed_limit, min(speed_limit, velocity))
        bounded = max((limits[0] - position) / dt, bounded)
        bounded = min((limits[1] - position) / dt, bounded)
        target = max(limits[0], min(limits[1], position + bounded * dt))
        velocities.append(bounded)
        targets.append(target)
    return JointVelocityCommand(tuple(velocities), tuple(targets))


def _chain_state(
    joints: Sequence[object],
) -> tuple[list[Vector], Matrix3, Vector, list[Vector]]:
    values = _validated_joints(joints)
    position: Vector = (0.0, 0.0, 0.0)
    rotation: Matrix3 = _IDENTITY
    axes: list[Vector] = []
    origins: list[Vector] = []
    for value, (translation, rpy) in zip(values, _JOINT_ORIGINS):
        position = _add(position, _mat_vec(rotation, translation))
        rotation = _mat_mul3(rotation, _rpy_rotation(rpy))
        origins.append(position)
        axes.append(_mat_vec(rotation, _LOCAL_Z))
        rotation = _mat_mul3(rotation, _z_rotation(value))

    end_translation, end_rpy = _END_EFFECTOR_ORIGIN
    tip = _add(position, _mat_vec(rotation, end_translation))
    rotation = _mat_mul3(rotation, _rpy_rotation(end_rpy))
    return axes, rotation, tip, origins


def _validated_joints(joints: Sequence[object]) -> tuple[float, ...]:
    if isinstance(joints, (str, bytes)):
        raise ValueError("exactly six joint positions are required")
    try:
        values = tuple(joints)
    except TypeError as error:
        raise ValueError("exactly six joint positions are required") from error
    if len(values) != JOINT_COUNT:
        raise ValueError("exactly six joint positions are required")
    numeric = tuple(_finite(value, "joint position") for value in values)
    for name, value, limits in zip(JOINT_NAMES, numeric, JOINT_LIMITS):
        if not limits[0] <= value <= limits[1]:
            raise ValueError(f"{name} is outside its configured limits")
    return numeric


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


def _rpy_rotation(rpy: Vector) -> Matrix3:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _z_rotation(angle: float) -> Matrix3:
    cosine, sine = math.cos(angle), math.sin(angle)
    return ((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0))


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
    """Solve a small positive-definite system by pivoted elimination."""
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
