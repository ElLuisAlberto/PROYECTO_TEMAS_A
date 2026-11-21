import math

from thesis_core.clearance_geometry import (
    closest_point_on_segment as geometry_closest_point,
    minimum_configuration_clearance,
    point_distance,
)


SEGMENT_NAMES = (
    'base_to_shoulder',
    'shoulder_to_upper_arm',
    'upper_arm_to_forearm',
    'forearm_to_wrist_1',
    'wrist_1_to_wrist_2',
    'wrist_2_to_tool',
)

# Keep the full six-capsule model for kinematics and visualization. This
# historical JACO2 subset is used only by preventive clearance calculations.
CLEARANCE_SEGMENT_NAMES = SEGMENT_NAMES[2:]
CLEARANCE_EXCLUDED_SEGMENT_NAMES = SEGMENT_NAMES[:2]

CAPSULE_RADII = (0.105, 0.105, 0.085, 0.075, 0.085, 0.090)

JOINT_ORIGINS = (
    ((0.0, 0.0, 0.15675), (0.0, math.pi, 0.0)),
    ((0.0, 0.0016, -0.11875), (-math.pi / 2.0, 0.0, math.pi)),
    ((0.0, -0.410, 0.0), (0.0, math.pi, 0.0)),
    ((0.0, 0.2073, -0.0114), (-math.pi / 2.0, 0.0, math.pi)),
    ((0.0, -0.03703, -0.06414), (math.pi / 3.0, 0.0, math.pi)),
    ((0.0, -0.03703, -0.06414), (math.pi / 3.0, 0.0, math.pi)),
)

END_EFFECTOR_ORIGIN = (
    (0.0, 0.0, -0.1600),
    (math.pi, 0.0, math.pi / 2.0),
)


def identity_matrix():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _validated_homogeneous_transform(matrix, name='world_from_base'):
    """Return a finite rigid 4x4 transform or reject an unsafe frame map."""
    try:
        result = tuple(tuple(float(value) for value in row) for row in matrix)
    except (TypeError, ValueError) as exception:
        raise ValueError(f'{name} must be a finite 4x4 rigid transform') from exception
    if len(result) != 4 or any(len(row) != 4 for row in result):
        raise ValueError(f'{name} must be a finite 4x4 rigid transform')
    if not all(math.isfinite(value) for row in result for value in row):
        raise ValueError(f'{name} must be a finite 4x4 rigid transform')
    if any(abs(result[3][index]) > 1.0e-9 for index in range(3)) or not math.isclose(
            result[3][3], 1.0, abs_tol=1.0e-9):
        raise ValueError(f'{name} must be homogeneous')

    rotation = tuple(row[:3] for row in result[:3])
    for row in rotation:
        if not math.isclose(sum(value * value for value in row), 1.0, abs_tol=1.0e-9):
            raise ValueError(f'{name} rotation must be orthonormal')
    for first in range(3):
        for second in range(first + 1, 3):
            if not math.isclose(
                    sum(rotation[first][index] * rotation[second][index]
                        for index in range(3)),
                    0.0,
                    abs_tol=1.0e-9):
                raise ValueError(f'{name} rotation must be orthonormal')
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2]
                          - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2]
                            - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1]
                            - rotation[1][1] * rotation[2][0])
    )
    if not math.isclose(determinant, 1.0, abs_tol=1.0e-9):
        raise ValueError(f'{name} rotation must be right-handed')
    return [list(row) for row in result]


def transform_point(world_from_base, point):
    """Transform one base-frame point into world without accepting bad TF."""
    transform = _validated_homogeneous_transform(world_from_base)
    try:
        x_value, y_value, z_value = (float(value) for value in point)
    except (TypeError, ValueError) as exception:
        raise ValueError('point must contain three finite values') from exception
    if not all(math.isfinite(value) for value in (x_value, y_value, z_value)):
        raise ValueError('point must contain three finite values')
    return tuple(
        transform[row][0] * x_value
        + transform[row][1] * y_value
        + transform[row][2] * z_value
        + transform[row][3]
        for row in range(3)
    )


def transform_vector(world_from_base, vector):
    """Rotate one base-frame vector into world; translation never applies."""
    transform = _validated_homogeneous_transform(world_from_base)
    try:
        x_value, y_value, z_value = (float(value) for value in vector)
    except (TypeError, ValueError) as exception:
        raise ValueError('vector must contain three finite values') from exception
    if not all(math.isfinite(value) for value in (x_value, y_value, z_value)):
        raise ValueError('vector must contain three finite values')
    return tuple(
        transform[row][0] * x_value
        + transform[row][1] * y_value
        + transform[row][2] * z_value
        for row in range(3)
    )


def multiply(first, second):
    """Multiply two 4x4 matrices without generator overhead."""
    result = []
    for row in range(4):
        a0, a1, a2, a3 = first[row]
        result.append([
            a0 * second[0][0] + a1 * second[1][0]
            + a2 * second[2][0] + a3 * second[3][0],
            a0 * second[0][1] + a1 * second[1][1]
            + a2 * second[2][1] + a3 * second[3][1],
            a0 * second[0][2] + a1 * second[1][2]
            + a2 * second[2][2] + a3 * second[3][2],
            a0 * second[0][3] + a1 * second[1][3]
            + a2 * second[2][3] + a3 * second[3][3],
        ])
    return result


def transform_matrix(xyz, rpy):
    x_value, y_value, z_value = xyz
    roll, pitch, yaw = rpy

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return [
        [cy * cp, cy * sp * sr - sy * cr,
         cy * sp * cr + sy * sr, x_value],
        [sy * cp, sy * sp * sr + cy * cr,
         sy * sp * cr - cy * sr, y_value],
        [-sp, cp * sr, cp * cr, z_value],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rotation_z(angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return [
        [cosine, -sine, 0.0, 0.0],
        [sine, cosine, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


JOINT_ORIGIN_TRANSFORMS = tuple(
    transform_matrix(xyz, rpy) for xyz, rpy in JOINT_ORIGINS
)
END_EFFECTOR_TRANSFORM = transform_matrix(*END_EFFECTOR_ORIGIN)


def translation(matrix):
    return (matrix[0][3], matrix[1][3], matrix[2][3])


def rotation_matrix_to_rpy(matrix):
    """Return intrinsic roll, pitch and yaw from a homogeneous transform."""
    pitch_sine = -matrix[2][0]
    pitch_sine = max(-1.0, min(1.0, pitch_sine))
    pitch = math.asin(pitch_sine)
    pitch_cosine = math.cos(pitch)

    if abs(pitch_cosine) > 1.0e-9:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        roll = math.atan2(-matrix[1][2], matrix[1][1])
        yaw = 0.0
    return roll, pitch, yaw


def end_effector_transform(joint_positions, world_from_base=None):
    """
    Return the world-to-tool transform for six joints.

    ``world_from_base`` is optional to preserve the legacy single-arm model,
    whose base is coincident with ``world``.  Multi-instance callers must pass
    the exact-stamp base transform rather than assuming that coincidence.
    """
    if len(joint_positions) != 6:
        raise ValueError('JACO2 forward kinematics requires six joints')
    if not all(math.isfinite(value) for value in joint_positions):
        raise ValueError('JACO2 joint positions must be finite')

    current = identity_matrix()
    for joint_position, origin_transform in zip(
            joint_positions, JOINT_ORIGIN_TRANSFORMS):
        current = multiply(current, origin_transform)
        current = multiply(current, rotation_z(joint_position))

    base_from_tool = multiply(current, END_EFFECTOR_TRANSFORM)
    if world_from_base is None:
        return base_from_tool
    return multiply(
        _validated_homogeneous_transform(world_from_base), base_from_tool
    )


def end_effector_pose(joint_positions, world_from_base=None):
    """Return tool X, Y, Z, roll, pitch and yaw in the world frame."""
    transform = end_effector_transform(joint_positions, world_from_base)
    return (
        *translation(transform),
        *rotation_matrix_to_rpy(transform),
    )


def capsule_segments(joint_positions, world_from_base=None):
    """
    Return six world-frame capsule axes for a JACO2 configuration.

    With no base transform this is exactly the legacy base-at-world result.
    """
    if len(joint_positions) != 6:
        raise ValueError('JACO2 forward kinematics requires six joints')

    current = identity_matrix()
    frames = [translation(current)]

    for joint_position, origin_transform in zip(
            joint_positions,
            JOINT_ORIGIN_TRANSFORMS):
        current = multiply(current, origin_transform)
        current = multiply(current, rotation_z(joint_position))
        frames.append(translation(current))

    end_effector = multiply(current, END_EFFECTOR_TRANSFORM)

    capsule_points = frames[:6] + [translation(end_effector)]
    if world_from_base is not None:
        transform = _validated_homogeneous_transform(world_from_base)
        capsule_points = [transform_point(transform, point)
                          for point in capsule_points]
    return tuple(zip(capsule_points[:-1], capsule_points[1:]))


def closest_point_on_segment(point, start, end):
    """Compatibility wrapper for the canonical geometric primitive."""
    return geometry_closest_point(point, start, end)


def distance(first, second):
    """Compatibility wrapper for finite Euclidean distance."""
    return point_distance(first, second)


def minimum_sphere_clearance(
    joint_positions,
    obstacle_center,
    obstacle_radius,
    world_from_base=None,
):
    """Return the closest of all six canonical robot capsules."""
    result = minimum_configuration_clearance(
        capsule_segments(joint_positions, world_from_base),
        SEGMENT_NAMES,
        CAPSULE_RADII,
        obstacle_center,
        obstacle_radius,
    )
    return result.legacy_tuple()
