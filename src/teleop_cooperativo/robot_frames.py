"""Shared JACO2 link and simulated finger transforms."""
import numpy as np
from . import jaco2_kinematics as kin

FINGER_ORIGINS = (
    ((0.00279, 0.03126, -0.11467), (-1.7047873384941834, 0.6476144647144773, 1.67317415161155)),
    ((0.02226, -0.02707, -0.11482), (-1.570796327, 0.649262481663582, -1.38614049188413)),
    ((-0.02226, -0.02707, -0.11482), (-1.570796327, 0.649262481663582, -1.75545216211587)),
)


def link_transforms(joints):
    """World transforms for base and six links, following the Jaco2 Xacro."""
    values = kin._validated_joints(joints)
    result = [np.eye(4)]
    transform = np.eye(4)
    for value, (translation, rpy) in zip(values, kin._JOINT_ORIGINS):
        origin = np.eye(4)
        origin[:3, :3] = kin._rpy_rotation(rpy)
        origin[:3, 3] = translation
        rotation = np.eye(4)
        rotation[:3, :3] = kin._z_rotation(value)
        transform = transform @ origin @ rotation
        result.append(transform.copy())
    return result


def _origin_transform(translation, rpy):
    result = np.eye(4)
    result[:3, :3] = kin._rpy_rotation(rpy)
    result[:3, 3] = translation
    return result


def finger_link_transform(hand, origin, angle, distal=False):
    """Kinova proximal joint rotation; distal joint remains unmeasured."""
    rotation = np.eye(4)
    rotation[:3, :3] = kin._z_rotation(angle)
    transform = hand @ origin @ rotation
    if distal:
        transform = transform @ _origin_transform((0.044, -0.003, 0.0), (0.0, 0.0, 0.0))
    return transform
