"""Validated robot metadata consumed by the historical operator GUI."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

from thesis_core.joint_model import (
    CONTINUOUS_JOINTS as JACO2_CONTINUOUS_JOINTS,
    JOINT_NAMES as JACO2_JOINT_NAMES,
    JOINT_POSITION_LIMITS as JACO2_POSITION_LIMITS,
    JOINT_VELOCITY_LIMITS as JACO2_VELOCITY_LIMITS,
)


JACO2_LABELS = (
    'J1 - Base',
    'J2 - Hombro',
    'J3 - Codo',
    'J4 - Muñeca 1',
    'J5 - Muñeca 2',
    'J6 - Muñeca 3',
)


@dataclass(frozen=True)
class GuiRobotProfile:
    """Small immutable view of any ``thesis_core.robot_profile`` object."""

    name: str
    display_name: str
    profile_id: str
    joint_names: tuple[str, ...]
    joint_labels: tuple[str, ...]
    position_limits: tuple[tuple[float, float], ...]
    velocity_limits: tuple[float, ...]
    continuous_joints: frozenset[str]
    initial_positions: tuple[float, ...]
    world_frame: str
    base_link: str
    tip_link: str
    test_positions: tuple[float, ...]
    cartesian_ready_positions: tuple[float, ...]

    @property
    def joint_count(self) -> int:
        return len(self.joint_names)

    @property
    def is_jaco2(self) -> bool:
        return self.profile_id.lower() == 'jaco2'


def _finite_vector(
    values: Sequence[float], length: int, label: str
) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise ValueError(f'{label} must contain {length} finite values')
    return result


def _ordered_values(raw, joint_names, label):
    if isinstance(raw, Mapping):
        missing = [name for name in joint_names if name not in raw]
        if missing:
            raise ValueError(f'{label} is missing {missing[0]}')
        return tuple(raw[name] for name in joint_names)
    return tuple(raw)


def _derived_test_pose(
    initial: tuple[float, ...],
    limits: tuple[tuple[float, float], ...],
) -> tuple[float, ...]:
    result = list(initial)
    lower, upper = limits[0]
    positive = min(0.20, max(0.0, upper - initial[0]))
    negative = max(-0.20, min(0.0, lower - initial[0]))
    result[0] += positive if positive > 1.0e-12 else negative
    return tuple(result)


def default_gui_robot_profile() -> GuiRobotProfile:
    """Return the unchanged historical JACO2 operational profile."""
    initial = tuple(
        math.radians(value)
        for value in (-45.0, 160.0, 200.0, 0.0, 15.0, 0.0)
    )
    return GuiRobotProfile(
        name='jaco2',
        display_name='JACO2',
        profile_id='jaco2',
        joint_names=tuple(JACO2_JOINT_NAMES),
        joint_labels=JACO2_LABELS,
        position_limits=tuple(
            tuple(JACO2_POSITION_LIMITS[name])
            for name in JACO2_JOINT_NAMES
        ),
        velocity_limits=tuple(
            float(JACO2_VELOCITY_LIMITS[name])
            for name in JACO2_JOINT_NAMES
        ),
        continuous_joints=frozenset(JACO2_CONTINUOUS_JOINTS),
        initial_positions=initial,
        world_frame='world',
        base_link='root',
        tip_link='j2n6s300_end_effector',
        test_positions=(0.20, math.pi, math.pi, 0.0, 0.0, 0.0),
        cartesian_ready_positions=tuple(
            math.radians(value)
            for value in (-45.0, 160.0, 200.0, 0.0, 15.0, 0.0)
        ),
    )


DEFAULT_GUI_ROBOT_PROFILE = default_gui_robot_profile()


def normalize_gui_robot_profile(profile=None) -> GuiRobotProfile:
    """
    Validate a core or multi-robot profile for presentation in one GUI.

    The primary API is ``thesis_core.robot_profile.RobotProfile``.  The
    ``joint_limits`` fallback keeps the already existing multi-robot manifest
    view usable until its backend is migrated to the core profile loader.
    """
    if profile is None:
        return DEFAULT_GUI_ROBOT_PROFILE
    if isinstance(profile, GuiRobotProfile):
        return profile

    joint_names = tuple(str(name) for name in profile.joint_names)
    if not joint_names or len(set(joint_names)) != len(joint_names):
        raise ValueError('joint_names must be nonempty and unique')
    count = len(joint_names)

    raw_position_limits = getattr(profile, 'joint_position_limits', None)
    raw_velocity_limits = getattr(profile, 'joint_velocity_limits', None)
    if raw_position_limits is None or raw_velocity_limits is None:
        legacy_limits = tuple(getattr(profile, 'joint_limits', ()))
        if len(legacy_limits) != count:
            raise ValueError('profile does not provide one limit per joint')
        raw_position_limits = tuple(limit[:2] for limit in legacy_limits)
        raw_velocity_limits = tuple(limit[2] for limit in legacy_limits)

    ordered_limits = _ordered_values(
        raw_position_limits, joint_names, 'joint_position_limits'
    )
    position_limits = []
    for joint_name, raw_limit in zip(joint_names, ordered_limits):
        if len(raw_limit) != 2:
            raise ValueError(f'{joint_name} position limit must contain two values')
        lower, upper = (float(value) for value in raw_limit)
        if not all(math.isfinite(value) for value in (lower, upper)):
            raise ValueError(f'{joint_name} position limit must be finite')
        if lower >= upper:
            raise ValueError(f'{joint_name} position limit must be increasing')
        position_limits.append((lower, upper))
    position_limits = tuple(position_limits)

    velocity_limits = _finite_vector(
        _ordered_values(
            raw_velocity_limits, joint_names, 'joint_velocity_limits'
        ),
        count,
        'joint_velocity_limits',
    )
    if any(value <= 0.0 for value in velocity_limits):
        raise ValueError('joint_velocity_limits must be positive')

    initial = _finite_vector(
        getattr(profile, 'initial_positions'), count, 'initial_positions'
    )
    for name, value, (lower, upper) in zip(
        joint_names, initial, position_limits
    ):
        if not lower <= value <= upper:
            raise ValueError(f'initial position for {name} is outside its limits')

    raw_labels = getattr(profile, 'joint_labels', ())
    if raw_labels:
        labels = tuple(str(value) for value in raw_labels)
        if len(labels) != count or any(not value.strip() for value in labels):
            raise ValueError('joint_labels must contain one nonempty label per joint')
    else:
        labels = tuple(
            f'J{index} - {joint_name}'
            for index, joint_name in enumerate(joint_names, start=1)
        )

    continuous = frozenset(
        str(name) for name in getattr(profile, 'continuous_joints', ())
    )
    if not continuous.issubset(joint_names):
        raise ValueError('continuous_joints contains an unknown joint')

    test = getattr(profile, 'test_positions', None)
    test_positions = (
        _derived_test_pose(initial, position_limits)
        if test is None else _finite_vector(test, count, 'test_positions')
    )
    ready = getattr(profile, 'cartesian_ready_positions', None)
    ready_positions = (
        initial
        if ready is None
        else _finite_vector(ready, count, 'cartesian_ready_positions')
    )

    name = str(getattr(profile, 'name', '')).strip()
    profile_id = str(
        getattr(profile, 'profile_id', getattr(profile, 'profile', name))
    ).strip()
    display_name = str(
        getattr(profile, 'display_name', profile_id or name)
    ).strip()
    world_frame = str(getattr(profile, 'world_frame', 'world')).strip()
    base_link = str(getattr(profile, 'base_link', '')).strip()
    tip_link = str(getattr(profile, 'tip_link', '')).strip()
    if not all((name, profile_id, display_name, world_frame, base_link, tip_link)):
        raise ValueError('profile identity and frames must be nonempty')

    return GuiRobotProfile(
        name=name,
        display_name=display_name,
        profile_id=profile_id,
        joint_names=joint_names,
        joint_labels=labels,
        position_limits=position_limits,
        velocity_limits=velocity_limits,
        continuous_joints=continuous,
        initial_positions=initial,
        world_frame=world_frame,
        base_link=base_link,
        tip_link=tip_link,
        test_positions=test_positions,
        cartesian_ready_positions=ready_positions,
    )
