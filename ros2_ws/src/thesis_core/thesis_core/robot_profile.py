"""Validated robot profiles shared by simulation and safety components."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, NamedTuple, Sequence
import xml.etree.ElementTree as ET

import numpy as np

from thesis_core.jaco_kinematics import CAPSULE_RADII, SEGMENT_NAMES
from thesis_core.joint_model import (
    CONTINUOUS_JOINTS,
    JOINT_NAMES,
    JOINT_POSITION_LIMITS,
    JOINT_VELOCITY_LIMITS,
)
from thesis_core.robot_kinematics import RobotKinematics


DISPLAY_NAMES = MappingProxyType({
    'jaco2': 'Kinova JACO2',
    'ur17g': 'Universal Robots UR17g',
    'p3020': 'Doosan P3020',
    'cs625': 'Elite CS625',
})


class CapsuleSegment(NamedTuple):
    """One capsule expressed in ``RobotProfile.world_frame``."""

    name: str
    start: tuple[float, float, float]
    end: tuple[float, float, float]
    radius: float


def _finite_vector(
    value: Sequence[float],
    size: int,
    field: str,
) -> tuple[float, ...]:
    result = tuple(float(component) for component in value)
    if len(result) != size or not all(math.isfinite(v) for v in result):
        raise ValueError(f'{field} must contain {size} finite values')
    return result


@dataclass(frozen=True)
class CollisionCapsule:
    """A local-link or between-frame collision capsule description."""

    name: str
    radius: float
    link: str | None = None
    start: tuple[float, float, float] | None = None
    end: tuple[float, float, float] | None = None
    start_frame: str | None = None
    end_frame: str | None = None

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError('capsule name must be nonempty')
        radius = float(self.radius)
        if not math.isfinite(radius) or radius <= 0.0:
            raise ValueError('capsule radius must be positive and finite')
        object.__setattr__(self, 'radius', radius)
        local_mode = (
            self.link is not None
            and self.start is not None
            and self.end is not None
            and self.start_frame is None
            and self.end_frame is None
        )
        frame_mode = (
            self.link is None
            and self.start is None
            and self.end is None
            and self.start_frame is not None
            and self.end_frame is not None
        )
        if local_mode == frame_mode:
            raise ValueError(
                'capsule must use exactly one geometry representation'
            )
        if local_mode:
            object.__setattr__(
                self, 'start', _finite_vector(self.start, 3, 'capsule start')
            )
            object.__setattr__(
                self, 'end', _finite_vector(self.end, 3, 'capsule end')
            )

    def world_segment(
        self,
        poses: Mapping[str, np.ndarray],
    ) -> CapsuleSegment:
        """Resolve this capsule using world-from-link transforms."""
        if self.link is not None:
            if self.link not in poses:
                raise ValueError(f'capsule references unknown link: {self.link}')
            transform = poses[self.link]
            start = (
                transform[:3, :3] @ np.asarray(self.start)
                + transform[:3, 3]
            )
            end = (
                transform[:3, :3] @ np.asarray(self.end)
                + transform[:3, 3]
            )
        else:
            if self.start_frame not in poses or self.end_frame not in poses:
                raise ValueError(
                    'capsule references an unknown start or end frame'
                )
            start = poses[self.start_frame][:3, 3]
            end = poses[self.end_frame][:3, 3]
        return CapsuleSegment(
            self.name,
            tuple(float(value) for value in start),
            tuple(float(value) for value in end),
            self.radius,
        )


@dataclass(frozen=True)
class ControllerPaths:
    """Namespaced observation and command endpoints for one robot."""

    namespace: str
    config: Path | None
    controller_manager: str
    trajectory_action: str
    joint_states_topic: str

    @classmethod
    def for_robot(
        cls,
        name: str,
        config: Path | None,
        legacy: bool = False,
    ) -> 'ControllerPaths':
        prefix = '' if legacy else f'/{name}'
        return cls(
            namespace=prefix or '/',
            config=config,
            controller_manager=f'{prefix}/controller_manager',
            trajectory_action=(
                f'{prefix}/arm_controller/follow_joint_trajectory'
            ),
            joint_states_topic=f'{prefix}/joint_states',
        )


@dataclass(frozen=True)
class RobotProfile:
    """Complete immutable contract for one simulated arm."""

    name: str
    display_name: str
    profile_id: str
    joint_names: tuple[str, ...]
    joint_position_limits: Mapping[str, tuple[float, float]]
    joint_velocity_limits: Mapping[str, float]
    continuous_joints: frozenset[str]
    initial_positions: tuple[float, ...]
    world_frame: str
    base_link: str
    tip_link: str
    urdf_xml: str
    urdf_path: Path | None
    kinematics: RobotKinematics
    collision_capsules: tuple[CollisionCapsule, ...]
    controller_paths: ControllerPaths
    base_xyz: tuple[float, float, float]
    world_name: str

    def __post_init__(self) -> None:
        names = tuple(self.joint_names)
        if not names or len(set(names)) != len(names):
            raise ValueError('profile joint_names must be nonempty and unique')
        if names != self.kinematics.joint_names:
            raise ValueError('profile and kinematic joint orders differ')
        if self.world_frame != self.kinematics.root_link:
            raise ValueError('world_frame must be the URDF root link')
        if self.base_link not in self.kinematics.link_names:
            raise ValueError('base_link is absent from the URDF')
        if self.tip_link not in self.kinematics.link_names:
            raise ValueError('tip_link is absent from the URDF')
        if set(self.joint_position_limits) != set(names):
            raise ValueError('position limits must cover every selected joint')
        if set(self.joint_velocity_limits) != set(names):
            raise ValueError('velocity limits must cover every selected joint')
        if not self.continuous_joints.issubset(names):
            raise ValueError('continuous_joints contains an unknown joint')
        initial = _finite_vector(
            self.initial_positions, len(names), 'initial_positions'
        )
        object.__setattr__(self, 'initial_positions', initial)
        object.__setattr__(
            self,
            'base_xyz',
            _finite_vector(self.base_xyz, 3, 'base_xyz'),
        )
        object.__setattr__(
            self,
            'joint_position_limits',
            MappingProxyType(dict(self.joint_position_limits)),
        )
        object.__setattr__(
            self,
            'joint_velocity_limits',
            MappingProxyType(dict(self.joint_velocity_limits)),
        )
        if not self.collision_capsules:
            raise ValueError('profile must contain collision capsules')
        self.kinematics.forward(initial)
        self.capsule_segments(initial)

    @property
    def joint_count(self) -> int:
        """Return the number of commanded arm axes."""
        return len(self.joint_names)

    @property
    def continuous_flags(self) -> tuple[bool, ...]:
        """Continuous-joint flags in command-vector order."""
        return tuple(
            name in self.continuous_joints for name in self.joint_names
        )

    @property
    def capsules(self) -> tuple[CollisionCapsule, ...]:
        """Compatibility alias for ``collision_capsules``."""
        return self.collision_capsules

    @property
    def model(self) -> RobotKinematics:
        """Compatibility alias for the parsed URDF kinematic model."""
        return self.kinematics

    @property
    def controller_config(self) -> Path | None:
        return self.controller_paths.config

    @property
    def controller_action(self) -> str:
        return self.controller_paths.trajectory_action

    @property
    def joint_states_topic(self) -> str:
        return self.controller_paths.joint_states_topic

    @property
    def urdf_root(self) -> ET.Element:
        """Return a fresh XML tree so callers cannot mutate the profile."""
        return ET.fromstring(self.urdf_xml)

    def forward(
        self, positions: Sequence[float]
    ) -> dict[str, np.ndarray]:
        """Return world-from-link transforms for one joint vector."""
        return self.kinematics.forward(positions)

    def inverse(
        self,
        target: np.ndarray,
        seed: Sequence[float],
        position_only: bool = False,
    ) -> tuple[float, ...]:
        """Solve local bounded IK for this profile's configured tip."""
        return self.kinematics.inverse(
            target, seed, self.tip_link, position_only=position_only
        )

    def capsule_segments(
        self,
        positions: Sequence[float],
    ) -> tuple[CapsuleSegment, ...]:
        """Return collision capsules in ``world_frame`` coordinates."""
        poses = self.forward(positions)
        return tuple(
            capsule.world_segment(poses)
            for capsule in self.collision_capsules
        )


def _resolve_file(
    value: Any,
    manifest_dir: Path,
    field: str,
) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError(f'{field} must be a nonempty path')
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = manifest_dir / path
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f'{field} does not exist: {path}')
    return path


def _matching_jaco_name(joint_name: str) -> str | None:
    for canonical in JOINT_NAMES:
        if joint_name == canonical or joint_name.endswith('_' + canonical):
            return canonical
    return None


def _joint_contract(
    root: ET.Element,
    joint_names: tuple[str, ...],
    profile_id: str,
) -> tuple[
    dict[str, tuple[float, float]],
    dict[str, float],
    frozenset[str],
]:
    positions: dict[str, tuple[float, float]] = {}
    velocities: dict[str, float] = {}
    continuous: set[str] = set()
    for name in joint_names:
        joint = root.find(f"joint[@name='{name}']")
        if joint is None or joint.get('type') == 'fixed':
            raise ValueError(f'unknown or fixed selected joint: {name}')
        kind = joint.get('type')
        limit = joint.find('limit')
        if limit is None or limit.get('velocity') is None:
            raise ValueError(f'missing velocity limit at {name}')
        velocity = float(limit.get('velocity'))
        if not math.isfinite(velocity) or velocity <= 0.0:
            raise ValueError(f'invalid velocity limit at {name}')
        if kind == 'continuous':
            lower, upper = -math.inf, math.inf
            continuous.add(name)
        else:
            if limit.get('lower') is None or limit.get('upper') is None:
                raise ValueError(f'missing position limits at {name}')
            lower = float(limit.get('lower'))
            upper = float(limit.get('upper'))
            if (
                not math.isfinite(lower)
                or not math.isfinite(upper)
                or lower >= upper
            ):
                raise ValueError(f'invalid position limits at {name}')
        canonical = (
            _matching_jaco_name(name) if profile_id == 'jaco2' else None
        )
        if canonical is not None:
            lower, upper = JOINT_POSITION_LIMITS[canonical]
            velocity = JOINT_VELOCITY_LIMITS[canonical]
        positions[name] = (lower, upper)
        velocities[name] = velocity
    return positions, velocities, frozenset(continuous)


def _local_capsules(
    value: Any,
    links: set[str],
) -> tuple[CollisionCapsule, ...]:
    if not isinstance(value, list) or not value:
        raise ValueError('capsules must be a nonempty list')
    result = []
    for index, raw in enumerate(value):
        if not isinstance(raw, dict):
            raise ValueError('each capsule must be an object')
        link = raw.get('link')
        if link not in links:
            raise ValueError(f'capsule references unknown link: {link}')
        result.append(CollisionCapsule(
            name=str(raw.get('name') or f'{link}_collision_{index}'),
            link=link,
            start=_finite_vector(raw.get('start', ()), 3, 'capsule start'),
            end=_finite_vector(raw.get('end', ()), 3, 'capsule end'),
            radius=float(raw.get('radius', math.nan)),
        ))
    return tuple(result)


def _profile_from_manifest(
    name: str,
    raw: Any,
    manifest_dir: Path,
    world_name: str,
) -> RobotProfile:
    if not isinstance(raw, dict):
        raise ValueError(f'robot {name} must be an object')
    profile_id = raw.get('profile')
    if not isinstance(profile_id, str) or not profile_id:
        raise ValueError(f'robot {name} has no profile identifier')
    urdf_path = _resolve_file(raw.get('urdf'), manifest_dir, 'urdf')
    try:
        root = ET.parse(urdf_path).getroot()
    except ET.ParseError as error:
        raise ValueError(f'invalid URDF for {name}: {error}') from error
    if root.tag != 'robot':
        raise ValueError(f'URDF for {name} has no robot root')
    urdf_xml = ET.tostring(root, encoding='unicode')

    raw_names = raw.get('joint_names')
    if not isinstance(raw_names, list):
        raise ValueError(f'robot {name} joint_names must be a list')
    joint_names = tuple(str(joint) for joint in raw_names)
    positions, velocities, continuous = _joint_contract(
        root, joint_names, profile_id
    )
    model = RobotKinematics(root, joint_names, positions)
    world_frame = str(raw.get('world_frame') or model.root_link)
    controller_config = _resolve_file(
        raw.get('controller_config'), manifest_dir, 'controller_config'
    )
    capsules = _local_capsules(
        raw.get('capsules'), set(model.link_names)
    )
    initial = _finite_vector(
        raw.get('initial_positions', ()),
        len(joint_names),
        'initial_positions',
    )
    return RobotProfile(
        name=name,
        display_name=str(
            raw.get('display_name')
            or DISPLAY_NAMES.get(profile_id, profile_id)
        ),
        profile_id=profile_id,
        joint_names=joint_names,
        joint_position_limits=positions,
        joint_velocity_limits=velocities,
        continuous_joints=continuous,
        initial_positions=initial,
        world_frame=world_frame,
        base_link=str(raw.get('base_link', '')),
        tip_link=str(raw.get('tip_link', '')),
        urdf_xml=urdf_xml,
        urdf_path=urdf_path,
        kinematics=model,
        collision_capsules=capsules,
        controller_paths=ControllerPaths.for_robot(
            name, controller_config
        ),
        base_xyz=_finite_vector(raw.get('base_xyz', ()), 3, 'base_xyz'),
        world_name=world_name,
    )


def _legacy_jaco2_profile() -> RobotProfile:
    """Build the original unnamespaced JACO2 contract on demand."""
    try:
        from ament_index_python.packages import get_package_share_directory
        import xacro
    except ImportError as error:
        raise RuntimeError(
            'legacy JACO2 requires a sourced ROS environment with xacro'
        ) from error

    description = Path(get_package_share_directory('thesis_description'))
    source = description / 'urdf' / 'j2n6s300_standalone.xacro'
    root = ET.fromstring(xacro.process_file(str(source)).toxml())
    urdf_xml = ET.tostring(root, encoding='unicode')
    positions = dict(JOINT_POSITION_LIMITS)
    velocities = dict(JOINT_VELOCITY_LIMITS)
    model = RobotKinematics(root, JOINT_NAMES, positions)
    frame_pairs = (
        ('j2n6s300_link_base', 'j2n6s300_link_1'),
        ('j2n6s300_link_1', 'j2n6s300_link_2'),
        ('j2n6s300_link_2', 'j2n6s300_link_3'),
        ('j2n6s300_link_3', 'j2n6s300_link_4'),
        ('j2n6s300_link_4', 'j2n6s300_link_5'),
        ('j2n6s300_link_5', 'j2n6s300_end_effector'),
    )
    capsules = tuple(
        CollisionCapsule(
            name=segment_name,
            radius=radius,
            start_frame=start,
            end_frame=end,
        )
        for segment_name, radius, (start, end) in zip(
            SEGMENT_NAMES, CAPSULE_RADII, frame_pairs
        )
    )
    return RobotProfile(
        name='jaco2',
        display_name=DISPLAY_NAMES['jaco2'],
        profile_id='jaco2',
        joint_names=JOINT_NAMES,
        joint_position_limits=positions,
        joint_velocity_limits=velocities,
        continuous_joints=CONTINUOUS_JOINTS,
        initial_positions=(
            -math.pi / 4.0,
            math.radians(160.0),
            math.radians(200.0),
            0.0,
            math.radians(15.0),
            0.0,
        ),
        world_frame=model.root_link,
        base_link='j2n6s300_link_base',
        tip_link='j2n6s300_end_effector',
        urdf_xml=urdf_xml,
        urdf_path=None,
        kinematics=model,
        collision_capsules=capsules,
        controller_paths=ControllerPaths.for_robot(
            'jaco2', None, legacy=True
        ),
        base_xyz=(0.0, 0.0, 0.0),
        world_name='jaco_world',
    )


def load_robot_profiles(
    manifest_path: str | Path | None,
) -> dict[str, RobotProfile]:
    """Load all profiles from ``cell.json`` or the legacy JACO2 default."""
    if manifest_path is None:
        return {'jaco2': _legacy_jaco2_profile()}
    path = Path(manifest_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f'cell manifest does not exist: {path}')
    try:
        document = json.loads(path.read_text(encoding='utf-8'))
    except json.JSONDecodeError as error:
        raise ValueError(f'invalid cell manifest: {error}') from error
    if not isinstance(document, dict) or document.get('schema_version') != 1:
        raise ValueError('cell manifest schema_version must be 1')
    robots = document.get('robots')
    if not isinstance(robots, dict) or not robots:
        raise ValueError('cell manifest robots must be a nonempty object')
    world_name = document.get('world')
    if not isinstance(world_name, str) or not world_name:
        raise ValueError('cell manifest world must be nonempty')
    return {
        str(name): _profile_from_manifest(
            str(name), raw, path.parent, world_name
        )
        for name, raw in robots.items()
    }


def load_robot_profile(
    manifest_path: str | Path | None = None,
    name: str = 'jaco2',
) -> RobotProfile:
    """Load one named profile; no manifest means legacy JACO2."""
    profiles = load_robot_profiles(manifest_path)
    if name not in profiles:
        raise ValueError(
            f'unknown robot {name!r}; available: {", ".join(profiles)}'
        )
    return profiles[name]


def load_default_robot_profile(
    manifest_path: str | Path | None = None,
) -> RobotProfile:
    """Explicit convenience alias for the JACO2-compatible default."""
    return load_robot_profile(manifest_path, 'jaco2')
