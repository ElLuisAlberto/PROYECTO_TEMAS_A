"""
Pure validation for the offline three-Gazebo hybrid-preparation scenario.

The contract intentionally distinguishes four logical identities from runtime
backends: only arm_02 through arm_04 may own Gazebo entities and simulation
outputs.  arm_01 is a synthetic state source, never a hidden hardware path.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Any, Mapping, Optional, Sequence, Tuple


CANONICAL_ROBOT_IDS = ('arm_01', 'arm_02', 'arm_03', 'arm_04')
GAZEBO_ROBOT_IDS = ('arm_02', 'arm_03', 'arm_04')
REFERENCE_FRAME = 'world'
_IDENTIFIER = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')
_TOOL_MODES = frozenset(('legacy', 'interface', 'fixture', 'synthetic_gripper'))


@dataclass(frozen=True)
class HybridInstance:
    """One validated logical identity and, where allowed, its sim backend."""

    robot_id: str
    source: str
    output: str
    namespace: str
    tf_prefix: str
    entity_name: Optional[str]
    controller_manager_name: Optional[str]
    arm_controller_name: Optional[str]
    base_pose: Tuple[float, float, float, float, float, float]
    world_from_base: Tuple[float, float, float, float, float, float]
    tool_mode: str


@dataclass(frozen=True)
class HybridThreeSimScenario:
    """Exact four-identity contract for the no-hardware offline campaign."""

    reference_frame: str
    instances: Tuple[HybridInstance, ...]

    def instance(self, robot_id: str) -> HybridInstance:
        for item in self.instances:
            if item.robot_id == robot_id:
                return item
        raise KeyError(f'unknown robot_id: {robot_id!r}')


def _identifier(value: Any, field: str) -> str:
    if not isinstance(value, str) or not _IDENTIFIER.fullmatch(value):
        raise ValueError(f'{field} must be a relative identifier')
    return value


def _optional_identifier(value: Any, field: str) -> Optional[str]:
    if value is None:
        return None
    return _identifier(value, field)


def _pose(value: Any, field: str) -> Tuple[float, float, float, float, float, float]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError(f'{field} must contain six numeric values')
    if len(value) != 6:
        raise ValueError(f'{field} must contain six numeric values')
    try:
        result = tuple(float(component) for component in value)
    except (TypeError, ValueError) as exception:
        raise ValueError(f'{field} must contain six numeric values') from exception
    if not all(math.isfinite(component) for component in result):
        raise ValueError(f'{field} must contain finite values')
    return result  # type: ignore[return-value]


def _instance(value: Any) -> HybridInstance:
    if not isinstance(value, Mapping):
        raise ValueError('every hybrid instance must be a mapping')
    robot_id = _identifier(value.get('robot_id'), 'robot_id')
    source = value.get('source')
    output = value.get('output')
    if not isinstance(source, str) or not isinstance(output, str):
        raise ValueError(f'{robot_id} source and output must be strings')
    tool_mode = value.get('tool_mode')
    if tool_mode not in _TOOL_MODES:
        raise ValueError(f'{robot_id}.tool_mode is not an approved offline mode')
    base_pose = _pose(value.get('base_pose'), f'{robot_id}.base_pose')
    world_from_base = _pose(
        value.get('world_from_base'), f'{robot_id}.world_from_base'
    )
    if base_pose != world_from_base:
        raise ValueError(f'{robot_id} base_pose and world_from_base must match')
    return HybridInstance(
        robot_id=robot_id,
        source=source,
        output=output,
        namespace=_identifier(value.get('namespace'), f'{robot_id}.namespace'),
        tf_prefix=_identifier(value.get('tf_prefix'), f'{robot_id}.tf_prefix'),
        entity_name=_optional_identifier(
            value.get('entity_name'), f'{robot_id}.entity_name'
        ),
        controller_manager_name=_optional_identifier(
            value.get('controller_manager_name'),
            f'{robot_id}.controller_manager_name',
        ),
        arm_controller_name=_optional_identifier(
            value.get('arm_controller_name'), f'{robot_id}.arm_controller_name'
        ),
        base_pose=base_pose,
        world_from_base=world_from_base,
        tool_mode=tool_mode,
    )


def _unique(values: Sequence[str], field: str) -> None:
    if len(set(values)) != len(values):
        raise ValueError(f'{field} must be unique across hybrid instances')


def _validate_role(instance: HybridInstance) -> None:
    resources = (
        instance.entity_name,
        instance.controller_manager_name,
        instance.arm_controller_name,
    )
    if instance.robot_id == 'arm_01':
        if instance.source != 'synthetic_state' or instance.output != 'none':
            raise ValueError('arm_01 must be synthetic_state with output none')
        if any(resource is not None for resource in resources):
            raise ValueError('arm_01 synthetic_state must not own Gazebo resources')
        return
    if instance.robot_id in GAZEBO_ROBOT_IDS:
        if instance.source != 'gazebo' or instance.output != 'simulation':
            raise ValueError(
                f'{instance.robot_id} must be gazebo with simulation output'
            )
        if any(resource is None for resource in resources):
            raise ValueError(f'{instance.robot_id} requires Gazebo resources')
        return
    raise ValueError(f'unsupported robot_id: {instance.robot_id!r}')


def build_hybrid_three_sim_scenario(
    value: Mapping[str, Any],
) -> HybridThreeSimScenario:
    """Validate the fixed offline topology without implicit hardware defaults."""
    if not isinstance(value, Mapping):
        raise ValueError('hybrid scenario must be a mapping')
    raw = value.get('hybrid_three_sim', value)
    if not isinstance(raw, Mapping):
        raise ValueError('hybrid_three_sim must be a mapping')
    if raw.get('reference_frame') != REFERENCE_FRAME:
        raise ValueError('reference_frame must be world')
    raw_instances = raw.get('instances')
    if isinstance(raw_instances, (str, bytes)) or not isinstance(
            raw_instances, Sequence):
        raise ValueError('instances must be a sequence of four mappings')
    instances = tuple(_instance(item) for item in raw_instances)
    if len(instances) != len(CANONICAL_ROBOT_IDS):
        raise ValueError('exactly four logical robot instances are required')
    robot_ids = tuple(item.robot_id for item in instances)
    if set(robot_ids) != set(CANONICAL_ROBOT_IDS):
        raise ValueError('robot_id values must be arm_01 through arm_04')
    _unique(robot_ids, 'robot_id')
    _unique(tuple(item.namespace for item in instances), 'namespace')
    _unique(tuple(item.tf_prefix for item in instances), 'tf_prefix')
    for instance in instances:
        _validate_role(instance)
    gazebo = tuple(item for item in instances if item.source == 'gazebo')
    if tuple(sorted(item.robot_id for item in gazebo)) != GAZEBO_ROBOT_IDS:
        raise ValueError('exactly arm_02 through arm_04 may use Gazebo')
    _unique(
        tuple(item.entity_name for item in gazebo if item.entity_name is not None),
        'entity_name',
    )
    _unique(
        tuple(
            item.controller_manager_name for item in gazebo
            if item.controller_manager_name is not None
        ),
        'controller_manager_name',
    )
    _unique(
        tuple(
            item.arm_controller_name for item in gazebo
            if item.arm_controller_name is not None
        ),
        'arm_controller_name',
    )
    return HybridThreeSimScenario(
        reference_frame=REFERENCE_FRAME,
        instances=tuple(sorted(instances, key=lambda item: item.robot_id)),
    )
