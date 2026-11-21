"""
Versioned, immutable safety geometry profiles for robot tools.

This module deliberately has no ROS dependency.  It provides the contract that
will connect a future gripper CAD model to URDF, predictive safety, telemetry,
and multi-arm coordination without relying on an unversioned visual mesh.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import re
from typing import Tuple, Union

from thesis_core.jaco_kinematics import CAPSULE_RADII, SEGMENT_NAMES


Point3 = Tuple[float, float, float]

CLOSED = 'cerrado'
NOMINAL = 'nominal'
OPEN = 'abierto'
WITH_PAYLOAD = 'con_carga'
UNKNOWN = 'desconocido'
CONFIGURED_STATES = (CLOSED, NOMINAL, OPEN, WITH_PAYLOAD)
VALID_STATES = CONFIGURED_STATES + (UNKNOWN,)

_SHA256 = re.compile(r'^[0-9a-f]{64}$')


def _identifier(value: str, field_name: str) -> str:
    """Return a non-empty identifier while rejecting implicit whitespace."""
    if not isinstance(value, str) or not value.strip() or value != value.strip():
        raise ValueError(f'{field_name} must be a non-empty trimmed string')
    return value


def _point(values, field_name: str) -> Point3:
    """Return exactly three finite coordinates."""
    try:
        point = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exception:
        raise ValueError(f'{field_name} must contain three finite values') from exception
    if len(point) != 3 or not all(math.isfinite(value) for value in point):
        raise ValueError(f'{field_name} must contain three finite values')
    return point


def _radius(value: float, field_name: str) -> float:
    """Return a strictly positive finite radius."""
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f'{field_name} must be finite and greater than zero')
    return result


@dataclass(frozen=True)
class CapsulePrimitive:
    """A capsule with endpoint offsets expressed in their respective frames."""

    name: str
    start_frame: str
    end_frame: str
    radius: float
    start_offset: Point3 = (0.0, 0.0, 0.0)
    end_offset: Point3 = (0.0, 0.0, 0.0)

    def __post_init__(self):
        object.__setattr__(self, 'name', _identifier(self.name, 'capsule name'))
        object.__setattr__(
            self, 'start_frame', _identifier(self.start_frame, 'start_frame')
        )
        object.__setattr__(
            self, 'end_frame', _identifier(self.end_frame, 'end_frame')
        )
        object.__setattr__(self, 'radius', _radius(self.radius, 'capsule radius'))
        object.__setattr__(
            self, 'start_offset', _point(self.start_offset, 'start_offset')
        )
        object.__setattr__(
            self, 'end_offset', _point(self.end_offset, 'end_offset')
        )

    def as_dict(self):
        """Return a JSON-compatible description with explicit primitive kind."""
        return {
            'type': 'capsule',
            'name': self.name,
            'start_frame': self.start_frame,
            'end_frame': self.end_frame,
            'radius': self.radius,
            'start_offset': self.start_offset,
            'end_offset': self.end_offset,
        }


@dataclass(frozen=True)
class SpherePrimitive:
    """A sphere whose center is expressed in one named frame."""

    name: str
    frame: str
    radius: float
    center: Point3 = (0.0, 0.0, 0.0)

    def __post_init__(self):
        object.__setattr__(self, 'name', _identifier(self.name, 'sphere name'))
        object.__setattr__(self, 'frame', _identifier(self.frame, 'sphere frame'))
        object.__setattr__(self, 'radius', _radius(self.radius, 'sphere radius'))
        object.__setattr__(self, 'center', _point(self.center, 'sphere center'))

    def as_dict(self):
        """Return a JSON-compatible description with explicit primitive kind."""
        return {
            'type': 'sphere',
            'name': self.name,
            'frame': self.frame,
            'radius': self.radius,
            'center': self.center,
        }


Primitive = Union[CapsulePrimitive, SpherePrimitive]


@dataclass(frozen=True)
class StateEnvelope:
    """Non-empty set of safety primitives for one declared tool state."""

    state: str
    primitives: Tuple[Primitive, ...]

    def __post_init__(self):
        state = _identifier(self.state, 'state')
        if state not in CONFIGURED_STATES:
            raise ValueError(f'unsupported configured state: {state!r}')
        primitives = tuple(self.primitives)
        if not primitives:
            raise ValueError('state envelope must contain at least one primitive')
        if not all(isinstance(item, (CapsulePrimitive, SpherePrimitive))
                   for item in primitives):
            raise ValueError('state envelope contains an unsupported primitive')
        names = tuple(item.name for item in primitives)
        if len(names) != len(set(names)):
            raise ValueError('primitive names must be unique inside one state')
        object.__setattr__(self, 'state', state)
        object.__setattr__(self, 'primitives', primitives)

    def as_dict(self):
        """Return the envelope as a JSON-compatible dictionary."""
        return {
            'state': self.state,
            'primitives': tuple(item.as_dict() for item in self.primitives),
        }


@dataclass(frozen=True)
class GeometryProfile:
    """Immutable safety geometry tied to a robot, tool revision, and assets."""

    robot_id: str
    profile_id: str
    geometry_version: str
    asset_hash: str
    mount_frame: str
    tcp_frame: str
    envelopes: Tuple[StateEnvelope, ...]

    def __post_init__(self):
        object.__setattr__(self, 'robot_id', _identifier(self.robot_id, 'robot_id'))
        object.__setattr__(
            self, 'profile_id', _identifier(self.profile_id, 'profile_id')
        )
        object.__setattr__(
            self, 'geometry_version',
            _identifier(self.geometry_version, 'geometry_version'),
        )
        asset_hash = _identifier(self.asset_hash, 'asset_hash')
        if _SHA256.fullmatch(asset_hash) is None:
            raise ValueError('asset_hash must be a lowercase SHA-256 digest')
        object.__setattr__(self, 'asset_hash', asset_hash)
        object.__setattr__(
            self, 'mount_frame', _identifier(self.mount_frame, 'mount_frame')
        )
        object.__setattr__(
            self, 'tcp_frame', _identifier(self.tcp_frame, 'tcp_frame')
        )
        envelopes = tuple(self.envelopes)
        if not envelopes:
            raise ValueError('geometry profile must define at least one envelope')
        if not all(isinstance(item, StateEnvelope) for item in envelopes):
            raise ValueError('geometry profile contains an invalid envelope')
        states = tuple(item.state for item in envelopes)
        if len(states) != len(set(states)):
            raise ValueError('geometry profile states must be unique')
        object.__setattr__(self, 'envelopes', envelopes)

    def envelope_for(self, state: str) -> StateEnvelope:
        """Resolve a state, failing closed when unknown could carry a payload."""
        requested = _identifier(state, 'requested state')
        if requested not in VALID_STATES:
            raise ValueError(f'unsupported requested state: {requested!r}')
        if requested == UNKNOWN and any(
                envelope.state == WITH_PAYLOAD
                for envelope in self.envelopes):
            raise ValueError(
                'unknown state with a payload envelope fails closed'
            )
        effective = OPEN if requested == UNKNOWN else requested
        for envelope in self.envelopes:
            if envelope.state == effective:
                return envelope
        if requested == UNKNOWN:
            raise ValueError('unknown state requires a configured open envelope')
        raise ValueError(
            f'profile {self.profile_id!r} has no envelope for {requested!r}'
        )

    def as_dict(self):
        """Return a deterministic JSON-compatible profile description."""
        return {
            'robot_id': self.robot_id,
            'profile_id': self.profile_id,
            'geometry_version': self.geometry_version,
            'asset_hash': self.asset_hash,
            'mount_frame': self.mount_frame,
            'tcp_frame': self.tcp_frame,
            'envelopes': tuple(item.as_dict() for item in self.envelopes),
        }


@dataclass(frozen=True)
class GeometrySnapshot:
    """Immutable, time-bounded selection of a versioned geometry profile."""

    profile: GeometryProfile
    state: str
    stamp_sec: float
    ttl_sec: float

    def __post_init__(self):
        if not isinstance(self.profile, GeometryProfile):
            raise ValueError('snapshot profile must be a GeometryProfile')
        state = _identifier(self.state, 'snapshot state')
        if state not in VALID_STATES:
            raise ValueError(f'unsupported snapshot state: {state!r}')
        stamp = float(self.stamp_sec)
        ttl = float(self.ttl_sec)
        if not math.isfinite(stamp) or stamp < 0.0:
            raise ValueError('snapshot stamp_sec must be finite and non-negative')
        if not math.isfinite(ttl) or ttl <= 0.0:
            raise ValueError('snapshot ttl_sec must be finite and positive')
        self.profile.envelope_for(state)
        object.__setattr__(self, 'state', state)
        object.__setattr__(self, 'stamp_sec', stamp)
        object.__setattr__(self, 'ttl_sec', ttl)

    @property
    def effective_state(self) -> str:
        """Return the state actually used by geometry evaluation."""
        return OPEN if self.state == UNKNOWN else self.state

    @property
    def primitives(self) -> Tuple[Primitive, ...]:
        """Return the selected conservative envelope primitives."""
        return self.profile.envelope_for(self.state).primitives

    def is_fresh(self, now_sec: float) -> bool:
        """Return whether the snapshot is valid at a finite clock value."""
        now = float(now_sec)
        if not math.isfinite(now):
            return False
        return self.stamp_sec <= now <= self.stamp_sec + self.ttl_sec

    def as_dict(self):
        """Return a deterministic, JSON-compatible immutable snapshot view."""
        return {
            'profile': self.profile.as_dict(),
            'state': self.state,
            'effective_state': self.effective_state,
            'stamp_sec': self.stamp_sec,
            'ttl_sec': self.ttl_sec,
            'primitives': tuple(item.as_dict() for item in self.primitives),
        }

    def to_json(self) -> str:
        """Serialize the snapshot without mutable object references."""
        return json.dumps(self.as_dict(), sort_keys=True, separators=(',', ':'))


_LEGACY_FRAME_PAIRS = (
    ('j2n6s300_link_base', 'j2n6s300_link_1'),
    ('j2n6s300_link_1', 'j2n6s300_link_2'),
    ('j2n6s300_link_2', 'j2n6s300_link_3'),
    ('j2n6s300_link_3', 'j2n6s300_link_4'),
    ('j2n6s300_link_4', 'j2n6s300_link_5'),
    ('j2n6s300_link_5', 'j2n6s300_end_effector'),
)
_LEGACY_ASSET_HASH = hashlib.sha256(
    b'jaco2-legacy-six-capsule-profile-v1'
).hexdigest()
_OFFLINE_FIXTURE_ASSET_HASH = hashlib.sha256(
    b'jaco2-plus-offline-box-fixture-0.10x0.10x0.12-v1'
).hexdigest()
_SYNTHETIC_GRIPPER_ASSET_HASH = hashlib.sha256(
    b'jaco2-synthetic-gripper-swept-box-0.11x0.08x0.13-v1'
).hexdigest()


def legacy_jaco_geometry_profile(
    robot_id: str = 'arm_01',
    profile_id: str = 'jaco2_legacy',
    geometry_version: str = '1.0.0',
    frame_prefix: str = 'j2n6s300',
) -> GeometryProfile:
    """
    Build the immutable profile matching the current six-capsule model.

    The legacy model declares only empty-tool states. A held object therefore
    remains unavailable and cannot be mistaken for the old arm envelope.
    """
    prefix = _identifier(frame_prefix, 'frame_prefix')
    frame_pairs = tuple(
        (start.replace('j2n6s300', prefix, 1),
         end.replace('j2n6s300', prefix, 1))
        for start, end in _LEGACY_FRAME_PAIRS
    )
    primitives = tuple(
        CapsulePrimitive(name, start_frame, end_frame, radius)
        for name, radius, (start_frame, end_frame) in zip(
            SEGMENT_NAMES, CAPSULE_RADII, frame_pairs
        )
    )
    return GeometryProfile(
        robot_id=robot_id,
        profile_id=profile_id,
        geometry_version=geometry_version,
        asset_hash=_LEGACY_ASSET_HASH,
        mount_frame=f'{prefix}_link_6',
        tcp_frame=f'{prefix}_end_effector',
        envelopes=tuple(
            StateEnvelope(state, primitives)
            for state in (CLOSED, NOMINAL, OPEN)
        ),
    )


def offline_fixture_jaco_geometry_profile(
    robot_id: str,
    frame_prefix: str,
) -> GeometryProfile:
    """
    Compose the full legacy arm with the synthetic offline box fixture.

    The sphere is the circumscribed envelope of the 0.10 x 0.10 x 0.12 m
    test box declared by ``thesis_gripper_fixture.xacro``. It is test data,
    not a claimed dimension or safety model for the future thesis gripper.
    """
    prefix = _identifier(frame_prefix, 'frame_prefix')
    legacy = legacy_jaco_geometry_profile(
        robot_id=robot_id,
        frame_prefix=prefix,
    )
    arm_primitives = legacy.envelope_for(NOMINAL).primitives
    fixture_radius = math.sqrt(0.05 ** 2 + 0.05 ** 2 + 0.06 ** 2)
    primitives = arm_primitives + (
        SpherePrimitive(
            name='offline_fixture_envelope',
            frame=f'{prefix}_fixture_body',
            radius=fixture_radius,
            center=(0.0, 0.0, 0.06),
        ),
    )
    return GeometryProfile(
        robot_id=robot_id,
        profile_id='jaco2_offline_fixture',
        geometry_version='1.0.0',
        asset_hash=_OFFLINE_FIXTURE_ASSET_HASH,
        mount_frame=f'{prefix}_tool_mount',
        tcp_frame=f'{prefix}_gripper_tcp',
        envelopes=tuple(
            StateEnvelope(state, primitives)
            for state in (CLOSED, NOMINAL, OPEN)
        ),
    )


def synthetic_gripper_jaco_geometry_profile(
    robot_id: str,
    frame_prefix: str,
) -> GeometryProfile:
    """Cover the simulated gripper throughout its entire unloaded stroke.

    The sphere encloses the swept box x +/-0.055, y +/-0.04, z 0..0.13 m
    relative to gripper_base, with an additional 1 mm margin. The same
    envelope is used for every state, including unknown and transitions.
    This is a synthetic test model, not the measured thesis gripper. Payloads
    are deliberately unsupported and must not be represented as empty space.
    """
    prefix = _identifier(frame_prefix, 'frame_prefix')
    legacy = legacy_jaco_geometry_profile(robot_id=robot_id, frame_prefix=prefix)
    radius = math.sqrt(0.055 ** 2 + 0.04 ** 2 + 0.065 ** 2) + 0.001
    primitives = legacy.envelope_for(NOMINAL).primitives + (
        SpherePrimitive('synthetic_gripper_swept_envelope',
                        f'{prefix}_gripper_base', radius, (0.0, 0.0, 0.065)),
    )
    return GeometryProfile(
        robot_id=robot_id,
        profile_id='jaco2_synthetic_gripper',
        geometry_version='1.0.0',
        asset_hash=_SYNTHETIC_GRIPPER_ASSET_HASH,
        mount_frame=f'{prefix}_tool_mount',
        tcp_frame=f'{prefix}_gripper_tcp',
        envelopes=tuple(StateEnvelope(state, primitives)
                        for state in (CLOSED, NOMINAL, OPEN)),
    )
