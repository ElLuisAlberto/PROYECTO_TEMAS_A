"""URDF-driven kinematics for robot profiles with arbitrary arm DOF."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def _vector(text: str | None, default: Sequence[float]) -> np.ndarray:
    values = np.asarray(default if text is None else text.split(), dtype=float)
    if values.shape != (3,) or not np.isfinite(values).all():
        raise ValueError('URDF vector must contain three finite values')
    return values


def _origin(element: ET.Element | None) -> np.ndarray:
    result = np.eye(4)
    if element is not None:
        result[:3, :3] = Rotation.from_euler(
            'xyz', _vector(element.get('rpy'), (0.0, 0.0, 0.0))
        ).as_matrix()
        result[:3, 3] = _vector(
            element.get('xyz'), (0.0, 0.0, 0.0)
        )
    return result


@dataclass(frozen=True)
class _Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    velocity: float | None
    mimic: tuple[str, float, float] | None


class RobotKinematics:
    """
    Evaluate one connected URDF tree in an explicit joint-vector order.

    Fixed and mimic joints participate in FK. Only independent actuated joints
    listed in ``joint_names`` form the command vector. Optional application
    limits can make a continuous URDF joint finite, as required by the legacy
    JACO2 safety contract.
    """

    def __init__(
        self,
        root: ET.Element,
        joint_names: Sequence[str],
        position_limits: Mapping[str, tuple[float, float]] | None = None,
    ) -> None:
        self.joint_names = tuple(str(name) for name in joint_names)
        if (
            not self.joint_names
            or len(set(self.joint_names)) != len(self.joint_names)
        ):
            raise ValueError('joint_names must be nonempty and unique')

        links = tuple(link.attrib['name'] for link in root.findall('link'))
        if not links or len(set(links)) != len(links):
            raise ValueError('URDF links must be nonempty and unique')
        self.link_names = links

        parsed: dict[str, _Joint] = {}
        children: set[str] = set()
        supported = {'fixed', 'revolute', 'continuous', 'prismatic'}
        for element in root.findall('joint'):
            name = element.attrib['name']
            kind = element.attrib['type']
            if name in parsed or kind not in supported:
                raise ValueError(f'unsupported or duplicate joint: {name}')
            parent_element = element.find('parent')
            child_element = element.find('child')
            if parent_element is None or child_element is None:
                raise ValueError(f'missing parent or child at {name}')
            parent = parent_element.attrib['link']
            child = child_element.attrib['link']
            if parent not in links or child not in links or child in children:
                raise ValueError(f'invalid joint tree at {name}')
            children.add(child)

            axis_element = element.find('axis')
            axis = _vector(
                axis_element.get('xyz')
                if axis_element is not None else None,
                (1.0, 0.0, 0.0),
            )
            if kind != 'fixed':
                norm = float(np.linalg.norm(axis))
                if norm <= 0.0:
                    raise ValueError(f'zero joint axis at {name}')
                axis = axis / norm

            lower, upper = -math.inf, math.inf
            limit = element.find('limit')
            if kind in {'revolute', 'prismatic'}:
                if (
                    limit is None
                    or 'lower' not in limit.attrib
                    or 'upper' not in limit.attrib
                ):
                    raise ValueError(f'missing position limits at {name}')
                lower = float(limit.attrib['lower'])
                upper = float(limit.attrib['upper'])
                if (
                    not math.isfinite(lower)
                    or not math.isfinite(upper)
                    or lower >= upper
                ):
                    raise ValueError(f'invalid position limits at {name}')
            velocity = None
            if kind != 'fixed' and limit is not None:
                raw_velocity = limit.get('velocity')
                if raw_velocity is not None:
                    velocity = float(raw_velocity)
                    if not math.isfinite(velocity) or velocity <= 0.0:
                        raise ValueError(f'invalid velocity limit at {name}')

            mimic_element = element.find('mimic')
            mimic = None
            if mimic_element is not None:
                multiplier = float(mimic_element.get('multiplier', '1'))
                offset = float(mimic_element.get('offset', '0'))
                if not math.isfinite(multiplier) or not math.isfinite(offset):
                    raise ValueError(f'non-finite mimic parameters at {name}')
                mimic = (
                    mimic_element.attrib['joint'], multiplier, offset
                )
            parsed[name] = _Joint(
                name=name,
                kind=kind,
                parent=parent,
                child=child,
                origin=_origin(element.find('origin')),
                axis=axis,
                lower=lower,
                upper=upper,
                velocity=velocity,
                mimic=mimic,
            )

        roots = set(links) - children
        if len(roots) != 1:
            raise ValueError('URDF must have exactly one root link')
        self.root_link = roots.pop()
        self._joints = parsed
        for name in self.joint_names:
            if (
                name not in parsed
                or parsed[name].kind == 'fixed'
                or parsed[name].mimic is not None
            ):
                raise ValueError(
                    f'not an independent actuated joint: {name}'
                )

        provided_limits = dict(position_limits or {})
        unknown_limits = set(provided_limits) - set(self.joint_names)
        if unknown_limits:
            raise ValueError(
                'position limits supplied for unknown selected joints: '
                + ', '.join(sorted(unknown_limits))
            )
        self.joint_limits: dict[str, tuple[float, float]] = {}
        for name in self.joint_names:
            lower, upper = provided_limits.get(
                name, (parsed[name].lower, parsed[name].upper)
            )
            lower, upper = float(lower), float(upper)
            if math.isnan(lower) or math.isnan(upper) or lower >= upper:
                raise ValueError(f'invalid application limits at {name}')
            if lower < parsed[name].lower or upper > parsed[name].upper:
                raise ValueError(
                    f'application limits exceed URDF limits at {name}'
                )
            self.joint_limits[name] = (lower, upper)

        self.joint_types = {
            name: parsed[name].kind for name in self.joint_names
        }
        self.velocity_limits = {
            name: parsed[name].velocity for name in self.joint_names
        }

        known = {self.root_link}
        pending = list(parsed.values())
        self._ordered: list[_Joint] = []
        while pending:
            ready = [joint for joint in pending if joint.parent in known]
            if not ready:
                raise ValueError('URDF contains a cycle or disconnected links')
            for joint in ready:
                self._ordered.append(joint)
                pending.remove(joint)
                known.add(joint.child)
        if known != set(links):
            raise ValueError('URDF contains disconnected links')

        # Resolve mimic dependencies now so invalid models fail on load.
        self._joint_values(dict.fromkeys(self.joint_names, 0.0))

    def _joint_values(
        self, supplied: Mapping[str, float]
    ) -> dict[str, float]:
        values = dict(supplied)

        def resolve(name: str, visiting: set[str]) -> float:
            if name in values:
                return values[name]
            if name not in self._joints or name in visiting:
                raise ValueError(f'unknown or cyclic mimic joint: {name}')
            joint = self._joints[name]
            if joint.mimic is not None:
                source, multiplier, offset = joint.mimic
                values[name] = (
                    multiplier * resolve(source, visiting | {name}) + offset
                )
            else:
                values[name] = 0.0
            return values[name]

        for name in self._joints:
            resolve(name, set())
        return values

    def _positions(self, positions: Sequence[float]) -> np.ndarray:
        values = np.asarray(positions, dtype=float)
        if (
            values.shape != (len(self.joint_names),)
            or not np.isfinite(values).all()
        ):
            raise ValueError(
                'positions must match the joint order and be finite'
            )
        for name, value in zip(self.joint_names, values):
            lower, upper = self.joint_limits[name]
            if value < lower - 1e-9 or value > upper + 1e-9:
                raise ValueError(
                    f'{name} position lies outside its configured limits'
                )
        return values

    def forward(
        self, positions: Sequence[float]
    ) -> dict[str, np.ndarray]:
        """Return every link's 4x4 pose in the URDF root frame."""
        vector = self._positions(positions)
        values = self._joint_values(dict(zip(self.joint_names, vector)))
        poses = {self.root_link: np.eye(4)}
        for joint in self._ordered:
            motion = np.eye(4)
            if joint.kind in {'revolute', 'continuous'}:
                motion[:3, :3] = Rotation.from_rotvec(
                    joint.axis * values[joint.name]
                ).as_matrix()
            elif joint.kind == 'prismatic':
                motion[:3, 3] = joint.axis * values[joint.name]
            poses[joint.child] = (
                poses[joint.parent] @ joint.origin @ motion
            )
        return poses

    def inverse(
        self,
        target: np.ndarray,
        seed: Sequence[float],
        tip_link: str,
        position_only: bool = False,
    ) -> tuple[float, ...]:
        """Solve bounded local IK or reject a target outside tolerance."""
        goal = np.asarray(target, dtype=float)
        if (
            goal.shape != (4, 4)
            or not np.isfinite(goal).all()
            or not np.allclose(
                goal[3], (0.0, 0.0, 0.0, 1.0), atol=1e-9
            )
            or not np.allclose(
                goal[:3, :3].T @ goal[:3, :3],
                np.eye(3),
                atol=1e-7,
            )
            or not np.isclose(
                np.linalg.det(goal[:3, :3]), 1.0, atol=1e-7
            )
        ):
            raise ValueError('target must be a finite rigid 4x4 transform')
        initial = self._positions(seed)
        if tip_link not in self.forward(initial):
            raise ValueError(f'unknown tip link: {tip_link}')

        def errors(values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            actual = self.forward(values)[tip_link]
            position = actual[:3, 3] - goal[:3, 3]
            orientation = Rotation.from_matrix(
                goal[:3, :3] @ actual[:3, :3].T
            ).as_rotvec()
            return position, orientation

        def residual(values: np.ndarray) -> np.ndarray:
            position, orientation = errors(values)
            if position_only:
                return position
            return np.concatenate((position, orientation))

        lower, upper = zip(*(
            self.joint_limits[name] for name in self.joint_names
        ))
        result = least_squares(
            residual,
            initial,
            bounds=(lower, upper),
            method='trf',
            max_nfev=500,
            ftol=1e-10,
            xtol=1e-10,
            gtol=1e-10,
        )
        position_error, orientation_error = errors(result.x)
        if (
            np.linalg.norm(position_error) > 0.003
            or (
                not position_only
                and np.linalg.norm(orientation_error) > math.radians(1.0)
            )
        ):
            raise ValueError(
                'IK target is unreachable from this seed within tolerance: '
                f'{np.linalg.norm(position_error):.6f} m, '
                f'{math.degrees(np.linalg.norm(orientation_error)):.3f} deg'
            )
        return tuple(float(value) for value in result.x)
