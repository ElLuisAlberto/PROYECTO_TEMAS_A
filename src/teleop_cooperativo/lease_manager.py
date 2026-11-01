"""Exclusive axis reservations and expiry rules."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable, Iterable, Mapping
from uuid import uuid4

from .domain import AXES, Axis, canonical_axes, canonical_values


class ClaimDenied(ValueError):
    """Raised when an atomic axis claim cannot be granted."""


class InputRejected(ValueError):
    """Raised when an input cannot update the current room state."""


@dataclass
class Lease:
    """Current ownership and last accepted state for one axis."""

    axis: Axis
    session_id: str
    lease_id: str
    generation: int
    last_input_monotonic: float
    last_heartbeat_monotonic: float
    value: float = 0.0


@dataclass(frozen=True)
class MaintenanceResult:
    """Axes changed by one periodic expiry pass."""

    zeroed_axes: tuple[Axis, ...]
    released_axes: tuple[Axis, ...]


class LeaseManager:
    """Own Cartesian axis leases using only the mother's monotonic clock."""

    def __init__(
        self,
        *,
        max_axes_per_session: int = 3,
        input_timeout_sec: float = 0.15,
        lease_timeout_sec: float = 0.25,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if max_axes_per_session < 1 or max_axes_per_session > len(AXES):
            raise ValueError("max_axes_per_session is out of range")
        if input_timeout_sec <= 0.0 or lease_timeout_sec <= input_timeout_sec:
            raise ValueError(
                "lease timeout must be greater than the positive input timeout"
            )
        self.max_axes_per_session = max_axes_per_session
        self.input_timeout_sec = float(input_timeout_sec)
        self.lease_timeout_sec = float(lease_timeout_sec)
        self._clock = clock
        self._leases: dict[Axis, Lease] = {}
        self._last_sequence: dict[str, int] = {}
        self._generation_by_axis: dict[Axis, int] = {axis: 0 for axis in AXES}
        self._last_timestamp: float | None = None

    def claim(
        self,
        session_id: str,
        axes: Iterable[Axis | str],
        *,
        now: float | None = None,
    ) -> tuple[Lease, ...]:
        """Atomically reserve free axes for a session."""
        session = self._validate_session(session_id)
        requested = canonical_axes(axes)
        if not requested:
            raise ClaimDenied("a claim must include at least one axis")
        owned = self.axes_for_session(session)
        if len(owned) + len(requested) > self.max_axes_per_session:
            raise ClaimDenied("session would exceed its axis limit")
        if any(axis in self._leases for axis in requested):
            raise ClaimDenied("one or more requested axes are unavailable")

        timestamp = self._timestamp(now)
        granted = []
        for axis in requested:
            generation = self._generation_by_axis[axis] + 1
            self._generation_by_axis[axis] = generation
            lease = Lease(
                axis=axis,
                session_id=session,
                lease_id=str(uuid4()),
                generation=generation,
                last_input_monotonic=timestamp,
                last_heartbeat_monotonic=timestamp,
            )
            self._leases[axis] = lease
            granted.append(lease)
        return tuple(granted)

    def submit_input(
        self,
        session_id: str,
        sequence: int,
        values: Mapping[Axis | str, object],
        tokens: Mapping[Axis | str, tuple[str, int]],
        *,
        now: float | None = None,
    ) -> None:
        """Atomically accept current values for axes owned by one session."""
        session = self._validate_session(session_id)
        if isinstance(sequence, bool) or not isinstance(sequence, int):
            raise InputRejected("sequence must be an integer")
        previous = self._last_sequence.get(session, -1)
        if sequence <= previous:
            raise InputRejected("sequence is stale or repeated")

        try:
            incoming = canonical_values(values)
        except ValueError as error:
            raise InputRejected(str(error)) from error
        if not isinstance(tokens, Mapping):
            raise InputRejected("lease tokens must be a mapping")

        validated: list[tuple[Lease, float]] = []
        for axis, value in incoming.items():
            lease = self._leases.get(axis)
            token = tokens.get(axis, tokens.get(axis.value))
            if lease is None or lease.session_id != session:
                raise InputRejected(f"session does not own {axis.value}")
            if (
                not isinstance(token, tuple)
                or len(token) != 2
                or token[0] != lease.lease_id
                or token[1] != lease.generation
            ):
                raise InputRejected(f"invalid lease token for {axis.value}")
            validated.append((lease, value))

        timestamp = self._timestamp(now)
        for lease, value in validated:
            lease.value = value
            lease.last_input_monotonic = timestamp
        for lease in self._leases.values():
            if lease.session_id == session:
                lease.last_heartbeat_monotonic = timestamp
        self._last_sequence[session] = sequence

    def heartbeat(self, session_id: str, *, now: float | None = None) -> None:
        """Keep all current leases for a session alive without adding motion."""
        session = self._validate_session(session_id)
        timestamp = self._timestamp(now)
        for lease in self._leases.values():
            if lease.session_id == session:
                lease.last_heartbeat_monotonic = timestamp

    def release(
        self,
        session_id: str,
        axes: Iterable[Axis | str] | None = None,
    ) -> tuple[Axis, ...]:
        """Release owned axes after clearing their command value."""
        session = self._validate_session(session_id)
        requested = (
            self.axes_for_session(session) if axes is None else canonical_axes(axes)
        )
        released = []
        for axis in requested:
            lease = self._leases.get(axis)
            if lease is not None and lease.session_id == session:
                lease.value = 0.0
                del self._leases[axis]
                released.append(axis)
        return tuple(released)

    def stop_all(self) -> tuple[Axis, ...]:
        """Clear every command and revoke every lease."""
        released = tuple(self._leases)
        self._leases.clear()
        return released

    def advance(self, *, now: float | None = None) -> MaintenanceResult:
        """Apply input and heartbeat expiry without relying on client clocks."""
        timestamp = self._timestamp(now)
        zeroed = []
        released = []
        for axis, lease in tuple(self._leases.items()):
            if timestamp - lease.last_heartbeat_monotonic >= self.lease_timeout_sec:
                lease.value = 0.0
                del self._leases[axis]
                released.append(axis)
                continue
            if (
                timestamp - lease.last_input_monotonic >= self.input_timeout_sec
                and lease.value != 0.0
            ):
                lease.value = 0.0
                zeroed.append(axis)
        return MaintenanceResult(tuple(zeroed), tuple(released))

    def axes_for_session(self, session_id: str) -> tuple[Axis, ...]:
        """Return the axes owned by a session in canonical order."""
        return tuple(
            axis for axis in AXES
            if axis in self._leases and self._leases[axis].session_id == session_id
        )

    def values(self) -> dict[Axis, float]:
        """Return all current axes, with zero for each unowned component."""
        return {
            axis: self._leases[axis].value if axis in self._leases else 0.0
            for axis in AXES
        }

    def lease(self, axis: Axis | str) -> Lease | None:
        """Read the active lease for one axis."""
        return self._leases.get(Axis.parse(axis))

    @staticmethod
    def _validate_session(session_id: str) -> str:
        if not isinstance(session_id, str) or not session_id.strip():
            raise ValueError("session_id must be a non-empty string")
        return session_id

    def _timestamp(self, now: float | None) -> float:
        timestamp = self._clock() if now is None else now
        if isinstance(timestamp, bool) or not isinstance(timestamp, (int, float)):
            raise ValueError("timestamp must be numeric")
        numeric = float(timestamp)
        if not math.isfinite(numeric):
            raise ValueError("timestamp must be finite")
        if self._last_timestamp is not None and numeric < self._last_timestamp:
            raise ValueError("monotonic timestamp moved backwards")
        self._last_timestamp = numeric
        return numeric
