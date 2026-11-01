"""Local authority that turns valid shared input into one safe twist."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Mapping

from .command_composer import Twist, compose_twist
from .domain import Axis
from .lease_manager import Lease, LeaseManager, MaintenanceResult


class RoomState(str, Enum):
    """Lifecycle of the local control room."""

    NOT_READY = "NOT_READY"
    READY = "READY"
    ACTIVE = "ACTIVE"
    FAULT = "FAULT"


@dataclass(frozen=True)
class ArbiterTick:
    """One complete local control cycle and the command it produced."""

    state: RoomState
    twist: Twist
    maintenance: MaintenanceResult


class CooperativeArbiter:
    """Own room readiness, leases and the unique resulting Cartesian command."""

    def __init__(
        self,
        leases: LeaseManager | None = None,
        *,
        max_linear_speed_mps: float = 0.05,
        max_angular_speed_radps: float = 0.175,
    ) -> None:
        self.leases = leases if leases is not None else LeaseManager()
        self.max_linear_speed_mps = max_linear_speed_mps
        self.max_angular_speed_radps = max_angular_speed_radps
        self._state = RoomState.NOT_READY

    @property
    def state(self) -> RoomState:
        """Return the room state without exposing mutable internals."""
        return self._state

    def mark_ready(self) -> None:
        """Permit control only after the local simulation is available."""
        if self._state in {RoomState.NOT_READY, RoomState.FAULT}:
            self._state = RoomState.READY

    def claim(
        self,
        session_id: str,
        axes: Iterable[Axis | str],
        *,
        now: float | None = None,
    ) -> tuple[Lease, ...]:
        """Grant an axis claim only while the room is operational."""
        self._require_operational()
        granted = self.leases.claim(session_id, axes, now=now)
        self._state = RoomState.ACTIVE
        return granted

    def submit_input(
        self,
        session_id: str,
        sequence: int,
        values: Mapping[Axis | str, object],
        tokens: Mapping[Axis | str, tuple[str, int]],
        *,
        now: float | None = None,
    ) -> None:
        """Forward an already authenticated input to the lease manager."""
        self._require_operational()
        self.leases.submit_input(session_id, sequence, values, tokens, now=now)

    def heartbeat(self, session_id: str, *, now: float | None = None) -> None:
        """Forward a liveness signal for an existing session."""
        self._require_operational()
        self.leases.heartbeat(session_id, now=now)

    def release(
        self,
        session_id: str,
        axes: Iterable[Axis | str] | None = None,
    ) -> tuple[Axis, ...]:
        """Release axes and return to READY when the room becomes empty."""
        self._require_operational()
        released = self.leases.release(session_id, axes)
        self._state = RoomState.ACTIVE if self._has_leases() else RoomState.READY
        return released

    def tick(self, *, now: float | None = None) -> ArbiterTick:
        """Expire stale state, compose one command and expose current room state."""
        if self._state in {RoomState.NOT_READY, RoomState.FAULT}:
            return ArbiterTick(self._state, self._zero_twist(), MaintenanceResult((), ()))

        maintenance = self.leases.advance(now=now)
        self._state = RoomState.ACTIVE if self._has_leases() else RoomState.READY
        return ArbiterTick(
            self._state,
            compose_twist(
                self.leases.values(),
                max_linear_speed_mps=self.max_linear_speed_mps,
                max_angular_speed_radps=self.max_angular_speed_radps,
            ),
            maintenance,
        )

    def fault(self) -> tuple[Axis, ...]:
        """Immediately clear authority after a simulation or host failure."""
        released = self.leases.stop_all()
        self._state = RoomState.FAULT
        return released

    def stop_global(self) -> tuple[Axis, ...]:
        """Host-controlled stop that requires a later explicit mark_ready call."""
        return self.fault()

    def _has_leases(self) -> bool:
        return any(self.leases.lease(axis) is not None for axis in Axis)

    def _require_operational(self) -> None:
        if self._state not in {RoomState.READY, RoomState.ACTIVE}:
            raise RuntimeError(f"room is not operational: {self._state.value}")

    def _zero_twist(self) -> Twist:
        return compose_twist(
            {Axis.X: 0.0},
            max_linear_speed_mps=self.max_linear_speed_mps,
            max_angular_speed_radps=self.max_angular_speed_radps,
        )
