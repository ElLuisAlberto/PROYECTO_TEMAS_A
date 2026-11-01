"""Fail-safe runner between the cooperative arbiter and a simulator port."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .arbiter import ArbiterTick, CooperativeArbiter, RoomState
from .simulation_port import SimulationPort, ToolPose


class RunnerFault(RuntimeError):
    """Raised after simulator or timing failure has revoked all authority."""


@dataclass(frozen=True)
class RunnerStep:
    """One successful arbiter-to-simulator step."""

    arbiter: ArbiterTick
    pose: ToolPose
    dt_sec: float


class SimulationRunner:
    """Advance one simulator without allowing stale time to create pose jumps."""

    def __init__(
        self,
        arbiter: CooperativeArbiter,
        simulator: SimulationPort,
        *,
        maximum_step_sec: float = 0.10,
    ) -> None:
        if not math.isfinite(maximum_step_sec) or maximum_step_sec <= 0.0:
            raise ValueError("maximum_step_sec must be positive and finite")
        self.arbiter = arbiter
        self.simulator = simulator
        self.maximum_step_sec = float(maximum_step_sec)
        self._last_step_monotonic: float | None = None

    def start(self, *, now: float) -> None:
        """Arm the local room only when the simulator is already ready."""
        timestamp = self._finite_time(now)
        if not self.simulator.ready:
            raise RunnerFault("simulator is not ready")
        self._last_step_monotonic = timestamp
        self.arbiter.mark_ready()

    def step(self, *, now: float) -> RunnerStep:
        """Compose and apply one command; any failure moves the room to FAULT."""
        timestamp = self._finite_time(now)
        if self._last_step_monotonic is None:
            raise RunnerFault("runner has not been started")
        dt_sec = timestamp - self._last_step_monotonic
        if dt_sec <= 0.0 or dt_sec > self.maximum_step_sec:
            self._fail("invalid or excessive simulation time step")
        if not self.simulator.ready:
            self._fail("simulator lost readiness")
        try:
            tick = self.arbiter.tick(now=timestamp)
            if tick.state in {RoomState.NOT_READY, RoomState.FAULT}:
                self._fail(f"arbiter is not operational: {tick.state.value}")
            pose = self.simulator.apply_twist(tick.twist, dt_sec)
        except Exception as error:
            self._fail(f"simulator step failed: {error}", cause=error)
        self._last_step_monotonic = timestamp
        return RunnerStep(tick, pose, dt_sec)

    def stop(self) -> None:
        """Stop output, revoke leases and require a fresh start."""
        self.simulator.stop()
        self.arbiter.stop_global()
        self._last_step_monotonic = None

    def _fail(self, detail: str, *, cause: Exception | None = None) -> None:
        self.simulator.stop()
        if self.arbiter.state is not RoomState.FAULT:
            self.arbiter.fault()
        self._last_step_monotonic = None
        if cause is None:
            raise RunnerFault(detail)
        raise RunnerFault(detail) from cause

    @staticmethod
    def _finite_time(value: object) -> float:
        if isinstance(value, bool):
            raise RunnerFault("runner time must be finite")
        try:
            timestamp = float(value)
        except (TypeError, ValueError) as error:
            raise RunnerFault("runner time must be finite") from error
        if not math.isfinite(timestamp):
            raise RunnerFault("runner time must be finite")
        return timestamp
