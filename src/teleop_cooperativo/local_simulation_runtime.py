"""Deterministic local runtime used to exercise H3 without ROS or Gazebo.

The runtime owns the only call to :class:`SimulationRunner`. A gateway uses
its ``tick`` result instead of calling ``arbiter.tick`` separately, so leases
expire and the simulated tool move from the very same command cycle.
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable

from .arbiter import ArbiterTick, CooperativeArbiter
from .command_composer import Twist
from .simulation_port import SimulationPort, ToolPose
from .simulation_runner import RunnerFault, SimulationRunner


@dataclass(frozen=True)
class LocalSimulationTick:
    """One coherent command, pose and elapsed-time observation."""

    arbiter_tick: ArbiterTick
    pose: ToolPose
    twist: Twist
    dt_sec: float


class LocalSimulationRuntime:
    """Connect one arbiter to a deterministic simulation using one clock."""

    def __init__(
        self,
        arbiter: CooperativeArbiter,
        simulator: SimulationPort,
        *,
        clock: Callable[[], float] = time.monotonic,
        maximum_step_sec: float = 0.10,
    ) -> None:
        self.arbiter = arbiter
        self.simulator = simulator
        self._clock = clock
        self._runner = SimulationRunner(
            arbiter, simulator, maximum_step_sec=maximum_step_sec
        )

    @property
    def started(self) -> bool:
        """Whether the simulator has passed its readiness gate."""
        return self._runner.started

    def start(self) -> None:
        """Arm the simulation and room from the canonical local clock."""
        self._runner.start(now=self._clock())

    def tick(self) -> LocalSimulationTick:
        """Advance the arbiter and simulator exactly once."""
        step = self._runner.step(now=self._clock())
        return LocalSimulationTick(
            arbiter_tick=step.arbiter,
            pose=step.pose,
            twist=step.arbiter.twist,
            dt_sec=step.dt_sec,
        )

    def stop(self) -> None:
        """Force the same safe stop behavior as the regular runner."""
        self._runner.stop()

    def safe_telemetry(self) -> LocalSimulationTick:
        """Return a zero-command observation after a runner fault.

        ``SimulationRunner`` has already stopped the simulator and faulted the
        arbiter before raising ``RunnerFault``. This method lets the gateway
        report that safe state without attempting another simulator step.
        """
        tick = self.arbiter.tick()
        return LocalSimulationTick(
            arbiter_tick=tick,
            pose=self.simulator.pose,
            twist=tick.twist,
            dt_sec=0.0,
        )


__all__ = ["LocalSimulationRuntime", "LocalSimulationTick", "RunnerFault"]
