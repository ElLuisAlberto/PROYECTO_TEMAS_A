"""Independent virtual three-finger gripper; never sends hardware commands."""
from __future__ import annotations

import math
import time
from typing import Callable


class SimulatedGripper:
    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._last = clock()
        self._angle = 0.0
        self._maximum = math.radians(80.0)
        self._max_speed = math.radians(20.0)

    @property
    def positions(self) -> tuple[float, float, float]:
        return (self._angle, self._angle, self._angle)

    def step(self, command: float) -> None:
        now = self._clock()
        dt = min(max(now - self._last, 0.0), 0.1)
        self._last = now
        if not math.isfinite(command):
            command = 0.0
        velocity = max(-1.0, min(1.0, command)) * self._max_speed
        self._angle = max(0.0, min(self._maximum, self._angle + velocity * dt))
