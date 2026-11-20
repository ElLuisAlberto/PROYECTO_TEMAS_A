#!/usr/bin/env python3
"""Runtime movement-and-return validation for the Jaco2 Cartesian bridge.

Start Gazebo and ``jaco2_cartesian_ik_bridge.py`` first.  This validator uses
only the bridge input topic; it never sends joint trajectories or talks to a
controller directly.  It confirms readiness, a measurable small Cartesian
motion, a zero-twist hold, and an inverse Cartesian motion back near the
initial joint configuration.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from typing import Callable, Sequence

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from teleop_cooperativo.jaco2_kinematics import JOINT_NAMES


EXPECTED_JOINT_NAMES = tuple(f"j2n6s300_joint_{index}" for index in range(1, 7))
_CONTINUOUS_INDEXES = frozenset((0, 3, 4, 5))


class Jaco2CartesianBridgeValidator(Node):
    """Exercise only the public Cartesian input of the Jaco2 bridge."""

    def __init__(
        self,
        *,
        linear_speed: float,
        command_duration_sec: float,
        settle_duration_sec: float,
        movement_threshold_rad: float,
        return_tolerance_rad: float,
    ) -> None:
        if tuple(JOINT_NAMES) != EXPECTED_JOINT_NAMES:
            raise RuntimeError("jaco2_kinematics does not expose J2N6S300 joint names")
        super().__init__("jaco2_cartesian_bridge_validator")
        self._linear_speed = linear_speed
        self._command_duration_sec = command_duration_sec
        self._settle_duration_sec = settle_duration_sec
        self._movement_threshold_rad = movement_threshold_rad
        self._return_tolerance_rad = return_tolerance_rad
        self.ready = False
        self._ready_seen_at: float | None = None
        self.positions: tuple[float, ...] | None = None
        self._state_seen_at: float | None = None
        self.create_subscription(Bool, "/teleop/bridge_ready", self._on_ready, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.publisher = self.create_publisher(Twist, "/teleop/cartesian_twist", 10)

    def _on_ready(self, message: Bool) -> None:
        self.ready = bool(message.data)
        self._ready_seen_at = time.monotonic()

    def _on_joint_state(self, message: JointState) -> None:
        indexed = dict(zip(message.name, message.position))
        if not all(name in indexed for name in JOINT_NAMES):
            return
        candidate = tuple(float(indexed[name]) for name in JOINT_NAMES)
        if all(math.isfinite(value) for value in candidate):
            self.positions = candidate
            self._state_seen_at = time.monotonic()

    def wait_until(self, predicate: Callable[[], bool], timeout_sec: float, detail: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {detail}")

    def _ready_and_fresh(self) -> bool:
        now = time.monotonic()
        return (
            self.ready
            and self.positions is not None
            and self._ready_seen_at is not None
            and self._state_seen_at is not None
            and now - self._ready_seen_at <= 1.0
            and now - self._state_seen_at <= 1.0
        )

    def publish_for(self, message: Twist, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self.publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _hold(self) -> None:
        self.publish_for(Twist(), self._settle_duration_sec)
        self.wait_until(self._ready_and_fresh, 2.0, "fresh bridge readiness during hold")

    def run(self) -> dict[str, object]:
        self.wait_until(self._ready_and_fresh, 20.0, "Jaco2 bridge readiness and joint states")
        initial = _require_positions(self.positions)

        forward = Twist()
        forward.linear.x = self._linear_speed
        self.publish_for(forward, self._command_duration_sec)
        self._hold()
        moved = _require_positions(self.positions)
        movement = _joint_displacement(initial, moved)
        max_movement = max(abs(value) for value in movement)
        if max_movement < self._movement_threshold_rad:
            raise RuntimeError(f"no measurable Jaco2 joint response: {movement}")

        reverse = Twist()
        reverse.linear.x = -self._linear_speed
        self.publish_for(reverse, self._command_duration_sec)
        self._hold()
        returned = _require_positions(self.positions)
        return_error = _joint_displacement(initial, returned)
        max_return_error = max(abs(value) for value in return_error)
        if max_return_error > self._return_tolerance_rad:
            raise RuntimeError(
                "Jaco2 did not return near its initial configuration: "
                f"error={return_error}, tolerance={self._return_tolerance_rad}"
            )
        return {
            "bridge_ready": self.ready,
            "joint_names": list(JOINT_NAMES),
            "initial_positions": initial,
            "moved_positions": moved,
            "joint_displacement": movement,
            "returned_positions": returned,
            "return_error": return_error,
            "max_movement_rad": max_movement,
            "max_return_error_rad": max_return_error,
            "result": "PASS",
        }


def _require_positions(positions: tuple[float, ...] | None) -> tuple[float, ...]:
    if positions is None:
        raise RuntimeError("Jaco2 joint state disappeared during validation")
    return positions


def _joint_displacement(
    before: Sequence[float], after: Sequence[float]
) -> tuple[float, ...]:
    """Measure continuous Jaco joints by their shortest angular displacement."""
    result = []
    for index, (start, finish) in enumerate(zip(before, after)):
        delta = finish - start
        if index in _CONTINUOUS_INDEXES:
            delta = math.atan2(math.sin(delta), math.cos(delta))
        result.append(delta)
    return tuple(result)


def _positive(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return parsed


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate Jaco2 Cartesian bridge movement and return")
    parser.add_argument("--linear-speed", type=_positive, default=0.01)
    parser.add_argument("--command-duration-sec", type=_positive, default=0.5)
    parser.add_argument("--settle-duration-sec", type=_positive, default=0.35)
    parser.add_argument("--movement-threshold-rad", type=_positive, default=0.002)
    parser.add_argument("--return-tolerance-rad", type=_positive, default=0.04)
    arguments = parser.parse_args()
    if arguments.linear_speed > 0.05:
        parser.error("linear-speed must not exceed the 0.05 m/s gateway safety bound")

    rclpy.init()
    node = Jaco2CartesianBridgeValidator(
        linear_speed=arguments.linear_speed,
        command_duration_sec=arguments.command_duration_sec,
        settle_duration_sec=arguments.settle_duration_sec,
        movement_threshold_rad=arguments.movement_threshold_rad,
        return_tolerance_rad=arguments.return_tolerance_rad,
    )
    try:
        print(json.dumps(node.run(), indent=2))
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
