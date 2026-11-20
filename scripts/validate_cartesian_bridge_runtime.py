#!/usr/bin/env python3
"""Validate the single-host Cartesian bridge against the H2c substitute.

Start Gazebo and ``cartesian_ik_bridge.py`` first.  This script intentionally
publishes only to the bridge input; it never bypasses the controller with joint
commands and is not a substitute for the gateway's protocol tests.
"""

from __future__ import annotations

import json
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from teleop_cooperativo.differential_kinematics import JOINT_NAMES


class CartesianBridgeValidator(Node):
    def __init__(self) -> None:
        super().__init__("course_cartesian_bridge_validator")
        self.ready = False
        self.positions: tuple[float, ...] | None = None
        self.create_subscription(Bool, "/teleop/bridge_ready", self._on_ready, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.publisher = self.create_publisher(Twist, "/teleop/cartesian_twist", 10)

    def _on_ready(self, message: Bool) -> None:
        self.ready = bool(message.data)

    def _on_joint_state(self, message: JointState) -> None:
        indexed = dict(zip(message.name, message.position))
        if all(name in indexed for name in JOINT_NAMES):
            self.positions = tuple(float(indexed[name]) for name in JOINT_NAMES)

    def wait_until(self, predicate, timeout_sec: float, detail: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {detail}")

    def publish_for(self, message: Twist, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            self.publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self) -> dict[str, object]:
        self.wait_until(lambda: self.ready, 20.0, "bridge readiness")
        self.wait_until(lambda: self.positions is not None, 5.0, "six joint states")
        initial = self.positions
        assert initial is not None
        forward = Twist()
        forward.linear.x = 0.02
        self.publish_for(forward, 0.7)
        moved = self.positions
        assert moved is not None
        movement = [after - before for before, after in zip(initial, moved)]
        if max(abs(value) for value in movement) < 0.002:
            raise RuntimeError(f"no measurable joint response: {movement}")
        self.publish_for(Twist(), 0.25)
        stopped = self.positions
        assert stopped is not None
        self.wait_until(lambda: self.positions is not None, 0.2, "post-stop joint state")
        time.sleep(0.25)
        rclpy.spin_once(self, timeout_sec=0.1)
        settled = self.positions
        assert settled is not None
        residual = [after - before for before, after in zip(stopped, settled)]
        if max(abs(value) for value in residual) > 0.02:
            raise RuntimeError(f"joint motion did not settle after zero twist: {residual}")
        return {
            "bridge_ready": self.ready,
            "initial_positions": initial,
            "moved_positions": moved,
            "joint_displacement": movement,
            "post_zero_residual": residual,
            "result": "PASS",
        }


def main() -> int:
    rclpy.init()
    node = CartesianBridgeValidator()
    try:
        print(json.dumps(node.run(), indent=2))
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
