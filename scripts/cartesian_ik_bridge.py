#!/usr/bin/env python3
"""Convert the gateway's sole Cartesian Twist into safe Gazebo trajectories.

Run only with the independent H2c substitute.  This is not a Jaco2 driver.
The browser never communicates with this process; only the mother gateway may
publish ``/teleop/cartesian_twist``.
"""

from __future__ import annotations

import argparse
import math
import time

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist as RosTwist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from teleop_cooperativo.command_composer import Twist
from teleop_cooperativo.differential_kinematics import (
    JOINT_LIMITS,
    JOINT_NAMES,
    twist_to_joint_command,
)


class CartesianIkBridge(Node):
    """Short-horizon, position-controlled bridge for the Gazebo substitute."""

    def __init__(self, *, rate_hz: float, input_timeout_sec: float) -> None:
        super().__init__("course_cartesian_ik_bridge")
        self._period = 1.0 / rate_hz
        self._input_timeout_sec = input_timeout_sec
        self._positions: tuple[float, ...] | None = None
        self._state_seen_at: float | None = None
        self._twist = Twist((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self._twist_seen_at: float | None = None
        self._twist_valid = True
        self._controllers_active = False
        self._last_controller_check = 0.0
        self._controller_request = None
        self.create_subscription(RosTwist, "/teleop/cartesian_twist", self._on_twist, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._controllers = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._trajectory = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self._ready = self.create_publisher(Bool, "/teleop/bridge_ready", 10)
        self.create_timer(self._period, self._control_tick)

    def _on_twist(self, message: RosTwist) -> None:
        candidate = Twist(
            (message.linear.x, message.linear.y, message.linear.z),
            (message.angular.x, message.angular.y, message.angular.z),
        )
        self._twist_valid = all(
            math.isfinite(value)
            for value in (*candidate.linear, *candidate.angular)
        )
        self._twist = candidate if self._twist_valid else Twist((0.0,) * 3, (0.0,) * 3)
        self._twist_seen_at = time.monotonic()
        if not self._twist_valid:
            self.get_logger().error("rejecting non-finite Cartesian Twist; bridge is unsafe")

    def _on_joint_state(self, message: JointState) -> None:
        indexed = dict(zip(message.name, message.position))
        if all(name in indexed for name in JOINT_NAMES):
            candidate = tuple(float(indexed[name]) for name in JOINT_NAMES)
            if all(
                math.isfinite(value) and limits[0] <= value <= limits[1]
                for value, limits in zip(candidate, JOINT_LIMITS)
            ):
                self._positions = candidate
                self._state_seen_at = time.monotonic()
            else:
                self._positions = None
                self._state_seen_at = None
                self.get_logger().error("joint state is non-finite or outside configured limits")

    def _poll_controllers(self, now: float) -> None:
        if now - self._last_controller_check < 0.5:
            return
        self._last_controller_check = now
        if not self._controllers.service_is_ready():
            self._controllers_active = False
            return
        if self._controller_request is not None and not self._controller_request.done():
            return
        self._controller_request = self._controllers.call_async(ListControllers.Request())
        self._controller_request.add_done_callback(self._on_controller_list)

    def _on_controller_list(self, future) -> None:
        try:
            response = future.result()
            states = {controller.name: controller.state for controller in response.controller}
            self._controllers_active = (
                states.get("joint_state_broadcaster") == "active"
                and states.get("arm_controller") == "active"
            )
        except Exception as error:
            self._controllers_active = False
            self.get_logger().error(f"cannot read controller status: {error}")

    def _control_tick(self) -> None:
        now = time.monotonic()
        self._poll_controllers(now)
        state_fresh = (
            self._positions is not None
            and self._state_seen_at is not None
            and now - self._state_seen_at <= self._input_timeout_sec
        )
        bridge_ready = state_fresh and self._controllers_active and self._twist_valid
        self._ready.publish(Bool(data=bridge_ready))
        if not bridge_ready:
            return
        active_twist = self._twist
        if self._twist_seen_at is None or now - self._twist_seen_at > self._input_timeout_sec:
            active_twist = Twist((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        try:
            command = twist_to_joint_command(self._positions, active_twist, self._period)
        except ValueError as error:
            self.get_logger().error(f"rejecting invalid Cartesian command: {error}")
            return
        trajectory = JointTrajectory()
        trajectory.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = list(command.target_positions)
        point.time_from_start.sec = int(self._period)
        point.time_from_start.nanosec = int((self._period % 1.0) * 1_000_000_000)
        trajectory.points = [point]
        self._trajectory.publish(trajectory)


def main() -> int:
    parser = argparse.ArgumentParser(description="Cartesian IK bridge for H2c Gazebo substitute")
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--input-timeout-sec", type=float, default=0.15)
    arguments = parser.parse_args()
    if arguments.rate_hz <= 0.0 or arguments.input_timeout_sec <= 0.0:
        parser.error("rate and timeout must be positive")
    rclpy.init()
    node = CartesianIkBridge(
        rate_hz=arguments.rate_hz, input_timeout_sec=arguments.input_timeout_sec
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # SIGINT can shut down the rclpy context before a waiting executor
        # observes it on Humble.  Treat only that shutdown race as clean.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
