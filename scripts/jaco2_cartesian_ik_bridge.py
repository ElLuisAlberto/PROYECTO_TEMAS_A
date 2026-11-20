#!/usr/bin/env python3
"""Safely translate Cartesian twists into short Jaco2 Gazebo trajectories.

This node is exclusively for the simulated Kinova Jaco2 J2N6S300.  It accepts
Cartesian commands only from ``/teleop/cartesian_twist`` and sends bounded,
short-horizon position targets to ``arm_controller``.  It does not connect to
Kinova hardware and it deliberately publishes no command until the controller
and a fresh, valid six-joint state are available.

The future ``teleop_cooperativo.jaco2_kinematics`` module is the kinematic
authority.  Its required public contract mirrors the existing generic bridge:
``JOINT_NAMES``, ``JOINT_LIMITS`` and ``twist_to_joint_command`` returning an
object with finite ``target_positions``.
"""

from __future__ import annotations

import argparse
import math
import time
from typing import Sequence

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist as RosTwist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from teleop_cooperativo.command_composer import Twist
from teleop_cooperativo.jaco2_kinematics import (
    JOINT_LIMITS,
    JOINT_NAMES,
    twist_to_joint_command,
)


EXPECTED_JOINT_NAMES = tuple(f"j2n6s300_joint_{index}" for index in range(1, 7))
_CONTROLLER_POLL_SEC = 0.5
_MIN_HORIZON_SEC = 0.05
_MAX_HORIZON_SEC = 0.10
# Intent freshness stops movement quickly. Joint-state freshness is a separate
# watchdog: Gazebo can briefly skip a sample while controller services are
# polled, and that must not falsely fault the entire coordination room.
_JOINT_STATE_TIMEOUT_SEC = 0.50


def _validate_kinematics_contract() -> None:
    """Fail closed if the Jaco2 module is not the model this node expects."""
    if tuple(JOINT_NAMES) != EXPECTED_JOINT_NAMES:
        raise RuntimeError(
            "jaco2_kinematics must expose the canonical J2N6S300 joints "
            f"{EXPECTED_JOINT_NAMES}; received {tuple(JOINT_NAMES)}"
        )
    if len(JOINT_LIMITS) != len(EXPECTED_JOINT_NAMES):
        raise RuntimeError("jaco2_kinematics must provide six joint limits")
    for joint_name, limits in zip(JOINT_NAMES, JOINT_LIMITS):
        if len(limits) != 2 or not all(math.isfinite(float(value)) for value in limits):
            raise RuntimeError(f"invalid limits for {joint_name}")
        if float(limits[0]) >= float(limits[1]):
            raise RuntimeError(f"invalid ordered limits for {joint_name}")


class Jaco2CartesianIkBridge(Node):
    """Readiness-gated, position-controlled Cartesian bridge for J2N6S300."""

    def __init__(self, *, rate_hz: float, input_timeout_sec: float) -> None:
        _validate_kinematics_contract()
        super().__init__("jaco2_cartesian_ik_bridge")
        self._period_sec = 1.0 / rate_hz
        # The kinematic module receives the actual trajectory horizon.  Keeping
        # it in [50, 100] ms bounds every command even if the timer is changed.
        self._horizon_sec = min(
            _MAX_HORIZON_SEC, max(_MIN_HORIZON_SEC, self._period_sec)
        )
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
        self.create_timer(self._period_sec, self._control_tick)

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
        if not all(name in indexed for name in JOINT_NAMES):
            return
        candidate = tuple(float(indexed[name]) for name in JOINT_NAMES)
        if _positions_are_valid(candidate):
            self._positions = candidate
            self._state_seen_at = time.monotonic()
            return
        self._positions = None
        self._state_seen_at = None
        self.get_logger().error("Jaco2 joint state is non-finite or outside configured limits")

    def _poll_controllers(self, now: float) -> None:
        if now - self._last_controller_check < _CONTROLLER_POLL_SEC:
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
            and now - self._state_seen_at <= _JOINT_STATE_TIMEOUT_SEC
        )
        bridge_ready = state_fresh and self._controllers_active and self._twist_valid
        self._ready.publish(Bool(data=bridge_ready))
        if not bridge_ready:
            return

        active_twist = self._twist
        if self._twist_seen_at is None or now - self._twist_seen_at > self._input_timeout_sec:
            # A stale command is an explicit hold: do not reuse a prior motion.
            active_twist = Twist((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        try:
            command = twist_to_joint_command(
                self._positions, active_twist, self._horizon_sec
            )
            targets = tuple(float(value) for value in command.target_positions)
            if not _positions_are_valid(targets):
                raise ValueError("kinematics returned an invalid Jaco2 target")
        except (AttributeError, TypeError, ValueError) as error:
            self.get_logger().error(f"rejecting invalid Jaco2 Cartesian command: {error}")
            return
        self._trajectory.publish(_trajectory_message(targets, self._horizon_sec))


def _positions_are_valid(positions: Sequence[float]) -> bool:
    if len(positions) != len(JOINT_NAMES):
        return False
    return all(
        math.isfinite(value) and float(limits[0]) <= value <= float(limits[1])
        for value, limits in zip(positions, JOINT_LIMITS)
    )


def _trajectory_message(targets: Sequence[float], duration_sec: float) -> JointTrajectory:
    """Create one bounded, immediate, short-horizon controller trajectory."""
    seconds = int(duration_sec)
    nanoseconds = int(round((duration_sec - seconds) * 1_000_000_000))
    if nanoseconds == 1_000_000_000:
        seconds += 1
        nanoseconds = 0
    point = JointTrajectoryPoint()
    point.positions = list(targets)
    point.time_from_start.sec = seconds
    point.time_from_start.nanosec = nanoseconds
    trajectory = JointTrajectory()
    trajectory.joint_names = list(JOINT_NAMES)
    trajectory.points = [point]
    return trajectory


def main() -> int:
    parser = argparse.ArgumentParser(description="Safe Cartesian IK bridge for Jaco2 J2N6S300 Gazebo")
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--input-timeout-sec", type=float, default=0.15)
    arguments = parser.parse_args()
    if not 10.0 <= arguments.rate_hz <= 20.0:
        parser.error("rate-hz must be between 10 and 20 so trajectories remain 50-100 ms")
    if arguments.input_timeout_sec <= 0.0:
        parser.error("input-timeout-sec must be positive")

    rclpy.init()
    node = Jaco2CartesianIkBridge(
        rate_hz=arguments.rate_hz, input_timeout_sec=arguments.input_timeout_sec
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # ROS 2 Humble can shut down its context before an executor wakes up.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
