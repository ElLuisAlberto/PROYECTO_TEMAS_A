#!/usr/bin/env python3
"""Validate the running H2c Gazebo substitute through ROS 2 interfaces."""

from __future__ import annotations

import json
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS = [f"joint_{index}" for index in range(1, 7)]
MOVE_TARGET = [0.12, -0.10, 0.08, 0.06, -0.05, 0.10]
ZERO_TARGET = [0.0] * 6


class RuntimeValidator(Node):
    """Observe clock/state and issue two small controlled trajectories."""

    def __init__(self) -> None:
        super().__init__("course_h2c_runtime_validator")
        self.clock_seen = False
        self.positions: dict[str, float] = {}
        self.create_subscription(Clock, "/clock", self._clock_callback, 10)
        self.create_subscription(
            JointState, "/joint_states", self._joint_callback, 10
        )
        self.controllers = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.trajectory = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )

    def _clock_callback(self, _message: Clock) -> None:
        self.clock_seen = True

    def _joint_callback(self, message: JointState) -> None:
        self.positions = {
            name: float(position)
            for name, position in zip(message.name, message.position)
        }

    def wait_until(self, predicate, timeout_sec: float, detail: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {detail}")

    def verify_controllers(self) -> None:
        if not self.controllers.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("controller manager is unavailable")
        future = self.controllers.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None:
            raise RuntimeError("controller manager did not respond")
        states = {
            controller.name: controller.state
            for controller in response.controller
        }
        expected = {
            "joint_state_broadcaster": "active",
            "arm_controller": "active",
        }
        if any(states.get(name) != state for name, state in expected.items()):
            raise RuntimeError(f"controllers are not active: {states}")

    def send_target(self, positions: list[float]) -> None:
        if not self.trajectory.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("trajectory action server is unavailable")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 2
        goal.trajectory.points = [point]
        send_future = self.trajectory.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("trajectory goal was rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        wrapped = result_future.result()
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError("trajectory did not succeed")
        self.wait_until(
            lambda: self._within_tolerance(positions, 0.01),
            5.0,
            "joint target",
        )

    def _within_tolerance(
        self, expected: list[float], tolerance: float
    ) -> bool:
        return all(
            joint in self.positions
            and abs(self.positions[joint] - target) <= tolerance
            for joint, target in zip(JOINTS, expected)
        )


def main() -> int:
    """Validate active controllers, motion and return to zero."""
    rclpy.init()
    node = RuntimeValidator()
    try:
        node.wait_until(lambda: node.clock_seen, 20.0, "/clock")
        node.wait_until(
            lambda: all(joint in node.positions for joint in JOINTS),
            20.0,
            "six joint states",
        )
        node.verify_controllers()
        node.send_target(MOVE_TARGET)
        measured_move = [node.positions[joint] for joint in JOINTS]
        node.send_target(ZERO_TARGET)
        measured_return = [node.positions[joint] for joint in JOINTS]
        print(
            json.dumps(
                {
                    "controllers": "active",
                    "clock": "advancing",
                    "move_target": MOVE_TARGET,
                    "measured_move": measured_move,
                    "measured_return": measured_return,
                    "result": "PASS",
                },
                indent=2,
            )
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
