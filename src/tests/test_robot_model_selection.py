"""Model-selection tests that do not require ROS 2 or Gazebo."""

from __future__ import annotations

import math
from pathlib import Path
import unittest
from unittest.mock import patch

from teleop_cooperativo.gateway import build_argument_parser, main
from teleop_cooperativo.jaco2_kinematics import JOINT_NAMES as JACO2_JOINT_NAMES
from teleop_cooperativo.jaco2_kinematics import forward_kinematics as jaco2_fk
from teleop_cooperativo.ros_twist_sink import RosTwistSink, robot_kinematics


class _JointState:
    def __init__(self, names: tuple[str, ...], positions: tuple[float, ...]) -> None:
        self.name = names
        self.position = positions


class _Rclpy:
    @staticmethod
    def ok() -> bool:
        return True

    @staticmethod
    def spin_once(node: object, timeout_sec: float) -> None:
        del node, timeout_sec


class RobotModelSelectionTest(unittest.TestCase):
    def test_jaco2_uses_its_canonical_joint_names_and_fk(self) -> None:
        kinematics = robot_kinematics("jaco2")
        sink = object.__new__(RosTwistSink)
        sink._kinematics = kinematics
        sink._joint_state = None
        sink._joint_state_at = None
        positions = (0.0, math.radians(150.0), math.radians(210.0), 0.0, 0.0, 0.0)

        sink._on_joint_state(_JointState(JACO2_JOINT_NAMES, positions))

        self.assertEqual(sink._joint_state, positions)
        self.assertEqual(
            sink._kinematics.forward_kinematics(sink._joint_state).position,
            jaco2_fk(positions).position,
        )
        self.assertEqual(kinematics.telemetry_source, "ros_gazebo_jaco2_fk")
        sink._rclpy = _Rclpy()
        sink._node = object()
        telemetry = sink.telemetry()
        self.assertIsNotNone(telemetry)
        assert telemetry is not None
        self.assertEqual(telemetry.source, "ros_gazebo_jaco2_fk")
        self.assertEqual(telemetry.pose, jaco2_fk(positions))

    def test_jaco2_does_not_accept_generic_joint_names(self) -> None:
        sink = object.__new__(RosTwistSink)
        sink._kinematics = robot_kinematics("jaco2")
        sink._joint_state = None
        sink._joint_state_at = None

        sink._on_joint_state(
            _JointState(
                tuple(f"joint_{index}" for index in range(1, 7)),
                (0.0,) * 6,
            )
        )

        self.assertIsNone(sink._joint_state)

    def test_gateway_rejects_jaco2_without_its_ros_path(self) -> None:
        parser = build_argument_parser()
        arguments = parser.parse_args(["--robot-model", "jaco2"])
        self.assertEqual(arguments.robot_model, "jaco2")
        self.assertIsNone(arguments.ros_twist_topic)
        with patch("sys.argv", ["gateway", "--robot-model", "jaco2"]):
            with self.assertRaises(SystemExit) as raised:
                main()
        self.assertEqual(raised.exception.code, 2)

    def test_base_station_targets_only_jaco2_simulation(self) -> None:
        script = (
            Path(__file__).resolve().parents[2] / "scripts" / "start_base.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("jaco2_sim.launch.py", script)
        self.assertIn("jaco2_cartesian_ik_bridge.py", script)
        self.assertIn('--robot-model "$ROBOT_MODEL"', script)
        self.assertIn("teleop_cooperativo.desktop_console", script)
        self.assertIn("import PyQt5, vtk, collada", script)
        self.assertIn('"headless:=true"', script)
        self.assertIn("lan_https_proxy.py", script)
        self.assertNotIn("xdg-open", script)
        self.assertNotIn("course_arm_sim.launch.py", script)
        self.assertNotIn('scripts/cartesian_ik_bridge.py"', script)


if __name__ == "__main__":
    unittest.main()
