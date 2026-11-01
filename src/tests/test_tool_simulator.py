"""Tests for the deterministic Cartesian simulation harness."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from teleop_cooperativo.command_composer import Twist
from teleop_cooperativo.simulation_port import ToolPose
from teleop_cooperativo.tool_simulator import (
    DeterministicToolSimulator,
    SimulationFault,
)


class DeterministicToolSimulatorTest(unittest.TestCase):
    """Verify bounded, atomic pose integration."""

    def test_linear_twist_produces_expected_displacement(self) -> None:
        simulator = DeterministicToolSimulator()

        pose = simulator.apply_twist(
            Twist((0.05, -0.02, 0.01), (0.0, 0.0, 0.0)), 0.10
        )

        self.assertAlmostEqual(pose.position[0], 0.005)
        self.assertAlmostEqual(pose.position[1], -0.002)
        self.assertAlmostEqual(pose.position[2], 0.601)

    def test_angular_twist_keeps_unit_quaternion(self) -> None:
        simulator = DeterministicToolSimulator()

        pose = simulator.apply_twist(
            Twist((0.0, 0.0, 0.0), (0.2, -0.1, 0.3)), 0.10
        )

        norm = math.sqrt(sum(value * value for value in pose.quaternion_xyzw))
        self.assertAlmostEqual(norm, 1.0)
        self.assertNotEqual(pose.quaternion_xyzw, (0.0, 0.0, 0.0, 1.0))

    def test_zero_twist_has_no_drift(self) -> None:
        simulator = DeterministicToolSimulator()
        initial = simulator.pose

        for _ in range(20):
            simulator.apply_twist(
                Twist((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)), 0.05
            )

        self.assertEqual(simulator.pose, initial)

    def test_workspace_violation_is_atomic(self) -> None:
        initial = ToolPose((0.0, 0.0, 0.10), (0.0, 0.0, 0.0, 1.0))
        simulator = DeterministicToolSimulator(
            initial_pose=initial,
            workspace_min=(-0.1, -0.1, 0.05),
            workspace_max=(0.1, 0.1, 0.2),
        )

        with self.assertRaises(SimulationFault):
            simulator.apply_twist(
                Twist((0.0, 0.0, -0.6), (0.0, 0.0, 0.0)), 0.10
            )

        self.assertEqual(simulator.pose, initial)

    def test_invalid_step_and_not_ready_are_rejected(self) -> None:
        simulator = DeterministicToolSimulator(maximum_step_sec=0.10)
        initial = simulator.pose

        with self.assertRaises(ValueError):
            simulator.apply_twist(
                Twist((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)), float("nan")
            )
        with self.assertRaises(SimulationFault):
            simulator.apply_twist(
                Twist((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)), 0.11
            )
        simulator.set_ready(False)
        with self.assertRaises(SimulationFault):
            simulator.apply_twist(
                Twist((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)), 0.05
            )

        self.assertEqual(simulator.pose, initial)


if __name__ == "__main__":
    unittest.main()
