"""Checks for the pure, substitute-only differential kinematics layer."""

from __future__ import annotations

from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from teleop_cooperativo.command_composer import Twist
from teleop_cooperativo.differential_kinematics import (
    JOINT_LIMITS,
    JOINT_VELOCITY_LIMITS,
    forward_kinematics,
    geometric_jacobian,
    twist_to_joint_command,
)


class DifferentialKinematicsTest(unittest.TestCase):
    def test_zero_configuration_matches_urdf_chain(self) -> None:
        pose = forward_kinematics([0.0] * 6)

        self.assertEqual(pose.position, (0.0, 0.0, 1.33))
        self.assertEqual(pose.quaternion_xyzw, (0.0, 0.0, 0.0, 1.0))

    def test_jacobian_linear_columns_match_finite_difference(self) -> None:
        joints = [0.3, -0.4, 0.5, 0.2, -0.3, 0.4]
        jacobian = geometric_jacobian(joints)
        epsilon = 1e-6
        for column in range(6):
            plus = list(joints)
            minus = list(joints)
            plus[column] += epsilon
            minus[column] -= epsilon
            upper = forward_kinematics(plus).position
            lower = forward_kinematics(minus).position
            approximation = tuple((a - b) / (2.0 * epsilon) for a, b in zip(upper, lower))
            for row in range(3):
                self.assertAlmostEqual(jacobian[row][column], approximation[row], places=5)

    def test_zero_twist_keeps_all_joint_targets_still(self) -> None:
        joints = [0.2, -0.4, 0.5, 0.3, -0.2, 0.1]
        command = twist_to_joint_command(joints, Twist((0.0,) * 3, (0.0,) * 3), 0.05)

        self.assertEqual(command.velocities, (0.0,) * 6)
        self.assertEqual(command.target_positions, tuple(joints))

    def test_command_obeys_velocity_and_position_limits(self) -> None:
        joints = [upper - 0.001 for _, upper in JOINT_LIMITS]
        command = twist_to_joint_command(
            joints, Twist((20.0, -20.0, 20.0), (20.0, -20.0, 20.0)), 0.1
        )

        for velocity, speed_limit in zip(command.velocities, JOINT_VELOCITY_LIMITS):
            self.assertLessEqual(abs(velocity), speed_limit)
        for target, limits in zip(command.target_positions, JOINT_LIMITS):
            self.assertGreaterEqual(target, limits[0])
            self.assertLessEqual(target, limits[1])

    def test_damping_keeps_singular_configuration_finite(self) -> None:
        command = twist_to_joint_command(
            [0.0] * 6, Twist((0.05, 0.0, 0.0), (0.0, 0.0, 0.1)), 0.05
        )

        self.assertTrue(all(math.isfinite(value) for value in command.velocities))
        self.assertTrue(all(math.isfinite(value) for value in command.target_positions))

    def test_invalid_joint_state_or_step_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            forward_kinematics([0.0] * 5)
        with self.assertRaises(ValueError):
            twist_to_joint_command([0.0] * 6, Twist((0.0,) * 3, (0.0,) * 3), 0.0)


if __name__ == "__main__":
    unittest.main()
