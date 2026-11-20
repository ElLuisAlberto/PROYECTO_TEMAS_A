"""Checks for the self-contained Jaco2 J2N6S300 kinematics layer."""

from __future__ import annotations

from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from teleop_cooperativo.command_composer import Twist
from teleop_cooperativo.jaco2_kinematics import (
    JOINT_LIMITS,
    JOINT_NAMES,
    JOINT_VELOCITY_LIMITS,
    forward_kinematics,
    geometric_jacobian,
    twist_to_joint_command,
)


class Jaco2KinematicsTest(unittest.TestCase):
    _SAFE_JOINTS = (0.2, math.radians(150.0), math.radians(210.0), 0.15, 0.3, -0.2)

    def test_contract_names_limits_and_conservative_speeds(self) -> None:
        self.assertEqual(JOINT_NAMES, tuple(f"j2n6s300_joint_{index}" for index in range(1, 7)))
        self.assertEqual(JOINT_LIMITS[1], (math.radians(47.0), math.radians(313.0)))
        self.assertEqual(JOINT_LIMITS[2], (math.radians(19.0), math.radians(341.0)))
        self.assertEqual(JOINT_VELOCITY_LIMITS[:3], (math.radians(18.0),) * 3)
        self.assertEqual(JOINT_VELOCITY_LIMITS[3:], (math.radians(24.0),) * 3)

    def test_fk_matches_official_chain_reference_pose(self) -> None:
        pose = forward_kinematics(self._SAFE_JOINTS)

        # Independent reference values from the Xacro transforms in metres.
        self.assertAlmostEqual(pose.position[0], 0.611258, places=6)
        self.assertAlmostEqual(pose.position[1], -0.047319, places=6)
        self.assertAlmostEqual(pose.position[2], 0.909306, places=6)
        self.assertAlmostEqual(sum(value * value for value in pose.quaternion_xyzw), 1.0, places=12)

    def test_jacobian_matches_central_finite_differences(self) -> None:
        joints = list(self._SAFE_JOINTS)
        jacobian = geometric_jacobian(joints)
        epsilon = 1e-6
        for column in range(6):
            plus, minus = list(joints), list(joints)
            plus[column] += epsilon
            minus[column] -= epsilon
            upper = forward_kinematics(plus).position
            lower = forward_kinematics(minus).position
            approximation = tuple((a - b) / (2.0 * epsilon) for a, b in zip(upper, lower))
            for row in range(3):
                self.assertAlmostEqual(jacobian[row][column], approximation[row], places=5)

            # q(+) * inverse(q(-)) is the small world-frame rotation over
            # 2*epsilon.  Its vector part is omega*epsilon to first order.
            upper_q = forward_kinematics(plus).quaternion_xyzw
            lower_q = forward_kinematics(minus).quaternion_xyzw
            differential = self._quaternion_product(
                upper_q, (-lower_q[0], -lower_q[1], -lower_q[2], lower_q[3])
            )
            for row in range(3):
                self.assertAlmostEqual(
                    jacobian[row + 3][column], differential[row] / epsilon, places=5
                )

    def test_zero_twist_keeps_joint_targets_still(self) -> None:
        command = twist_to_joint_command(
            self._SAFE_JOINTS, Twist((0.0,) * 3, (0.0,) * 3), 0.05
        )

        self.assertEqual(command.velocities, (0.0,) * 6)
        self.assertEqual(command.target_positions, self._SAFE_JOINTS)

    def test_command_obeys_real_position_and_speed_limits(self) -> None:
        joints = tuple(upper - 0.001 for _, upper in JOINT_LIMITS)
        command = twist_to_joint_command(
            joints, Twist((20.0, -20.0, 20.0), (20.0, -20.0, 20.0)), 0.1
        )

        for velocity, speed_limit in zip(command.velocities, JOINT_VELOCITY_LIMITS):
            self.assertLessEqual(abs(velocity), speed_limit)
        for target, limits in zip(command.target_positions, JOINT_LIMITS):
            self.assertGreaterEqual(target, limits[0])
            self.assertLessEqual(target, limits[1])

    def test_invalid_joint_twist_and_step_inputs_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            forward_kinematics([0.0] * 6)  # J2 and J3 cannot be at zero.
        with self.assertRaises(ValueError):
            forward_kinematics([0.0] * 5)
        with self.assertRaises(ValueError):
            forward_kinematics([0.0, math.nan, math.radians(20.0), 0.0, 0.0, 0.0])
        with self.assertRaises(ValueError):
            twist_to_joint_command(self._SAFE_JOINTS, Twist((0.0,) * 3, (0.0,) * 3), 0.0)
        with self.assertRaises(ValueError):
            twist_to_joint_command(self._SAFE_JOINTS, object(), 0.05)  # type: ignore[arg-type]

    @staticmethod
    def _quaternion_product(
        left: tuple[float, float, float, float],
        right: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        lx, ly, lz, lw = left
        rx, ry, rz, rw = right
        return (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )


if __name__ == "__main__":
    unittest.main()
