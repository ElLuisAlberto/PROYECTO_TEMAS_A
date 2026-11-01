"""Unit tests for safe Cartesian command composition."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from teleop_cooperativo.command_composer import compose_twist


class CommandComposerTest(unittest.TestCase):
    """Verify normalization and invalid input rejection."""

    def test_maps_each_linear_and_angular_axis(self) -> None:
        twist = compose_twist(
            {
                "X": 0.5,
                "Y": -0.5,
                "Z": 0.0,
                "ROLL": 1.0,
                "PITCH": 0.0,
                "YAW": -1.0,
            },
            max_linear_speed_mps=0.04,
            max_angular_speed_radps=0.20,
        )

        self.assertAlmostEqual(twist.linear[0], 0.02)
        self.assertAlmostEqual(twist.linear[1], -0.02)
        self.assertAlmostEqual(twist.linear[2], 0.0)
        self.assertLessEqual(_norm(twist.angular), 0.20)
        self.assertGreater(twist.angular[0], 0.0)
        self.assertLess(twist.angular[2], 0.0)

    def test_normalizes_resultant_without_changing_direction(self) -> None:
        twist = compose_twist(
            {"X": 1.0, "Y": 1.0, "Z": 1.0},
            max_linear_speed_mps=0.06,
        )

        self.assertAlmostEqual(_norm(twist.linear), 0.06)
        self.assertAlmostEqual(twist.linear[0], twist.linear[1])
        self.assertAlmostEqual(twist.linear[1], twist.linear[2])

    def test_rejects_invalid_normalized_values(self) -> None:
        with self.assertRaises(ValueError):
            compose_twist({"X": 1.01})
        with self.assertRaises(ValueError):
            compose_twist({"X": float("nan")})
        with self.assertRaises(ValueError):
            compose_twist({"unknown": 0.0})


def _norm(values: tuple[float, float, float]) -> float:
    return math.sqrt(sum(value * value for value in values))


if __name__ == "__main__":
    unittest.main()
