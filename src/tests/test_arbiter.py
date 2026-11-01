"""Tests for the local mother-laptop authority."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from teleop_cooperativo.arbiter import CooperativeArbiter, RoomState
from teleop_cooperativo.domain import Axis
from teleop_cooperativo.lease_manager import LeaseManager


class CooperativeArbiterTest(unittest.TestCase):
    """Verify readiness gates and one composed output."""

    def setUp(self) -> None:
        self.arbiter = CooperativeArbiter(
            LeaseManager(input_timeout_sec=0.15, lease_timeout_sec=0.25),
            max_linear_speed_mps=0.05,
            max_angular_speed_radps=0.175,
        )

    def test_claim_requires_explicit_ready_state(self) -> None:
        with self.assertRaises(RuntimeError):
            self.arbiter.claim("operator-a", ["X"], now=0.0)

        self.arbiter.mark_ready()
        self.arbiter.claim("operator-a", ["X"], now=0.0)
        self.assertEqual(self.arbiter.state, RoomState.ACTIVE)

    def test_tick_composes_two_operators_into_one_twist(self) -> None:
        self.arbiter.mark_ready()
        first = self.arbiter.claim("operator-a", ["X", "Y"], now=0.0)
        second = self.arbiter.claim("operator-b", ["YAW"], now=0.0)
        first_tokens = {lease.axis: (lease.lease_id, lease.generation) for lease in first}
        second_tokens = {
            lease.axis: (lease.lease_id, lease.generation) for lease in second
        }
        self.arbiter.submit_input(
            "operator-a",
            1,
            {"X": 1.0, "Y": -1.0},
            first_tokens,
            now=0.01,
        )
        self.arbiter.submit_input(
            "operator-b", 1, {"YAW": 1.0}, second_tokens, now=0.01
        )

        result = self.arbiter.tick(now=0.02)

        self.assertEqual(result.state, RoomState.ACTIVE)
        self.assertGreater(result.twist.linear[0], 0.0)
        self.assertLess(result.twist.linear[1], 0.0)
        self.assertEqual(result.twist.linear[2], 0.0)
        self.assertEqual(result.twist.angular[0], 0.0)
        self.assertGreater(result.twist.angular[2], 0.0)

    def test_expiry_returns_room_to_ready_and_zero_output(self) -> None:
        self.arbiter.mark_ready()
        lease = self.arbiter.claim("operator-a", ["X"], now=0.0)[0]
        self.arbiter.submit_input(
            "operator-a",
            1,
            {"X": 1.0},
            {Axis.X: (lease.lease_id, lease.generation)},
            now=0.01,
        )

        input_expired = self.arbiter.tick(now=0.17)
        self.assertEqual(input_expired.twist.linear, (0.0, 0.0, 0.0))
        self.assertEqual(input_expired.state, RoomState.ACTIVE)

        released = self.arbiter.tick(now=0.27)
        self.assertEqual(released.state, RoomState.READY)
        self.assertEqual(released.twist.linear, (0.0, 0.0, 0.0))
        self.assertEqual(released.maintenance.released_axes, (Axis.X,))

    def test_fault_revokes_authority_until_rearmed(self) -> None:
        self.arbiter.mark_ready()
        self.arbiter.claim("operator-a", ["X"], now=0.0)

        self.assertEqual(self.arbiter.fault(), (Axis.X,))
        self.assertEqual(self.arbiter.state, RoomState.FAULT)
        self.assertEqual(self.arbiter.tick(now=0.1).twist.linear, (0.0, 0.0, 0.0))

        with self.assertRaises(RuntimeError):
            self.arbiter.claim("operator-a", ["X"], now=0.2)

        self.arbiter.mark_ready()
        self.arbiter.claim("operator-a", ["X"], now=0.3)
        self.assertEqual(self.arbiter.state, RoomState.ACTIVE)

    def test_releasing_last_lease_returns_room_to_ready(self) -> None:
        self.arbiter.mark_ready()
        self.arbiter.claim("operator-a", ["X"], now=0.0)

        self.assertEqual(self.arbiter.release("operator-a"), (Axis.X,))
        self.assertEqual(self.arbiter.state, RoomState.READY)


if __name__ == "__main__":
    unittest.main()
