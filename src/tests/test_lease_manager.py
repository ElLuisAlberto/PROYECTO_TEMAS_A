"""Unit tests for exclusive Cartesian axis reservations."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from teleop_cooperativo.domain import Axis
from teleop_cooperativo.lease_manager import (
    ClaimDenied,
    InputRejected,
    LeaseManager,
)


class LeaseManagerTest(unittest.TestCase):
    """Verify atomicity, freshness and ownership invariants."""

    def setUp(self) -> None:
        self.manager = LeaseManager(
            input_timeout_sec=0.15, lease_timeout_sec=0.25
        )

    def test_claim_is_exclusive_and_atomic(self) -> None:
        self.manager.claim("operator-a", ["X", "Y"], now=0.0)

        with self.assertRaises(ClaimDenied):
            self.manager.claim("operator-b", ["Y", "Z"], now=0.0)

        self.assertEqual(self.manager.axes_for_session("operator-b"), ())
        self.assertEqual(
            self.manager.axes_for_session("operator-a"), (Axis.X, Axis.Y)
        )
        self.assertIsNone(self.manager.lease(Axis.Z))

    def test_session_cannot_hold_more_than_three_axes(self) -> None:
        self.manager.claim("operator-a", ["X", "Y", "Z"], now=0.0)

        with self.assertRaises(ClaimDenied):
            self.manager.claim("operator-a", ["ROLL"], now=0.01)

        self.assertEqual(len(self.manager.axes_for_session("operator-a")), 3)

    def test_stale_sequence_does_not_refresh_or_change_input(self) -> None:
        lease = self.manager.claim("operator-a", ["X"], now=0.0)[0]
        tokens = {Axis.X: (lease.lease_id, lease.generation)}
        self.manager.submit_input(
            "operator-a", 8, {"X": 1.0}, tokens, now=0.01
        )

        with self.assertRaises(InputRejected):
            self.manager.submit_input(
                "operator-a", 8, {"X": -1.0}, tokens, now=0.10
            )

        self.assertEqual(self.manager.values()[Axis.X], 1.0)
        update = self.manager.advance(now=0.17)
        self.assertEqual(update.zeroed_axes, (Axis.X,))
        self.assertEqual(self.manager.values()[Axis.X], 0.0)

    def test_input_timeout_zeroes_before_heartbeat_timeout_releases(self) -> None:
        lease = self.manager.claim("operator-a", ["X"], now=0.0)[0]
        self.manager.submit_input(
            "operator-a",
            1,
            {"X": 0.8},
            {Axis.X: (lease.lease_id, lease.generation)},
            now=0.01,
        )

        first = self.manager.advance(now=0.17)
        self.assertEqual(first.zeroed_axes, (Axis.X,))
        self.assertEqual(first.released_axes, ())
        self.assertEqual(self.manager.axes_for_session("operator-a"), (Axis.X,))

        second = self.manager.advance(now=0.27)
        self.assertEqual(second.released_axes, (Axis.X,))
        self.assertEqual(self.manager.axes_for_session("operator-a"), ())

    def test_disconnect_only_releases_affected_operator(self) -> None:
        self.manager.claim("operator-a", ["X", "Y"], now=0.0)
        self.manager.claim("operator-b", ["Z"], now=0.0)

        released = self.manager.release("operator-a")

        self.assertEqual(released, (Axis.X, Axis.Y))
        self.assertEqual(self.manager.axes_for_session("operator-b"), (Axis.Z,))
        self.assertEqual(self.manager.values()[Axis.Z], 0.0)

    def test_old_generation_cannot_change_reassigned_axis(self) -> None:
        old_lease = self.manager.claim("operator-a", ["X"], now=0.0)[0]
        self.manager.release("operator-a", ["X"])
        new_lease = self.manager.claim("operator-b", ["X"], now=0.01)[0]

        with self.assertRaises(InputRejected):
            self.manager.submit_input(
                "operator-a",
                1,
                {"X": 1.0},
                {Axis.X: (old_lease.lease_id, old_lease.generation)},
                now=0.02,
            )

        self.assertNotEqual(old_lease.generation, new_lease.generation)
        self.assertEqual(self.manager.values()[Axis.X], 0.0)

    def test_stop_all_revokes_every_lease(self) -> None:
        self.manager.claim("operator-a", ["X", "Y"], now=0.0)
        self.manager.claim("operator-b", ["Z"], now=0.0)

        released = self.manager.stop_all()

        self.assertEqual(released, (Axis.X, Axis.Y, Axis.Z))
        self.assertEqual(self.manager.values(), {axis: 0.0 for axis in Axis})

    def test_invalid_or_backwards_time_is_rejected(self) -> None:
        self.manager.claim("operator-a", ["X"], now=1.0)

        with self.assertRaises(ValueError):
            self.manager.advance(now=float("nan"))
        with self.assertRaises(ValueError):
            self.manager.advance(now=float("inf"))
        with self.assertRaises(ValueError):
            self.manager.advance(now=0.9)

        self.assertEqual(self.manager.axes_for_session("operator-a"), (Axis.X,))


if __name__ == "__main__":
    unittest.main()
