"""Integration tests for arbiter, leases and deterministic simulator."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from teleop_cooperativo.arbiter import CooperativeArbiter, RoomState
from teleop_cooperativo.domain import Axis
from teleop_cooperativo.lease_manager import LeaseManager
from teleop_cooperativo.simulation_runner import RunnerFault, SimulationRunner
from teleop_cooperativo.tool_simulator import DeterministicToolSimulator


class SimulationRunnerTest(unittest.TestCase):
    """Verify one fail-safe path from two operators to one evolving pose."""

    def setUp(self) -> None:
        self.arbiter = CooperativeArbiter(
            LeaseManager(input_timeout_sec=0.15, lease_timeout_sec=0.25)
        )
        self.simulator = DeterministicToolSimulator()
        self.runner = SimulationRunner(
            self.arbiter, self.simulator, maximum_step_sec=0.10
        )

    def test_two_operators_produce_one_pose_update(self) -> None:
        self.runner.start(now=0.0)
        translation = self.arbiter.claim("operator-a", ["X", "Y"], now=0.0)
        rotation = self.arbiter.claim("operator-b", ["YAW"], now=0.0)
        self.arbiter.submit_input(
            "operator-a",
            1,
            {"X": 1.0, "Y": -0.5},
            _tokens(translation),
            now=0.01,
        )
        self.arbiter.submit_input(
            "operator-b",
            1,
            {"YAW": 1.0},
            _tokens(rotation),
            now=0.01,
        )

        result = self.runner.step(now=0.05)

        self.assertEqual(result.arbiter.state, RoomState.ACTIVE)
        self.assertGreater(result.pose.position[0], 0.0)
        self.assertLess(result.pose.position[1], 0.0)
        self.assertNotEqual(
            result.pose.quaternion_xyzw, (0.0, 0.0, 0.0, 1.0)
        )

    def test_input_timeout_stops_pose_before_lease_release(self) -> None:
        self.runner.start(now=0.0)
        lease = self.arbiter.claim("operator-a", ["X"], now=0.0)[0]
        self.arbiter.submit_input(
            "operator-a",
            1,
            {"X": 1.0},
            _tokens((lease,)),
            now=0.01,
        )
        moving = self.runner.step(now=0.05)
        self.runner.step(now=0.10)
        stopped = self.runner.step(now=0.16)

        self.assertGreater(moving.pose.position[0], 0.0)
        self.assertEqual(stopped.arbiter.maintenance.zeroed_axes, (Axis.X,))
        position_at_stop = stopped.pose.position
        after_stop = self.runner.step(now=0.20)
        self.assertEqual(after_stop.pose.position, position_at_stop)
        self.assertEqual(after_stop.arbiter.state, RoomState.ACTIVE)

    def test_excessive_time_step_faults_and_revokes_leases(self) -> None:
        self.runner.start(now=0.0)
        self.arbiter.claim("operator-a", ["X"], now=0.0)

        with self.assertRaises(RunnerFault):
            self.runner.step(now=0.11)

        self.assertEqual(self.arbiter.state, RoomState.FAULT)
        self.assertEqual(self.arbiter.leases.axes_for_session("operator-a"), ())
        self.assertEqual(
            self.simulator.last_twist.linear, (0.0, 0.0, 0.0)
        )

    def test_lost_simulator_readiness_faults_room(self) -> None:
        self.runner.start(now=0.0)
        self.arbiter.claim("operator-a", ["X"], now=0.0)
        self.simulator.set_ready(False)

        with self.assertRaises(RunnerFault):
            self.runner.step(now=0.05)

        self.assertEqual(self.arbiter.state, RoomState.FAULT)
        self.assertEqual(self.arbiter.leases.axes_for_session("operator-a"), ())


def _tokens(leases):
    return {lease.axis: (lease.lease_id, lease.generation) for lease in leases}


if __name__ == "__main__":
    unittest.main()
