"""End-to-end deterministic H3 tests without sockets, ROS or Gazebo."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from teleop_cooperativo.arbiter import CooperativeArbiter, RoomState
from teleop_cooperativo.event_log import RunEventLogger
from teleop_cooperativo.gateway import CooperativeGateway
from teleop_cooperativo.lease_manager import LeaseManager
from teleop_cooperativo.local_simulation_runtime import LocalSimulationRuntime
from teleop_cooperativo.tool_simulator import DeterministicToolSimulator


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


class LocalSimulationGatewayTest(unittest.TestCase):
    """Verify one gateway maintenance pass means one simulator step."""

    def setUp(self) -> None:
        self.clock = FakeClock()
        self.arbiter = CooperativeArbiter(LeaseManager(clock=self.clock))
        self.simulator = DeterministicToolSimulator()
        self.runtime = LocalSimulationRuntime(
            self.arbiter, self.simulator, clock=self.clock
        )
        self.gateway = CooperativeGateway(
            self.arbiter, local_runtime=self.runtime
        )
        self.gateway.mark_ready()
        self.gateway.open_connection("operator")
        replies = self.gateway.receive(
            "operator",
            {"type": "hello", "room_id": "default", "operator_name": "Ana"},
        )
        self.assertEqual(replies[0]["type"], "welcome")

    def test_input_moves_pose_and_returns_serializable_telemetry(self) -> None:
        grant = self.gateway.receive(
            "operator", {"type": "claim", "axes": ["X", "YAW"]}
        )[0]
        leases = {
            lease["axis"]: {
                "lease_id": lease["lease_id"],
                "generation": lease["generation"],
            }
            for lease in grant["leases"]
        }
        self.clock.now = 0.01
        accepted = self.gateway.receive(
            "operator",
            {
                "type": "input",
                "sequence": 1,
                "values": {"X": 1.0, "YAW": 1.0},
                "leases": leases,
            },
        )
        self.assertEqual(accepted[0]["type"], "input_accepted")

        self.clock.now = 0.05
        result = self.gateway.maintenance_tick()

        self.assertEqual(result.arbiter_tick.state, RoomState.ACTIVE)
        self.assertIsNotNone(result.telemetry)
        assert result.telemetry is not None
        self.assertEqual(result.telemetry["type"], "telemetry")
        self.assertAlmostEqual(result.telemetry["dt_sec"], 0.05)
        self.assertAlmostEqual(result.telemetry["twist"]["linear"][0], 0.05)
        self.assertAlmostEqual(result.telemetry["pose"]["position"][0], 0.0025)
        self.assertNotEqual(
            result.telemetry["pose"]["quaternion_xyzw"], [0.0, 0.0, 0.0, 1.0]
        )

    def test_timeout_zeroes_simulation_command_without_repeating_motion(self) -> None:
        grant = self.gateway.receive(
            "operator", {"type": "claim", "axes": ["X"]}
        )[0]
        lease = grant["leases"][0]
        self.clock.now = 0.01
        self.gateway.receive(
            "operator",
            {
                "type": "input",
                "sequence": 1,
                "values": {"X": 1.0},
                "leases": {
                    "X": {
                        "lease_id": lease["lease_id"],
                        "generation": lease["generation"],
                    }
                },
            },
        )
        self.clock.now = 0.05
        moving = self.gateway.maintenance_tick()
        self.clock.now = 0.10
        self.gateway.maintenance_tick()
        self.clock.now = 0.17
        stopped = self.gateway.maintenance_tick()
        self.clock.now = 0.20
        after_stop = self.gateway.maintenance_tick()

        assert moving.telemetry is not None
        assert stopped.telemetry is not None
        assert after_stop.telemetry is not None
        self.assertEqual(stopped.arbiter_tick.maintenance.zeroed_axes[0].value, "X")
        self.assertEqual(stopped.telemetry["twist"]["linear"], [0.0, 0.0, 0.0])
        self.assertEqual(
            stopped.telemetry["pose"]["position"],
            after_stop.telemetry["pose"]["position"],
        )

    def test_lost_local_simulator_faults_room_and_reports_zero_telemetry(self) -> None:
        self.gateway.receive("operator", {"type": "claim", "axes": ["X"]})
        self.simulator.set_ready(False)
        self.clock.now = 0.05

        result = self.gateway.maintenance_tick()

        self.assertEqual(result.arbiter_tick.state, RoomState.FAULT)
        assert result.telemetry is not None
        self.assertEqual(result.telemetry["state"], RoomState.FAULT.value)
        self.assertEqual(result.telemetry["twist"]["linear"], [0.0, 0.0, 0.0])
        self.assertFalse(result.room_state["axes"]["X"]["assigned"])

    def test_optional_logger_records_the_same_pose_reported_to_gateway(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "run.jsonl"
            logger = RunEventLogger(
                path,
                run_id="local-001",
                wall_clock=lambda: 1_700_000_000.0,
                monotonic_clock=self.clock,
            )
            gateway = CooperativeGateway(
                self.arbiter, local_runtime=self.runtime, event_logger=logger
            )
            self.clock.now = 0.05
            reported = gateway.maintenance_tick()
            logger.close()
            records = [json.loads(line) for line in path.read_text().splitlines()]

        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]["event_type"], "telemetry")
        self.assertEqual(
            records[0]["payload"]["pose"]["position"],
            reported.telemetry["pose"]["position"],
        )

    def test_runtime_must_share_the_gateway_arbiter(self) -> None:
        other = CooperativeArbiter(LeaseManager(clock=self.clock))
        with self.assertRaises(ValueError):
            CooperativeGateway(other, local_runtime=self.runtime)


if __name__ == "__main__":
    unittest.main()
