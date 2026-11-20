"""Protocol-level tests for the dependency-free cooperative gateway."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from teleop_cooperativo.arbiter import CooperativeArbiter, RoomState
from teleop_cooperativo.gateway import CooperativeGateway, load_credentials
from teleop_cooperativo.event_log import RunEventLogger
from teleop_cooperativo.lease_manager import LeaseManager
from teleop_cooperativo.simulation_port import TelemetrySample, ToolPose
from teleop_cooperativo.local_simulation_runtime import LocalSimulationRuntime
from teleop_cooperativo.tool_simulator import DeterministicToolSimulator


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


class CooperativeGatewayTest(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = FakeClock()
        arbiter = CooperativeArbiter(LeaseManager(clock=self.clock))
        self.gateway = CooperativeGateway(arbiter)
        self.gateway.mark_ready()
        self.gateway.open_connection("connection-a")
        self.gateway.open_connection("connection-b")

    def hello(self, connection: str, name: str) -> dict[str, object]:
        replies = self.gateway.receive(
            connection,
            json.dumps(
                {"type": "hello", "operator_name": name, "room_id": "default"}
            ),
        )
        self.assertEqual(replies[0]["type"], "welcome")
        return replies[0]

    def claim(self, connection: str, axes: list[str]) -> dict[str, object]:
        replies = self.gateway.receive(connection, {"type": "claim", "axes": axes})
        self.assertEqual(replies[0]["type"], "claim_granted")
        return replies[0]

    def test_hello_claim_input_and_public_room_state(self) -> None:
        welcome = self.hello("connection-a", "Ana")
        self.assertEqual(welcome["operator_name"], "Ana")
        self.assertEqual(welcome["state"], RoomState.READY.value)
        self.hello("connection-b", "Bruno")
        grant = self.claim("connection-a", ["X", "Y", "Z"])
        leases = {
            item["axis"]: {
                "lease_id": item["lease_id"],
                "generation": item["generation"],
            }
            for item in grant["leases"]
        }
        replies = self.gateway.receive(
            "connection-a",
            {
                "type": "input",
                "sequence": 1,
                "values": {"X": 1.0, "Y": 0.0, "Z": 0.0},
                "leases": leases,
            },
        )
        self.assertEqual(replies, [{"type": "input_accepted", "sequence": 1}])
        tick = self.gateway.maintenance_tick()
        self.assertAlmostEqual(tick.arbiter_tick.twist.linear[0], 0.05)
        self.assertAlmostEqual(tick.arbiter_tick.twist.linear[2], 0.0)
        state = self.gateway.room_state()
        self.assertEqual(state["state"], RoomState.ACTIVE.value)
        self.assertEqual(state["axes"]["X"]["owner"], "Ana")
        self.assertNotIn("lease_id", state["axes"]["X"])

    def test_conflicting_claim_is_rejected_atomically(self) -> None:
        self.hello("connection-a", "Ana")
        self.hello("connection-b", "Bruno")
        self.claim("connection-a", ["X"])
        before = self.gateway.room_state()
        replies = self.gateway.receive(
            "connection-b", {"type": "claim", "axes": ["X", "Y"]}
        )
        self.assertEqual(replies[0]["type"], "error")
        self.assertEqual(replies[0]["code"], "claim_denied")
        self.assertEqual(self.gateway.room_state(), before)

    def test_close_releases_authority_for_another_operator(self) -> None:
        self.hello("connection-a", "Ana")
        self.hello("connection-b", "Bruno")
        self.claim("connection-a", ["ROLL"])
        self.assertTrue(self.gateway.close_connection("connection-a"))
        self.assertFalse(self.gateway.room_state()["axes"]["ROLL"]["assigned"])
        grant = self.claim("connection-b", ["ROLL"])
        self.assertEqual(grant["leases"][0]["axis"], "ROLL")

    def test_malformed_or_pre_hello_message_cannot_change_authority(self) -> None:
        self.assertEqual(
            self.gateway.receive("connection-a", "not-json")[0]["code"],
            "invalid_message",
        )
        self.assertEqual(
            self.gateway.receive("connection-a", {"type": "claim", "axes": ["X"]})[
                0
            ]["code"],
            "hello_required",
        )
        self.hello("connection-a", "Ana")
        self.assertEqual(
            self.gateway.receive("connection-a", b"binary")[0]["code"],
            "invalid_message",
        )
        self.assertEqual(self.gateway.room_state()["state"], RoomState.READY.value)

    def test_server_clock_expiry_releases_without_a_browser_message(self) -> None:
        self.hello("connection-a", "Ana")
        self.claim("connection-a", ["YAW"])
        self.clock.now = 0.3
        tick = self.gateway.maintenance_tick()
        self.assertTrue(tick.changed)
        self.assertEqual(tick.arbiter_tick.maintenance.released_axes[0].value, "YAW")
        self.assertEqual(tick.room_state["state"], RoomState.READY.value)

    def test_authoritative_tick_forwards_one_safe_twist_to_optional_sink(self) -> None:
        observed = []
        arbiter = CooperativeArbiter(LeaseManager(clock=self.clock))
        gateway = CooperativeGateway(arbiter, twist_sink=observed.append)
        gateway.mark_ready()
        gateway.open_connection("operator")
        welcome = gateway.receive(
            "operator", {"type": "hello", "operator_name": "Ana", "room_id": "default"}
        )[0]
        self.assertEqual(welcome["type"], "welcome")
        grant = gateway.receive("operator", {"type": "claim", "axes": ["X"]})[0]
        lease = grant["leases"][0]
        gateway.receive(
            "operator",
            {
                "type": "input",
                "sequence": 1,
                "values": {"X": 1.0},
                "leases": {"X": {"lease_id": lease["lease_id"], "generation": lease["generation"]}},
            },
        )

        gateway.maintenance_tick()
        self.assertAlmostEqual(observed[-1].linear[0], 0.05)
        self.clock.now = 0.3
        gateway.maintenance_tick()
        self.assertEqual(observed[-1].linear, (0.0, 0.0, 0.0))
        self.assertEqual(observed[-1].angular, (0.0, 0.0, 0.0))

    def test_unready_bridge_blocks_claims_and_faults_an_active_room(self) -> None:
        ready = [True]
        arbiter = CooperativeArbiter(LeaseManager(clock=self.clock))
        gateway = CooperativeGateway(arbiter, readiness_probe=lambda: ready[0])
        gateway.mark_ready()
        gateway.open_connection("operator")
        gateway.receive(
            "operator", {"type": "hello", "operator_name": "Ana", "room_id": "default"}
        )
        gateway.receive("operator", {"type": "claim", "axes": ["X"]})

        ready[0] = False
        tick = gateway.maintenance_tick()

        self.assertEqual(tick.arbiter_tick.state, RoomState.FAULT)
        self.assertFalse(tick.room_state["axes"]["X"]["assigned"])

    def test_ros_telemetry_and_protocol_outcomes_are_logged_without_local_runtime(self) -> None:
        sample = TelemetrySample(
            source="ros_gazebo_h2c_fk",
            pose=ToolPose((0.1, 0.2, 0.3), (0.0, 0.0, 0.0, 1.0)),
            joint_positions=(0.0,) * 6,
            age_sec=0.01,
        )
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "ros.jsonl"
            with RunEventLogger(path, run_id="ros") as logger:
                gateway = CooperativeGateway(
                    CooperativeArbiter(LeaseManager(clock=self.clock)),
                    event_logger=logger,
                    telemetry_source=lambda: sample,
                )
                gateway.mark_ready()
                gateway.open_connection("operator")
                gateway.receive(
                    "operator",
                    {"type": "hello", "operator_name": "Ana", "room_id": "default"},
                )
                gateway.maintenance_tick()
            records = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual(records[0]["event_type"], "protocol")
        self.assertEqual(records[0]["payload"]["outcome"], "welcome")
        self.assertEqual(records[1]["event_type"], "telemetry")
        self.assertEqual(records[1]["payload"]["source"], "ros_gazebo_h2c_fk")
        self.assertEqual(records[1]["payload"]["pose"]["position"], [0.1, 0.2, 0.3])

    def test_ros_telemetry_is_live_without_requiring_an_event_log(self) -> None:
        sample = TelemetrySample(
            source="ros_gazebo_jaco2_fk",
            pose=ToolPose((0.4, -0.2, 0.7), (0.0, 0.0, 0.0, 1.0)),
            joint_positions=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
            age_sec=0.012,
        )
        gateway = CooperativeGateway(
            CooperativeArbiter(LeaseManager(clock=self.clock)),
            telemetry_source=lambda: sample,
        )
        gateway.mark_ready()

        tick = gateway.maintenance_tick()

        self.assertIsNotNone(tick.telemetry)
        assert tick.telemetry is not None
        self.assertEqual(tick.telemetry["type"], "telemetry")
        self.assertTrue(tick.telemetry["telemetry_available"])
        self.assertEqual(tick.telemetry["source"], "ros_gazebo_jaco2_fk")
        self.assertEqual(tick.telemetry["pose"]["position"], [0.4, -0.2, 0.7])
        self.assertEqual(
            tick.telemetry["joint_positions"], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        )
        self.assertEqual(tick.telemetry["twist"]["linear"], [0.0, 0.0, 0.0])

    def test_stale_ros_telemetry_reports_unavailable_without_fake_pose(self) -> None:
        gateway = CooperativeGateway(
            CooperativeArbiter(LeaseManager(clock=self.clock)),
            telemetry_source=lambda: None,
        )
        gateway.mark_ready()

        tick = gateway.maintenance_tick()

        self.assertIsNotNone(tick.telemetry)
        assert tick.telemetry is not None
        self.assertFalse(tick.telemetry["telemetry_available"])
        self.assertNotIn("pose", tick.telemetry)
        self.assertNotIn("joint_positions", tick.telemetry)


class SecuredGatewayTest(unittest.TestCase):
    def setUp(self) -> None:
        self.credentials = {"operators": ("a" * 40,), "observers": ("b" * 40,), "admins": ("c" * 40,)}
        self.ready = True
        self.telemetry = TelemetrySample(source="test", pose=ToolPose((0, 0, 0), (0, 0, 0, 1)), joint_positions=(0.0,) * 6, age_sec=0.0)
        self.gateway = CooperativeGateway(credentials=self.credentials, readiness_probe=lambda: self.ready, telemetry_source=lambda: self.telemetry)
        self.gateway.mark_ready()

    def login(self, name: str, role: str, credential: str, remote: bool = True) -> dict[str, object]:
        self.gateway.open_connection(name, remote_control_allowed=remote)
        return self.gateway.receive(name, {"type": "hello", "room_id": "default", "operator_name": name, "role": role, "credential": credential})[0]

    def test_role_credentials_and_local_restriction_are_enforced(self) -> None:
        self.assertEqual(self.login("local", "operator", "a" * 40, False)["code"], "authentication_failed")
        self.assertEqual(self.login("bad", "operator", "b" * 40)["code"], "authentication_failed")
        self.assertEqual(self.login("unicode", "operator", "ñ" * 40)["code"], "authentication_failed")
        self.assertEqual(self.login("observer", "observer", "b" * 40, False)["role"], "observer")
        for action in ("claim", "input", "release", "stop_global", "rearm"):
            self.assertEqual(self.gateway.receive("observer", {"type": action})[0]["code"], "forbidden")
        self.assertEqual(self.gateway.room_state()["state"], "READY")

    def test_stop_and_rearm_require_remote_admin_and_readiness(self) -> None:
        self.login("operator", "operator", "a" * 40)
        self.login("admin", "admin", "c" * 40)
        self.gateway.receive("operator", {"type": "claim", "axes": ["X"]})
        self.assertEqual(self.gateway.receive("operator", {"type": "stop_global"})[0]["code"], "forbidden")
        self.assertEqual(self.gateway.receive("admin", {"type": "claim", "axes": ["Y"]})[0]["code"], "forbidden")
        self.assertEqual(self.gateway.receive("admin", {"type": "stop_global"})[0]["type"], "stop_confirmed")
        tick = self.gateway.maintenance_tick()
        self.assertEqual(tick.arbiter_tick.twist.linear, (0.0, 0.0, 0.0))
        self.assertFalse(tick.room_state["axes"]["X"]["assigned"])
        self.ready = False
        self.assertEqual(self.gateway.receive("admin", {"type": "rearm"})[0]["code"], "room_not_operational")
        self.ready = True
        self.telemetry = None
        self.assertEqual(self.gateway.receive("admin", {"type": "rearm"})[0]["code"], "room_not_operational")
        self.telemetry = TelemetrySample(source="test", pose=ToolPose((0, 0, 0), (0, 0, 0, 1)), joint_positions=(0.0,) * 6, age_sec=0.0)
        self.assertEqual(self.gateway.receive("admin", {"type": "rearm"})[0]["type"], "rearm_confirmed")
        self.assertFalse(self.gateway.room_state()["axes"]["X"]["assigned"])

    def test_stale_telemetry_faults_room_and_blocks_rearm(self) -> None:
        self.login("operator", "operator", "a" * 40)
        self.login("admin", "admin", "c" * 40)
        self.gateway.receive("operator", {"type": "claim", "axes": ["X"]})
        self.telemetry = TelemetrySample(source="test", pose=ToolPose((0, 0, 0), (0, 0, 0, 1)), joint_positions=(0.0,) * 6, age_sec=0.31)
        tick = self.gateway.maintenance_tick()
        self.assertEqual(tick.room_state["state"], "FAULT")
        self.assertFalse(tick.room_state["axes"]["X"]["assigned"])
        self.assertFalse(tick.telemetry["telemetry_available"])
        self.assertEqual(self.gateway.receive("admin", {"type": "rearm"})[0]["code"], "room_not_operational")

    def test_local_simulator_stop_and_rearm_resets_runner_clock(self) -> None:
        clock = FakeClock()
        arbiter = CooperativeArbiter(LeaseManager(clock=clock))
        runtime = LocalSimulationRuntime(arbiter, DeterministicToolSimulator(), clock=clock)
        gateway = CooperativeGateway(arbiter, local_runtime=runtime, credentials=self.credentials)
        gateway.mark_ready()
        gateway.open_connection("admin")
        gateway.receive("admin", {"type": "hello", "room_id": "default", "operator_name": "admin", "role": "admin", "credential": "c" * 40})
        gateway.receive("admin", {"type": "stop_global"})
        clock.now = 1.0
        self.assertEqual(gateway.receive("admin", {"type": "rearm"})[0]["type"], "rearm_confirmed")
        clock.now += 0.02
        self.assertEqual(gateway.maintenance_tick().room_state["state"], "READY")

    def test_credential_file_permissions_and_roles(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "credentials.json"
            path.write_text(json.dumps(self.credentials))
            path.chmod(0o644)
            with self.assertRaises(ValueError):
                load_credentials(str(path))
            path.chmod(0o600)
            self.assertEqual(load_credentials(str(path)), self.credentials)
            path.write_text(json.dumps({"operators": ["a" * 40], "observers": ["a" * 40], "admins": ["c" * 40]}))
            with self.assertRaises(ValueError):
                load_credentials(str(path))

    def test_credential_cannot_be_shared_by_concurrent_sessions(self) -> None:
        self.assertEqual(self.login("one", "operator", "a" * 40)["type"], "welcome")
        self.assertEqual(self.login("two", "operator", "a" * 40)["code"], "authentication_failed")
        self.gateway.close_connection("one")
        self.gateway.close_connection("two")
        self.assertEqual(self.login("three", "operator", "a" * 40)["type"], "welcome")


if __name__ == "__main__":
    unittest.main()
