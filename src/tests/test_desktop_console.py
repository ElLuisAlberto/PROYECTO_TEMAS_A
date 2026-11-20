"""Tests for the native console model without requiring a graphical display."""

from __future__ import annotations

import asyncio
import json
import unittest

import websockets

from teleop_cooperativo.desktop_console import DesktopConsole, DesktopState, GatewayObserver


class DesktopStateTest(unittest.TestCase):
    def test_room_and_real_telemetry_are_reduced_for_rendering(self) -> None:
        state = DesktopState("laboratorio")
        self.assertTrue(
            state.apply(
                {
                    "type": "room_state",
                    "state": "ACTIVE",
                    "revision": 4,
                    "axes": {
                        "X": {"assigned": True, "owner": "Ana"},
                        "Y": {"assigned": False, "owner": None},
                    },
                }
            )
        )
        self.assertTrue(
            state.apply(
                {
                    "type": "telemetry",
                    "state": "ACTIVE",
                    "source": "ros_gazebo_jaco2_fk",
                    "pose_age_sec": 0.012,
                    "pose": {
                        "position": [0.1, 0.2, 0.3],
                        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                    "twist": {
                        "linear": [0.05, 0.0, 0.0],
                        "angular": [0.0, 0.0, 0.1],
                    },
                    "joint_positions": [0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
                }
            )
        )

        snapshot = state.snapshot()
        self.assertEqual(snapshot.room_id, "laboratorio")
        self.assertEqual(snapshot.room_state, "ACTIVE")
        self.assertEqual(snapshot.revision, 4)
        self.assertEqual(snapshot.owners["X"], "Ana")
        self.assertEqual(snapshot.owners["Y"], "Disponible")
        self.assertEqual(snapshot.telemetry_source, "ros_gazebo_jaco2_fk")
        self.assertEqual(snapshot.position, (0.1, 0.2, 0.3))
        self.assertEqual(snapshot.linear, (0.05, 0.0, 0.0))
        self.assertEqual(snapshot.joint_positions, (0.0, 1.0, 2.0, 3.0, 4.0, 5.0))

    def test_malformed_or_unknown_messages_do_not_corrupt_state(self) -> None:
        state = DesktopState()
        self.assertFalse(state.apply({"state": "FAULT"}))
        self.assertFalse(state.apply({"type": "unrelated", "state": "FAULT"}))
        state.apply(
            {
                "type": "telemetry",
                "pose": {"position": [float("nan"), 0, 0]},
                "twist": {"linear": [True, 0, 0]},
            }
        )
        snapshot = state.snapshot()
        self.assertEqual(snapshot.room_state, "—")
        self.assertIsNone(snapshot.position)
        self.assertIsNone(snapshot.linear)

    def test_unavailable_telemetry_clears_previous_robot_observation(self) -> None:
        state = DesktopState()
        state.apply(
            {
                "type": "telemetry",
                "telemetry_available": True,
                "source": "ros_gazebo_jaco2_fk",
                "pose_age_sec": 0.01,
                "pose": {
                    "position": [0.1, 0.2, 0.3],
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
                "joint_positions": [0.0, 1.0, 1.0, 0.0, 0.0, 0.0],
                "twist": {"linear": [0.05, 0.0, 0.0], "angular": [0.0] * 3},
            }
        )

        state.apply(
            {
                "type": "telemetry",
                "state": "READY",
                "telemetry_available": False,
                "twist": {"linear": [0.0] * 3, "angular": [0.0] * 3},
            }
        )

        snapshot = state.snapshot()
        self.assertEqual(snapshot.telemetry_source, "Telemetría no disponible")
        self.assertIsNone(snapshot.telemetry_age_sec)
        self.assertIsNone(snapshot.position)
        self.assertIsNone(snapshot.quaternion)
        self.assertIsNone(snapshot.joint_positions)
        self.assertIsNone(snapshot.updated_monotonic)
        self.assertEqual(snapshot.linear, (0.0, 0.0, 0.0))

    def test_error_and_connection_events_are_bounded(self) -> None:
        state = DesktopState(max_events=2)
        state.set_connection(True, "Conectado")
        state.apply({"type": "error", "detail": "fallo controlado"})
        state.add_event("último")
        snapshot = state.snapshot()
        self.assertEqual(len(snapshot.events), 2)
        self.assertIn("último", snapshot.events[0])
        self.assertIn("fallo controlado", snapshot.events[1])

    def test_disconnect_invalidates_robot_sample(self) -> None:
        state = DesktopState()
        state.set_connection(True, "Conectado")
        state.apply({"type": "telemetry", "source": "ros_gazebo_jaco2_fk", "pose_age_sec": 0.1,
                     "pose": {"position": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]},
                     "joint_positions": [0, 1, 1, 0, 0, 0]})
        self.assertIsNotNone(state.snapshot().position)
        state.set_connection(False, "Desconectado")
        self.assertIsNone(state.snapshot().position)
        self.assertIsNone(state.snapshot().joint_positions)
        self.assertIsNone(state.snapshot().updated_monotonic)
        self.assertEqual(state.snapshot().owners["X"], "Desconocido")

    def test_partial_jaco_sample_invalidates_older_observation(self) -> None:
        state = DesktopState()
        sample = {"type": "telemetry", "source": "ros_gazebo_jaco2_fk", "pose_age_sec": 0.1,
                  "pose": {"position": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]},
                  "joint_positions": [0, 1, 1, 0, 0, 0]}
        state.apply(sample)
        state.apply({**sample, "joint_positions": [0] * 5})
        self.assertIsNone(state.snapshot().joint_positions)
        self.assertIsNone(state.snapshot().position)

    def test_jaco_sample_without_age_is_not_fresh(self) -> None:
        state = DesktopState()
        state.apply({"type": "telemetry", "source": "ros_gazebo_jaco2_fk",
                     "pose": {"position": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]},
                     "joint_positions": [0, 1, 1, 0, 0, 0]})
        self.assertIsNone(state.snapshot().updated_monotonic)
        self.assertIsNone(state.snapshot().position)

    def test_vector_format_is_stable_for_operator_display(self) -> None:
        self.assertEqual(DesktopConsole._format_vector(None), "—")
        self.assertEqual(
            DesktopConsole._format_vector((0.1, -0.2, 0.0)),
            "[0.1000, -0.2000, 0.0000]",
        )


class GatewayObserverTest(unittest.TestCase):
    def test_rejects_non_websocket_url(self) -> None:
        with self.assertRaises(ValueError):
            GatewayObserver(DesktopState(), url="http://127.0.0.1:8765/ws")


class GatewayObserverHeartbeatTest(unittest.IsolatedAsyncioTestCase):
    async def test_heartbeat_continues_during_constant_telemetry(self) -> None:
        received = asyncio.Event()

        async def handler(socket):
            await socket.recv()  # hello
            async def publish():
                while True:
                    await socket.send(json.dumps({"type": "telemetry"}))
                    await asyncio.sleep(0.02)
            sender = asyncio.create_task(publish())
            try:
                async for raw in socket:
                    if json.loads(raw).get("type") == "heartbeat":
                        received.set()
                        break
            finally:
                sender.cancel()
                await asyncio.gather(sender, return_exceptions=True)

        async with websockets.serve(handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            observer = GatewayObserver(DesktopState(), url=f"ws://127.0.0.1:{port}/ws")
            observer.start()
            try:
                await asyncio.wait_for(received.wait(), timeout=2)
            finally:
                await asyncio.to_thread(observer.stop)


if __name__ == "__main__":
    unittest.main()
