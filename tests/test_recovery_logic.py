import unittest

from teleop_cooperativo.arbiter import CooperativeArbiter, RoomState
from teleop_cooperativo.domain import Axis
from teleop_cooperativo.gateway import CooperativeGateway
from teleop_cooperativo.lease_manager import LeaseManager


class Bridge:
    def __init__(self):
        self.active = False
        self.permits = []
        self.web_permits = []

    def recovery_state(self):
        return self.active

    def recovery_permit(self, allowed):
        self.permits.append(allowed)

    def web_permit(self, allowed):
        self.web_permits.append(allowed)


class RecoveryArbitrationTest(unittest.TestCase):
    def setUp(self):
        self.ready = True
        self.bridge = Bridge()
        self.arbiter = CooperativeArbiter()
        self.gateway = CooperativeGateway(self.arbiter, readiness_probe=lambda: self.ready,
                                          recovery_bridge=self.bridge, wait_for_initial_ready=True)
        self.gateway.maintenance_tick()
        self.gateway.maintenance_tick()
        self.assertEqual(self.arbiter.state, RoomState.READY)

    def test_pause_retains_leases_and_blocks_commands(self):
        self.arbiter.claim('operator', [Axis.X])
        self.bridge.active = True
        result = self.gateway.maintenance_tick()
        self.assertEqual(result.arbiter_tick.state, RoomState.PAUSED)
        self.assertIsNotNone(self.arbiter.leases.lease(Axis.X))
        self.assertEqual(result.arbiter_tick.twist.linear, (0.0, 0.0, 0.0))
        with self.assertRaises(RuntimeError):
            self.arbiter.claim('operator', [Axis.Y])
        self.assertTrue(self.bridge.permits[-1])

    def test_stop_revokes_recovery_permission(self):
        self.bridge.active = True
        self.gateway.maintenance_tick()
        self.gateway.fault()
        self.assertEqual(self.arbiter.state, RoomState.FAULT)
        self.assertFalse(self.bridge.permits[-1])
        self.assertFalse(self.bridge.web_permits[-1])
        self.bridge.active = False
        self.gateway.maintenance_tick()
        self.assertEqual(self.arbiter.state, RoomState.FAULT)

    def test_exit_restores_live_leases_only_after_neutral(self):
        lease = self.arbiter.claim('operator', [Axis.X])[0]
        token = {Axis.X: (lease.lease_id, lease.generation)}
        self.arbiter.submit_input('operator', 1, {Axis.X: 1}, token)
        self.bridge.active = True
        self.gateway.maintenance_tick()
        self.assertFalse(self.bridge.web_permits[-1])
        self.assertEqual(lease.value, 0.0)
        self.arbiter.heartbeat('operator')
        self.bridge.active = False
        self.gateway.maintenance_tick()
        self.assertEqual(self.arbiter.state, RoomState.ACTIVE)
        self.assertTrue(self.bridge.web_permits[-1])
        self.assertIs(self.arbiter.leases.lease(Axis.X), lease)
        self.arbiter.submit_input('operator', 2, {Axis.X: 1}, token)
        self.assertEqual(lease.value, 0.0)
        self.arbiter.submit_input('operator', 3, {Axis.X: 0}, token)
        self.arbiter.submit_input('operator', 4, {Axis.X: 1}, token)
        self.assertEqual(lease.value, 1.0)

    def test_paused_heartbeats_preserve_only_live_reservations(self):
        now = [0.0]
        arbiter = CooperativeArbiter(LeaseManager(clock=lambda: now[0]))
        arbiter.mark_ready()
        arbiter.claim('live', [Axis.X])
        arbiter.claim('lost', [Axis.Y])
        arbiter.pause()
        for step in range(1, 7):
            now[0] = step * 0.1
            arbiter.heartbeat('live')
            arbiter.tick()
        self.assertIsNotNone(arbiter.leases.lease(Axis.X))
        self.assertIsNone(arbiter.leases.lease(Axis.Y))
        arbiter.resume()
        self.assertEqual(arbiter.state, RoomState.ACTIVE)

    def test_disconnected_operator_releases_lease_while_paused(self):
        self.arbiter.claim('operator', [Axis.X])
        self.bridge.active = True
        self.gateway.maintenance_tick()
        self.arbiter.release('operator')
        self.assertIsNone(self.arbiter.leases.lease(Axis.X))
        self.assertEqual(self.arbiter.state, RoomState.PAUSED)

    def test_stop_revokes_paused_reservations(self):
        self.arbiter.claim('operator', [Axis.X])
        self.bridge.active = True
        self.gateway.maintenance_tick()
        self.gateway.fault()
        self.assertIsNone(self.arbiter.leases.lease(Axis.X))
        self.assertEqual(self.arbiter.state, RoomState.FAULT)

    def test_missing_bridge_heartbeat_faults(self):
        self.bridge.active = True
        self.gateway.maintenance_tick()
        self.bridge.active = None
        self.gateway.maintenance_tick()
        self.assertEqual(self.arbiter.state, RoomState.FAULT)
        self.assertFalse(self.bridge.permits[-1])


if __name__ == '__main__':
    unittest.main()
