import math
import unittest
from teleop_cooperativo.arbiter import RoomState
from teleop_cooperativo.gateway import CooperativeGateway
from teleop_cooperativo.simulated_gripper import SimulatedGripper
from teleop_cooperativo.simulation_port import TelemetrySample, ToolPose


class Bridge:
    def __init__(self): self.active = False; self.reset_requested = False
    def recovery_state(self): return self.active
    def recovery_permit(self, value): pass
    def web_permit(self, value): pass
    def take_game_reset(self):
        result = self.reset_requested
        self.reset_requested = False
        return result
    def finger_velocity(self, value):
        raise AssertionError('La pinza simulada nunca debe mandar comandos al bridge')


class FingerControlTest(unittest.TestCase):
    def setUp(self):
        self.bridge = Bridge()
        self.gateway = CooperativeGateway(
            readiness_probe=lambda: True, telemetry_source=self.telemetry,
            twist_sink=lambda _twist: None, recovery_bridge=self.bridge,
            wait_for_initial_ready=True)
        self.now = [0.0]
        self.gateway._sim_gripper = SimulatedGripper(clock=lambda: self.now[0])
        self.gateway.maintenance_tick()
        self.gateway.open_connection('op')
        reply = self.gateway.receive('op', {'type':'hello', 'room_id':'default',
                                           'operator_name':'Operador', 'role':'operator'})
        self.assertEqual(reply[0]['type'], 'welcome')

    def telemetry(self):
        # The physical fingers do not report any valid state.
        return TelemetrySample('ros_physical_jaco2_fk', ToolPose((0, 0, 0), (0, 0, 0, 1)),
                               (0.2, 2.4, 2.7, 0.4, 0.8, 0.1), 0.0, None)

    def test_virtual_gripper_works_without_physical_feedback_and_never_calls_hardware(self):
        response = self.gateway.receive('op', {'type':'claim', 'axes':['PINZA']})
        self.assertEqual(response[0]['type'], 'claim_granted')
        item = response[0]['leases'][0]
        token = {'PINZA': {'lease_id': item['lease_id'], 'generation': item['generation']}}
        self.gateway.receive('op', {'type':'input', 'sequence':1, 'values':{'PINZA':1}, 'leases':token})
        self.now[0] += 0.05
        tick = self.gateway.maintenance_tick()
        self.assertEqual(tick.telemetry['finger_source'], 'simulated')
        self.assertGreater(tick.telemetry['finger_positions'][0], 0)
        self.assertEqual(len(set(tick.telemetry['finger_positions'])), 1)
        before = self.gateway._sim_gripper.positions
        self.gateway.fault()
        self.now[0] += 0.05
        tick = self.gateway.maintenance_tick()
        self.assertEqual(tick.arbiter_tick.state, RoomState.FAULT)
        self.assertEqual(self.gateway._sim_gripper.positions, before)

    def test_pause_preserves_reservation_but_stops_virtual_fingers(self):
        grant = self.gateway.receive('op', {'type':'claim', 'axes':['PINZA']})[0]['leases'][0]
        token = {'PINZA': {'lease_id': grant['lease_id'], 'generation': grant['generation']}}
        self.gateway.receive('op', {'type':'input', 'sequence':1, 'values':{'PINZA':1}, 'leases':token})
        self.now[0] += 0.05
        self.gateway.maintenance_tick()
        stopped = self.gateway._sim_gripper.positions
        self.bridge.active = True
        for _ in range(8):
            self.now[0] += 0.05
            self.gateway.arbiter.heartbeat(self.gateway._connections['op'])
            result = self.gateway.maintenance_tick()
            self.assertEqual(result.arbiter_tick.state, RoomState.PAUSED)
            self.assertEqual(self.gateway._sim_gripper.positions, stopped)
        self.bridge.active = False
        self.now[0] += 0.05
        self.gateway.maintenance_tick()
        self.assertEqual(self.gateway._sim_gripper.positions, stopped)
        self.gateway.receive('op', {'type':'input', 'sequence':2, 'values':{'PINZA':1}, 'leases':token})
        self.now[0] += 0.05
        self.gateway.maintenance_tick()
        self.assertEqual(self.gateway._sim_gripper.positions, stopped)
        self.gateway.receive('op', {'type':'input', 'sequence':3, 'values':{'PINZA':0}, 'leases':token})
        self.gateway.receive('op', {'type':'input', 'sequence':4, 'values':{'PINZA':1}, 'leases':token})
        self.now[0] += 0.05
        self.gateway.maintenance_tick()
        self.assertGreater(self.gateway._sim_gripper.positions[0], stopped[0])

    def test_virtual_only_works_with_physical_output_disarmed(self):
        sent = []
        gateway = CooperativeGateway(readiness_probe=lambda: False, telemetry_source=self.telemetry,
                                     twist_sink=sent.append, recovery_bridge=self.bridge,
                                     wait_for_initial_ready=True)
        clock = [0.0]
        gateway._sim_gripper = SimulatedGripper(clock=lambda: clock[0])
        gateway.maintenance_tick()
        self.assertEqual(gateway.arbiter.state, RoomState.SIM_ONLY)
        self.assertTrue(gateway.room_state()['axes']['PINZA']['available'])
        self.assertFalse(gateway.room_state()['axes']['X']['available'])
        gateway.open_connection('op')
        gateway.receive('op', {'type':'hello','room_id':'default','operator_name':'op','role':'operator'})
        denied = gateway.receive('op', {'type':'claim','axes':['X']})
        self.assertEqual(denied[0]['type'], 'error')
        grant = gateway.receive('op', {'type':'claim','axes':['PINZA']})[0]['leases'][0]
        token = {'PINZA': {'lease_id':grant['lease_id'], 'generation':grant['generation']}}
        gateway.receive('op', {'type':'input','sequence':1,'values':{'PINZA':1},'leases':token})
        clock[0] += 0.05
        tick = gateway.maintenance_tick()
        self.assertGreater(tick.telemetry['finger_positions'][0], 0)
        self.assertEqual(sent[-1].linear, (0.0, 0.0, 0.0))
        self.assertEqual(sent[-1].angular, (0.0, 0.0, 0.0))
        gateway.fault()
        self.assertEqual(gateway.arbiter.state, RoomState.FAULT)
        gateway.open_connection('admin')
        gateway.receive('admin', {'type':'hello','room_id':'default','operator_name':'admin','role':'admin'})
        response = gateway.receive('admin', {'type':'rearm'})
        self.assertEqual(response[0]['type'], 'rearm_confirmed')
        self.assertEqual(gateway.arbiter.state, RoomState.SIM_ONLY)

    def test_task_reset_is_forwarded_without_robot_commands(self):
        class Task:
            def __init__(self): self.reset_count = 0
            def reset(self): self.reset_count += 1
            def update(self, sample, fingers): return {'total':5, 'delivered':0, 'objects':[], 'held':None}
        if self.gateway._virtual_task is not None:
            self.gateway._virtual_task.close()
        task = Task()
        self.gateway._virtual_task = task
        self.bridge.reset_requested = True
        tick = self.gateway.maintenance_tick()
        self.assertEqual(task.reset_count, 1)
        self.assertEqual(tick.telemetry['virtual_task']['total'], 5)
        self.gateway.maintenance_tick()
        self.assertEqual(task.reset_count, 1)

    def test_reset_one_object_requires_authorized_role_and_valid_id(self):
        self.gateway.open_connection('observer')
        self.gateway.receive('observer', {'type':'hello', 'room_id':'default', 'operator_name':'Observador', 'role':'observer'})
        self.assertEqual(self.gateway.receive('observer', {'type':'reset_object', 'object_id':0})[0]['code'], 'forbidden')
        for invalid in (-1, 5, True, '0'):
            self.assertEqual(self.gateway.receive('op', {'type':'reset_object', 'object_id':invalid})[0]['code'], 'validation_error')
        result = self.gateway.receive('op', {'type':'reset_object', 'object_id':2})[0]
        self.assertEqual(result, {'type':'object_reset', 'object_id':2})
        self.bridge.active = True
        self.gateway.maintenance_tick()
        self.assertEqual(self.gateway.receive('op', {'type':'reset_object', 'object_id':2})[0]['code'], 'room_not_operational')

    def test_simulator_bounds_and_no_jump_after_long_delay(self):
        now = [0.0]
        gripper = SimulatedGripper(clock=lambda: now[0])
        gripper.step(-1)
        self.assertEqual(gripper.positions, (0.0, 0.0, 0.0))
        now[0] = 100.0
        gripper.step(1)
        self.assertLessEqual(gripper.positions[0], math.radians(2.01))
        for _ in range(100):
            now[0] += 0.1
            gripper.step(1)
        self.assertAlmostEqual(gripper.positions[0], math.radians(80))


if __name__ == '__main__': unittest.main()
