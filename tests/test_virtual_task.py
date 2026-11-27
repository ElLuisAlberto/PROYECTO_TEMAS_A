import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import pybullet as pb
from teleop_cooperativo.virtual_task import VirtualTask, OBJECTS, GOALS, GOAL_HEIGHTS
from teleop_cooperativo.virtual_task_spec import INITIAL
from teleop_cooperativo.jaco2_kinematics import forward_kinematics
from teleop_cooperativo.desktop_console import _parse_virtual_task


class VirtualTaskTest(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.game = VirtualTask(clock=lambda: self.now)

    def tearDown(self): self.game.close()

    def advance(self, seconds, sample=None, fingers=None):
        for _ in range(round(seconds / .05)):
            self.now += .05
            self.game.update(sample, fingers)

    def test_snapshot_is_validated_for_the_desktop(self):
        snapshot = self.game.snapshot()
        parsed = _parse_virtual_task(snapshot)
        self.assertIsNotNone(parsed)
        self.assertEqual(len(parsed['objects']), 5)
        snapshot['objects'][0]['position'][0] = float('nan')
        self.assertIsNone(_parse_virtual_task(snapshot))

    def test_five_100g_objects_and_gravity_on_table(self):
        self.assertEqual(len(self.game.bodies), 5)
        self.assertEqual(len(set(x['name'] for x in OBJECTS)), 5)
        for body in self.game.bodies:
            self.assertAlmostEqual(pb.getDynamicsInfo(body, -1, physicsClientId=self.game.client)[0], .1)
            lower, upper = pb.getAABB(body, physicsClientId=self.game.client)
            self.assertAlmostEqual(upper[2] - lower[2], .10, delta=.004)
        body = self.game.bodies[0]
        pb.resetBasePositionAndOrientation(body, [.19, -.26, .30], [0, 0, 0, 1], physicsClientId=self.game.client)
        self.advance(2)
        position, _ = pb.getBasePositionAndOrientation(body, physicsClientId=self.game.client)
        self.assertGreater(position[2], .012)
        self.assertLess(position[2], .08)

    def test_assisted_grip_lift_release_and_scoring(self):
        sample = SimpleNamespace(joint_positions=(0,)*6, pose=SimpleNamespace(quaternion_xyzw=(0,0,0,1)))
        center = [.19, -.26, .052]
        def tips(*_):
            return [(center[0] + .025, center[1], center[2]), (center[0] - .025, center[1], center[2]), (center[0], center[1]+.025, center[2])]
        with patch.object(VirtualTask, 'tip_positions', side_effect=tips), patch.object(VirtualTask, 'phalanx_poses', return_value=[((0, 0, -10), (0, 0, 0, 1))] * 6):
            self.advance(.1, sample, (0.7,)*3)
            self.advance(.1, sample, (0.9,)*3)
            self.assertEqual(self.game.held, 0)
            center[2] = .12
            self.advance(.4, sample, (1.0,)*3)
            lifted, _ = pb.getBasePositionAndOrientation(self.game.bodies[0], physicsClientId=self.game.client)
            self.assertGreater(lifted[2], .08)
            self.advance(.1, sample, (.3,)*3)
            self.assertIsNone(self.game.held)
            center[2] = .30
            self.advance(1.5, sample, (.3,)*3)
            dropped, _ = pb.getBasePositionAndOrientation(self.game.bodies[0], physicsClientId=self.game.client)
            self.assertLess(dropped[2], lifted[2])
        pb.resetBasePositionAndOrientation(self.game.bodies[0], [.19,.27,.05], [0,0,0,1], physicsClientId=self.game.client)
        pb.resetBaseVelocity(self.game.bodies[0], [0,0,0], [0,0,0], physicsClientId=self.game.client)
        self.advance(1.5)
        self.assertTrue(self.game.scored[0])
        pb.resetBasePositionAndOrientation(self.game.bodies[0], [.19, -.26, .05], [0,0,0,1], physicsClientId=self.game.client)
        self.advance(.1)
        self.assertFalse(self.game.scored[0])
        self.game.reset()
        self.assertEqual(self.game.snapshot()['delivered'], 0)

    def test_real_finger_geometry_grips_each_shape_lifts_and_releases(self):
        # Recorded reachable joint poses; no patched fingertip positions.
        low = (.934033, 2.360420, 5.096854, -3.868472, .740220, 6.039871)
        high = (.947006, 2.743695, 5.171131, -3.865051, .864780, 6.033402)
        for index in range(5):
            with self.subTest(object=OBJECTS[index]['name']):
                self.game.reset()
                for i, body in enumerate(self.game.bodies):
                    position = (.19, -.26, .052) if i == index else (.65, -.38 + i * .07, .052)
                    pb.resetBasePositionAndOrientation(body, position, (0, 0, 0, 1), physicsClientId=self.game.client)
                    pb.resetBaseVelocity(body, (0, 0, 0), (0, 0, 0), physicsClientId=self.game.client)
                def sample(joints):
                    return SimpleNamespace(joint_positions=joints, pose=forward_kinematics(joints))
                for step in range(65):
                    angle = min(step * .015, .9)
                    self.advance(.05, sample(low), (angle,) * 3)
                self.assertEqual(self.game.held, index)
                for step in range(1, 31):
                    joints = tuple(a + (b - a) * step / 30 for a, b in zip(low, high))
                    self.advance(.05, sample(joints), (.9,) * 3)
                lifted, _ = pb.getBasePositionAndOrientation(self.game.bodies[index], physicsClientId=self.game.client)
                self.assertGreater(lifted[2], .12)
                self.advance(.1, sample(high), (.3,) * 3)
                self.assertIsNone(self.game.held)
                self.advance(1.0, sample(high), (.3,) * 3)
                dropped, _ = pb.getBasePositionAndOrientation(self.game.bodies[index], physicsClientId=self.game.client)
                self.assertLess(dropped[2], lifted[2] - .03)

    def test_grip_releases_under_sustained_pressure_and_requires_opening(self):
        sample = SimpleNamespace(joint_positions=(0,) * 6, pose=SimpleNamespace(quaternion_xyzw=(0, 0, 0, 1)))
        center = [.19, -.26, .052]
        def tips(*_):
            return [(center[0] + .025, center[1], center[2]), (center[0] - .025, center[1], center[2]), (center[0], center[1] + .025, center[2])]
        with patch.object(VirtualTask, 'tip_positions', side_effect=tips), patch.object(VirtualTask, 'phalanx_poses', return_value=[((0, 0, -10), (0, 0, 0, 1))] * 6):
            self.advance(.2, sample, (.9,) * 3)
            self.assertEqual(self.game.held, 0)
            self.assertEqual(self.game.snapshot()['objects'][0]['state'], 'sujeto')
            for _ in range(18):
                center[2] -= .003
                self.advance(.05, sample, (.9,) * 3)
            self.assertIsNone(self.game.held)
            self.assertEqual(self.game.release_reason, 'impacto')
            position, _ = pb.getBasePositionAndOrientation(self.game.bodies[0], physicsClientId=self.game.client)
            self.assertGreater(position[2], .025)
            center[2] = .052
            self.advance(.1, sample, (.9,) * 3)
            self.assertIsNone(self.game.held)
            self.advance(.1, sample, (.3,) * 3)
            self.assertIsNone(self.game.release_reason)
            self.advance(.1, sample, (.9,) * 3)
            self.assertEqual(self.game.held, 0)

    def test_reset_one_object_preserves_other_objects_and_score(self):
        body = self.game.bodies[2]
        pb.resetBasePositionAndOrientation(body, (.4, .1, -.60), pb.getQuaternionFromEuler((1, 0, 0)), physicsClientId=self.game.client)
        pb.resetBaseVelocity(body, (1, 0, 0), (0, 1, 0), physicsClientId=self.game.client)
        other_before = pb.getBasePositionAndOrientation(self.game.bodies[1], physicsClientId=self.game.client)
        self.game.scored[2] = self.game.scored[1] = True
        self.game.goal_dwell[2] = 1
        self.game.reset_object(2)
        position, rotation = pb.getBasePositionAndOrientation(body, physicsClientId=self.game.client)
        velocity = pb.getBaseVelocity(body, physicsClientId=self.game.client)
        self.assertEqual(position, INITIAL[2])
        self.assertEqual(rotation, (0, 0, 0, 1))
        self.assertEqual(velocity, ((0, 0, 0), (0, 0, 0)))
        self.assertFalse(self.game.scored[2]); self.assertTrue(self.game.scored[1])
        self.assertEqual(other_before, pb.getBasePositionAndOrientation(self.game.bodies[1], physicsClientId=self.game.client))
        self.assertIn(2, self.game.reset_blocked)
        with self.assertRaises(ValueError): self.game.reset_object(True)
        pb.resetBasePositionAndOrientation(self.game.bodies[1], INITIAL[2], (0, 0, 0, 1), physicsClientId=self.game.client)
        with self.assertRaisesRegex(ValueError, 'ocupada'): self.game.reset_object(2)

    def test_reset_held_object_releases_without_immediate_regrip(self):
        sample = SimpleNamespace(joint_positions=(0,) * 6, pose=SimpleNamespace(quaternion_xyzw=(0, 0, 0, 1)))
        tips = [(.215, -.26, .052), (.165, -.26, .052), (.19, -.235, .052)]
        with patch.object(VirtualTask, 'tip_positions', return_value=tips), patch.object(VirtualTask, 'phalanx_poses', return_value=[((0, 0, -10), (0, 0, 0, 1))] * 6):
            self.advance(.1, sample, (.9,) * 3)
            self.assertEqual(self.game.held, 0)
            self.game.reset_object(0)
            self.advance(.1, sample, (.9,) * 3)
            self.assertIsNone(self.game.held)
            self.advance(.1, sample, (.3,) * 3)
            self.advance(.1, sample, (.9,) * 3)
            self.assertEqual(self.game.held, 0)

    def test_fallen_object_is_not_counted_as_delivered(self):
        body = self.game.bodies[0]
        pb.resetBasePositionAndOrientation(body, (GOALS[0][0], GOALS[0][1], .05),
            pb.getQuaternionFromEuler((0, math.pi / 2, 0)), physicsClientId=self.game.client)
        self.advance(2.0)
        self.assertEqual(self.game.snapshot()['objects'][0]['state'], 'caido')
        self.assertFalse(self.game.scored[0])

    def test_each_start_and_destination_has_a_kinematic_point_solution(self):
        # Point reachability only: this is not a collision-free trajectory check.
        solutions = [
            (.926, 2.254, 5.073, -3.872, .711, 6.041),
            (.015, 1.451, 4.845, 2.887, -3.046, 2.162),
            (6.270, 1.592, 4.594, 2.591, -2.389, -3.961),
            (6.275, 1.621, 4.256, 2.472, -1.997, -6.279),
            (-2.824, 5.011, 3.960, -1.865, -2.398, -3.882),
            (4.848, 2.161, 5.239, 1.753, -.798, -1.458),
            (-1.376, 1.674, 4.823, -3.001, -3.852, -3.862),
            (-.495, 2.382, 5.217, 5.762, .278, -2.975),
            (-.063, 1.646, 4.213, 6.282, 2.809, -5.318),
            (-.236, 2.168, 4.452, -5.696, -5.796, -5.073),
        ]
        targets = list(INITIAL) + [(x, y, h + .05) for (x, y, _), h in zip(GOALS, GOAL_HEIGHTS)]
        for joints, target in zip(solutions, targets):
            self.assertLess(math.dist(forward_kinematics(joints).position, target), .004)

    def test_every_destination_catches_an_object_at_the_visible_height(self):
        for index, (goal, height) in enumerate(zip(GOALS, GOAL_HEIGHTS)):
            with self.subTest(destination=index):
                self.game.reset()
                body = self.game.bodies[1]
                pb.resetBasePositionAndOrientation(body, (goal[0], goal[1], height + .16),
                    (0, 0, 0, 1), physicsClientId=self.game.client)
                self.advance(2.0)
                lower, _ = pb.getAABB(body, physicsClientId=self.game.client)
                self.assertAlmostEqual(lower[2], height, delta=.012)

    def test_objects_collide_and_transfer_momentum(self):
        first, second = self.game.bodies[:2]
        pb.resetBasePositionAndOrientation(first, (.20, -.10, .052), (0, 0, 0, 1), physicsClientId=self.game.client)
        pb.resetBasePositionAndOrientation(second, (.25, -.10, .052), (0, 0, 0, 1), physicsClientId=self.game.client)
        pb.resetBaseVelocity(first, (1.0, 0, 0), (0, 0, 0), physicsClientId=self.game.client)
        self.advance(.8)
        position, _ = pb.getBasePositionAndOrientation(second, physicsClientId=self.game.client)
        self.assertGreater(position[0], .255)

    def test_high_destination_supports_a_falling_object(self):
        body = self.game.bodies[4]
        pb.resetBasePositionAndOrientation(body, [GOALS[4][0], GOALS[4][1], .34], [0,0,0,1], physicsClientId=self.game.client)
        self.advance(2.0)
        position, _ = pb.getBasePositionAndOrientation(body, physicsClientId=self.game.client)
        self.assertGreater(position[2], GOAL_HEIGHTS[4])
        self.assertTrue(self.game.scored[4])

    def test_virtual_finger_pushes_object_by_contact(self):
        sample = SimpleNamespace(joint_positions=(0,)*6, pose=SimpleNamespace(quaternion_xyzw=(0,0,0,1)))
        x = [.13]
        def tips(*_): return [(x[0], -.26, .04), (0, 0, .5), (0, .1, .5)]
        start, _ = pb.getBasePositionAndOrientation(self.game.bodies[0], physicsClientId=self.game.client)
        with patch.object(VirtualTask, 'tip_positions', side_effect=tips), patch.object(VirtualTask, 'phalanx_poses', return_value=[((0, 0, -10), (0, 0, 0, 1))] * 6):
            for step in range(25):
                x[0] = .13 + step * .003
                self.advance(.05, sample, (.1,)*3)
        end, _ = pb.getBasePositionAndOrientation(self.game.bodies[0], physicsClientId=self.game.client)
        self.assertGreater(end[0] - start[0], .015)

    def test_stale_telemetry_releases_grasp(self):
        sample = SimpleNamespace(joint_positions=(0,)*6, pose=SimpleNamespace(quaternion_xyzw=(0,0,0,1)))
        tips = [(.215,-.26,.052),(.165,-.26,.052),(.19,-.235,.052)]
        with patch.object(VirtualTask, 'tip_positions', return_value=tips), patch.object(VirtualTask, 'phalanx_poses', return_value=[((0, 0, -10), (0, 0, 0, 1))] * 6):
            self.advance(.1, sample, (.7,)*3)
            self.advance(.1, sample, (.9,)*3)
            self.assertEqual(self.game.held, 0)
            self.advance(.05, None, None)
            self.assertIsNone(self.game.held)


if __name__ == '__main__': unittest.main()
