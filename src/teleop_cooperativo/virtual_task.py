"""Isolated PyBullet tabletop task driven by observed JACO pose only.

The rigid bodies and gripper are virtual. Nothing here publishes robot commands.
Grasping uses a proximity-assisted fixed constraint between three virtual tips.
"""
from __future__ import annotations

import math
import time

import numpy as np
import pybullet as pb

from .robot_frames import FINGER_ORIGINS, _origin_transform, finger_link_transform, link_transforms
from .jaco2_kinematics import _quaternion

from .virtual_task_spec import OBJECTS, INITIAL, GOALS, GOAL_HEIGHTS, _parts


class VirtualTask:
    """Five independent dynamic objects and a kinematic, virtual gripper."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.client = pb.connect(pb.DIRECT)
        if self.client < 0:
            raise RuntimeError('No se pudo iniciar la física del juego')
        pb.setGravity(0, 0, -9.81, physicsClientId=self.client)
        pb.setTimeStep(1.0 / 120.0, physicsClientId=self.client)
        pb.setPhysicsEngineParameter(numSolverIterations=60, physicsClientId=self.client)
        table = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[.60, .45, .03], physicsClientId=self.client)
        self.table = pb.createMultiBody(baseMass=0, baseCollisionShapeIndex=table, basePosition=[.30, 0, -.03], physicsClientId=self.client)
        floor = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[2.5, 2.5, .01], physicsClientId=self.client)
        self.floor = pb.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor, basePosition=[.1, 0, -.73], physicsClientId=self.client)
        self.goal_supports = [self.table]
        for goal, height in zip(GOALS, GOAL_HEIGHTS):
            if height > 0:
                pedestal = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[.047, .047, height / 2], physicsClientId=self.client)
                self.goal_supports.append(pb.createMultiBody(baseMass=0, baseCollisionShapeIndex=pedestal,
                                   basePosition=[goal[0], goal[1], height / 2], physicsClientId=self.client))
        self.bodies = []
        shape_ids = {'box': pb.GEOM_BOX, 'sphere': pb.GEOM_SPHERE, 'cylinder': pb.GEOM_CYLINDER, 'capsule': pb.GEOM_CAPSULE}
        for spec, start in zip(OBJECTS, INITIAL):
            parts = _parts(spec)
            shape = pb.createCollisionShapeArray(
                shapeTypes=[shape_ids[p['shape']] for p in parts],
                radii=[p['radius'] for p in parts], lengths=[p['height'] for p in parts],
                halfExtents=[p['half'] for p in parts], collisionFramePositions=[p['center'] for p in parts],
                physicsClientId=self.client)
            body = pb.createMultiBody(baseMass=.1, baseCollisionShapeIndex=shape, basePosition=start, physicsClientId=self.client)
            pb.changeDynamics(body, -1, lateralFriction=.85, spinningFriction=.02, restitution=.05,
                              linearDamping=.08, angularDamping=.12, physicsClientId=self.client)
            self.bodies.append(body)
        tip_shape = pb.createCollisionShape(pb.GEOM_SPHERE, radius=.009, physicsClientId=self.client)
        self.tips = [pb.createMultiBody(baseMass=0, baseCollisionShapeIndex=tip_shape,
                                      basePosition=[0, 0, -10], physicsClientId=self.client) for _ in range(3)]
        # Proxies cover the visible proximal and distal phalanges as well as the tips.
        self.phalanges = []
        for height in (.038, .032):
            shape = pb.createCollisionShape(pb.GEOM_CAPSULE, radius=.009, height=height,
                collisionFrameOrientation=pb.getQuaternionFromEuler([0, math.pi / 2, 0]), physicsClientId=self.client)
            self.phalanges.append([pb.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape,
                basePosition=[0, 0, -10], physicsClientId=self.client) for _ in range(3)])
        self.previous_proxies = None
        self.previous_anchor = None
        self.grip_blocked = False
        self.reset_blocked = set()
        self.contact_load_time = 0.0
        self.release_reason = None
        self.anchor = pb.createMultiBody(baseMass=0, basePosition=[0, 0, -10], physicsClientId=self.client)
        self.held = None
        self.constraint = None
        self.last_time = self.clock()
        self.previous_angle = None
        self.previous_center = None
        self.scored = [False] * 5
        self.goal_dwell = [0.0] * 5
        self._accumulator = 0.0

    def close(self):
        if self.client is not None:
            pb.disconnect(self.client)
            self.client = None

    def reset(self):
        self._release()
        for body, start in zip(self.bodies, INITIAL):
            pb.resetBasePositionAndOrientation(body, start, [0, 0, 0, 1], physicsClientId=self.client)
            pb.resetBaseVelocity(body, [0, 0, 0], [0, 0, 0], physicsClientId=self.client)
        self.scored = [False] * 5
        self.goal_dwell = [0.0] * 5
        self.previous_angle = None
        self.previous_center = None
        self.previous_proxies = None
        self.previous_anchor = None
        self.grip_blocked = False
        self.reset_blocked.clear()
        self.contact_load_time = 0.0
        self.release_reason = None
        self._accumulator = 0.0
        self.last_time = self.clock()

    def reset_object(self, index):
        """Return one virtual object to its initial pose without moving the robot."""
        if type(index) is not int or not 0 <= index < len(self.bodies):
            raise ValueError('Muñeco inválido')
        start = INITIAL[index]
        for other, body in enumerate(self.bodies):
            if other != index:
                position, _ = pb.getBasePositionAndOrientation(body, physicsClientId=self.client)
                if math.dist(position, start) < .09:
                    raise ValueError('La posición inicial está ocupada por otro muñeco')
        if self.held == index:
            self._release()
        body = self.bodies[index]
        pb.resetBasePositionAndOrientation(body, start, (0, 0, 0, 1), physicsClientId=self.client)
        pb.resetBaseVelocity(body, (0, 0, 0), (0, 0, 0), physicsClientId=self.client)
        self.scored[index] = False
        self.goal_dwell[index] = 0.0
        self.reset_blocked.add(index)

    def _release(self, reason=None):
        if self.constraint is not None:
            pb.removeConstraint(self.constraint, physicsClientId=self.client)
            self.constraint = None
        if self.held is not None:
            for tip in self.tips + sum(self.phalanges, []):
                pb.setCollisionFilterPair(self.bodies[self.held], tip, -1, -1, 1, physicsClientId=self.client)
        self.held = None
        self.contact_load_time = 0.0
        if reason:
            self.grip_blocked = True
            self.release_reason = reason

    def _attach(self, index, center, orientation):
        body = self.bodies[index]
        position, rotation = pb.getBasePositionAndOrientation(body, physicsClientId=self.client)
        inv_position, inv_rotation = pb.invertTransform(center, orientation)
        rel_position, rel_rotation = pb.multiplyTransforms(inv_position, inv_rotation, position, rotation)
        self.constraint = pb.createConstraint(self.anchor, -1, body, -1, pb.JOINT_FIXED, [0, 0, 0],
                                              rel_position, [0, 0, 0], parentFrameOrientation=rel_rotation,
                                              physicsClientId=self.client)
        pb.changeConstraint(self.constraint, maxForce=12, erp=.3, physicsClientId=self.client)
        self.held = index
        self.hold_relative = (rel_position, rel_rotation)
        self.release_reason = None
        self.contact_load_time = 0.0
        for tip in self.tips + sum(self.phalanges, []):
            pb.setCollisionFilterPair(body, tip, -1, -1, 0, physicsClientId=self.client)

    @staticmethod
    def tip_positions(joints, fingers):
        hand = link_transforms(joints)[6]
        result = []
        for (xyz, rpy), angle in zip(FINGER_ORIGINS, fingers):
            frame = finger_link_transform(hand, _origin_transform(xyz, rpy), angle, distal=True)
            result.append(tuple((frame @ np.array([.033, -.004, 0.0, 1.0]))[:3]))
        return result

    @staticmethod
    def phalanx_poses(joints, fingers):
        hand = link_transforms(joints)[6]
        result = [[], []]
        for (xyz, rpy), angle in zip(FINGER_ORIGINS, fingers):
            for distal in (False, True):
                frame = finger_link_transform(hand, _origin_transform(xyz, rpy), angle, distal=distal)
                local = [.016, -.0045, 0, 1] if distal else [.021, -.008, 0, 1]
                position = tuple(float(v) for v in (frame @ np.array(local))[:3])
                result[int(distal)].append((position, _quaternion(frame[:3, :3])))
        return sum(result, [])

    def _candidate(self, center, tips):
        candidates = []
        for index, body in enumerate(self.bodies):
            if index in self.reset_blocked:
                continue
            position, _ = pb.getBasePositionAndOrientation(body, physicsClientId=self.client)
            distance = math.dist(center, position)
            if distance > .09:
                continue
            # Surface distance accounts for the different shapes and the full fingers.
            near = 0
            for finger in range(3):
                proxies = (self.tips[finger], self.phalanges[0][finger], self.phalanges[1][finger])
                if any(pb.getClosestPoints(body, proxy, .014, physicsClientId=self.client) for proxy in proxies):
                    near += 1
            if near >= 2 or (distance < .055 and sum(math.dist(tip, position) < .060 for tip in tips) >= 2):
                candidates.append((distance, index))
        return min(candidates)[1] if candidates else None

    def update(self, sample, fingers):
        now = self.clock()
        dt = min(max(now - self.last_time, 0.0), .10)
        self.last_time = now
        valid = sample is not None and fingers is not None and len(fingers) == 3
        if valid:
            try:
                tips = self.tip_positions(sample.joint_positions, fingers)
                proxies = self.phalanx_poses(sample.joint_positions, fingers)
                center = tuple(float(x) for x in np.mean(tips, axis=0))
                orientation = sample.pose.quaternion_xyzw
                valid = all(math.isfinite(v) for p in tips for v in p) and all(math.isfinite(v) for v in orientation)
            except (ValueError, TypeError, IndexError):
                valid = False
        if not valid:
            self._release()
            self.previous_angle = self.previous_center = self.previous_proxies = self.previous_anchor = None
            for tip in self.tips + sum(self.phalanges, []):
                pb.resetBasePositionAndOrientation(tip, [0, 0, -10], [0, 0, 0, 1], physicsClientId=self.client)
        else:
            # A discontinuous telemetric jump must not fling an attached object.
            if self.previous_center is not None and math.dist(center, self.previous_center) > .12:
                self._release()
                self.previous_proxies = self.previous_anchor = None
            self.previous_center = center
            for proxy, (position, rotation) in zip(sum(self.phalanges, []), proxies):
                pb.resetBasePositionAndOrientation(proxy, position, rotation, physicsClientId=self.client)
            for tip, position in zip(self.tips, tips):
                pb.resetBasePositionAndOrientation(tip, position, [0, 0, 0, 1], physicsClientId=self.client)
            pb.resetBasePositionAndOrientation(self.anchor, center, orientation, physicsClientId=self.client)
            angle = sum(fingers) / 3
            if angle < .55:
                self.reset_blocked.clear()
                self.grip_blocked = False
                self.release_reason = None
                if self.held is not None:
                    self._release()
            elif self.held is None and angle >= .65 and not self.grip_blocked:
                # Keep checking while closed, including after the user approaches an
                # object with the gripper already closed; opening always releases it.
                candidate = self._candidate(center, tips)
                if candidate is not None:
                    self._attach(candidate, center, orientation)
            self.previous_angle = angle
        self._accumulator += dt
        steps = int((self._accumulator + 1e-10) * 120)
        current_proxies = ([(p, (0, 0, 0, 1)) for p in tips] + proxies) if valid else None
        previous = self.previous_proxies if self.previous_proxies is not None else current_proxies
        current_anchor = (center, orientation) if valid else None
        previous_anchor = self.previous_anchor if self.previous_anchor is not None else current_anchor
        for step in range(steps):
            if current_proxies is not None:
                fraction = (step + 1) / steps
                anchor_position = tuple(a + fraction * (b - a) for a, b in zip(previous_anchor[0], current_anchor[0]))
                anchor_orientation = pb.getQuaternionSlerp(previous_anchor[1], current_anchor[1], fraction)
                pb.resetBasePositionAndOrientation(self.anchor, anchor_position, anchor_orientation, physicsClientId=self.client)
                for proxy, start, end in zip(self.tips + sum(self.phalanges, []), previous, current_proxies):
                    position = tuple(a + fraction * (b - a) for a, b in zip(start[0], end[0]))
                    rotation = pb.getQuaternionSlerp(start[1], end[1], fraction)
                    pb.resetBasePositionAndOrientation(proxy, position, rotation, physicsClientId=self.client)
            pb.stepSimulation(physicsClientId=self.client)
            if self.held is not None:
                body = self.bodies[self.held]
                contacts = pb.getContactPoints(bodyA=body, physicsClientId=self.client)
                load = sum(max(0.0, c[9]) for c in contacts)
                expected, _ = pb.multiplyTransforms(anchor_position, anchor_orientation, *self.hold_relative)
                actual, _ = pb.getBasePositionAndOrientation(body, physicsClientId=self.client)
                strained = load > 3.0 and math.dist(expected, actual) > .018
                self.contact_load_time = self.contact_load_time + 1 / 120 if strained else 0.0
                if self.contact_load_time >= .12:
                    self._release('impacto')
        self.previous_proxies = current_proxies
        self.previous_anchor = current_anchor
        self._accumulator = max(0.0, self._accumulator - steps / 120.0)
        for i, body in enumerate(self.bodies):
            position, _ = pb.getBasePositionAndOrientation(body, physicsClientId=self.client)
            speed, _ = pb.getBaseVelocity(body, physicsClientId=self.client)
            contacts = pb.getContactPoints(bodyA=body, bodyB=self.goal_supports[i], physicsClientId=self.client)
            _, rotation = pb.getBasePositionAndOrientation(body, physicsClientId=self.client)
            upright = pb.getMatrixFromQuaternion(rotation)[8] > .65
            on_goal = self.held != i and bool(contacts) and upright and math.dist(position[:2], GOALS[i][:2]) < .045 and GOAL_HEIGHTS[i] + .012 < position[2] < GOAL_HEIGHTS[i] + .12 and math.dist(speed, (0, 0, 0)) < .04
            self.goal_dwell[i] = self.goal_dwell[i] + dt if on_goal else 0.0
            self.scored[i] = self.goal_dwell[i] >= .6
        return self.snapshot()

    def snapshot(self):
        objects = []
        for i, body in enumerate(self.bodies):
            position, orientation = pb.getBasePositionAndOrientation(body, physicsClientId=self.client)
            contacts = pb.getContactPoints(bodyA=body, physicsClientId=self.client)
            supported = any(c[2] in self.goal_supports or c[2] == self.floor or c[2] in self.bodies for c in contacts)
            upright = pb.getMatrixFromQuaternion(orientation)[8] > .65
            state = ('entregado' if self.scored[i] else 'sujeto' if self.held == i else
                     'caido' if supported and (not upright or position[2] < -.2) else
                     'apoyado' if supported else 'en_aire')
            objects.append({'id': i, 'name': OBJECTS[i]['name'], 'position': list(position), 'quaternion_xyzw': list(orientation), 'delivered': self.scored[i], 'state': state})
        return {'objects': objects, 'held': self.held, 'delivered': sum(self.scored), 'total': len(self.bodies), 'release_reason': self.release_reason}
