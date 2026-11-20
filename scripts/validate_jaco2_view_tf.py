#!/usr/bin/env python3
"""Compare the reconstructed Jaco2 viewer links with ROS TF at matching times.

Run against an isolated simulation; this script observes only and sends no
motion commands. Fail when no matching samples or a transform differs.
"""
from __future__ import annotations

import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from teleop_cooperativo.desktop_qt import link_transforms
from teleop_cooperativo.jaco2_kinematics import JOINT_NAMES


def matrix_from_ros(transform):
    t = transform.translation
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    matrix = np.eye(4)
    matrix[:3, :3] = (
        (1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)),
        (2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)),
        (2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)),
    )
    matrix[:3, 3] = (t.x, t.y, t.z)
    return matrix


class Checker(Node):
    def __init__(self):
        super().__init__("jaco2_view_tf_check")
        self.buffer = Buffer(cache_time=Duration(seconds=5))
        self.listener = TransformListener(self.buffer, self)
        self.latest = None
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)

    def on_joint_state(self, msg):
        lookup = dict(zip(msg.name, msg.position))
        if all(name in lookup for name in JOINT_NAMES):
            self.latest = (Time.from_msg(msg.header.stamp), tuple(lookup[name] for name in JOINT_NAMES))

    def run(self, seconds):
        deadline = time.monotonic() + seconds
        max_position = max_rotation = 0.0
        observations = 0
        last_stamp = None
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest is None:
                continue
            stamp, joints = self.latest
            if stamp.nanoseconds == last_stamp:
                continue
            try:
                ros = [self.buffer.lookup_transform("world", name, stamp).transform for name in (
                    "j2n6s300_link_base", *(f"j2n6s300_link_{i}" for i in range(1, 7))
                )]
                expected = link_transforms(joints)
            except (TransformException, ValueError):
                continue
            for reference, actual in zip(ros, expected):
                matrix = matrix_from_ros(reference)
                max_position = max(max_position, float(np.linalg.norm(matrix[:3, 3] - actual[:3, 3])))
                delta = matrix[:3, :3].T @ actual[:3, :3]
                cosine = max(-1.0, min(1.0, (float(np.trace(delta)) - 1.0) / 2.0))
                max_rotation = max(max_rotation, float(np.arccos(cosine)))
            observations += 1
            last_stamp = stamp.nanoseconds
        result = {"samples": observations, "links_per_sample": 7,
                  "max_translation_error_m": max_position, "max_rotation_error_rad": max_rotation}
        if observations < 10 or max_position > 1e-4 or max_rotation > 1e-3:
            raise RuntimeError(f"TF/view comparison failed: {result}")
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=8.0)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    rclpy.init()
    node = Checker()
    try:
        print(json.dumps(node.run(args.seconds), indent=2))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
