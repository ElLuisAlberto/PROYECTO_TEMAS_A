#!/usr/bin/env python3
"""Connect the gateway to the direct bridge, with an explicit readiness heartbeat."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class DirectWebRelay(Node):
    def __init__(self):
        super().__init__('direct_web_relay')
        self.enabled = False
        self.enabled_at = 0.0
        self.joints_at = 0.0
        self.output = self.create_publisher(Twist, '/kinova_direct/cartesian_velocity', 10)
        self.ready_pub = self.create_publisher(Bool, '/teleop/bridge_ready', 10)
        self.create_subscription(Bool, '/kinova_direct/output_enabled', self.on_enabled, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Twist, '/teleop/cartesian_twist', self.on_command, 10)
        self.create_timer(0.05, self.publish_ready)

    def on_enabled(self, msg):
        self.enabled = msg.data
        self.enabled_at = time.monotonic()

    def on_joints(self, msg):
        joints = dict(zip(msg.name, msg.position))
        names = [f'j2n6s300_joint_{i}' for i in range(1, 7)]
        if all(name in joints and math.isfinite(joints[name]) for name in names):
            self.joints_at = time.monotonic()
        else:
            self.joints_at = 0.0

    def ready(self):
        now = time.monotonic()
        return (self.enabled and now - self.enabled_at < 0.3
                and now - self.joints_at < 0.3
                and self.output.get_subscription_count() == 1)

    def publish_ready(self):
        msg = Bool()
        msg.data = self.ready()
        self.ready_pub.publish(msg)

    def on_command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        if self.ready() and all(math.isfinite(value) for value in values):
            self.output.publish(msg)
        else:
            self.output.publish(Twist())


def main():
    rclpy.init()
    node = DirectWebRelay()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.output.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
