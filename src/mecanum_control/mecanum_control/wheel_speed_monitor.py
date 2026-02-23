#!/usr/bin/env python3
# Copyright (c) 2025 Tarik Kahraman
# SPDX-License-Identifier: MIT

"""
Wheel speed monitor node.

Subscribes to wheel_speeds (Float64MultiArray) and prints FL, FR, RL, RR (rad/s).
Useful for debugging kinematics in test_kinematics.launch.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class WheelSpeedMonitor(Node):
    """Subscribe to wheel_speeds and print them."""

    def __init__(self):
        super().__init__('wheel_speed_monitor')
        self.sub = self.create_subscription(
            Float64MultiArray,
            'wheel_speeds',
            self.cb,
            10
        )

    def cb(self, msg):
        """Print wheel speeds [FL, FR, RL, RR]."""
        if len(msg.data) >= 4:
            self.get_logger().info(
                'Wheels [rad/s]: FL=%.2f FR=%.2f RL=%.2f RR=%.2f' % (
                    msg.data[0], msg.data[1], msg.data[2], msg.data[3]
                ),
                throttle_duration_sec=0.2
            )


def main(args=None):
    rclpy.init(args=args)
    node = WheelSpeedMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
