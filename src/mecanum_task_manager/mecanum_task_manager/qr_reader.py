#!/usr/bin/env python3
# Copyright (c) 2025 Tarik Kahraman
# SPDX-License-Identifier: MIT
"""
ROS 2 node that reads QR codes from the camera and publishes the payload.

Subscribed: /camera/image_raw (sensor_msgs/Image)
Published:  /qr/data (std_msgs/String), /qr/goal_pose (geometry_msgs/PoseStamped)

QR format: "x:<float>;y:<float>;yaw:<float>"  e.g. "x:3.5;y:1.2;yaw:1.57"
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import math


class QRReader(Node):

    def __init__(self):
        super().__init__('qr_reader')

        self.bridge    = CvBridge()
        self.detector  = cv2.QRCodeDetector()
        self.last_data = None   # avoid publishing the same QR repeatedly

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.qr_pub   = self.create_publisher(String,      '/qr/data',      10)
        self.goal_pub = self.create_publisher(PoseStamped, '/qr/goal_pose', 10)

        self.get_logger().info('QR Reader started, waiting for camera...')

    # ------------------------------------------------------------------ #

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        data, _, _ = self.detector.detectAndDecode(frame)

        if not data or data == self.last_data:
            return

        self.last_data = data
        self.get_logger().info(f'QR okundu: {data}')

        # Publish raw payload
        str_msg = String()
        str_msg.data = data
        self.qr_pub.publish(str_msg)

        # Try to parse as goal pose
        goal = self.parse_goal(data)
        if goal:
            self.goal_pub.publish(goal)

    # ------------------------------------------------------------------ #

    def parse_goal(self, data: str) -> PoseStamped | None:
        """
        Converts "x:3.5;y:1.2;yaw:1.57" format to PoseStamped.
        """
        try:
            parts  = {k: float(v) for k, v in
                      (pair.split(':') for pair in data.split(';'))}
            x   = parts['x']
            y   = parts['y']
            yaw = parts.get('yaw', 0.0)

            pose = PoseStamped()
            pose.header.stamp    = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # yaw -> quaternion (z and w only)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            self.get_logger().info(f'Goal parsed: x={x}, y={y}, yaw={yaw}')
            return pose

        except Exception as e:
            self.get_logger().warning(
                f'QR parse error ("{data}"): {e}. '
                'Expected format: "x:<float>;y:<float>;yaw:<float>"'
            )
            return None


def main(args=None):
    rclpy.init(args=args)
    node = QRReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

