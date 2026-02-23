#!/usr/bin/env python3
# Copyright (c) 2025 Tarik Kahraman
# SPDX-License-Identifier: MIT
"""
Simple Patrol — waypoint patrol using only odometry (no Nav2, no map).
Subscribes to /odom, publishes /cmd_vel. When close enough to a waypoint, moves to the next; loops.
Services: /patrol/start, /patrol/stop (std_srvs/Trigger)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import yaml
import os
import math
from enum import Enum, auto


class State(Enum):
    IDLE = auto()
    PATROLLING = auto()
    STOPPED = auto()


class SimplePatrol(Node):
    def __init__(self):
        super().__init__('simple_patrol')

        self.declare_parameter('waypoints_yaml', os.path.expanduser('~/patrol_waypoints.yaml'))
        self.declare_parameter('waypoint_tolerance', 0.4)
        self.declare_parameter('pause_at_waypoint', 0.9)
        self.declare_parameter('depart_turn_only_duration', 0.5)
        self.declare_parameter('approach_slow_dist', 1.2)
        self.declare_parameter('max_linear', 0.4)
        self.declare_parameter('max_angular', 0.6)
        self.declare_parameter('k_linear', 0.8)
        self.declare_parameter('k_angular', 1.2)

        self.waypoints_yaml = self.get_parameter('waypoints_yaml').value
        self.tolerance = self.get_parameter('waypoint_tolerance').value
        self.pause_at_waypoint = self.get_parameter('pause_at_waypoint').value
        self.depart_turn_only = self.get_parameter('depart_turn_only_duration').value
        self.approach_slow_dist = self.get_parameter('approach_slow_dist').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.k_linear = self.get_parameter('k_linear').value
        self.k_angular = self.get_parameter('k_angular').value

        self.state = State.IDLE
        self.waypoints = []
        self.current_idx = 0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.waypoint_reached_at = None
        self.depart_waypoint_at = None

        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_service(Trigger, 'patrol/start', self._start_cb)
        self.create_service(Trigger, 'patrol/stop', self._stop_cb)
        self.create_timer(0.05, self._control_loop)  # 20 Hz

        self.get_logger().info('Simple Patrol ready (odom + cmd_vel only, no Nav2).')

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def _load_waypoints(self):
        if not os.path.exists(self.waypoints_yaml):
            self.get_logger().error(f'Waypoint file not found: {self.waypoints_yaml}')
            return []
        with open(self.waypoints_yaml, 'r') as f:
            data = yaml.safe_load(f)
        return data.get('patrol_waypoints', [])

    def _start_cb(self, req, res):
        if self.state not in (State.IDLE, State.STOPPED):
            res.success = False
            res.message = f'Already running: {self.state.name}'
            return res
        self.waypoints = self._load_waypoints()
        if not self.waypoints:
            res.success = False
            res.message = f'No waypoints in: {self.waypoints_yaml}'
            return res
        self.current_idx = 0
        self.waypoint_reached_at = None
        self.depart_waypoint_at = None
        self.state = State.PATROLLING
        res.success = True
        res.message = f'Patrol started, {len(self.waypoints)} waypoints.'
        self.get_logger().info(res.message)
        return res

    def _stop_cb(self, req, res):
        self.state = State.STOPPED
        self._pub_vel(0.0, 0.0, 0.0)
        res.success = True
        res.message = 'Patrol stopped.'
        return res

    def _pub_vel(self, vx, vy, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)

    def _control_loop(self):
        if self.state != State.PATROLLING or not self.waypoints:
            if self.state == State.STOPPED:
                self._pub_vel(0.0, 0.0, 0.0)
            return

        wp = self.waypoints[self.current_idx]
        wx, wy = wp['x'], wp['y']
        dx = wx - self.x
        dy = wy - self.y
        dist = math.hypot(dx, dy)
        now = self.get_clock().now().nanoseconds / 1e9

        # Reset reached time when outside tolerance
        if dist >= self.tolerance:
            self.waypoint_reached_at = None

        # Reached waypoint: stop, wait a bit, then go to next
        if dist < self.tolerance:
            self._pub_vel(0.0, 0.0, 0.0)
            if self.waypoint_reached_at is None:
                self.waypoint_reached_at = now
                return
            if (now - self.waypoint_reached_at) < self.pause_at_waypoint:
                return
            self.current_idx = (self.current_idx + 1) % len(self.waypoints)
            self.waypoint_reached_at = None
            self.depart_waypoint_at = now
            self.get_logger().info(
                f'Waypoint [{self.current_idx}/{len(self.waypoints)}] target: '
                f'({self.waypoints[self.current_idx]["x"]:.2f}, {self.waypoints[self.current_idx]["y"]:.2f})'
            )
            return

        # Waypoint’ten yeni ayrıldıysak turn only briefly to avoid jerk
        if self.depart_waypoint_at is not None:
            if (now - self.depart_waypoint_at) < self.depart_turn_only:
                target_yaw = math.atan2(dy, dx)
                yaw_err = math.atan2(math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))
                wz = self.k_angular * 0.7 * yaw_err
                wz = max(-self.max_angular * 0.8, min(self.max_angular * 0.8, wz))
                self._pub_vel(0.0, 0.0, wz)
                return
            self.depart_waypoint_at = None

        # Heading and distance to target
        target_yaw = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))
        yaw_tolerance = 0.12

        # Slow down when approaching
        if dist < self.approach_slow_dist:
            speed_scale = dist / self.approach_slow_dist
        else:
            speed_scale = 1.0

        if abs(yaw_err) > yaw_tolerance:
            wz = self.k_angular * yaw_err
            wz = max(-self.max_angular, min(self.max_angular, wz))
            self._pub_vel(0.0, 0.0, wz)
        else:
            vx = self.k_linear * min(dist, 1.0) * speed_scale
            vx = max(0.0, min(self.max_linear, vx))
            wz = self.k_angular * 0.5 * yaw_err
            wz = max(-self.max_angular, min(self.max_angular, wz))
            self._pub_vel(vx, 0.0, wz)


def main(args=None):
    rclpy.init(args=args)
    node = SimplePatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._pub_vel(0.0, 0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
