#!/usr/bin/env python3
# Copyright (c) 2025 Tarik Kahraman
# SPDX-License-Identifier: MIT
"""
Patrol Manager — sends waypoints to Nav2 in order. When a QR is detected, interrupts
patrol to run an unload mission, then resumes from the nearest waypoint.
Subscribes: /qr/goal_pose. Publishes: /robot_state. Action: navigate_to_pose.
Services: /patrol/start, /patrol/stop.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Trigger

import yaml
import math
import os
from enum import Enum, auto


class PatrolState(Enum):
    IDLE        = auto()
    MAPPING     = auto()
    PATROLLING  = auto()
    ON_MISSION  = auto()   # QR detected → run UNLOADING mission
    RETURNING   = auto()   # After mission → go back to nearest waypoint
    STOPPED     = auto()


class PatrolManager(Node):

    def __init__(self):
        super().__init__('patrol_manager')

        self.declare_parameter('waypoints_yaml',
            os.path.expanduser('~/patrol_waypoints.yaml'))
        self.declare_parameter('waypoint_tolerance', 0.3)   # metre

        self.waypoints_yaml  = self.get_parameter('waypoints_yaml').value
        self.wp_tolerance    = self.get_parameter('waypoint_tolerance').value

        self.cb_group = ReentrantCallbackGroup()

        # State
        self.state           = PatrolState.IDLE
        self.waypoints       = []
        self.current_wp_idx  = 0
        self.visited_qrs     = set()          # Avoid processing same QR again
        self.active_goal_handle = None

        # Nav2 action client
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.cb_group
        )

        # QR subscriber
        self.qr_sub = self.create_subscription(
            PoseStamped, '/qr/goal_pose',
            self.qr_callback, 10,
            callback_group=self.cb_group
        )

        # State publisher
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.create_timer(0.5, self.publish_state)

        # Services
        self.start_srv = self.create_service(
            Trigger, 'patrol/start', self.handle_start,
            callback_group=self.cb_group
        )
        self.stop_srv = self.create_service(
            Trigger, 'patrol/stop', self.handle_stop,
            callback_group=self.cb_group
        )

        self.get_logger().info('Patrol Manager ready.')

    # ------------------------------------------------------------------ #
    # Service handlers
    # ------------------------------------------------------------------ #

    def handle_start(self, request, response):
        if self.state not in (PatrolState.IDLE, PatrolState.STOPPED):
            response.success = False
            response.message = f'Cannot start patrol, current state: {self.state.name}'
            return response

        self.waypoints = self.load_waypoints()
        if not self.waypoints:
            response.success = False
            response.message = f'No waypoints found: {self.waypoints_yaml}'
            return response

        self.current_wp_idx = 0
        self.state = PatrolState.PATROLLING
        self.get_logger().info(
            f'Patrol started. {len(self.waypoints)} waypoints loaded.'
        )
        # Start async patrol loop
        self.create_timer(0.1, self.patrol_loop_once)

        response.success = True
        response.message = f'Patrol started: {len(self.waypoints)} waypoints'
        return response

    def handle_stop(self, request, response):
        self.state = PatrolState.STOPPED
        self.cancel_current_goal()
        response.success = True
        response.message = 'Patrol stopped.'
        return response

    # ------------------------------------------------------------------ #
    # QR callback — interrupt patrol for mission
    # ------------------------------------------------------------------ #

    def qr_callback(self, msg: PoseStamped):
        # Use pose as key so we don't process same QR again
        qr_key = f"{msg.pose.position.x:.1f}_{msg.pose.position.y:.1f}"

        if qr_key in self.visited_qrs:
            self.get_logger().debug(f'QR already visited: {qr_key}')
            return

        if self.state != PatrolState.PATROLLING:
            self.get_logger().warning(
                f'QR received but state is {self.state.name}, mission postponed.'
            )
            return

        self.get_logger().info(
            f'QR detected: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) — '
            f'Interrupting patrol!'
        )

        # Cancel current nav goal
        self.cancel_current_goal()
        self.state = PatrolState.ON_MISSION

        # Run mission in a thread
        import threading
        t = threading.Thread(
            target=self.execute_mission,
            args=(msg, qr_key),
            daemon=True
        )
        t.start()

    # ------------------------------------------------------------------ #
    # Mission flow
    # ------------------------------------------------------------------ #

    def execute_mission(self, unload_pose: PoseStamped, qr_key: str):
        """
        1. Go to UNLOADING pose
        2. Lower lift (lift down service)
        3. Return to nearest patrol waypoint
        4. Resume patrol
        """
        self.get_logger().info('Mission: navigating to UNLOADING pose...')
        success = self.navigate_to_sync(unload_pose)

        if success:
            self.visited_qrs.add(qr_key)
            self.get_logger().info(f'Mission done: {qr_key}')
        else:
            self.get_logger().error('UNLOADING navigation failed.')

        # Return to nearest waypoint
        self.state = PatrolState.RETURNING
        nearest_wp = self.find_nearest_waypoint(unload_pose)
        self.current_wp_idx = nearest_wp
        self.get_logger().info(
            f'Returning to nearest waypoint: idx={nearest_wp}'
        )
        home_pose = self.waypoint_to_pose(self.waypoints[nearest_wp])
        self.navigate_to_sync(home_pose)

        # Resume patrol
        self.state = PatrolState.PATROLLING
        self.get_logger().info('Resuming patrol...')

    # ------------------------------------------------------------------ #
    # Patrol loop
    # ------------------------------------------------------------------ #

    def patrol_loop_once(self):
        """Timer callback — if patrolling, go to next waypoint."""
        if self.state != PatrolState.PATROLLING:
            return

        if not self.waypoints:
            return

        wp = self.waypoints[self.current_wp_idx]
        pose = self.waypoint_to_pose(wp)

        self.get_logger().info(
            f'Waypoint [{self.current_wp_idx}/{len(self.waypoints)-1}]: '
            f'({wp["x"]:.2f}, {wp["y"]:.2f})'
        )

        success = self.navigate_to_sync(pose)

        if self.state == PatrolState.PATROLLING:
            # Loop: after last waypoint go back to first
            self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)

    # ------------------------------------------------------------------ #
    # Nav2 helpers
    # ------------------------------------------------------------------ #

    def navigate_to_sync(self, pose: PoseStamped) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available.')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = pose

        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return False

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)

        self.active_goal_handle = None
        return True

    def cancel_current_goal(self):
        if self.active_goal_handle:
            try:
                self.active_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.active_goal_handle = None

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def load_waypoints(self) -> list[dict]:
        if not os.path.exists(self.waypoints_yaml):
            self.get_logger().error(f'File not found: {self.waypoints_yaml}')
            return []
        with open(self.waypoints_yaml, 'r') as f:
            data = yaml.safe_load(f)
        return data.get('patrol_waypoints', [])

    def waypoint_to_pose(self, wp: dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']
        pose.pose.orientation.w = math.cos(wp.get('yaw', 0.0) / 2.0)
        pose.pose.orientation.z = math.sin(wp.get('yaw', 0.0) / 2.0)
        return pose

    def find_nearest_waypoint(self, pose: PoseStamped) -> int:
        px = pose.pose.position.x
        py = pose.pose.position.y
        min_dist = float('inf')
        nearest  = 0
        for i, wp in enumerate(self.waypoints):
            d = math.hypot(wp['x'] - px, wp['y'] - py)
            if d < min_dist:
                min_dist = d
                nearest  = i
        return nearest

    def publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolManager()
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

