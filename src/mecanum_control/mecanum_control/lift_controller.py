#!/usr/bin/env python3
# Copyright (c) 2025 Tarik Kahraman
# SPDX-License-Identifier: MIT

"""
Lift service interface: /lift/up and /lift/down only.
No Gazebo animation here; provides API for task/integration layer.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class LiftController(Node):

    def __init__(self):
        super().__init__('lift_controller')

        self.declare_parameter('lift_height_up', 0.25)
        self.lift_height_up = self.get_parameter('lift_height_up').value

        self.srv_up = self.create_service(Trigger, 'lift/up', self.handle_lift_up)
        self.srv_down = self.create_service(Trigger, 'lift/down', self.handle_lift_down)

        self.get_logger().info('Lift Controller started (services only).')

    def handle_lift_up(self, request, response):
        self.get_logger().info(f'Lift UP -> target: {self.lift_height_up:.3f} m')
        response.success = True
        response.message = f'Lifting to {self.lift_height_up:.3f} m'
        return response

    def handle_lift_down(self, request, response):
        self.get_logger().info('Lift DOWN -> target: 0.000 m')
        response.success = True
        response.message = 'Lowering to ground'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LiftController()
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
