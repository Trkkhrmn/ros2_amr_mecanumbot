#!/usr/bin/env python3
# Copyright (c) 2025 Tarik Kahraman
# SPDX-License-Identifier: MIT

"""
Hardware bridge node: runs on Jetson Nano, forwards /cmd_vel and lift
service calls to the STM32 over UART. Protocol: ASCII line-based
(M,w_fl,w_fr,w_rl,w_rr and L,0 / L,1). See hardware/docs/PROTOCOL.md.
"""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Trigger

from kinematics import (
    format_motion_frame,
    inverse_kinematics,
    normalize,
)

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    serial = None  # type: ignore
    HAS_SERIAL = False


class HwBridgeNode(Node):
    """
    ROS 2 node that bridges /cmd_vel and lift services to the STM32 over UART.
    Implements a command timeout (watchdog): sends zero motion if no cmd_vel received.
    """

    def __init__(self) -> None:
        super().__init__("hw_bridge_node")

        self._declare_parameters()
        self._load_parameters()
        self._serial: Optional[serial.Serial] = None
        self._open_serial()
        self._last_cmd_time = 0.0
        self._last_twist = Twist()

        self._cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_cb, 10
        )
        self._lift_up_srv = self.create_service(
            Trigger, "lift/up", self._lift_up_cb
        )
        self._lift_down_srv = self.create_service(
            Trigger, "lift/down", self._lift_down_cb
        )
        motion_rate = self._motion_rate_hz
        self._timer = self.create_timer(1.0 / motion_rate, self._timer_cb)

        self.get_logger().info(
            "hw_bridge_node started (serial=%s, timeout=%.2fs, rate=%.0f Hz)"
            % (
                self._serial_port,
                self._cmd_timeout_sec,
                self._motion_rate_hz,
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("serial_port", "/dev/ttyTHS1")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("wheel_radius", 0.075)
        self.declare_parameter("lx", 0.25)
        self.declare_parameter("ly", 0.35)
        self.declare_parameter("max_wheel_speed", 15.0)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("motion_rate_hz", 25.0)

    def _load_parameters(self) -> None:
        self._serial_port = self.get_parameter("serial_port").value
        self._baud_rate = self.get_parameter("baud_rate").value
        self._wheel_radius = self.get_parameter("wheel_radius").value
        self._lx = self.get_parameter("lx").value
        self._ly = self.get_parameter("ly").value
        self._max_wheel_speed = self.get_parameter("max_wheel_speed").value
        self._cmd_timeout_sec = self.get_parameter("cmd_timeout_sec").value
        self._motion_rate_hz = self.get_parameter("motion_rate_hz").value

    def _open_serial(self) -> None:
        if not HAS_SERIAL:
            self.get_logger().warn(
                "pyserial not installed; install with: pip install pyserial"
            )
            return
        try:
            self._serial = serial.Serial(
                port=self._serial_port,
                baudrate=self._baud_rate,
                timeout=0.02,
                write_timeout=0.1,
            )
            self.get_logger().info(
                "Serial opened: %s @ %d" % (self._serial_port, self._baud_rate)
            )
        except Exception as e:
            self.get_logger().error(
                "Serial open failed: %s. Check port and permissions." % e
            )
            self._serial = None

    def _cmd_vel_cb(self, msg: Twist) -> None:
        self._last_twist = msg
        self._last_cmd_time = time.monotonic()

    def _timer_cb(self) -> None:
        if self._serial is None or not self._serial.is_open:
            return
        now = time.monotonic()
        if now - self._last_cmd_time > self._cmd_timeout_sec:
            self._send_motion(0.0, 0.0, 0.0)
            return
        t = self._last_twist
        self._send_motion(
            float(t.linear.x),
            float(t.linear.y),
            float(t.angular.z),
        )

    def _send_motion(self, vx: float, vy: float, omega: float) -> None:
        ws = inverse_kinematics(
            vx, vy, omega,
            self._wheel_radius, self._lx, self._ly,
        )
        ws = normalize(ws, self._max_wheel_speed)
        line = format_motion_frame(ws)
        self._write_serial(line)

    def _write_serial(self, s: str) -> None:
        if self._serial is None or not self._serial.is_open:
            return
        try:
            self._serial.write(s.encode("ascii"))
        except Exception as e:
            self.get_logger().error("Serial write failed: %s" % e)

    def _lift_up_cb(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self._write_serial("L,1\n")
        response.success = True
        response.message = "Lift up sent"
        return response

    def _lift_down_cb(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self._write_serial("L,0\n")
        response.success = True
        response.message = "Lift down sent"
        return response

    def destroy_node(self, *args, **kwargs) -> None:
        if self._serial is not None and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        super().destroy_node(*args, **kwargs)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = HwBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
