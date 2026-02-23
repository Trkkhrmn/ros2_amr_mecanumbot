#!/usr/bin/env python3
# Copyright (c) 2025 Tarik Kahraman
# SPDX-License-Identifier: MIT

"""
Mecanum wheel kinematics: forward (wheel speeds -> body velocity) and
inverse (body velocity -> wheel speeds). Wheel order (ROS): 0=front_left,
1=front_right, 2=rear_left, 3=rear_right.
"""


class MecanumKinematics:
    """
    wheel_radius (m), lx = half front-rear axle distance (m), ly = half left-right (m).
    """

    def __init__(self, wheel_radius: float = 0.075,
                 lx: float = 0.25, ly: float = 0.35):
        self.r = wheel_radius
        self.lx = lx
        self.ly = ly

    # Inverse kinematics: body velocity -> wheel angular speeds (rad/s)
    def inverse(self, vx: float, vy: float, omega: float) -> list:
        """Returns [w_fl, w_fr, w_rl, w_rr] in rad/s."""
        k = self.lx + self.ly
        r = self.r
        w_fl = (1 / r) * (vx - vy - k * omega)
        w_fr = (1 / r) * (vx + vy + k * omega)
        w_rl = (1 / r) * (vx + vy - k * omega)
        w_rr = (1 / r) * (vx - vy + k * omega)
        return [w_fl, w_fr, w_rl, w_rr]

    # ------------------------------------------------------------------ #
    # İleri Kinematik: Tekerlek açısal hızları → Gövde hızı
    # ------------------------------------------------------------------ #
    def forward(self, w_fl: float, w_fr: float,
                w_rl: float, w_rr: float) -> tuple:
        """
        Gövde hızını hesapla.

        Returns
        -------
        (vx, vy, omega) – gövde doğrusal ve açısal hızları

        """
        r = self.r / 4.0
        k = self.lx + self.ly
        vx = r * (w_fl + w_fr + w_rl + w_rr)
        vy = r * (-w_fl + w_fr + w_rl - w_rr)
        omega = r * (-w_fl + w_fr - w_rl + w_rr) / k
        return (vx, vy, omega)

    # Normalize wheel speeds to max_speed
    def normalize(self, wheel_speeds: list,
                  max_speed: float = 10.0) -> list:
        max_w = max(abs(w) for w in wheel_speeds)
        if max_w > max_speed:
            factor = max_speed / max_w
            return [w * factor for w in wheel_speeds]
        return wheel_speeds
