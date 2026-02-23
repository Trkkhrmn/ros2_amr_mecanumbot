// Copyright (c) 2025 Tarik Kahraman
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace mecanum_control
{

/**
 * @brief Mecanum wheel kinematics (C++). Wheel order (ROS): 0=front_left,
 * 1=front_right, 2=rear_left, 3=rear_right. Ref: Taheri et al. (2015).
 */
class MecanumKinematics
{
public:
  /** wheel_radius (m), lx/ly = half front-rear and left-right distances (m) */
  explicit MecanumKinematics(
    double wheel_radius = 0.075,
    double lx = 0.25,
    double ly = 0.35);

  /** Inverse: body (vx, vy, omega) -> wheel speeds (rad/s) */
  std::array<double, 4> inverse(double vx, double vy, double omega) const;

  /** Forward: wheel speeds -> body (vx, vy, omega) */
  struct BodyVelocity { double vx, vy, omega; };
  BodyVelocity forward(const std::array<double, 4> & wheel_speeds) const;

  /** Normalize wheel speeds to max_speed, keeping ratios. */
  std::array<double, 4> normalize(
    const std::array<double, 4> & speeds,
    double max_speed = 10.0) const;

  // Getters
  double wheelRadius() const {return r_;}
  double lx()         const {return lx_;}
  double ly()         const {return ly_;}

private:
  double r_;   ///< wheel radius
  double lx_;  ///< half front-rear distance
  double ly_;  ///< half left-right distance
};

}  // namespace mecanum_control
