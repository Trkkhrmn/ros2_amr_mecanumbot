// Copyright (c) 2025 Tarik Kahraman
// SPDX-License-Identifier: MIT

#include "mecanum_control/mecanum_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mecanum_control
{

MecanumKinematics::MecanumKinematics(double wheel_radius, double lx, double ly)
: r_(wheel_radius), lx_(lx), ly_(ly)
{
  if (r_ <= 0.0) {throw std::invalid_argument("wheel_radius must be > 0");}
  if (lx_ <= 0.0) {throw std::invalid_argument("lx must be > 0");}
  if (ly_ <= 0.0) {throw std::invalid_argument("ly must be > 0");}
}

// Inverse kinematics (standard mecanum matrix):
//
//  |w_fl|   1/r * | 1  -1  -(lx+ly) | |vx |
//  |w_fr| =       | 1   1   (lx+ly) | |vy |
//  |w_rl|         | 1   1  -(lx+ly) | |ω  |
//  |w_rr|         | 1  -1   (lx+ly) |
//
// -------------------------------------------------------------------------- //
std::array<double, 4> MecanumKinematics::inverse(
  double vx, double vy, double omega) const
{
  const double k = lx_ + ly_;
  const double inv_r = 1.0 / r_;

  return {
    inv_r * (vx - vy - k * omega),  // front_left
    inv_r * (vx + vy + k * omega),  // front_right
    inv_r * (vx + vy - k * omega),  // rear_left
    inv_r * (vx - vy + k * omega)   // rear_right
  };
}

// Forward kinematics:
//  vx    = r/4 * ( w_fl + w_fr + w_rl + w_rr)
//  vy    = r/4 * (-w_fl + w_fr + w_rl - w_rr)
//  omega = r/4 * (-w_fl + w_fr - w_rl + w_rr) / (lx+ly)
//
// -------------------------------------------------------------------------- //
MecanumKinematics::BodyVelocity MecanumKinematics::forward(
  const std::array<double, 4> & ws) const
{
  const double r4 = r_ / 4.0;
  const double k = lx_ + ly_;

  return {
    r4 * ( ws[0] + ws[1] + ws[2] + ws[3]),           // vx
    r4 * (-ws[0] + ws[1] + ws[2] - ws[3]),           // vy
    r4 * (-ws[0] + ws[1] - ws[2] + ws[3]) / k        // omega
  };
}

// -------------------------------------------------------------------------- //
// Normalizasyon
// -------------------------------------------------------------------------- //
std::array<double, 4> MecanumKinematics::normalize(
  const std::array<double, 4> & speeds, double max_speed) const
{
  double max_abs = 0.0;
  for (const auto & s : speeds) {
    max_abs = std::max(max_abs, std::abs(s));
  }

  if (max_abs <= max_speed || max_abs < 1e-9) {
    return speeds;
  }

  const double factor = max_speed / max_abs;
  std::array<double, 4> normalized;
  for (size_t i = 0; i < 4; ++i) {
    normalized[i] = speeds[i] * factor;
  }
  return normalized;
}

}  // namespace mecanum_control
