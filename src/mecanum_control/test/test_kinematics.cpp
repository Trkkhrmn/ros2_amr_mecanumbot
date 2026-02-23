// Copyright (c) 2025 Tarik Kahraman
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <cmath>
#include <array>

#include "mecanum_control/mecanum_kinematics.hpp"

using mecanum_control::MecanumKinematics;

// ------------------------------------------------------------------ //
// Test Fixture
// ------------------------------------------------------------------ //
class MecanumKinematicsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Parameters close to real robot
    kin = std::make_unique<MecanumKinematics>(
      0.075,  // wheel_radius
      0.25,   // lx
      0.35    // ly
    );
  }

  std::unique_ptr<MecanumKinematics> kin;

  // Helper: are two doubles close?
  static void expectNear(
    double a, double b, double tol = 1e-6,
    const std::string & label = "")
  {
    EXPECT_NEAR(a, b, tol) << (label.empty() ? "" : "[" + label + "]");
  }
};

// ------------------------------------------------------------------ //
// 1. Constructor tests
// ------------------------------------------------------------------ //

TEST_F(MecanumKinematicsTest, ConstructorStoresParameters)
{
  EXPECT_DOUBLE_EQ(kin->wheelRadius(), 0.075);
  EXPECT_DOUBLE_EQ(kin->lx(), 0.25);
  EXPECT_DOUBLE_EQ(kin->ly(), 0.35);
}

TEST(MecanumKinematicsConstructorTest, ThrowsOnInvalidRadius)
{
  EXPECT_THROW(MecanumKinematics(0.0, 0.25, 0.35), std::invalid_argument);
  EXPECT_THROW(MecanumKinematics(-1.0, 0.25, 0.35), std::invalid_argument);
}

TEST(MecanumKinematicsConstructorTest, ThrowsOnInvalidLx)
{
  EXPECT_THROW(MecanumKinematics(0.075, 0.0, 0.35), std::invalid_argument);
}

TEST(MecanumKinematicsConstructorTest, ThrowsOnInvalidLy)
{
  EXPECT_THROW(MecanumKinematics(0.075, 0.25, 0.0), std::invalid_argument);
}

// ------------------------------------------------------------------ //
// 2. Inverse kinematics — basic motion tests
// ------------------------------------------------------------------ //

TEST_F(MecanumKinematicsTest, InverseZeroInputGivesZeroOutput)
{
  auto ws = kin->inverse(0.0, 0.0, 0.0);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(ws[i], 0.0) << "wheel " << i;
  }
}

TEST_F(MecanumKinematicsTest, InverseForwardMotion)
{
  // Forward: vx=1, vy=0, omega=0 — all wheels should turn same way, positive
  auto ws = kin->inverse(1.0, 0.0, 0.0);

  EXPECT_GT(ws[0], 0.0) << "FL positive";
  EXPECT_GT(ws[1], 0.0) << "FR positive";
  EXPECT_GT(ws[2], 0.0) << "RL positive";
  EXPECT_GT(ws[3], 0.0) << "RR positive";

  // All wheels same speed for forward
  expectNear(ws[0], ws[1], 1e-9, "FL==FR");
  expectNear(ws[0], ws[2], 1e-9, "FL==RL");
  expectNear(ws[0], ws[3], 1e-9, "FL==RR");
}

TEST_F(MecanumKinematicsTest, InverseBackwardMotion)
{
  // Düz geri: vx=-1
  auto ws_fwd = kin->inverse(1.0, 0.0, 0.0);
  auto ws_bwd = kin->inverse(-1.0, 0.0, 0.0);

  for (size_t i = 0; i < 4; ++i) {
    expectNear(
      ws_fwd[i], -ws_bwd[i], 1e-9,
      "Backward should be opposite of forward");
  }
}

TEST_F(MecanumKinematicsTest, InverseLateralMotionRight)
{
  // Sağa yanal hareket: vy=-1 (ROS convention: sola pozitif)
  // Mecanum: diagonal wheels opposite direction
  auto ws = kin->inverse(0.0, -1.0, 0.0);

  // FL ve RR pozitif, FR ve RL negatif (ya da tam tersi — doğrulama)
  // Sağa gidişte: FL(+), FR(-), RL(-), RR(+)
  EXPECT_GT(ws[0], 0.0) << "FL pozitif (sağa yanal)";
  EXPECT_LT(ws[1], 0.0) << "FR negatif (sağa yanal)";
  EXPECT_LT(ws[2], 0.0) << "RL negatif (sağa yanal)";
  EXPECT_GT(ws[3], 0.0) << "RR pozitif (sağa yanal)";
}

TEST_F(MecanumKinematicsTest, InverseLateralMotionLeft)
{
  // Sola yanal: vy=+1
  auto ws_r = kin->inverse(0.0, -1.0, 0.0);
  auto ws_l = kin->inverse(0.0, 1.0, 0.0);

  for (size_t i = 0; i < 4; ++i) {
    expectNear(ws_r[i], -ws_l[i], 1e-9, "Left/right lateral symmetry");
  }
}

TEST_F(MecanumKinematicsTest, InverseRotationInPlace)
{
  // In-place rotate: omega > 0 -> turn left. Left wheels back, right forward.
  auto ws = kin->inverse(0.0, 0.0, 1.0);

  EXPECT_LT(ws[0], 0.0) << "FL negative (turn left)";
  EXPECT_GT(ws[1], 0.0) << "FR positive (turn left)";
  EXPECT_LT(ws[2], 0.0) << "RL negative (turn left)";
  EXPECT_GT(ws[3], 0.0) << "RR positive (turn left)";
}

// ------------------------------------------------------------------ //
// 3. Forward kinematics tests
// ------------------------------------------------------------------ //

TEST_F(MecanumKinematicsTest, ForwardZeroGivesZero)
{
  auto body = kin->forward({0.0, 0.0, 0.0, 0.0});
  expectNear(body.vx, 0.0, 1e-9, "vx");
  expectNear(body.vy, 0.0, 1e-9, "vy");
  expectNear(body.omega, 0.0, 1e-9, "omega");
}

TEST_F(MecanumKinematicsTest, ForwardInverseRoundTrip)
{
  // Inverse then forward should give same body velocity
  const double vx_in = 0.5, vy_in = 0.3, omega_in = 0.2;

  auto ws = kin->inverse(vx_in, vy_in, omega_in);
  auto body = kin->forward(ws);

  expectNear(body.vx, vx_in, 1e-6, "vx round-trip");
  expectNear(body.vy, vy_in, 1e-6, "vy round-trip");
  expectNear(body.omega, omega_in, 1e-6, "omega round-trip");
}

TEST_F(MecanumKinematicsTest, ForwardOnlyForwardMotion)
{
  // All wheels same speed -> forward only
  const double w = 5.0;
  auto body = kin->forward({w, w, w, w});

  EXPECT_GT(body.vx, 0.0);
  expectNear(body.vy, 0.0, 1e-9, "vy zero");
  expectNear(body.omega, 0.0, 1e-9, "omega zero");
}

// ------------------------------------------------------------------ //
// 4. Normalizasyon Testleri
// ------------------------------------------------------------------ //

TEST_F(MecanumKinematicsTest, NormalizeNoChangeIfBelowMax)
{
  std::array<double, 4> speeds = {1.0, 2.0, 3.0, 4.0};
  auto normalized = kin->normalize(speeds, 10.0);

  for (size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(normalized[i], speeds[i]);
  }
}

TEST_F(MecanumKinematicsTest, NormalizeScalesDownCorrectly)
{
  std::array<double, 4> speeds = {5.0, 10.0, 15.0, 20.0};
  auto normalized = kin->normalize(speeds, 10.0);

  // Maksimum 10.0 olmalı
  for (const auto & s : normalized) {
    EXPECT_LE(std::abs(s), 10.0 + 1e-9);
  }

  // Ratios preserved
  EXPECT_NEAR(normalized[0] / normalized[3], 5.0 / 20.0, 1e-9);
}

TEST_F(MecanumKinematicsTest, NormalizeZeroInput)
{
  auto normalized = kin->normalize({0.0, 0.0, 0.0, 0.0}, 10.0);
  for (const auto & s : normalized) {
    EXPECT_DOUBLE_EQ(s, 0.0);
  }
}

// ------------------------------------------------------------------ //
// 5. Numerical accuracy
// ------------------------------------------------------------------ //

TEST_F(MecanumKinematicsTest, InverseNumericalAccuracy)
{
  // For vx=1, vy=0, omega=0 expected wheel speed: 1/r = 1/0.075 ≈ 13.333
  double expected = 1.0 / 0.075;
  auto ws = kin->inverse(1.0, 0.0, 0.0);

  for (size_t i = 0; i < 4; ++i) {
    expectNear(ws[i], expected, 1e-6, "Numerical accuracy");
  }
}

TEST_F(MecanumKinematicsTest, InverseLateralNumericalAccuracy)
{
  // vy=1, others 0 -> FL=-1/r, FR=+1/r, RL=+1/r, RR=-1/r
  double inv_r = 1.0 / 0.075;
  auto ws = kin->inverse(0.0, 1.0, 0.0);

  expectNear(ws[0], -inv_r, 1e-6, "FL yanal");
  expectNear(ws[1], inv_r, 1e-6, "FR yanal");
  expectNear(ws[2], inv_r, 1e-6, "RL yanal");
  expectNear(ws[3], -inv_r, 1e-6, "RR yanal");
}

// ------------------------------------------------------------------ //
// main
// ------------------------------------------------------------------ //

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
