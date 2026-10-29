#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/control/friction_compensation.hpp"

// Friction feed-forward (feature `friction-comp`):
//   tau_ff = coulomb .* tanh(qd_ref / eps) + viscous .* qd_ref, all zero when disabled or when
//   qd_ref has a non-finite element.
namespace cobot::control {
namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kTol = 1e-12;

// Two joints: coulomb (2.0, 0.5) N m, viscous (0.3, 0.1) N m s/rad, eps 0.01 rad/s.
FrictionCompensationConfig<2> two_joint_config() {
  FrictionCompensationConfig<2> c;
  c.enabled = true;
  c.coulomb_nm << 2.0, 0.5;
  c.viscous_nm_s_rad << 0.3, 0.1;
  c.velocity_eps_rad_s = 0.01;
  return c;
}

void expect_all_zero(const common::VecN<2>& tau) {
  for (int i = 0; i < 2; ++i) {
    EXPECT_EQ(tau(i), 0.0) << "joint " << i;
  }
}

TEST(FrictionCompensationConfig, DefaultsAreDisabledAndZero) {
  const FrictionCompensationConfig<7> c;
  EXPECT_FALSE(c.enabled);
  EXPECT_TRUE(c.coulomb_nm.isZero(0.0));
  EXPECT_TRUE(c.viscous_nm_s_rad.isZero(0.0));
  EXPECT_DOUBLE_EQ(c.velocity_eps_rad_s, 0.01);
}

TEST(FrictionCompensation, DisabledGivesZeroWhateverTheParameters) {
  FrictionCompensationConfig<2> c = two_joint_config();
  c.enabled = false;
  const FrictionCompensation<2> fc(c);
  expect_all_zero(fc.torque(common::VecN<2>(1.0, -2.0)));
  expect_all_zero(fc.torque(common::VecN<2>(0.01, 0.0)));
}

TEST(FrictionCompensation, DefaultConfigGivesZero) {
  const FrictionCompensation<7> fc(FrictionCompensationConfig<7>{});
  const common::VecN<7> tau = fc.torque(common::VecN<7>::Constant(0.5));
  EXPECT_TRUE(tau.isZero(0.0)) << tau.transpose();
}

TEST(FrictionCompensation, ZeroVelocityGivesZero) {
  const FrictionCompensation<2> fc(two_joint_config());
  expect_all_zero(fc.torque(common::VecN<2>::Zero()));
}

TEST(FrictionCompensation, VelocityEqualToEpsByHand) {
  const FrictionCompensation<2> fc(two_joint_config());
  const common::VecN<2> tau = fc.torque(common::VecN<2>(0.01, 0.01));
  // tanh(0.01 / 0.01) = tanh(1) = 0.7615941559557649
  // joint 0: 2.0 * 0.7615941559557649 + 0.3 * 0.01 = 1.5231883119115298 + 0.003
  //          = 1.5261883119115296
  // joint 1: 0.5 * 0.7615941559557649 + 0.1 * 0.01 = 0.38079707797788245 + 0.001
  //          = 0.3817970779778824
  EXPECT_NEAR(tau(0), 1.5261883119115296, kTol);
  EXPECT_NEAR(tau(1), 0.3817970779778824, kTol);
}

TEST(FrictionCompensation, SmoothingWidthComesFromConfig) {
  FrictionCompensationConfig<1> c;
  c.enabled = true;
  c.coulomb_nm << 1.0;
  c.viscous_nm_s_rad << 0.0;
  c.velocity_eps_rad_s = 0.1;
  const FrictionCompensation<1> fc(c);
  // 1.0 * tanh(0.05 / 0.1) = tanh(0.5) = 0.46211715726000974
  EXPECT_NEAR(fc.torque(common::VecN<1>(0.05))(0), 0.46211715726000974, kTol);
  // 1.0 * tanh(-0.2 / 0.1) = -tanh(2) = -0.9640275800758169
  EXPECT_NEAR(fc.torque(common::VecN<1>(-0.2))(0), -0.9640275800758169, kTol);
}

TEST(FrictionCompensation, LargeSpeedIsCoulombSignPlusViscous) {
  const FrictionCompensation<2> fc(two_joint_config());
  // |qd / eps| = 200 and 300: tanh rounds to exactly +-1 in double (above ~19.1).
  // joint 0: 2.0 * 1 + 0.3 * 2.0 = 2.6
  // joint 1: 0.5 * (-1) + 0.1 * (-3.0) = -0.5 - 0.3 = -0.8
  const common::VecN<2> tau = fc.torque(common::VecN<2>(2.0, -3.0));
  EXPECT_NEAR(tau(0), 2.6, kTol);
  EXPECT_NEAR(tau(1), -0.8, kTol);
}

TEST(FrictionCompensation, OddInVelocity) {
  const FrictionCompensation<2> fc(two_joint_config());
  const common::VecN<2> samples[] = {
      {0.001, -0.004}, {0.01, 0.02}, {-0.3, 0.05}, {1.5, -2.5}, {1e-6, 7.0}};
  for (const common::VecN<2>& qd : samples) {
    const common::VecN<2> plus = fc.torque(qd);
    const common::VecN<2> minus = fc.torque(-qd);
    for (int i = 0; i < 2; ++i) {
      EXPECT_NEAR(minus(i), -plus(i), kTol) << "qd = " << qd.transpose() << ", joint " << i;
      // Friction opposes motion, so its compensation has the sign of the velocity.
      EXPECT_GT(plus(i) * qd(i), 0.0) << "qd = " << qd.transpose() << ", joint " << i;
    }
  }
}

TEST(FrictionCompensation, BoundedByCoulombPlusViscous) {
  const FrictionCompensation<2> fc(two_joint_config());
  for (double v = -5.0; v <= 5.0; v += 0.0137) {
    const common::VecN<2> qd(v, -v);
    const common::VecN<2> tau = fc.torque(qd);
    // |c tanh(x) + b qd| <= c + b |qd|
    EXPECT_LE(std::abs(tau(0)), 2.0 + 0.3 * std::abs(v) + kTol) << v;
    EXPECT_LE(std::abs(tau(1)), 0.5 + 0.1 * std::abs(v) + kTol) << v;
  }
}

TEST(FrictionCompensation, JointsAreIndependent) {
  const FrictionCompensation<2> fc(two_joint_config());
  // Changing joint 1's velocity leaves joint 0's torque untouched.
  const double a = fc.torque(common::VecN<2>(0.02, 0.0))(0);
  const double b = fc.torque(common::VecN<2>(0.02, -4.0))(0);
  EXPECT_EQ(a, b);
}

TEST(FrictionCompensation, NonFiniteVelocityGivesAllZero) {
  const FrictionCompensation<2> fc(two_joint_config());
  expect_all_zero(fc.torque(common::VecN<2>(kNan, 0.5)));
  expect_all_zero(fc.torque(common::VecN<2>(0.5, kNan)));
  expect_all_zero(fc.torque(common::VecN<2>(kInf, 0.5)));
  expect_all_zero(fc.torque(common::VecN<2>(0.5, -kInf)));
}

TEST(FrictionCompensation, SevenJointsUsesEachJointsParameters) {
  FrictionCompensationConfig<7> c;
  c.enabled = true;
  c.coulomb_nm << 1.36, 1.36, 1.36, 1.36, 0.192, 0.192, 0.192;
  c.viscous_nm_s_rad << 0.4, 0.4, 0.4, 0.4, 0.08, 0.08, 0.08;
  const FrictionCompensation<7> fc(c);
  common::VecN<7> qd;
  qd << 1.0, -1.0, 0.0, 0.5, -0.5, 2.0, 0.01;
  const common::VecN<7> tau = fc.torque(qd);
  // |qd| >= 0.5 -> tanh(>= 50) = +-1 exactly.
  // j1: 1.36 + 0.4 * 1.0 = 1.76         j2: -1.36 - 0.4 = -1.76     j3: 0
  // j4: 1.36 + 0.4 * 0.5 = 1.56         j5: -0.192 - 0.08 * 0.5 = -0.232
  // j6: 0.192 + 0.08 * 2.0 = 0.352
  // j7: 0.192 * tanh(1) + 0.08 * 0.01 = 0.14622607794350687 + 0.0008 = 0.14702607794350686
  common::VecN<7> expected;
  expected << 1.76, -1.76, 0.0, 1.56, -0.232, 0.352, 0.14702607794350686;
  for (int i = 0; i < 7; ++i) {
    EXPECT_NEAR(tau(i), expected(i), kTol) << "joint " << i + 1;
  }
}

}  // namespace
}  // namespace cobot::control
