#include <cmath>
#include <limits>
#include <random>

#include <gtest/gtest.h>

#include "cobot/control/pd_gravity.hpp"
#include "cobot/dynamics/dynamics.hpp"

#include "registry_util.hpp"

// PD plus gravity compensation (brief controller-registry 2, owner question C6):
//   tau = J .* (kp .* (q_ref - q) + kd .* (qd_ref - qd)) + G(q),  J = diag(M(q_inertia)),
// clamped to tau_max; a non-finite input gives the all-zero command. Model: config/default.yaml,
// controller model (tau_max = peak torque P), J taken at home_q.
namespace cobot::control {
namespace {

using registry_test::clamped;
using registry_test::kDof;
using registry_test::load_arm_setup;
using registry_test::random_q;
using registry_test::random_vec;
using registry_test::Ref7;
using registry_test::State7;
using registry_test::tau_max_of;
using registry_test::Vec7;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

// kp_j = 100 + 10 j (1/s^2), kd_j = 20 + j (1/s): distinct per joint, so an index slip shows.
PdGravityGains<kDof> distinct_gains() {
  PdGravityGains<kDof> g;
  for (int j = 0; j < kDof; ++j) {
    g.kp(j) = 100.0 + 10.0 * j;
    g.kd(j) = 20.0 + j;
  }
  return g;
}

TEST(PdGravity, InertiaIsDiagonalOfMAtHome) {
  const auto arm = load_arm_setup();
  ASSERT_TRUE(arm);
  const Vec7& home = arm->config.home_q_rad;
  const PdGravity<kDof> pd(arm->nominal, PdGravityGains<kDof>{}, home);
  const Vec7 want = dynamics::mass_matrix(arm->nominal, home).diagonal();
  for (int j = 0; j < kDof; ++j) {
    EXPECT_EQ(pd.inertia()(j), want(j)) << "joint " << j;
    EXPECT_GT(pd.inertia()(j), 0.0) << "joint " << j;
  }
}

TEST(PdGravity, ZeroErrorGivesGravity) {
  const auto arm = load_arm_setup();
  ASSERT_TRUE(arm);
  const PdGravity<kDof> pd(arm->nominal, distinct_gains(), arm->config.home_q_rad);
  std::mt19937_64 rng(31);
  for (int k = 0; k < 50; ++k) {
    State7 s;
    Ref7 ref;
    s.q = random_q(arm->nominal, rng);
    s.qd = random_vec(1.0, rng);
    ref.q = s.q;
    ref.qd = s.qd;
    ref.qdd = random_vec(2.0, rng);  // ignored by the law
    const Vec7 tau = pd.compute(s, ref).tau;
    // Zero errors: J .* (kp .* 0 + kd .* 0) = 0 exactly, so tau = clamp(G(q)) bit for bit.
    const Vec7 want = clamped(arm->nominal, dynamics::gravity_torque(arm->nominal, s.q));
    for (int j = 0; j < kDof; ++j) {
      EXPECT_EQ(tau(j), want(j)) << "sample " << k << " joint " << j;
    }
  }
}

TEST(PdGravity, PositionAndVelocityTerms) {
  const auto arm = load_arm_setup();
  ASSERT_TRUE(arm);
  const Vec7& home = arm->config.home_q_rad;
  const PdGravityGains<kDof> g = distinct_gains();
  const PdGravity<kDof> pd(arm->nominal, g, home);
  const Vec7 J = dynamics::mass_matrix(arm->nominal, home).diagonal();
  const Vec7 G = dynamics::gravity_torque(arm->nominal, home);
  const Vec7 lim = tau_max_of(arm->nominal);

  // Position term: joint 2 (index 1) reference 0.01 rad ahead, qd = 0, everything else at home.
  // tau(1) = G(1) + J(1) * kp(1) * 0.01 with kp(1) = 100 + 10 = 110; qd_ref = qd = 0.
  State7 s;
  s.q = home;
  Ref7 ref;
  ref.q = home;
  ref.q(1) += 0.01;
  const double dp = J(1) * g.kp(1) * 0.01;
  ASSERT_LT(std::abs(G(1) + dp), lim(1)) << "fixture must not saturate joint 2";
  const Vec7 tau_p = pd.compute(s, ref).tau;
  EXPECT_NEAR(tau_p(1) - G(1), dp, 1e-9 * std::abs(dp));
  for (int j = 0; j < kDof; ++j) {
    if (j != 1) {
      ASSERT_LT(std::abs(G(j)), lim(j));
      EXPECT_EQ(tau_p(j), G(j)) << "joint " << j;
    }
  }

  // Velocity term: joint 7 (index 6) qd_ref = 0.1 rad/s, q = q_ref = home, qd = 0.
  // tau(6) = G(6) + J(6) * kd(6) * 0.1 with kd(6) = 20 + 6 = 26.
  Ref7 ref_v;
  ref_v.q = home;
  ref_v.qd(6) = 0.1;
  const double dv = J(6) * g.kd(6) * 0.1;
  ASSERT_LT(std::abs(G(6) + dv), lim(6)) << "fixture must not saturate joint 7";
  const Vec7 tau_v = pd.compute(s, ref_v).tau;
  EXPECT_NEAR(tau_v(6) - G(6), dv, 1e-9 * std::abs(dv));
  for (int j = 0; j < kDof - 1; ++j) {
    EXPECT_EQ(tau_v(j), G(j)) << "joint " << j;
  }
}

TEST(PdGravity, ClampAndNonFinite) {
  const auto arm = load_arm_setup();
  ASSERT_TRUE(arm);
  const Vec7& home = arm->config.home_q_rad;
  PdGravityGains<kDof> huge;
  huge.kp.setConstant(1e9);
  const PdGravity<kDof> pd(arm->nominal, huge, home);
  const Vec7 lim = tau_max_of(arm->nominal);
  State7 s;
  s.q = home;
  // +-0.1 rad on every joint: |J_j * 1e9 * 0.1| >= 1e8 J_j, far above every tau_max (<= 204 N m)
  // for any J_j > 2.04e-6 kg m^2, so each joint saturates with the sign of the error.
  for (const double sign : {1.0, -1.0}) {
    Ref7 ref;
    ref.q = home + Vec7::Constant(sign * 0.1);
    const Vec7 tau = pd.compute(s, ref).tau;
    for (int j = 0; j < kDof; ++j) {
      EXPECT_EQ(tau(j), sign * lim(j)) << "sign " << sign << " joint " << j;
    }
  }

  const PdGravity<kDof> nominal(arm->nominal, PdGravityGains<kDof>{}, home);
  for (int which = 0; which < 4; ++which) {
    State7 bad;
    bad.q = home;
    Ref7 ref;
    ref.q = home;
    ref.q(2) += 0.05;  // a nonzero command if the input were finite
    Vec7* target[] = {&bad.q, &bad.qd, &ref.q, &ref.qd};
    (*target[which])(3) = kNan;
    const Vec7 tau = nominal.compute(bad, ref).tau;
    for (int j = 0; j < kDof; ++j) {
      EXPECT_EQ(tau(j), 0.0) << "NaN in input " << which << " (q, qd, ref.q, ref.qd), joint " << j;
    }
  }
}

}  // namespace
}  // namespace cobot::control
