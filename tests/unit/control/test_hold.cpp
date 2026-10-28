#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "cobot/control/hold.hpp"

#include "control_test_util.hpp"

namespace cobot::control {
namespace {

using test::expect_near_rel;
using test::load_arm;
using test::make_pendulum;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

template <int N>
ComputedTorqueGains<N> gains(double kp, double kd) {
  ComputedTorqueGains<N> g;
  g.kp.setConstant(kp);
  g.kd.setConstant(kd);
  return g;
}

TEST(HoldController, EngageStoresTarget) {
  const auto m = make_pendulum();
  HoldController<1> hold(m, gains<1>(100.0, 20.0));
  EXPECT_EQ(hold.q_hold()(0), 0.0);
  hold.engage(VecN<1>(0.7));
  EXPECT_EQ(hold.q_hold()(0), 0.7);
}

TEST(HoldController, PendulumAtTargetIsGravityHandValue) {
  const auto m = make_pendulum();
  HoldController<1> hold(m, gains<1>(100.0, 20.0));
  hold.engage(VecN<1>(0.1));
  JointState<1> s;
  s.q << 0.1;
  // G(0.1) = -9.80665 cos(0.1) = -9.80665 * 0.995004165278026 = -9.757657597423751
  EXPECT_NEAR(hold.compute(s).tau(0), -9.757657597423751, 1e-9 * 9.76);
}

TEST(HoldController, PendulumPdScaledByInertiaHandValue) {
  const auto m = make_pendulum();
  HoldController<1> hold(m, gains<1>(100.0, 20.0));
  hold.engage(VecN<1>(0.1));
  JointState<1> s;
  s.q << 0.0;
  s.qd << 0.2;
  // tau = M (kp (q_hold - q) - kd qd) + G(0) = 0.6 * (100 * 0.1 - 20 * 0.2) - 9.80665
  //     = 0.6 * 6 - 9.80665 = 3.6 - 9.80665 = -6.20665
  EXPECT_NEAR(hold.compute(s).tau(0), -6.20665, 1e-9 * 6.21);
}

TEST(HoldController, ArmAtTargetIsGravityTorque) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  HoldController<7> hold(arm->model, gains<7>(100.0, 20.0));
  for (const double shift : {0.0, 0.2}) {
    VecN<7> q = arm->home;
    q += VecN<7>::Constant(shift);
    hold.engage(q);
    JointState<7> s;
    s.q = q;
    expect_near_rel<7>(hold.compute(s).tau, dynamics::gravity_torque(arm->model, q), 1e-9);
  }
}

TEST(HoldController, ArmGenericStateMatchesLaw) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  ComputedTorqueGains<7> g;
  g.kp << 100.0, 200.0, 100.0, 200.0, 100.0, 50.0, 20.0;
  g.kd << 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 20.0;
  HoldController<7> hold(arm->model, g);
  hold.engage(arm->home);
  JointState<7> s;
  s.q = arm->home;
  s.q += VecN<7>(0.01, -0.02, 0.015, -0.01, 0.02, -0.015, 0.01);
  s.qd << 0.05, -0.04, 0.03, -0.02, 0.05, -0.03, 0.02;
  // tau = diag(M(q)) .* (kp .* (q_hold - q) - kd .* qd) + G(q)
  const VecN<7> diag_m = dynamics::mass_matrix(arm->model, s.q).diagonal();
  const VecN<7> pd = g.kp.cwiseProduct(arm->home - s.q) - g.kd.cwiseProduct(s.qd);
  const VecN<7> expected = diag_m.cwiseProduct(pd) + dynamics::gravity_torque(arm->model, s.q);
  ASSERT_TRUE((expected.cwiseAbs().array() < test::tau_max_of(arm->model).array()).all())
      << "test state must not saturate: " << expected.transpose();
  expect_near_rel<7>(hold.compute(s).tau, expected, 1e-9);
}

TEST(HoldController, HugeGainClampsToTauMaxWithSign) {
  const auto m = make_pendulum();
  HoldController<1> hold(m, gains<1>(1e9, 0.0));
  hold.engage(VecN<1>(0.01));
  const JointState<1> s;
  // 0.6 * 1e9 * 0.01 = 6e6 >> 50, gravity -9.8 does not change the sign
  EXPECT_EQ(hold.compute(s).tau(0), test::kPendulumTauMax);
  hold.engage(VecN<1>(-0.01));
  EXPECT_EQ(hold.compute(s).tau(0), -test::kPendulumTauMax);
}

TEST(HoldController, NonFiniteGivesZeroTorque) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  HoldController<7> hold(arm->model, gains<7>(100.0, 20.0));
  hold.engage(arm->home);
  JointState<7> s0;
  s0.q = arm->home;
  {
    JointState<7> s = s0;
    s.q(3) = kNan;
    EXPECT_TRUE(hold.compute(s).tau.isZero(0.0));
  }
  {
    JointState<7> s = s0;
    s.qd(5) = kInf;
    EXPECT_TRUE(hold.compute(s).tau.isZero(0.0));
  }
  VecN<7> bad = arm->home;
  bad(0) = kNan;
  hold.engage(bad);
  EXPECT_TRUE(hold.compute(s0).tau.isZero(0.0));
}

// Real arm, exact plant, 1 ms, 3 s. Starts at rest 0.05 rad from home on every joint (signs
// alternate). Linearised at home the loop is M e'' = -diag(M) (kp e + kd e'); the modes of
// M^-1 diag(M) have eigenvalues lambda >= 1/7 (diag(M)^-1/2 M diag(M)^-1/2 has unit diagonal,
// so its largest eigenvalue is <= 7). With kp = 400, kd = 40 each mode solves
// s^2 + 40 lambda s + 400 lambda = 0, decay rate min(10, 20 lambda) >= 20/7 = 2.86 1/s, and
// 0.05 exp(-2.86 * 3) = 9.5e-6: a tenfold margin to 1e-4 even for the worst coupling.
TEST(HoldController, ArmConvergesToHoldWithinTauMax) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  constexpr double kDt = 1e-3;
  constexpr int kSteps = 3000;
  HoldController<7> hold(arm->model, gains<7>(400.0, 40.0));
  hold.engage(arm->home);
  const VecN<7> tau_max = test::tau_max_of(arm->model);
  JointState<7> s;
  s.q = arm->home;
  s.q += VecN<7>(0.05, -0.05, 0.05, -0.05, 0.05, -0.05, 0.05);
  for (int k = 0; k < kSteps; ++k) {
    const VecN<7> tau = hold.compute(s).tau;
    ASSERT_TRUE((tau.cwiseAbs().array() <= tau_max.array()).all()) << "step " << k;
    test::rk4_step(arm->model, s.q, s.qd, tau, kDt);
  }
  EXPECT_LT((s.q - arm->home).cwiseAbs().maxCoeff(), 1e-4);
}

}  // namespace
}  // namespace cobot::control
