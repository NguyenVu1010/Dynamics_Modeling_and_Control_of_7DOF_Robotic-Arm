#include <algorithm>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "cobot/control/computed_torque.hpp"

#include "control_test_util.hpp"

namespace cobot::control {
namespace {

using test::computed_torque_raw;
using test::expect_near_rel;
using test::load_arm;
using test::make_pendulum;

constexpr double kPi = 3.14159265358979323846;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

template <int N>
ComputedTorqueGains<N> gains(double kp, double kd) {
  ComputedTorqueGains<N> g;
  g.kp.setConstant(kp);
  g.kd.setConstant(kd);
  return g;
}

template <int N>
void expect_all_zero(const VecN<N>& tau) {
  for (int i = 0; i < N; ++i) {
    EXPECT_EQ(tau(i), 0.0) << "joint " << i;
  }
}

// ---------------------------------------------------------------- perfect / generic state

TEST(ComputedTorque, PendulumPerfectStateHandValue) {
  const auto m = make_pendulum();
  const ComputedTorque<1> ct(m, gains<1>(100.0, 20.0));
  JointState<1> s;
  JointReference<1> r;
  s.q << 0.3;
  s.qd << 0.4;
  r.q = s.q;
  r.qd = s.qd;
  r.qdd << 1.5;
  // tau = 0.6 * 1.5 + 0.1 * 0.4 - 9.80665 cos(0.3)
  //     = 0.9 + 0.04 - 9.80665 * 0.955336489125606 = 0.94 - 9.368650581083623
  //     = -8.428650581083623
  EXPECT_NEAR(ct.compute(s, r).tau(0), -8.428650581083623, 1e-9 * 8.43);
}

TEST(ComputedTorque, PendulumGenericStateHandValue) {
  const auto m = make_pendulum();
  const ComputedTorque<1> ct(m, gains<1>(100.0, 20.0));
  JointState<1> s;
  JointReference<1> r;
  s.q << 0.2;
  s.qd << 0.1;
  r.q << 0.3;
  r.qd << 0.4;
  r.qdd << 1.5;
  // v = 1.5 + 20 * (0.4 - 0.1) + 100 * (0.3 - 0.2) = 1.5 + 6 + 10 = 17.5
  // tau = 0.6 * 17.5 + 0.1 * 0.1 - 9.80665 cos(0.2) = 10.51 - 9.611169905586811
  //     = 0.8988300944131886
  EXPECT_NEAR(ct.compute(s, r).tau(0), 0.8988300944131886, 1e-9);
}

TEST(ComputedTorque, ArmPerfectStateIsInverseDynamicsPlusDamping) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  const ComputedTorque<7> ct(arm->model, gains<7>(100.0, 20.0));
  JointState<7> s;
  JointReference<7> r;
  s.q = arm->home;
  s.q += VecN<7>(0.1, -0.2, 0.15, 0.1, -0.3, 0.2, 0.25);
  s.qd << 0.3, -0.2, 0.25, -0.3, 0.4, -0.35, 0.5;
  r.q = s.q;
  r.qd = s.qd;
  r.qdd << 0.5, -0.4, 0.6, 0.3, -0.7, 0.8, -0.5;
  const VecN<7> expected = dynamics::rnea(arm->model, s.q, s.qd, r.qdd, arm->model.gravity_b) +
                           test::damping_of(arm->model).cwiseProduct(s.qd);
  ASSERT_TRUE((expected.cwiseAbs().array() < test::tau_max_of(arm->model).array()).all())
      << "test state must not saturate: " << expected.transpose();
  expect_near_rel<7>(ct.compute(s, r).tau, expected, 1e-9);
}

TEST(ComputedTorque, ArmGenericStateMatchesLaw) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  ComputedTorqueGains<7> g;
  g.kp << 100.0, 200.0, 100.0, 200.0, 100.0, 50.0, 20.0;
  g.kd << 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 20.0;
  const ComputedTorque<7> ct(arm->model, g);
  JointState<7> s;
  JointReference<7> r;
  s.q = arm->home;
  s.q += VecN<7>(0.05, -0.04, 0.03, 0.02, -0.05, 0.04, 0.01);
  s.qd << 0.1, -0.2, 0.15, -0.1, 0.2, -0.15, 0.1;
  r.q = arm->home;
  r.qd << 0.05, 0.0, -0.05, 0.1, 0.0, 0.05, -0.1;
  r.qdd << 0.2, -0.1, 0.3, 0.1, -0.2, 0.2, -0.1;
  const VecN<7> expected = computed_torque_raw(arm->model, g, s, r);
  ASSERT_TRUE((expected.cwiseAbs().array() < test::tau_max_of(arm->model).array()).all())
      << "test state must not saturate: " << expected.transpose();
  expect_near_rel<7>(ct.compute(s, r).tau, expected, 1e-9);
}

// ---------------------------------------------------------------- clamping

TEST(ComputedTorque, PendulumHugeGainClampsToTauMaxWithSign) {
  const auto m = make_pendulum();
  const ComputedTorque<1> ct(m, gains<1>(1e9, 0.0));
  const JointState<1> s;
  JointReference<1> r;
  r.q << 0.01;  // v = 1e9 * 0.01 = 1e7, tau ~ 0.6 * 1e7 >> 50
  EXPECT_EQ(ct.compute(s, r).tau(0), test::kPendulumTauMax);
  r.q << -0.01;
  EXPECT_EQ(ct.compute(s, r).tau(0), -test::kPendulumTauMax);
}

TEST(ComputedTorque, ArmHugeGainClampsEveryJointToTauMaxWithSign) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  const ComputedTorqueGains<7> g = gains<7>(1e9, 20.0);
  const ComputedTorque<7> ct(arm->model, g);
  JointState<7> s;
  JointReference<7> r;
  s.q = arm->home;
  r.q = arm->home;
  r.q += VecN<7>(0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01);
  const VecN<7> raw = computed_torque_raw(arm->model, g, s, r);
  const VecN<7> tau_max = test::tau_max_of(arm->model);
  const VecN<7> tau = ct.compute(s, r).tau;
  for (int i = 0; i < 7; ++i) {
    ASSERT_GT(std::abs(raw(i)), tau_max(i)) << "joint " << i << " must saturate";
    EXPECT_EQ(tau(i), std::copysign(tau_max(i), raw(i))) << "joint " << i;
  }
}

// ---------------------------------------------------------------- non-finite data

TEST(ComputedTorque, NonFiniteInputGivesZeroTorque) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  const ComputedTorque<7> ct(arm->model, gains<7>(100.0, 20.0));
  JointState<7> s0;
  JointReference<7> r0;
  s0.q = arm->home;
  r0.q = arm->home;
  r0.qdd.setConstant(0.1);
  {
    JointReference<7> r = r0;
    r.q(2) = kNan;
    expect_all_zero<7>(ct.compute(s0, r).tau);
  }
  {
    JointReference<7> r = r0;
    r.qd(0) = kInf;
    expect_all_zero<7>(ct.compute(s0, r).tau);
  }
  {
    JointReference<7> r = r0;
    r.qdd(6) = -kInf;
    expect_all_zero<7>(ct.compute(s0, r).tau);
  }
  {
    JointState<7> s = s0;
    s.q(4) = kNan;
    expect_all_zero<7>(ct.compute(s, r0).tau);
  }
  {
    JointState<7> s = s0;
    s.qd(1) = kNan;
    expect_all_zero<7>(ct.compute(s, r0).tau);
  }
}

TEST(ComputedTorque, PendulumNanReferenceGivesZeroTorque) {
  const auto m = make_pendulum();
  const ComputedTorque<1> ct(m, gains<1>(100.0, 20.0));
  const JointState<1> s;
  JointReference<1> r;
  r.q << kNan;
  expect_all_zero<1>(ct.compute(s, r).tau);
}

TEST(ComputedTorque, NonFiniteResultGivesZeroTorque) {
  const auto m = make_pendulum();
  // Finite inputs, but kp * (q_ref - q) = 1e308 * 10 overflows to +inf: the result is
  // non-finite, so the command is zero (not clamped to +tau_max).
  const ComputedTorque<1> ct(m, gains<1>(1e308, 0.0));
  const JointState<1> s;
  JointReference<1> r;
  r.q << 10.0;
  expect_all_zero<1>(ct.compute(s, r).tau);
}

// ---------------------------------------------------------------- closed loop

// Reference of joint i: q = c_i + A sin(w t + phi_i), A = 0.2 rad, w = 2 pi * 0.5 Hz = pi rad/s,
// phi_i = 0.5 i rad.
constexpr double kAmplitudeRad = 0.2;
constexpr double kOmegaRadS = 2.0 * kPi * 0.5;

template <int N>
JointReference<N> sinusoid(const VecN<N>& center, double t) {
  JointReference<N> r;
  for (int i = 0; i < N; ++i) {
    const double ph = kOmegaRadS * t + 0.5 * i;
    r.q(i) = center(i) + kAmplitudeRad * std::sin(ph);
    r.qd(i) = kAmplitudeRad * kOmegaRadS * std::cos(ph);
    r.qdd(i) = -kAmplitudeRad * kOmegaRadS * kOmegaRadS * std::sin(ph);
  }
  return r;
}

// 1 ms control period, 5 s, exact plant, starting on the reference. Returns max |q_ref - q|
// for t >= 1 s and checks that no command leaves [-tau_max, tau_max].
// Gains kp = 400 1/s^2, kd = 40 1/s (critically damped, wn = 20 rad/s). A double integrator
// driven through the same 1 ms zero-order hold tracks this reference to 7.5e-6 rad (worked
// out off line), so 1e-4 leaves room for the plant's state change inside a step.
template <int N>
double track_sinusoid(const Model<N>& m, const VecN<N>& center) {
  constexpr double kDt = 1e-3;
  constexpr int kSteps = 5000;
  const ComputedTorque<N> ct(m, gains<N>(400.0, 40.0));
  const VecN<N> tau_max = test::tau_max_of(m);
  JointReference<N> r = sinusoid<N>(center, 0.0);
  JointState<N> s;
  s.q = r.q;
  s.qd = r.qd;
  double max_err = 0.0;
  for (int k = 0; k < kSteps; ++k) {
    const double t = k * kDt;
    r = sinusoid<N>(center, t);
    if (t >= 1.0) {
      max_err = std::max(max_err, (r.q - s.q).cwiseAbs().maxCoeff());
    }
    const VecN<N> tau = ct.compute(s, r).tau;
    EXPECT_TRUE((tau.cwiseAbs().array() <= tau_max.array()).all()) << "t = " << t;
    test::rk4_step(m, s.q, s.qd, tau, kDt);
  }
  return max_err;
}

TEST(ComputedTorque, PendulumTracksSinusoid) {
  const auto m = make_pendulum();
  EXPECT_LT(track_sinusoid<1>(m, VecN<1>::Zero()), 1e-4);
}

TEST(ComputedTorque, ArmTracksSinusoidsAroundHome) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  // The torque is held for the 1 ms control period while M, C and G change on the coupled
  // wrist: a first-order zero-order-hold error of about 1.16e-4 rad (halves with the period).
  // 2e-4 keeps a 1.7x margin at the real 1 kHz rate.
  EXPECT_LT(track_sinusoid<7>(arm->model, arm->home), 2e-4);
}

// Constant reference, initial offset on every joint, qd = 0. With the exact model each joint
// obeys e'' + kd_i e' + kp_i e = 0 with e = q_ref - q. Critically damped gains kp_i = wn_i^2,
// kd_i = 2 wn_i give e_i(t) = e0 (1 + wn_i t) exp(-wn_i t), e0 = -0.01 rad.
// The control period here is 0.1 ms: a 1 ms zero-order hold alone moves a wn = 10 rad/s
// double integrator 1.9e-5 rad off the continuous solution (0.1 ms: 1.9e-6, about 2.3e-6
// for wn = 12), so 1e-5 checks the decoupling, not the sampling.
template <int N>
void check_error_dynamics(const Model<N>& m, const VecN<N>& q_ref, const VecN<N>& wn) {
  constexpr double kDt = 1e-4;
  constexpr int kSteps = 10000;  // 1 s
  constexpr double kE0 = -0.01;  // q(0) = q_ref + 0.01
  ComputedTorqueGains<N> g;
  g.kp = wn.cwiseProduct(wn);
  g.kd = 2.0 * wn;
  const ComputedTorque<N> ct(m, g);
  JointReference<N> r;
  r.q = q_ref;
  JointState<N> s;
  s.q = q_ref - VecN<N>::Constant(kE0);
  double worst = 0.0;
  for (int k = 0; k < kSteps; ++k) {
    const double t = k * kDt;
    for (int i = 0; i < N; ++i) {
      const double e = r.q(i) - s.q(i);
      const double analytic = kE0 * (1.0 + wn(i) * t) * std::exp(-wn(i) * t);
      worst = std::max(worst, std::abs(e - analytic));
    }
    const VecN<N> tau = ct.compute(s, r).tau;
    test::rk4_step(m, s.q, s.qd, tau, kDt);
  }
  EXPECT_LT(worst, 1e-5);
}

TEST(ComputedTorque, PendulumErrorDynamicsMatchAnalytic) {
  const auto m = make_pendulum();
  check_error_dynamics<1>(m, VecN<1>(0.3), VecN<1>(10.0));
}

TEST(ComputedTorque, ArmErrorDynamicsMatchAnalyticPerJoint) {
  const auto arm = load_arm();
  ASSERT_TRUE(arm);
  // wn_i = 6 + i rad/s, so every joint has its own decay.
  check_error_dynamics<7>(arm->model, arm->home, VecN<7>(6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0));
}

}  // namespace
}  // namespace cobot::control
