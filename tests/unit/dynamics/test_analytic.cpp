// Known-answer models: a single pendulum (Model<1>) and the planar 2-link arm of Spong,
// Hutchinson and Vidyasagar, "Robot Modeling and Control", sec. 7.4 (Model<2>).
#include <cmath>
#include <random>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "cobot/dynamics/dynamics.hpp"

#include "dynamics_test_util.hpp"

namespace cobot::dynamics {
namespace {

using common::Mat3;
using common::Vec3;
using test::all_close;

constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.80665;  // m/s^2
constexpr double kTol = 1e-12;  // relative, with the same absolute floor

// ---------------------------------------------------------------------------------------
// Single pendulum. Joint axis x of the joint frame, centre of mass at (0, 0, -l) in the link
// frame, gravity (0, 0, -g) in b. With T_parent_joint.R = I the CoM in b is
// R_x(q) (0, 0, -l) = (0, l sin q, -l cos q), so V = m g z = -m g l cos q and
//   G = dV/dq = m g l sin q,   M = I_xx + m l^2,   C qd = 0 (one DOF).
// Products of inertia do not enter: a^T I_c a = I_xx for a = x, and the x component of
// w x (I_c w) with w = (qd, 0, 0) is 0.
constexpr double kPendMass = 2.0;   // kg
constexpr double kPendLen = 0.3;    // m
constexpr double kPendIxx = 0.05;   // kg m^2
constexpr double kPendDamp = 0.04;  // N m s / rad
// M = 0.05 + 2.0 * 0.3^2 = 0.05 + 0.18 = 0.23 kg m^2
constexpr double kPendM = 0.23;
// m g l = 2.0 * 9.80665 * 0.3 = 5.88399 N m
constexpr double kPendMgl = 5.88399;

model::Model<1> pendulum(const Mat3& R_parent_joint) {
  model::Model<1> m;
  m.joints[0].name = "pendulum";
  m.joints[0].T_parent_joint.R = R_parent_joint;
  m.joints[0].T_parent_joint.p = Vec3(0.1, -0.2, 0.5);  // fixed base: the offset is irrelevant
  m.joints[0].axis_joint = Vec3::UnitX();
  m.joints[0].q_min = -kPi;
  m.joints[0].q_max = kPi;
  m.joints[0].qd_max = 3.0;
  m.joints[0].tau_max = 20.0;
  m.joints[0].damping = kPendDamp;
  m.links[0].mass_kg = kPendMass;
  m.links[0].com_i = Vec3(0.0, 0.0, -kPendLen);
  Mat3 I;
  I << kPendIxx, 0.001, -0.002,  //
      0.001, 0.04, 0.0015,       //
      -0.002, 0.0015, 0.03;
  m.links[0].I_com_i = I;
  m.T_last_tip.p = Vec3(0.0, 0.0, -2.0 * kPendLen);
  m.gravity_b = Vec3(0.0, 0.0, -kG);
  return m;
}

VecN<1> v1(double x) {
  VecN<1> v;
  v << x;
  return v;
}

TEST(Pendulum, MassMatrixIsIPlusMl2) {
  const model::Model<1> m = pendulum(Mat3::Identity());
  for (const double q : {-2.0, 0.0, 0.7, 3.0}) {
    const MatN<1> M = mass_matrix(m, v1(q));
    EXPECT_NEAR(M(0, 0), kPendM, kTol) << "q = " << q;
  }
}

TEST(Pendulum, GravityTorqueIsMglSinQ) {
  const model::Model<1> m = pendulum(Mat3::Identity());
  for (const double q : {-2.0, 0.0, 0.7, 1.5707963267948966, 3.0}) {
    const VecN<1> G = gravity_torque(m, v1(q));
    EXPECT_NEAR(G[0], kPendMgl * std::sin(q), kTol * (1.0 + kPendMgl)) << "q = " << q;
  }
  // q = 0.7: sin(0.7) = 0.644217687..., G = 5.88399 * 0.644217687 = 3.79057043 N m
  EXPECT_NEAR(gravity_torque(m, v1(0.7))[0], 3.7905704295297014, 1e-12 * 3.8);
}

// Rotating the joint frame by R_x(pi/2) about the axis itself turns the CoM direction:
// CoM in b = R_x(q + pi/2) (0, 0, -l) = (0, l cos q, l sin q), V = m g l sin q,
// G = m g l cos q. An implementation that ignores T_parent_joint.R gets sin q instead.
TEST(Pendulum, JointFrameRotationEntersGravity) {
  const Mat3 R = Eigen::AngleAxisd(kPi / 2.0, Vec3::UnitX()).toRotationMatrix();
  const model::Model<1> m = pendulum(R);
  for (const double q : {-2.0, 0.0, 0.7, 3.0}) {
    EXPECT_NEAR(gravity_torque(m, v1(q))[0], kPendMgl * std::cos(q), kTol * (1.0 + kPendMgl))
        << "q = " << q;
    EXPECT_NEAR(mass_matrix(m, v1(q))(0, 0), kPendM, kTol);
  }
  // q = 0.7: cos(0.7) = 0.764842187..., G = 5.88399 * 0.764842187 = 4.50032378 N m
  EXPECT_NEAR(gravity_torque(m, v1(0.7))[0], 4.500323781560057, 1e-12 * 4.5);
}

TEST(Pendulum, RneaIsMQddPlusG) {
  const model::Model<1> m = pendulum(Mat3::Identity());
  // q = 0.7, qd = 1.3, qdd = -0.4:
  // tau = 0.23 * (-0.4) + 5.88399 * sin(0.7) = -0.092 + 3.79057043 = 3.69857043 N m
  const VecN<1> tau = rnea(m, v1(0.7), v1(1.3), v1(-0.4), m.gravity_b);
  EXPECT_NEAR(tau[0], 3.6985704295297013, 1e-12 * 3.7);
  for (const double q : {-2.0, 0.4, 2.5}) {
    for (const double qd : {-1.5, 0.0, 2.0}) {
      for (const double qdd : {-3.0, 0.5}) {
        const double expected = kPendM * qdd + kPendMgl * std::sin(q);
        const VecN<1> t = rnea(m, v1(q), v1(qd), v1(qdd), m.gravity_b);
        EXPECT_NEAR(t[0], expected, kTol * (1.0 + std::abs(expected)))
            << "q " << q << " qd " << qd << " qdd " << qdd;
      }
    }
  }
}

TEST(Pendulum, ZeroGravityArgumentDropsG) {
  const model::Model<1> m = pendulum(Mat3::Identity());
  // tau = 0.23 * 1.5 = 0.345 N m, whatever q and qd
  const VecN<1> tau = rnea(m, v1(1.1), v1(-0.9), v1(1.5), Vec3::Zero());
  EXPECT_NEAR(tau[0], 0.345, kTol);
}

TEST(Pendulum, CoriolisVanishesForOneJoint) {
  const model::Model<1> m = pendulum(Mat3::Identity());
  for (const double qd : {-2.0, 0.5, 3.0}) {
    EXPECT_NEAR(coriolis_torque(m, v1(0.9), v1(qd))[0], 0.0, kTol);
  }
}

TEST(Pendulum, ForwardDynamicsIncludesDamping) {
  const model::Model<1> m = pendulum(Mat3::Identity());
  // qdd = (tau - G - d qd) / M = (1.0 - 3.79057043 - 0.04 * 1.3) / 0.23
  //     = (1.0 - 3.79057043 - 0.052) / 0.23 = -2.84257043 / 0.23 = -12.3590019 rad/s^2
  const VecN<1> qdd = forward_dynamics(m, v1(0.7), v1(1.3), v1(1.0));
  EXPECT_NEAR(qdd[0], -12.35900186752044, 1e-12 * 12.4);
}

// ---------------------------------------------------------------------------------------
// Planar 2-link arm (Spong sec. 7.4.1): both axes z, link 2 attached at (l1, 0, 0) of link 1,
// CoMs at (lc_i, 0, 0), gravity (0, -g, 0) in b, angles measured from the x axis.
//   d11 = m1 lc1^2 + m2 (l1^2 + lc2^2 + 2 l1 lc2 cos q2) + I1 + I2
//   d12 = d21 = m2 (lc2^2 + l1 lc2 cos q2) + I2,   d22 = m2 lc2^2 + I2
//   h = -m2 l1 lc2 sin q2,  (C qd)_1 = 2 h qd1 qd2 + h qd2^2,  (C qd)_2 = -h qd1^2
//   G_1 = (m1 lc1 + m2 l1) g cos q1 + m2 lc2 g cos(q1 + q2),  G_2 = m2 lc2 g cos(q1 + q2)
// I_i is I_zz about the CoM; I_xx, I_yy and I_xy do not enter for rotation about z.
constexpr double kM1 = 1.5;    // kg
constexpr double kM2 = 0.8;    // kg
constexpr double kL1 = 0.4;    // m
constexpr double kLc1 = 0.2;   // m
constexpr double kLc2 = 0.15;  // m
constexpr double kI1 = 0.012;  // kg m^2
constexpr double kI2 = 0.004;  // kg m^2

model::Model<2> planar() {
  model::Model<2> m;
  for (int j = 0; j < 2; ++j) {
    m.joints[j].name = j == 0 ? "joint_1" : "joint_2";
    m.joints[j].axis_joint = Vec3::UnitZ();
    m.joints[j].q_min = -kPi;
    m.joints[j].q_max = kPi;
    m.joints[j].qd_max = 3.0;
    m.joints[j].tau_max = 30.0;
    m.joints[j].damping = 0.0;
  }
  m.joints[1].T_parent_joint.p = Vec3(kL1, 0.0, 0.0);
  m.links[0].mass_kg = kM1;
  m.links[0].com_i = Vec3(kLc1, 0.0, 0.0);
  m.links[0].I_com_i << 0.002, 0.0003, 0.0,  //
      0.0003, 0.011, 0.0,                    //
      0.0, 0.0, kI1;
  m.links[1].mass_kg = kM2;
  m.links[1].com_i = Vec3(kLc2, 0.0, 0.0);
  m.links[1].I_com_i << 0.001, -0.0002, 0.0,  //
      -0.0002, 0.0035, 0.0,                   //
      0.0, 0.0, kI2;
  m.T_last_tip.p = Vec3(0.3, 0.0, 0.0);
  m.gravity_b = Vec3(0.0, -kG, 0.0);
  return m;
}

MatN<2> spong_m(const VecN<2>& q) {
  const double c2 = std::cos(q[1]);
  MatN<2> M;
  M(0, 0) = kM1 * kLc1 * kLc1 + kM2 * (kL1 * kL1 + kLc2 * kLc2 + 2.0 * kL1 * kLc2 * c2) + kI1 + kI2;
  M(0, 1) = kM2 * (kLc2 * kLc2 + kL1 * kLc2 * c2) + kI2;
  M(1, 0) = M(0, 1);
  M(1, 1) = kM2 * kLc2 * kLc2 + kI2;
  return M;
}

VecN<2> spong_cqd(const VecN<2>& q, const VecN<2>& qd) {
  const double h = -kM2 * kL1 * kLc2 * std::sin(q[1]);
  const VecN<2> cqd(2.0 * h * qd[0] * qd[1] + h * qd[1] * qd[1], -h * qd[0] * qd[0]);
  return cqd;
}

VecN<2> spong_g(const VecN<2>& q) {
  const double c12 = std::cos(q[0] + q[1]);
  const VecN<2> G((kM1 * kLc1 + kM2 * kL1) * kG * std::cos(q[0]) + kM2 * kLc2 * kG * c12,
                  kM2 * kLc2 * kG * c12);
  return G;
}

TEST(Planar2Link, LiteralValuesAtZero) {
  const model::Model<2> m = planar();
  const VecN<2> q = VecN<2>::Zero();
  // d11 = 1.5*0.04 + 0.8*(0.16 + 0.0225 + 2*0.4*0.15) + 0.012 + 0.004
  //     = 0.06 + 0.8*0.3025 + 0.016 = 0.06 + 0.242 + 0.016 = 0.318
  // d12 = 0.8*(0.0225 + 0.06) + 0.004 = 0.066 + 0.004 = 0.070
  // d22 = 0.8*0.0225 + 0.004 = 0.018 + 0.004 = 0.022
  MatN<2> M_expected;
  M_expected << 0.318, 0.070, 0.070, 0.022;
  EXPECT_TRUE(all_close(mass_matrix(m, q), M_expected, kTol, kTol));
  // G1 = (1.5*0.2 + 0.8*0.4)*g + 0.8*0.15*g = (0.3 + 0.32 + 0.12)*g = 0.74*9.80665 = 7.256921
  // G2 = 0.12*9.80665 = 1.176798
  EXPECT_TRUE(all_close(gravity_torque(m, q), VecN<2>(7.256921, 1.176798), kTol, kTol));
}

TEST(Planar2Link, LiteralTorqueAtOneState) {
  const model::Model<2> m = planar();
  // q = (0.3, -0.8), qd = (0.5, 1.2), qdd = (-0.7, 0.9); cos(-0.8) = 0.69670671,
  // sin(-0.8) = -0.71735609.
  // d11 = 0.06 + 0.8*(0.1825 + 0.12*0.69670671) + 0.016 = 0.28888384
  // d12 = 0.8*(0.0225 + 0.06*0.69670671) + 0.004 = 0.05544192,  d22 = 0.022
  // h = -0.8*0.4*0.15*(-0.71735609) = 0.03443309
  // C qd = (h*(2*0.5*1.2 + 1.44), -h*0.25) = (0.09090336, -0.00860827)
  // G1 = 0.62*g*cos(0.3) + 0.12*g*cos(-0.5) = 5.80856336 + 1.03273740 = 6.84130076
  // G2 = 0.12*g*cos(-0.5) = 1.03273740
  // tau1 = 0.28888384*(-0.7) + 0.05544192*0.9 + 0.09090336 + 6.84130076 = 6.77988317
  // tau2 = 0.05544192*(-0.7) + 0.022*0.9 - 0.00860827 + 1.03273740 = 1.00511979
  const VecN<2> tau =
      rnea(m, VecN<2>(0.3, -0.8), VecN<2>(0.5, 1.2), VecN<2>(-0.7, 0.9), m.gravity_b);
  EXPECT_TRUE(all_close(tau, VecN<2>(6.77988316675377, 1.0051197851426077), kTol, kTol));
}

TEST(Planar2Link, MatchesSpongClosedForm) {
  const model::Model<2> m = planar();
  std::mt19937 rng(5);
  for (int k = 0; k < 200; ++k) {
    const VecN<2> q = test::random_q(m, rng);
    const VecN<2> qd = test::random_sym<2>(2.0, rng);
    const VecN<2> qdd = test::random_sym<2>(3.0, rng);
    const MatN<2> M = spong_m(q);
    const VecN<2> cqd = spong_cqd(q, qd);
    const VecN<2> G = spong_g(q);
    EXPECT_TRUE(all_close(mass_matrix(m, q), M, kTol, kTol)) << "k = " << k;
    EXPECT_TRUE(all_close(gravity_torque(m, q), G, kTol, kTol)) << "k = " << k;
    EXPECT_TRUE(all_close(coriolis_torque(m, q, qd), cqd, kTol, kTol)) << "k = " << k;
    const VecN<2> tau_expected = M * qdd + cqd + G;
    EXPECT_TRUE(all_close(rnea(m, q, qd, qdd, m.gravity_b), tau_expected, kTol, kTol))
        << "k = " << k;
    // No damping in this model, so forward dynamics inverts rnea.
    const VecN<2> qdd_fd = forward_dynamics(m, q, qd, tau_expected);
    EXPECT_TRUE(all_close(qdd_fd, qdd, 1e-10, 1e-10)) << "k = " << k;
  }
}

}  // namespace
}  // namespace cobot::dynamics
