// Property checks of ADR-004 on the real arm (sim/urdf/cobot30_5.urdf): M symmetric positive
// definite, RNEA = M qdd + C qd + G, Mdot - 2C skew-symmetric with C from Christoffel symbols,
// forward/inverse round trip, kinetic energy conservation, and gravity torque vs rating.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include "cobot/dynamics/dynamics.hpp"

#include "dynamics_test_util.hpp"

namespace cobot::dynamics {
namespace {

using test::Arm7;
using test::kArmDof;
using test::norm_close;

constexpr int kN = kArmDof;
using Vec7 = VecN<kN>;
using Mat7 = MatN<kN>;

constexpr int kStates = 1000;
constexpr unsigned kSeed = 11;
constexpr double kQdBound = 1.0;   // rad/s
constexpr double kQddBound = 2.0;  // rad/s^2

struct State {
  Vec7 q;
  Vec7 qd;
  Vec7 qdd;
};

// 1000 states, seed 11: q uniform inside the joint limits, qd in [-1, 1], qdd in [-2, 2].
std::string sci(double x) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.3e", x);
  return buf;
}

std::vector<State> random_states(const Arm7& arm) {
  std::mt19937 rng(kSeed);
  std::vector<State> out;
  out.reserve(kStates);
  for (int k = 0; k < kStates; ++k) {
    State s;
    s.q = test::random_q(arm, rng);
    s.qd = test::random_sym<kN>(kQdBound, rng);
    s.qdd = test::random_sym<kN>(kQddBound, rng);
    out.push_back(s);
  }
  return out;
}

class ArmDynamics : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(test::load_arm(arm_));
    states_ = random_states(arm_);
  }
  Arm7 arm_;
  std::vector<State> states_;
};

TEST_F(ArmDynamics, MassMatrixSymmetricPositiveDefinite) {
  double worst_asym = 0.0;
  double worst_min_eig = 1e300;
  for (const State& s : states_) {
    const Mat7 M = mass_matrix(arm_, s.q);
    ASSERT_TRUE(M.allFinite());
    const double asym = (M - M.transpose()).cwiseAbs().maxCoeff();
    worst_asym = std::max(worst_asym, asym);
    EXPECT_LT(asym, 1e-9) << "q = " << s.q.transpose();
    const Eigen::SelfAdjointEigenSolver<Mat7> eig(M);
    const double min_eig = eig.eigenvalues().minCoeff();
    worst_min_eig = std::min(worst_min_eig, min_eig);
    EXPECT_GT(min_eig, 0.0) << "q = " << s.q.transpose();
  }
  RecordProperty("max_abs_asymmetry", sci(worst_asym));
  RecordProperty("min_eigenvalue", sci(worst_min_eig));
}

TEST_F(ArmDynamics, RneaEqualsMQddPlusCoriolisPlusGravity) {
  for (const State& s : states_) {
    const Vec7 tau = rnea(arm_, s.q, s.qd, s.qdd, arm_.gravity_b);
    const Mat7 M = mass_matrix(arm_, s.q);
    const Vec7 sum = M * s.qdd + coriolis_torque(arm_, s.q, s.qd) + gravity_torque(arm_, s.q);
    EXPECT_TRUE(norm_close(sum, tau, 1e-9)) << "q = " << s.q.transpose();
  }
}

// The helper functions are the documented special cases of rnea.
TEST_F(ArmDynamics, GravityAndCoriolisAreRneaSpecialCases) {
  const Vec7 zero = Vec7::Zero();
  for (int k = 0; k < 100; ++k) {
    const State& s = states_[k];
    EXPECT_TRUE(
        norm_close(gravity_torque(arm_, s.q), rnea(arm_, s.q, zero, zero, arm_.gravity_b), 1e-12));
    EXPECT_TRUE(norm_close(coriolis_torque(arm_, s.q, s.qd),
                           rnea(arm_, s.q, s.qd, zero, common::Vec3::Zero()),
                           1e-12));
    // rnea(q, 0, qdd, 0) = M qdd
    EXPECT_TRUE(norm_close(rnea(arm_, s.q, zero, s.qdd, common::Vec3::Zero()),
                           Vec7(mass_matrix(arm_, s.q) * s.qdd),
                           1e-9));
    // G is linear in the gravity vector passed in: g -> 2 g doubles it.
    EXPECT_TRUE(norm_close(rnea(arm_, s.q, zero, zero, 2.0 * arm_.gravity_b),
                           Vec7(2.0 * gravity_torque(arm_, s.q)),
                           1e-9));
  }
}

// gravity_torque uses model.gravity_b, not a hard-coded vector.
TEST_F(ArmDynamics, GravityTorqueFollowsModelGravity) {
  Arm7 tilted = arm_;
  tilted.gravity_b = common::Vec3(1.5, -2.0, -9.0);
  for (int k = 0; k < 20; ++k) {
    const Vec7 zero = Vec7::Zero();
    EXPECT_TRUE(norm_close(gravity_torque(tilted, states_[k].q),
                           rnea(arm_, states_[k].q, zero, zero, tilted.gravity_b),
                           1e-12));
  }
}

// dM/dq_i by central differences, h = 1e-6.
std::array<Mat7, kN> mass_matrix_partials(const Arm7& arm, const Vec7& q) {
  constexpr double kH = 1e-6;
  std::array<Mat7, kN> dm_dq;
  for (int i = 0; i < kN; ++i) {
    Vec7 qp = q;
    Vec7 qm = q;
    qp[i] += kH;
    qm[i] -= kH;
    dm_dq[i] = (mass_matrix(arm, qp) - mass_matrix(arm, qm)) / (2.0 * kH);
  }
  return dm_dq;
}

// Spong eq. (7.53): c_ijk = 1/2 (dM_kj/dq_i + dM_ki/dq_j - dM_ij/dq_k), C_kj = sum_i c_ijk qd_i.
Mat7 christoffel_c(const std::array<Mat7, kN>& dm_dq, const Vec7& qd) {
  Mat7 C = Mat7::Zero();
  for (int k = 0; k < kN; ++k) {
    for (int j = 0; j < kN; ++j) {
      double ckj = 0.0;
      for (int i = 0; i < kN; ++i) {
        ckj += 0.5 * (dm_dq[i](k, j) + dm_dq[j](k, i) - dm_dq[k](i, j)) * qd[i];
      }
      C(k, j) = ckj;
    }
  }
  return C;
}

TEST_F(ArmDynamics, MdotMinus2CIsSkewSymmetric) {
  std::mt19937 rng(kSeed + 1);
  std::normal_distribution<double> n(0.0, 1.0);
  for (const State& s : states_) {
    const std::array<Mat7, kN> dm_dq = mass_matrix_partials(arm_, s.q);
    Mat7 m_dot = Mat7::Zero();
    for (int i = 0; i < kN; ++i) {
      m_dot += dm_dq[i] * s.qd[i];
    }
    const Mat7 C = christoffel_c(dm_dq, s.qd);
    const Mat7 S = m_dot - 2.0 * C;
    Vec7 x;
    for (int j = 0; j < kN; ++j) {
      x[j] = n(rng);
    }
    const double scale = x.squaredNorm() * (m_dot.norm() + 2.0 * C.norm());
    EXPECT_LE(std::abs(x.dot(S * x)), 1e-5 * scale) << "q = " << s.q.transpose();

    // The same C, built from M alone, must reproduce RNEA's velocity term: this ties CRBA and
    // RNEA together independently of the KDL fixture.
    const Vec7 cqd_christoffel = C * s.qd;
    const Vec7 cqd_rnea = coriolis_torque(arm_, s.q, s.qd);
    EXPECT_LE((cqd_rnea - cqd_christoffel).norm(), 1e-5 * std::max(cqd_christoffel.norm(), 1.0))
        << "q = " << s.q.transpose() << "\n rnea " << cqd_rnea.transpose() << "\n christoffel "
        << cqd_christoffel.transpose();
  }
}

TEST_F(ArmDynamics, ForwardDynamicsInvertsRnea) {
  std::mt19937 rng(kSeed + 2);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  Vec7 damping;
  for (int j = 0; j < kN; ++j) {
    damping[j] = arm_.joints[j].damping;
  }
  ASSERT_GT(damping.norm(), 0.0) << "URDF damping expected to be non-zero";
  for (const State& s : states_) {
    Vec7 tau;
    for (int j = 0; j < kN; ++j) {
      tau[j] = unit(rng) * arm_.joints[j].tau_max;
    }
    const Vec7 qdd = forward_dynamics(arm_, s.q, s.qd, tau);
    ASSERT_TRUE(qdd.allFinite());
    const Vec7 back = rnea(arm_, s.q, s.qd, qdd, arm_.gravity_b) + damping.cwiseProduct(s.qd);
    EXPECT_TRUE(norm_close(back, tau, 1e-9)) << "q = " << s.q.transpose();
  }
}

// No gravity, no damping, tau = 0: the kinetic energy 1/2 qd^T M qd is conserved. RK4 of
// forward_dynamics, dt = 1 ms, 10 s; joint limits are not enforced here.
TEST_F(ArmDynamics, KineticEnergyConservedWithoutGravityAndDamping) {
  Arm7 free_arm = arm_;
  free_arm.gravity_b = common::Vec3::Zero();
  for (int j = 0; j < kN; ++j) {
    free_arm.joints[j].damping = 0.0;
  }
  constexpr double kDt = 1e-3;   // s
  constexpr int kSteps = 10000;  // 10 s
  const Vec7 tau = Vec7::Zero();
  std::mt19937 rng(kSeed + 3);
  Vec7 q = test::random_q(free_arm, rng);
  Vec7 qd = test::random_sym<kN>(kQdBound, rng);

  auto energy = [&](const Vec7& qq, const Vec7& vv) {
    return 0.5 * vv.dot(Vec7(mass_matrix(free_arm, qq) * vv));
  };
  const double e0 = energy(q, qd);
  ASSERT_GT(e0, 0.0);
  double worst = 0.0;
  for (int step = 0; step < kSteps; ++step) {
    const Vec7 k1v = forward_dynamics(free_arm, q, qd, tau);
    const Vec7 k1q = qd;
    const Vec7 q2 = q + 0.5 * kDt * k1q;
    const Vec7 v2 = qd + 0.5 * kDt * k1v;
    const Vec7 k2v = forward_dynamics(free_arm, q2, v2, tau);
    const Vec7 q3 = q + 0.5 * kDt * v2;
    const Vec7 v3 = qd + 0.5 * kDt * k2v;
    const Vec7 k3v = forward_dynamics(free_arm, q3, v3, tau);
    const Vec7 q4 = q + kDt * v3;
    const Vec7 v4 = qd + kDt * k3v;
    const Vec7 k4v = forward_dynamics(free_arm, q4, v4, tau);
    q += kDt / 6.0 * (k1q + 2.0 * v2 + 2.0 * v3 + v4);
    qd += kDt / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
    ASSERT_TRUE(qd.allFinite()) << "step " << step;
    worst = std::max(worst, std::abs(energy(q, qd) - e0) / e0);
  }
  RecordProperty("max_relative_energy_drift", sci(worst));
  EXPECT_LT(worst, 1e-4);
}

// Design 5.8: max over a 5^7 grid inside the limits (5 evenly spaced values per joint,
// endpoints included) of |G_j| / tau_max_j must stay below 1. A failure is an owner decision
// (ADR-005), not a reason to change this test.
TEST_F(ArmDynamics, GravityTorqueWithinRating) {
  constexpr int kPerJoint = 5;
  std::array<double, kN> worst_ratio{};
  std::array<Vec7, kN> worst_q;
  worst_q.fill(Vec7::Zero());
  std::array<int, kN> idx{};
  bool done = false;
  while (!done) {
    Vec7 q;
    for (int j = 0; j < kN; ++j) {
      const model::JointParams& p = arm_.joints[j];
      q[j] = p.q_min + (p.q_max - p.q_min) * idx[j] / (kPerJoint - 1);
    }
    const Vec7 G = gravity_torque(arm_, q);
    for (int j = 0; j < kN; ++j) {
      const double ratio = std::abs(G[j]) / arm_.joints[j].tau_max;
      if (ratio > worst_ratio[j]) {
        worst_ratio[j] = ratio;
        worst_q[j] = q;
      }
    }
    int j = 0;
    while (j < kN && ++idx[j] == kPerJoint) {
      idx[j] = 0;
      ++j;
    }
    done = j == kN;
  }
  double overall = 0.0;
  for (int j = 0; j < kN; ++j) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.6f", worst_ratio[j]);
    RecordProperty("max_G_over_tau_max_joint_" + std::to_string(j + 1), buf);
    overall = std::max(overall, worst_ratio[j]);
    EXPECT_LT(worst_ratio[j], 1.0)
        << "joint " << j + 1 << " gravity exceeds its rating at q = " << worst_q[j].transpose();
  }
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.6f", overall);
  RecordProperty("max_G_over_tau_max", buf);
}

}  // namespace
}  // namespace cobot::dynamics
