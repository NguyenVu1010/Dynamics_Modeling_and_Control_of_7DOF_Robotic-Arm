// Feature model-armature, dynamics side. JointParams::armature (kg m^2) is the rotor inertia
// reflected through the gear, J_in * N^2. In the standard approximation it adds only to the
// diagonal of the joint-space inertia:
//   M_a(q) = M(q) + diag(armature),   tau_a = rnea(q, qd, qdd) + armature .* qdd,
// and contributes nothing to G(q) or C(q, qd) qd. forward_dynamics uses M_a.
#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <gtest/gtest.h>

#include "cobot/dynamics/dynamics.hpp"

#include "dynamics_test_util.hpp"

namespace cobot::dynamics {
namespace {

using test::all_close;
using test::Arm7;
using test::kArmDof;
using test::norm_close;

constexpr int kN = kArmDof;
using Vec7 = VecN<kN>;
using Mat7 = MatN<kN>;

constexpr int kStates = 200;
constexpr unsigned kSeed = 23;
constexpr double kQdBound = 1.0;   // rad/s
constexpr double kQddBound = 2.0;  // rad/s^2
// "Exact" up to rounding: M entries are O(10) kg m^2 and torques O(100) N m, so a few ulps
// (~1e-15 and ~1e-14) stay far below these bounds.
constexpr double kExactM = 1e-12;    // kg m^2
constexpr double kExactTau = 1e-10;  // N m
constexpr double kRoundTrip = 1e-9;

// Distinct per joint so an index mix-up shows; kg m^2.
Vec7 test_armature() {
  Vec7 a;
  a << 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1;
  return a;
}

template <int N>
Model<N> with_armature(Model<N> m, const VecN<N>& a) {
  for (int j = 0; j < N; ++j) {
    m.joints[j].armature = a[j];
  }
  return m;
}

template <int N>
Model<N> without_armature(const Model<N>& m) {
  return with_armature<N>(m, VecN<N>::Zero());
}

template <int N>
VecN<N> armature_of(const Model<N>& m) {
  VecN<N> a;
  for (int j = 0; j < N; ++j) {
    a[j] = m.joints[j].armature;
  }
  return a;
}

template <int N>
VecN<N> damping_of(const Model<N>& m) {
  VecN<N> d;
  for (int j = 0; j < N; ++j) {
    d[j] = m.joints[j].damping;
  }
  return d;
}

struct State {
  Vec7 q;
  Vec7 qd;
  Vec7 qdd;
};

class ArmArmature : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(test::load_arm(urdf_arm_));
    bare_ = without_armature<kN>(urdf_arm_);
    arm_a_ = with_armature<kN>(bare_, test_armature());
    std::mt19937 rng(kSeed);
    for (int k = 0; k < kStates; ++k) {
      State s;
      s.q = test::random_q(bare_, rng);
      s.qd = test::random_sym<kN>(kQdBound, rng);
      s.qdd = test::random_sym<kN>(kQddBound, rng);
      states_.push_back(s);
    }
  }
  Arm7 urdf_arm_;  // armature as written in sim/urdf/cobot30_5.urdf
  Arm7 bare_;      // armature zeroed
  Arm7 arm_a_;     // test_armature()
  std::vector<State> states_;
};

TEST_F(ArmArmature, MassMatrixAddsDiagonal) {
  const Mat7 D = test_armature().asDiagonal();
  for (std::size_t k = 0; k < states_.size(); ++k) {
    const Vec7& q = states_[k].q;
    const Mat7 diff = mass_matrix(arm_a_, q) - mass_matrix(bare_, q);
    EXPECT_TRUE(all_close(diff, D, 0.0, kExactM)) << "state " << k;
  }
}

TEST_F(ArmArmature, MassMatrixOfUrdfArmAddsUrdfArmature) {
  const Vec7 a = armature_of(urdf_arm_);
  // The URDF carries armature on every joint (values checked by test_model_armature); without
  // it this test would be vacuous.
  EXPECT_GT(a.minCoeff(), 0.0);
  const Mat7 D = a.asDiagonal();
  for (std::size_t k = 0; k < states_.size(); ++k) {
    const Vec7& q = states_[k].q;
    const Mat7 diff = mass_matrix(urdf_arm_, q) - mass_matrix(bare_, q);
    EXPECT_TRUE(all_close(diff, D, 0.0, kExactM)) << "state " << k;
  }
}

TEST_F(ArmArmature, RneaAddsArmatureTimesQdd) {
  const Vec7 a = test_armature();
  for (std::size_t k = 0; k < states_.size(); ++k) {
    const State& s = states_[k];
    const Vec7 diff = rnea(arm_a_, s.q, s.qd, s.qdd, arm_a_.gravity_b) -
                      rnea(bare_, s.q, s.qd, s.qdd, bare_.gravity_b);
    const Vec7 expected = a.cwiseProduct(s.qdd);
    EXPECT_TRUE(all_close(diff, expected, 0.0, kExactTau)) << "state " << k;
  }
}

TEST_F(ArmArmature, RneaEqualsMassMatrixTimesQddPlusBias) {
  // tau = M_a qdd + C qd + G with the armature model throughout.
  for (std::size_t k = 0; k < states_.size(); ++k) {
    const State& s = states_[k];
    const Vec7 tau = rnea(arm_a_, s.q, s.qd, s.qdd, arm_a_.gravity_b);
    const Vec7 rebuilt = mass_matrix(arm_a_, s.q) * s.qdd + coriolis_torque(arm_a_, s.q, s.qd) +
                         gravity_torque(arm_a_, s.q);
    EXPECT_TRUE(norm_close(rebuilt, tau, 1e-9)) << "state " << k;
  }
}

TEST_F(ArmArmature, GravityAndCoriolisUnchanged) {
  const Vec7 zero = Vec7::Zero();
  for (std::size_t k = 0; k < states_.size(); ++k) {
    const State& s = states_[k];
    EXPECT_TRUE(all_close(gravity_torque(arm_a_, s.q), gravity_torque(bare_, s.q), 0.0, kExactTau))
        << "state " << k;
    EXPECT_TRUE(all_close(
        coriolis_torque(arm_a_, s.q, s.qd), coriolis_torque(bare_, s.q, s.qd), 0.0, kExactTau))
        << "state " << k;
    // qdd = 0: the armature term vanishes, so rnea itself is unchanged.
    EXPECT_TRUE(all_close(rnea(arm_a_, s.q, s.qd, zero, arm_a_.gravity_b),
                          rnea(bare_, s.q, s.qd, zero, bare_.gravity_b),
                          0.0,
                          kExactTau))
        << "state " << k;
  }
}

TEST_F(ArmArmature, ForwardDynamicsInvertsRnea) {
  // forward_dynamics subtracts damping .* qd, so feed it tau = rnea + damping .* qd.
  for (const Arm7* arm : {&arm_a_, &urdf_arm_}) {
    const Vec7 damping = damping_of(*arm);
    for (std::size_t k = 0; k < states_.size(); ++k) {
      const State& s = states_[k];
      const Vec7 tau = rnea(*arm, s.q, s.qd, s.qdd, arm->gravity_b) + damping.cwiseProduct(s.qd);
      const Vec7 qdd = forward_dynamics(*arm, s.q, s.qd, tau);
      EXPECT_TRUE(norm_close(qdd, s.qdd, kRoundTrip)) << "state " << k;
    }
  }
}

TEST_F(ArmArmature, ForwardDynamicsUsesArmature) {
  // Same torque, more inertia: at rest with gravity cancelled (tau = e7 + G), M_a qdd = e7.
  // Joint 7's own inertia about its axis is ~1.8e-4 kg m^2, so armature 0.1 kg m^2 cuts its
  // acceleration by far more than a factor of 10.
  const Vec7 q = states_.front().q;
  const Vec7 zero = Vec7::Zero();
  Vec7 e7 = Vec7::Zero();
  e7[kN - 1] = 1.0;
  const Vec7 tau_a = e7 + gravity_torque(arm_a_, q);
  const Vec7 tau_0 = e7 + gravity_torque(bare_, q);
  const Vec7 qdd_a = forward_dynamics(arm_a_, q, zero, tau_a);
  const Vec7 qdd_0 = forward_dynamics(bare_, q, zero, tau_0);
  const Mat7 M_a = mass_matrix(arm_a_, q);
  const Vec7 expected_a = M_a.ldlt().solve(e7);
  EXPECT_TRUE(norm_close(qdd_a, expected_a, kRoundTrip));
  EXPECT_LT(std::abs(qdd_a[kN - 1]), 0.1 * std::abs(qdd_0[kN - 1]));
}

TEST_F(ArmArmature, NoHeapAllocation) {
  const State& s = states_.front();
  const Vec7 tau = Vec7::Constant(1.0);
  Eigen::internal::set_is_malloc_allowed(false);
  const Mat7 M = mass_matrix(arm_a_, s.q);
  const Vec7 t = rnea(arm_a_, s.q, s.qd, s.qdd, arm_a_.gravity_b);
  const Vec7 qdd = forward_dynamics(arm_a_, s.q, s.qd, tau);
  Eigen::internal::set_is_malloc_allowed(true);
  EXPECT_TRUE(M.allFinite());
  EXPECT_TRUE(t.allFinite());
  EXPECT_TRUE(qdd.allFinite());
}

// Planar 2-link fixture tests/data/urdf/armature_2link.urdf: both axes +z, gravity along -z so
// it does no work; no products of inertia. joint_1 armature 0.25 kg m^2, joint_2 none.
// l1 = 0.5 (joint 1 -> joint 2), m1 = 2, c1 = 0.25, Izz1 = 0.0427; m2 = 1, c2 = 0.2, Izz2 = 0.0137.
// Planar arm at q2 = 0 (cos q2 = 1):
//   M11 = Izz1 + m1 c1^2 + Izz2 + m2 (l1^2 + c2^2 + 2 l1 c2) + a1
//       = 0.0427 + 0.125 + 0.0137 + (0.25 + 0.04 + 0.2) + 0.25 = 0.6714 + 0.25 = 0.9214
//   M12 = Izz2 + m2 (c2^2 + l1 c2) = 0.0137 + 0.04 + 0.1 = 0.1537
//   M22 = Izz2 + m2 c2^2 = 0.0137 + 0.04 = 0.0537
TEST(PlanarArmature, HandValuesAtZero) {
  const common::Result<model::UrdfChain> chain = model::parse_urdf_chain(
      test::repo_path("tests/data/urdf/armature_2link.urdf"), "base_link", "link_2");
  ASSERT_TRUE(chain.is_ok()) << chain.reason();
  const common::Result<Model<2>> m = model::make_model<2>(chain.value(), test::kGravityB);
  ASSERT_TRUE(m.is_ok()) << m.reason();
  const Model<2>& arm = m.value();
  const VecN<2> q = VecN<2>::Zero();
  const VecN<2> qd = VecN<2>::Zero();
  MatN<2> expected;
  expected << 0.9214, 0.1537, 0.1537, 0.0537;
  EXPECT_TRUE(all_close(mass_matrix(arm, q), expected, 0.0, 1e-12));

  // At rest (no C qd), gravity along the axes (G = 0): tau = M_a qdd with qdd = (1, -2):
  //   tau1 = 0.9214 * 1 + 0.1537 * (-2) = 0.9214 - 0.3074 = 0.6140
  //   tau2 = 0.1537 * 1 + 0.0537 * (-2) = 0.1537 - 0.1074 = 0.0463
  VecN<2> qdd;
  qdd << 1.0, -2.0;
  VecN<2> tau_expected;
  tau_expected << 0.6140, 0.0463;
  EXPECT_TRUE(all_close(rnea(arm, q, qd, qdd, arm.gravity_b), tau_expected, 0.0, 1e-12));

  // Armature zeroed: M11 drops by exactly 0.25 -> 0.6714, the rest is unchanged.
  MatN<2> expected_bare;
  expected_bare << 0.6714, 0.1537, 0.1537, 0.0537;
  EXPECT_TRUE(all_close(mass_matrix(without_armature<2>(arm), q), expected_bare, 0.0, 1e-12));
}

}  // namespace
}  // namespace cobot::dynamics
