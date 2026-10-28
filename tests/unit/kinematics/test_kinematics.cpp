#include <algorithm>
#include <cmath>
#include <filesystem>
#include <random>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <gtest/gtest.h>

#include "cobot/common/lie.hpp"
#include "cobot/common/types.hpp"
#include "cobot/kinematics/kinematics.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"

namespace cobot::kinematics {
namespace {

namespace fs = std::filesystem;
using common::Mat3;
using common::so3_log;

const fs::path kRepo = COBOT_REPO_DIR;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

// ---------------------------------------------------------------------------------------------
// Planar 2-link arm built in code: both axes z, link lengths l1 = 1.0 (joint 1 -> joint 2) and
// l2 = 0.7 (joint 2 -> tip), all along x of the moving frame.
//   T_b_0 = Rz(q1)
//   T_b_1 = T_b_0 * Trans(l1, 0, 0) * Rz(q2)
//   T_b_e = T_b_1 * Trans(l2, 0, 0)
//   => p_e = Rz(q1) (l1, 0, 0) + Rz(q1 + q2) (l2, 0, 0) = (l1 c1 + l2 c12, l1 s1 + l2 s12, 0)
//      R_b_e = Rz(q1 + q2)
// ---------------------------------------------------------------------------------------------
constexpr double kL1 = 1.0;
constexpr double kL2 = 0.7;

Model<2> planar_model() {
  Model<2> m;
  m.joints[0].name = "j1";
  m.joints[0].T_parent_joint = Pose{};  // identity
  m.joints[0].axis_joint = Vec3::UnitZ();
  m.joints[1].name = "j2";
  m.joints[1].T_parent_joint = Pose{Mat3::Identity(), Vec3(kL1, 0.0, 0.0)};
  m.joints[1].axis_joint = Vec3::UnitZ();
  m.T_last_tip = Pose{Mat3::Identity(), Vec3(kL2, 0.0, 0.0)};
  for (auto& j : m.joints) {
    j.q_min = -kPi;
    j.q_max = kPi;
    j.qd_max = 3.0;
    j.tau_max = 10.0;
  }
  return m;
}

Mat3 rz(double a) {
  return Eigen::AngleAxisd(a, Vec3::UnitZ()).toRotationMatrix();
}

VecN<2> random_q2(std::mt19937_64& rng) {
  std::uniform_real_distribution<double> u(-kPi, kPi);
  return VecN<2>(u(rng), u(rng));
}

TEST(Planar2Link, ForwardKinematicsMatchesClosedForm) {
  const Model<2> m = planar_model();
  std::mt19937_64 rng(7);
  for (int k = 0; k < 100; ++k) {
    const VecN<2> q = random_q2(rng);
    FkState<2> fk;
    forward_kinematics(m, q, fk);
    const double c1 = std::cos(q(0));
    const double s1 = std::sin(q(0));
    const double c12 = std::cos(q(0) + q(1));
    const double s12 = std::sin(q(0) + q(1));
    const Vec3 p_expected(kL1 * c1 + kL2 * c12, kL1 * s1 + kL2 * s12, 0.0);
    ASSERT_LT((fk.T_b_e.p - p_expected).norm(), 1e-12) << "q = " << q.transpose();
    ASSERT_LT((fk.T_b_e.R - rz(q(0) + q(1))).cwiseAbs().maxCoeff(), 1e-12);

    // Per-joint state: joint 1 at the origin, joint 2 at (c1, s1, 0); both axes are +z in b.
    EXPECT_LT(fk.p_b_joint[0].norm(), 1e-12);
    EXPECT_LT((fk.p_b_joint[1] - Vec3(kL1 * c1, kL1 * s1, 0.0)).norm(), 1e-12);
    EXPECT_LT((fk.axis_b[0] - Vec3::UnitZ()).norm(), 1e-12);
    EXPECT_LT((fk.axis_b[1] - Vec3::UnitZ()).norm(), 1e-12);
    // T_b_i is the link frame after the rotation q_i: R_b_0 = Rz(q1), R_b_1 = Rz(q1 + q2).
    EXPECT_LT((fk.T_b_i[0].R - rz(q(0))).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT((fk.T_b_i[1].R - rz(q(0) + q(1))).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT((fk.T_b_i[1].p - fk.p_b_joint[1]).norm(), 1e-12);
  }
}

// d p_e / d q1 = (-l1 s1 - l2 s12, l1 c1 + l2 c12, 0), d p_e / d q2 = (-l2 s12, l2 c12, 0);
// angular rows are z for both columns. Cross-check with the column formula
// z x (p_e - p_i): z x (x, y, 0) = (-y, x, 0); joint 1 p = 0, joint 2 p = l1 (c1, s1, 0) gives
// the same two columns.
TEST(Planar2Link, JacobianMatchesClosedForm) {
  const Model<2> m = planar_model();
  std::mt19937_64 rng(11);
  for (int k = 0; k < 100; ++k) {
    const VecN<2> q = random_q2(rng);
    FkState<2> fk;
    forward_kinematics(m, q, fk);
    Mat6N<2> J;
    geometric_jacobian(fk, J);
    const double c1 = std::cos(q(0));
    const double s1 = std::sin(q(0));
    const double c12 = std::cos(q(0) + q(1));
    const double s12 = std::sin(q(0) + q(1));
    Mat6N<2> J_expected;
    J_expected << -kL1 * s1 - kL2 * s12, -kL2 * s12,  //
        kL1 * c1 + kL2 * c12, kL2 * c12,              //
        0.0, 0.0,                                     //
        0.0, 0.0,                                     //
        0.0, 0.0,                                     //
        1.0, 1.0;
    ASSERT_LT((J - J_expected).cwiseAbs().maxCoeff(), 1e-12) << "q = " << q.transpose();
  }
}

// With qdd = 0, differentiate p_e twice:
//   xdot = -l1 s1 qd1 - l2 s12 (qd1 + qd2)        ydot = l1 c1 qd1 + l2 c12 (qd1 + qd2)
//   xddot = -l1 c1 qd1^2 - l2 c12 (qd1 + qd2)^2   yddot = -l1 s1 qd1^2 - l2 s12 (qd1 + qd2)^2
// The angular velocity is (0, 0, qd1 + qd2), constant when qdd = 0, so the angular part is 0.
TEST(Planar2Link, JdotQdotMatchesClosedForm) {
  const Model<2> m = planar_model();
  std::mt19937_64 rng(13);
  std::uniform_real_distribution<double> uqd(-3.0, 3.0);
  for (int k = 0; k < 100; ++k) {
    const VecN<2> q = random_q2(rng);
    const VecN<2> qd(uqd(rng), uqd(rng));
    FkState<2> fk;
    forward_kinematics(m, q, fk);
    const Vec6 a = jdot_qdot(m, fk, qd);
    const double c1 = std::cos(q(0));
    const double s1 = std::sin(q(0));
    const double c12 = std::cos(q(0) + q(1));
    const double s12 = std::sin(q(0) + q(1));
    const double w1 = qd(0);
    const double w12 = qd(0) + qd(1);
    Vec6 expected;
    expected << -kL1 * c1 * w1 * w1 - kL2 * c12 * w12 * w12,
        -kL1 * s1 * w1 * w1 - kL2 * s12 * w12 * w12, 0.0, 0.0, 0.0, 0.0;
    ASSERT_LT((a - expected).cwiseAbs().maxCoeff(), 1e-12)
        << "q = " << q.transpose() << " qd = " << qd.transpose();
  }
}

// ---------------------------------------------------------------------------------------------
// Real arm: sim/urdf/cobot30_5.urdf, base_link -> link_7, Model<7>.
// ---------------------------------------------------------------------------------------------
class ArmKinematics : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    auto chain = model::parse_urdf_chain(kRepo / "sim/urdf/cobot30_5.urdf", "base_link", "link_7");
    ASSERT_TRUE(chain.is_ok()) << chain.reason();
    chain_ = new model::UrdfChain(chain.value());
    auto m = model::make_model<7>(chain.value(), Vec3(0.0, 0.0, -9.80665));
    ASSERT_TRUE(m.is_ok()) << m.reason();
    model_ = new Model<7>(m.value());
  }
  static void TearDownTestSuite() {
    delete model_;
    delete chain_;
    model_ = nullptr;
    chain_ = nullptr;
  }
  void SetUp() override { ASSERT_NE(model_, nullptr) << "model failed to load"; }

  static const Model<7>& arm() { return *model_; }
  static const model::UrdfChain& chain() { return *chain_; }

  static VecN<7> random_q(std::mt19937_64& rng) {
    VecN<7> q;
    for (int i = 0; i < 7; ++i) {
      const auto& j = arm().joints[static_cast<std::size_t>(i)];
      std::uniform_real_distribution<double> u(j.q_min, j.q_max);
      q(i) = u(rng);
    }
    return q;
  }
  static VecN<7> random_qd(std::mt19937_64& rng) {
    VecN<7> qd;
    for (int i = 0; i < 7; ++i) {
      const double m = arm().joints[static_cast<std::size_t>(i)].qd_max;
      std::uniform_real_distribution<double> u(-m, m);
      qd(i) = u(rng);
    }
    return qd;
  }
  static FkState<7> fk_at(const VecN<7>& q) {
    FkState<7> fk;
    forward_kinematics(arm(), q, fk);
    return fk;
  }
  static Mat6N<7> jacobian_at(const VecN<7>& q) {
    const FkState<7> fk = fk_at(q);
    Mat6N<7> J;
    geometric_jacobian(fk, J);
    return J;
  }

 private:
  static Model<7>* model_;
  static model::UrdfChain* chain_;
};

Model<7>* ArmKinematics::model_ = nullptr;
model::UrdfChain* ArmKinematics::chain_ = nullptr;

// Reference FK written out in the test from the brief's recursion:
//   T_b_i = T_b_{i-1} * T_parent_joint_i * Rot(axis_i, q_i),  T_b_e = T_b_{N-1} * T_last_tip.
// The rotation is about the joint origin, so p_b_joint[i] = T_b_i.p and
// axis_b[i] = R_b_i * axis_joint_i (the axis is invariant under its own rotation).
TEST_F(ArmKinematics, ForwardKinematicsMatchesReferenceRecursion) {
  std::mt19937_64 rng(3);
  for (int k = 0; k < 100; ++k) {
    const VecN<7> q = random_q(rng);
    const FkState<7> fk = fk_at(q);
    Pose T;  // identity = T_b_{-1}
    for (std::size_t i = 0; i < 7; ++i) {
      const auto& j = arm().joints[i];
      const Pose rot{Eigen::AngleAxisd(q(static_cast<int>(i)), j.axis_joint).toRotationMatrix(),
                     Vec3::Zero()};
      T = common::compose(common::compose(T, j.T_parent_joint), rot);
      ASSERT_LT((fk.T_b_i[i].R - T.R).cwiseAbs().maxCoeff(), 1e-12) << "link " << i;
      ASSERT_LT((fk.T_b_i[i].p - T.p).norm(), 1e-12) << "link " << i;
      ASSERT_LT((fk.p_b_joint[i] - T.p).norm(), 1e-12) << "joint " << i;
      ASSERT_LT((fk.axis_b[i] - T.R * j.axis_joint).norm(), 1e-12) << "joint " << i;
      ASSERT_NEAR(fk.axis_b[i].norm(), 1.0, 1e-12) << "joint " << i;
    }
    const Pose T_b_e = common::compose(T, arm().T_last_tip);
    ASSERT_LT((fk.T_b_e.R - T_b_e.R).cwiseAbs().maxCoeff(), 1e-12);
    ASSERT_LT((fk.T_b_e.p - T_b_e.p).norm(), 1e-12);
  }
}

// At q = 0 every joint rotation is the identity, so the tip is the plain composition of the
// URDF origins taken from parse_urdf_chain (catches axis and ordering mistakes).
TEST_F(ArmKinematics, ZeroPoseEqualsComposedUrdfOrigins) {
  ASSERT_EQ(chain().joints.size(), 7U);
  const FkState<7> fk = fk_at(VecN<7>::Zero());
  Pose T;
  for (std::size_t i = 0; i < 7; ++i) {
    T = common::compose(T, chain().joints[i].T_parent_joint);
    EXPECT_LT((fk.p_b_joint[i] - T.p).norm(), 1e-12) << "joint " << i;
    EXPECT_LT((fk.axis_b[i] - T.R * chain().joints[i].axis_joint).norm(), 1e-12) << "joint " << i;
  }
  T = common::compose(T, chain().T_last_tip);
  EXPECT_LT((fk.T_b_e.p - T.p).norm(), 1e-12);
  EXPECT_LT((fk.T_b_e.R - T.R).cwiseAbs().maxCoeff(), 1e-12);
  // joint_1 <origin xyz="0 0 0.159" rpy="0 0 0"> in base_link, axis z: hand-checked anchor.
  EXPECT_LT((fk.p_b_joint[0] - Vec3(0.0, 0.0, 0.159)).norm(), 1e-12);
  EXPECT_LT((fk.axis_b[0] - Vec3::UnitZ()).norm(), 1e-12);
}

// Column j of J vs central differences of FK with h = 1e-6:
//   v_j = (p(q + h e_j) - p(q - h e_j)) / 2h
//   w_j = so3_log(R(q + h e_j) R(q - h e_j)^T) / 2h
// (R(q +- h e_j) = exp(+-w_j h) R(q) + O(h^2), so R+ R-^T = exp(2 w_j h) + O(h^3)).
// Truncation ~ h^2 = 1e-12, rounding ~ 1e-16 / h = 1e-10, both far below the 1e-6 tolerance.
TEST_F(ArmKinematics, JacobianMatchesCentralDifferences) {
  constexpr double kH = 1e-6;
  std::mt19937_64 rng(7);
  for (int k = 0; k < 1000; ++k) {
    const VecN<7> q = random_q(rng);
    const Mat6N<7> J = jacobian_at(q);
    for (int c = 0; c < 7; ++c) {
      VecN<7> qp = q;
      VecN<7> qm = q;
      qp(c) += kH;
      qm(c) -= kH;
      const FkState<7> fp = fk_at(qp);
      const FkState<7> fm = fk_at(qm);
      Vec6 num;
      num.head<3>() = (fp.T_b_e.p - fm.T_b_e.p) / (2.0 * kH);
      num.tail<3>() = so3_log(fp.T_b_e.R * fm.T_b_e.R.transpose()) / (2.0 * kH);
      const Vec6 col = J.col(c);
      const double err = (col - num).norm();
      const double scale = num.norm();
      if (scale < 1e-6) {
        ASSERT_LT(err, 1e-8) << "sample " << k << " column " << c;
      } else {
        ASSERT_LT(err / scale, 1e-6) << "sample " << k << " column " << c << "\nJ   "
                                     << col.transpose() << "\nnum " << num.transpose();
      }
    }
  }
}

// jdot_qdot vs d/dt [J(q + qd t) qd] at t = 0, central difference with h = 1e-6. With qdd = 0
// the tip acceleration is J qdd + Jdot qd = Jdot qd, so the two must agree.
TEST_F(ArmKinematics, JdotQdotMatchesCentralDifferences) {
  constexpr double kH = 1e-6;
  std::mt19937_64 rng(17);
  for (int k = 0; k < 200; ++k) {
    const VecN<7> q = random_q(rng);
    const VecN<7> qd = random_qd(rng);
    const FkState<7> fk = fk_at(q);
    const Vec6 a = jdot_qdot(arm(), fk, qd);
    const VecN<7> qp = q + qd * kH;
    const VecN<7> qm = q - qd * kH;
    const Vec6 num = (jacobian_at(qp) * qd - jacobian_at(qm) * qd) / (2.0 * kH);
    const double err = (a - num).norm();
    const double scale = num.norm();
    if (scale < 1e-6) {
      ASSERT_LT(err, 1e-8) << "sample " << k;
    } else {
      ASSERT_LT(err / scale, 1e-5) << "sample " << k << "\nanalytic " << a.transpose()
                                   << "\nnumeric  " << num.transpose();
    }
  }
}

// Jdot(q, qd) qd is a quadratic form in qd: zero at qd = 0, and scaling qd by s scales it by s^2.
TEST_F(ArmKinematics, JdotQdotIsQuadraticInQdot) {
  std::mt19937_64 rng(19);
  for (int k = 0; k < 50; ++k) {
    const VecN<7> q = random_q(rng);
    const VecN<7> qd = random_qd(rng);
    const FkState<7> fk = fk_at(q);
    const VecN<7> zero = VecN<7>::Zero();
    EXPECT_LT(jdot_qdot(arm(), fk, zero).norm(), 1e-14);
    const Vec6 a1 = jdot_qdot(arm(), fk, qd);
    const VecN<7> qd2 = -2.0 * qd;  // s = -2, s^2 = 4
    const Vec6 a2 = jdot_qdot(arm(), fk, qd2);
    EXPECT_LT((a2 - 4.0 * a1).norm(), 1e-12 * std::max(1.0, a1.norm()));
  }
}

// The design requires a home away from singularities. Home from config/default.yaml:
// home_q_deg = (0, 30, 0, -90, 0, 60, 0). Mixed units (m and rad rows) as in J_e_b itself.
// If this fails, report it; do not change the thresholds.
TEST_F(ArmKinematics, HomePoseIsWellConditioned) {
  const auto cfg = pipeline::load_config<7>(kRepo / "config/default.yaml");
  ASSERT_TRUE(cfg.is_ok()) << cfg.reason();
  const VecN<7> q_home = cfg.value().home_q_rad;
  VecN<7> expected_deg;
  expected_deg << 0.0, 30.0, 0.0, -90.0, 0.0, 60.0, 0.0;
  ASSERT_LT((q_home - expected_deg * kDeg).cwiseAbs().maxCoeff(), 1e-12);

  const Mat6N<7> J = jacobian_at(q_home);
  const Eigen::JacobiSVD<Mat6N<7>> svd(J);
  const Vec6 s = svd.singularValues();  // descending, 6 values
  EXPECT_GT(s(5), 0.05) << "singular values " << s.transpose();
  EXPECT_LT(s(0) / s(5), 100.0) << "singular values " << s.transpose();
}

// Hot path: no heap allocation in FK, J and Jdot qd (EIGEN_RUNTIME_NO_MALLOC is defined for
// tests, so an Eigen allocation asserts while malloc is disallowed).
TEST_F(ArmKinematics, NoHeapAllocation) {
  std::mt19937_64 rng(23);
  const VecN<7> q = random_q(rng);
  const VecN<7> qd = random_qd(rng);
  FkState<7> fk;
  Mat6N<7> J;
  Vec6 a = Vec6::Zero();
  Eigen::internal::set_is_malloc_allowed(false);
  forward_kinematics(arm(), q, fk);
  geometric_jacobian(fk, J);
  a = jdot_qdot(arm(), fk, qd);
  Eigen::internal::set_is_malloc_allowed(true);
  EXPECT_TRUE(a.allFinite());
  EXPECT_TRUE(J.allFinite());
}

}  // namespace
}  // namespace cobot::kinematics
