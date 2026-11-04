// planning::jacobian_condition_number (the former anonymous condition_number<N>(J) of clik.cpp,
// made public by feature manual-commands) and ClikReference::condition_number(): the condition
// number computed in the last step(), NaN before the first step, kept by reset().
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <random>

#include <Eigen/SVD>
#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/kinematics/kinematics.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/planning/cartesian_trajectory.hpp"
#include "cobot/planning/clik.hpp"

namespace cobot::planning {
namespace {

namespace fs = std::filesystem;
using Vec7 = common::VecN<7>;
using Jac = common::Mat6N<7>;

struct Arm {
  model::Model<7> model;
  pipeline::Config<7> cfg;
};

Arm load_arm() {
  const auto cfg = pipeline::load_config<7>(fs::path(COBOT_REPO_DIR) / "config/default.yaml");
  EXPECT_TRUE(cfg) << cfg.reason();
  const auto chain =
      model::parse_urdf_chain(cfg.value().urdf_path, cfg.value().base_link, cfg.value().tip_link);
  EXPECT_TRUE(chain) << chain.reason();
  const auto arm = model::make_model<7>(chain.value(), cfg.value().gravity_b);
  EXPECT_TRUE(arm) << arm.reason();
  return Arm{arm.value(), cfg.value()};
}

Jac jacobian_at(const model::Model<7>& m, const Vec7& q) {
  kinematics::FkState<7> fk;
  kinematics::forward_kinematics(m, q, fk);
  Jac J;
  kinematics::geometric_jacobian(fk, J);
  return J;
}

// sigma_max / sigma_min of a JacobiSVD, the reference.
double svd_ratio(const Jac& J) {
  const Eigen::JacobiSVD<Jac> svd(J);
  const auto& sv = svd.singularValues();
  return sv(0) / sv(sv.size() - 1);
}

// 50 random configurations inside the URDF limits, mt19937_64(5).
TEST(ClikCondition, EqualsTheSvdRatioOnRandomJacobians) {
  const Arm arm = load_arm();
  std::mt19937_64 rng(5);
  for (int k = 0; k < 50; ++k) {
    Vec7 q;
    for (int j = 0; j < 7; ++j) {
      const model::JointParams& p = arm.model.joints[static_cast<std::size_t>(j)];
      std::uniform_real_distribution<double> u(p.q_min, p.q_max);
      q(j) = u(rng);
    }
    const Jac J = jacobian_at(arm.model, q);
    const double expected = svd_ratio(J);
    ASSERT_TRUE(std::isfinite(expected)) << "sample " << k;
    const double got = jacobian_condition_number<7>(J);
    EXPECT_NEAR(got, expected, 1e-12 * expected) << "sample " << k;
    EXPECT_GE(got, 1.0) << "sample " << k;
  }
}

// Rank 5 (a zero row): sigma_min = 0 -> infinite. A zero matrix too.
TEST(ClikCondition, InfiniteForARankDeficientJacobian) {
  const Arm arm = load_arm();
  Jac J = jacobian_at(arm.model, arm.cfg.home_q_rad);
  J.row(4).setZero();
  const double c = jacobian_condition_number<7>(J);
  EXPECT_TRUE(std::isinf(c)) << c;
  EXPECT_GT(c, 0.0);
  const double zero = jacobian_condition_number<7>(Jac::Zero());
  EXPECT_TRUE(std::isinf(zero)) << zero;
}

// NaN before the first step; after one step the value of J(q_ref) at the start of that step
// (the q of reset()); reset() keeps the value of the last step.
TEST(ClikCondition, ClikReferenceReportsTheLastStepsConditionNumber) {
  const Arm arm = load_arm();
  ClikReference<7> clik(arm.model, arm.cfg.clik);
  Vec7 q = arm.cfg.home_q_rad;
  q(5) = 10.0 * 3.14159265358979323846 / 180.0;  // closer to the wrist singularity
  EXPECT_TRUE(std::isnan(clik.condition_number()));
  clik.reset(q);
  EXPECT_TRUE(std::isnan(clik.condition_number()));

  kinematics::FkState<7> fk;
  kinematics::forward_kinematics(arm.model, q, fk);
  CartesianSample x_d;
  x_d.T_b_d = fk.T_b_e;
  x_d.T_b_d.p += common::Vec3(0.01, 0.0, 0.0);  // some task error so that q_ref moves
  clik.step(x_d, 1e-3);
  const double c1 = jacobian_condition_number<7>(jacobian_at(arm.model, q));
  EXPECT_DOUBLE_EQ(clik.condition_number(), c1);
  EXPECT_NEAR(clik.condition_number(), svd_ratio(jacobian_at(arm.model, q)), 1e-12 * c1);

  // The second step uses the q_ref the first one produced.
  const Vec7 q1 = clik.reference().q;
  ASSERT_GT((q1 - q).norm(), 0.0);
  clik.step(x_d, 1e-3);
  EXPECT_DOUBLE_EQ(clik.condition_number(),
                   jacobian_condition_number<7>(jacobian_at(arm.model, q1)));
  const double c2 = clik.condition_number();

  clik.reset(arm.cfg.home_q_rad);
  EXPECT_DOUBLE_EQ(clik.condition_number(), c2);
}

}  // namespace
}  // namespace cobot::planning
