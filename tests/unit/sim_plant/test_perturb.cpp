#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include "cobot/imperfect/config.hpp"
#include "cobot/imperfect/perturb.hpp"
#include "cobot/model/model.hpp"

namespace cobot::imperfect {
namespace {

using common::Mat3;
using common::Vec3;
using V2 = VecN<2>;
using M2 = model::Model<2>;

// Two links with full (non-diagonal) inertia tensors and a tip origin away from the last
// joint, so the payload merge is visible on every component.
M2 two_link() {
  M2 m;
  m.joints[1].T_parent_joint.p = Vec3(0.3, 0.0, 0.1);
  for (auto& j : m.joints) {
    j.damping = 0.01;
  }
  m.links[0].mass_kg = 3.0;
  m.links[0].com_i = Vec3(0.1, -0.02, 0.05);
  m.links[0].I_com_i << 0.05, 0.001, -0.002, 0.001, 0.04, 0.003, -0.002, 0.003, 0.03;
  m.links[1].mass_kg = 1.5;
  m.links[1].com_i = Vec3(0.02, 0.01, -0.1);
  m.links[1].I_com_i << 0.02, -0.001, 0.0005, -0.001, 0.025, 0.002, 0.0005, 0.002, 0.01;
  m.T_last_tip.p = Vec3(0.1, -0.05, 0.2);
  return m;
}

void expect_same_link(const model::LinkInertia& a, const model::LinkInertia& b) {
  EXPECT_EQ(a.mass_kg, b.mass_kg);
  EXPECT_TRUE(a.com_i == b.com_i);
  EXPECT_TRUE(a.I_com_i == b.I_com_i);
}

TEST(PerturbModel, ZeroMismatchIsBitIdentical) {
  const M2 nominal = two_link();
  const M2 out = perturb_model(nominal, ModelMismatchConfig<2>{}, 42);
  for (int k = 0; k < 2; ++k) {
    expect_same_link(out.links[k], nominal.links[k]);
    EXPECT_TRUE(out.joints[k].T_parent_joint.R == nominal.joints[k].T_parent_joint.R);
    EXPECT_TRUE(out.joints[k].T_parent_joint.p == nominal.joints[k].T_parent_joint.p);
    EXPECT_EQ(out.joints[k].damping, nominal.joints[k].damping);
  }
  EXPECT_TRUE(out.T_last_tip.p == nominal.T_last_tip.p);
  EXPECT_TRUE(out.gravity_b == nominal.gravity_b);
}

TEST(PerturbModel, MassScaleReproducibleAndKeepsInertiaRatio) {
  const M2 nominal = two_link();
  ModelMismatchConfig<2> mm;
  mm.link_mass_scale_std = V2::Constant(0.1);
  const M2 a = perturb_model(nominal, mm, 5);
  const M2 b = perturb_model(nominal, mm, 5);
  const M2 c = perturb_model(nominal, mm, 6);
  bool seed_matters = false;
  for (int k = 0; k < 2; ++k) {
    expect_same_link(a.links[k], b.links[k]);
    seed_matters = seed_matters || a.links[k].mass_kg != c.links[k].mass_kg;
    const double s = a.links[k].mass_kg / nominal.links[k].mass_kg;
    EXPECT_NE(s, 1.0) << k;
    EXPECT_GE(s, 0.1) << k;
    // I_new / m_new == I_nom / m_nom  <=>  I_new == s I_nom
    const Mat3 diff = a.links[k].I_com_i - s * nominal.links[k].I_com_i;
    EXPECT_LT(diff.cwiseAbs().maxCoeff(), 1e-12 * nominal.links[k].I_com_i.norm()) << k;
    EXPECT_TRUE(a.links[k].com_i == nominal.links[k].com_i) << k;  // com_shift_std = 0
  }
  EXPECT_TRUE(seed_matters);
}

TEST(PerturbModel, ZeroStdLinkUntouchedAndScaleClampedAtTenth) {
  const M2 nominal = two_link();
  ModelMismatchConfig<2> mm;
  mm.link_mass_scale_std << 0.0, 10.0;  // P(1 + N(0, 10) < 0.1) = 0.46: clamp hit often
  int clamped = 0;
  for (std::uint64_t seed = 0; seed < 50; ++seed) {
    const M2 out = perturb_model(nominal, mm, seed);
    expect_same_link(out.links[0], nominal.links[0]);
    const double s = out.links[1].mass_kg / nominal.links[1].mass_kg;
    EXPECT_GE(s, 0.1 - 1e-15) << seed;
    clamped += std::abs(s - 0.1) < 1e-15 ? 1 : 0;
  }
  EXPECT_GT(clamped, 0);
}

TEST(PerturbModel, PayloadMergedIntoLastLink) {
  const M2 nominal = two_link();
  ModelMismatchConfig<2> mm;
  mm.payload_kg = 1.0;
  const M2 out = perturb_model(nominal, mm, 3);
  expect_same_link(out.links[0], nominal.links[0]);
  const model::LinkInertia& n = nominal.links[1];
  const model::LinkInertia& p = out.links[1];
  EXPECT_NEAR(p.mass_kg, 2.5, 1e-12);  // 1.5 + 1
  // c = (1.5 (0.02, 0.01, -0.1) + 1 (0.1, -0.05, 0.2)) / 2.5
  //   = ((0.03 + 0.1), (0.015 - 0.05), (-0.15 + 0.2)) / 2.5 = (0.052, -0.014, 0.02)
  EXPECT_LT((p.com_i - Vec3(0.052, -0.014, 0.02)).cwiseAbs().maxCoeff(), 1e-12);
  // Parallel axis to the new CoM: d0 = c_old - c = (-0.032, 0.024, -0.12),
  // dp = tip - c = (0.048, -0.036, 0.18); check 1.5 d0 + 1 dp = 0.
  const Vec3 d0(-0.032, 0.024, -0.12);
  const Vec3 dp(0.048, -0.036, 0.18);
  const Mat3 eye = Mat3::Identity();
  const Mat3 expected = n.I_com_i + 1.5 * (d0.squaredNorm() * eye - d0 * d0.transpose()) +
                        1.0 * (dp.squaredNorm() * eye - dp * dp.transpose());
  EXPECT_LT((p.I_com_i - expected).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(PerturbModel, ComShiftStatistics) {
  const M2 nominal = two_link();
  ModelMismatchConfig<2> mm;
  mm.com_shift_std_m = 0.01;
  Vec3 sum = Vec3::Zero();
  Vec3 sum_sq = Vec3::Zero();
  const double n = 2000.0 * 2.0;  // samples per axis: seeds x links
  for (std::uint64_t seed = 0; seed < 2000; ++seed) {
    const M2 out = perturb_model(nominal, mm, seed);
    for (int k = 0; k < 2; ++k) {
      EXPECT_EQ(out.links[k].mass_kg, nominal.links[k].mass_kg);
      const Vec3 d = out.links[k].com_i - nominal.links[k].com_i;
      sum += d;
      sum_sq += d.cwiseProduct(d);
    }
  }
  for (int ax = 0; ax < 3; ++ax) {
    const double mean = sum[ax] / n;
    const double std_dev = std::sqrt(sum_sq[ax] / n - mean * mean);
    // standard error of the mean: 0.01 / sqrt(4000) = 1.6e-4; of the std: 1.1 %
    EXPECT_LT(std::abs(mean), 6e-4) << ax;
    EXPECT_NEAR(std_dev, 0.01, 0.05 * 0.01) << ax;
  }
}

TEST(PlantEffects, CopiesFrictionAndMotorLag) {
  ImperfectionConfig<2> cfg;
  cfg.friction.coulomb_nm << 0.3, 0.7;
  cfg.friction.viscous_nm_s_rad << 0.05, 0.2;
  cfg.actuator.motor_time_constant_s << 0.002, 0.004;
  const dynamics::PlantEffects<2> fx = plant_effects(cfg);
  EXPECT_TRUE(fx.coulomb_nm == cfg.friction.coulomb_nm);
  EXPECT_TRUE(fx.viscous_nm_s_rad == cfg.friction.viscous_nm_s_rad);
  EXPECT_TRUE(fx.motor_time_constant_s == cfg.actuator.motor_time_constant_s);
}

}  // namespace
}  // namespace cobot::imperfect
