#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/imperfect/perturb.hpp"
#include "cobot/model/model.hpp"

namespace cobot::imperfect {
namespace {

using common::Mat3;
using common::Vec3;
constexpr int kN = 7;
constexpr double kDt = 1e-3;  // s
using Vec = common::VecN<kN>;

// Inner interface that always returns `state` and keeps the last command.
class ConstantInner final : public JointInterface<kN> {
 public:
  Status read(JointState<kN>& out) override {
    out = state;
    return Status::ok();
  }
  Status write(const JointCommand<kN>& cmd) override {
    last = cmd;
    return Status::ok();
  }
  JointState<kN> state;
  JointCommand<kN> last;
};

// ---------------------------------------------------------------- imperfect-io

// An all-zero config passes -0.0 through bit-exactly, also with apply_plant_effects (no lag,
// no friction configured: the torque must not become +0.0 through a "+ 0" or "- 0" term).
TEST(ImperfectIo, SignedZeroPassesThroughWithPlantEffects) {
  for (const bool plant_effects : {false, true}) {
    ConstantInner inner;
    inner.state.q.setConstant(-0.0);
    inner.state.qd.setConstant(-0.0);
    ImperfectJointInterface<kN> io(inner, ImperfectionConfig<kN>{}, kDt, plant_effects);
    for (int k = 0; k < 3; ++k) {
      JointState<kN> out;
      ASSERT_TRUE(io.read(out).is_ok());
      JointCommand<kN> cmd;
      cmd.tau.setConstant(-0.0);
      ASSERT_TRUE(io.write(cmd).is_ok());
      for (int j = 0; j < kN; ++j) {
        EXPECT_TRUE(std::signbit(out.q[j]) && out.q[j] == 0.0) << plant_effects << " j" << j;
        EXPECT_TRUE(std::signbit(out.qd[j]) && out.qd[j] == 0.0) << plant_effects << " j" << j;
        EXPECT_TRUE(std::signbit(inner.last.tau[j]) && inner.last.tau[j] == 0.0)
            << plant_effects << " j" << j;
      }
    }
  }
}

// qd = true qd + N(0, std_j) per sample (qd_from_diff = false); q stays exact.
TEST(ImperfectIo, VelocityNoiseStatistics) {
  constexpr int kSamples = 20000;
  ConstantInner inner;
  inner.state.q = Vec::LinSpaced(-0.3, 0.3);
  inner.state.qd.setConstant(0.5);
  ImperfectionConfig<kN> cfg;
  cfg.seed = 11;
  cfg.sensor.qd_noise_std_rad_s = Vec::LinSpaced(0.001, 0.007);
  ImperfectJointInterface<kN> io(inner, cfg, kDt);
  Vec sum = Vec::Zero();
  Vec sum_sq = Vec::Zero();
  for (int k = 0; k < kSamples; ++k) {
    JointState<kN> out;
    ASSERT_TRUE(io.read(out).is_ok());
    ASSERT_TRUE(out.q == inner.state.q);
    const Vec d = out.qd - inner.state.qd;
    sum += d;
    sum_sq += d.cwiseProduct(d);
  }
  for (int j = 0; j < kN; ++j) {
    const double sigma = cfg.sensor.qd_noise_std_rad_s[j];
    const double mean = sum[j] / kSamples;
    const double std = std::sqrt(sum_sq[j] / kSamples - mean * mean);
    EXPECT_LT(std::abs(mean), 4.0 * sigma / std::sqrt(double{kSamples})) << "joint " << j;
    EXPECT_NEAR(std / sigma, 1.0, 0.03) << "joint " << j;  // std estimate: 0.5 % (1 sigma)
  }
}

// ---------------------------------------------------------------- sim-plant

// One joint about z (gravity does no work), 4 kg at 0.5 m, limits +-0.5 rad.
model::Model<1> horizontal() {
  model::Model<1> m;
  m.joints[0].q_min = -0.5;
  m.joints[0].q_max = 0.5;
  m.joints[0].tau_max = 100.0;
  m.links[0].mass_kg = 4.0;
  m.links[0].com_i = Vec3(0.5, 0.0, 0.0);
  m.links[0].I_com_i = 0.01 * Mat3::Identity();
  return m;
}

TEST(SimPlant, LowerJointStopClampsPositionAndVelocity) {
  dynamics::SimPlant<1> plant(horizontal(), common::VecN<1>(0.0));
  int at_stop = 0;
  for (int k = 0; k < 1000; ++k) {  // -5 N m for 1 s: free motion would reach -2.5 rad
    plant.step(common::VecN<1>(-5.0), kDt);
    ASSERT_GE(plant.q()(0), -0.5);
    if (plant.q()(0) == -0.5) {
      ++at_stop;
      ASSERT_EQ(plant.qd()(0), 0.0) << "step " << k;
    }
  }
  EXPECT_GT(at_stop, 0);
  EXPECT_EQ(plant.q()(0), -0.5);
  // Released outward again by a positive torque.
  plant.step(common::VecN<1>(5.0), kDt);
  EXPECT_GT(plant.q()(0), -0.5);
  EXPECT_GT(plant.qd()(0), 0.0);
}

TEST(SimPlant, NegativeOrNanEffectsAreRejectedNamingTheField) {
  const auto expect_throw = [](const dynamics::PlantEffects<1>& fx, const std::string& field) {
    try {
      const dynamics::SimPlant<1> plant(horizontal(), common::VecN<1>(0.0), fx);
      ADD_FAILURE() << field << " accepted";
    } catch (const std::invalid_argument& e) {
      EXPECT_NE(std::string(e.what()).find(field + "[0]"), std::string::npos) << e.what();
    }
  };
  dynamics::PlantEffects<1> fx;
  fx.coulomb_nm(0) = -0.1;
  expect_throw(fx, "coulomb_nm");
  fx = {};
  fx.viscous_nm_s_rad(0) = -0.1;
  expect_throw(fx, "viscous_nm_s_rad");
  fx = {};
  fx.motor_time_constant_s(0) = std::nan("");
  expect_throw(fx, "motor_time_constant_s");
}

// The payload is added after the mass scale: it is not scaled itself, and it merges with the
// already scaled last link (same draws for the same seed, whatever the payload).
TEST(PerturbModel, PayloadAddedAfterMassScale) {
  model::Model<2> m;
  m.links[0].mass_kg = 3.0;
  m.links[1].mass_kg = 1.5;
  m.links[1].com_i = Vec3(0.02, 0.01, -0.1);
  m.links[1].I_com_i = Vec3(0.02, 0.025, 0.01).asDiagonal();
  m.T_last_tip.p = Vec3(0.1, -0.05, 0.2);
  ModelMismatchConfig<2> mm;
  mm.link_mass_scale_std.setConstant(0.2);
  constexpr std::uint64_t kSeed = 5;
  const model::Model<2> scaled = perturb_model<2>(m, mm, kSeed);
  ASSERT_NE(scaled.links[1].mass_kg, m.links[1].mass_kg);  // the scale did act
  mm.payload_kg = 2.0;
  const model::Model<2> loaded = perturb_model<2>(m, mm, kSeed);
  const model::LinkInertia& s = scaled.links[1];
  const model::LinkInertia& l = loaded.links[1];
  EXPECT_NEAR(l.mass_kg, s.mass_kg + 2.0, 1e-12);
  const Vec3 com = (s.mass_kg * s.com_i + 2.0 * m.T_last_tip.p) / (s.mass_kg + 2.0);
  EXPECT_LT((l.com_i - com).norm(), 1e-12);
  EXPECT_EQ(loaded.links[0].mass_kg, scaled.links[0].mass_kg);
}

}  // namespace
}  // namespace cobot::imperfect
