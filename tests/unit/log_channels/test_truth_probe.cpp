// pipeline::SimTruthProbe (feature log-channels, contract 4): the plant state the bench hands to
// ControlLoop::step as PlantTruth, sampled after the decorator's read(). q, qd, q_motor, tau_motor
// (= tau_applied) and tau_friction (= friction_torque()) come from the plant as they are;
// qdd = (qd - qd_prev) / (time_s - time_prev) on the plant clock (an overrun lengthens the
// interval), 0 on the first call.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"

namespace cobot::pipeline {
namespace {

using common::Mat3;
using common::Vec3;
using V1 = common::VecN<1>;

constexpr double kDt = 1e-3;  // s

bool same_bits(double a, double b) {
  std::uint64_t x = 0;
  std::uint64_t y = 0;
  std::memcpy(&x, &a, sizeof(double));
  std::memcpy(&y, &b, sizeof(double));
  return x == y;
}

// One joint about z, I_zz = 1e-4 kg m^2, with Coulomb and viscous friction so that the friction
// channel is not trivially zero (the 0.5 N m drive alternates between sliding and sticking).
model::Model<1> joint() {
  model::Model<1> mdl;
  mdl.joints[0].axis_joint = Vec3::UnitZ();
  mdl.joints[0].q_min = -1000.0;
  mdl.joints[0].q_max = 1000.0;
  mdl.joints[0].tau_max = 10.0;
  mdl.links[0].mass_kg = 0.1;
  mdl.links[0].com_i = Vec3::Zero();
  mdl.links[0].I_com_i = 1e-4 * Mat3::Identity();
  return mdl;
}

dynamics::PlantEffects<1> effects() {
  dynamics::PlantEffects<1> fx;
  fx.coulomb_nm = V1::Constant(0.24);
  fx.viscous_nm_s_rad = V1::Constant(0.1);
  return fx;
}

// 100 control cycles; each advance() runs 1 + overrun_cycles plant periods (overrun_prob 1).
// Checks every sample against the plant and qdd against the finite difference over
// (1 + overrun_cycles) * dt.
void run_and_check(int overrun_cycles) {
  dynamics::SimPlant<1> plant(joint(), V1::Zero(), effects());
  SimJointInterface<1> io(plant, kDt, 0.0, 1, overrun_cycles > 0 ? 1.0 : 0.0, overrun_cycles);
  SimTruthProbe<1> probe(plant);
  const double interval = (1.0 + overrun_cycles) * kDt;
  double qd_prev = 0.0;
  double t_prev = 0.0;
  bool moved = false;
  for (int k = 0; k < 100; ++k) {
    hal::JointState<1> s;
    ASSERT_TRUE(io.read(s).is_ok());
    const PlantTruth<1>& t = probe.sample();
    ASSERT_TRUE(same_bits(t.q(0), plant.q()(0))) << k;
    ASSERT_TRUE(same_bits(t.qd(0), plant.qd()(0))) << k;
    ASSERT_TRUE(same_bits(t.q_motor(0), plant.q_motor()(0))) << k;
    ASSERT_TRUE(same_bits(t.tau_motor(0), plant.tau_applied()(0))) << k;
    ASSERT_TRUE(same_bits(t.tau_friction(0), plant.friction_torque()(0))) << k;
    if (k == 0) {
      EXPECT_EQ(t.qdd(0), 0.0);
    } else {
      ASSERT_NEAR(plant.time_s() - t_prev, interval, 1e-15) << "plant clock at cycle " << k;
      const double want = (plant.qd()(0) - qd_prev) / interval;
      EXPECT_NEAR(t.qdd(0), want, 1e-12 * std::max(1.0, std::abs(want))) << "cycle " << k;
      moved = moved || want != 0.0;
    }
    qd_prev = plant.qd()(0);
    t_prev = plant.time_s();
    hal::JointCommand<1> cmd;
    cmd.tau(0) = 0.5 * std::sin(0.2 * k);
    ASSERT_TRUE(io.write(cmd).is_ok());
    io.advance();
  }
  EXPECT_TRUE(moved);
}

TEST(SimTruthProbe, AccelerationIsTheVelocityDifferenceOverOnePeriod) {
  run_and_check(0);
}

TEST(SimTruthProbe, AnOverrunOfTwoCyclesDividesByThreePeriods) {
  run_and_check(2);
}

}  // namespace
}  // namespace cobot::pipeline
