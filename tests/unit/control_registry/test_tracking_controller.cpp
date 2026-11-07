#include <random>

#include <gtest/gtest.h>

#include "cobot/control/computed_torque.hpp"
#include "cobot/control/pd_gravity.hpp"
#include "cobot/control/tracking_controller.hpp"

#include "registry_util.hpp"

// TrackingController switches on the algorithm and returns that law's command unchanged: the
// same bits as calling the law directly (ADR-023 "Hệ quả" 3, bit-identity of computed torque).
namespace cobot::control {
namespace {

using registry_test::kDof;
using registry_test::load_arm_setup;
using registry_test::random_pair;
using registry_test::Ref7;
using registry_test::State7;

constexpr int kPairs = 200;

// The gains of config/default.yaml (computed torque) and a distinct PD+G set.
ControllerConfig<kDof> make_config(const pipeline::Config<kDof>& c, Algorithm a) {
  ControllerConfig<kDof> cc;
  cc.algorithm = a;
  cc.computed_torque = c.computed_torque;
  cc.pd_gravity.kp.setConstant(2500.0);
  cc.pd_gravity.kd.setConstant(100.0);
  cc.q_inertia = c.home_q_rad;
  return cc;
}

bool same_bits(const JointCommand<kDof>& a, const JointCommand<kDof>& b) {
  return registry_test::bits_equal(a.tau, b.tau);
}

TEST(TrackingController, DefaultsToComputedTorque) {
  const ControllerConfig<kDof> cc;
  EXPECT_EQ(cc.algorithm, Algorithm::kComputedTorque);
  EXPECT_TRUE(cc.q_inertia.isZero(0.0));
}

TEST(TrackingController, ComputedTorqueDispatchIsBitIdentical) {
  const auto arm = load_arm_setup();
  ASSERT_TRUE(arm);
  const ControllerConfig<kDof> cc = make_config(arm->config, Algorithm::kComputedTorque);
  const TrackingController<kDof> tc(arm->nominal, cc);
  EXPECT_EQ(tc.algorithm(), Algorithm::kComputedTorque);
  const ComputedTorque<kDof> ct(arm->nominal, cc.computed_torque);
  std::mt19937_64 rng(23);
  for (int k = 0; k < kPairs; ++k) {
    State7 s;
    Ref7 ref;
    random_pair(arm->nominal, rng, s, ref);
    const JointCommand<kDof> got = tc.compute(s, ref);
    const JointCommand<kDof> want = ct.compute(s, ref);
    EXPECT_TRUE(same_bits(got, want))
        << "pair " << k << ": " << got.tau.transpose() << " vs " << want.tau.transpose();
  }
}

TEST(TrackingController, PdGravityDispatch) {
  const auto arm = load_arm_setup();
  ASSERT_TRUE(arm);
  const ControllerConfig<kDof> cc = make_config(arm->config, Algorithm::kPdGravity);
  const TrackingController<kDof> tc(arm->nominal, cc);
  EXPECT_EQ(tc.algorithm(), Algorithm::kPdGravity);
  const PdGravity<kDof> pd(arm->nominal, cc.pd_gravity, cc.q_inertia);
  const ComputedTorque<kDof> ct(arm->nominal, cc.computed_torque);
  std::mt19937_64 rng(23);
  int differs_from_ct = 0;
  for (int k = 0; k < kPairs; ++k) {
    State7 s;
    Ref7 ref;
    random_pair(arm->nominal, rng, s, ref);
    const JointCommand<kDof> got = tc.compute(s, ref);
    const JointCommand<kDof> want = pd.compute(s, ref);
    EXPECT_TRUE(same_bits(got, want))
        << "pair " << k << ": " << got.tau.transpose() << " vs " << want.tau.transpose();
    differs_from_ct += same_bits(got, ct.compute(s, ref)) ? 0 : 1;
  }
  EXPECT_GT(differs_from_ct, kPairs / 2) << "the selected law must really be PD+G";
}

}  // namespace
}  // namespace cobot::control
