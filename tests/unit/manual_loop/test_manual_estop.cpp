// ControlLoop::estop (feature manual-commands, ADR-022 "Lệnh tay", plan ambiguity 9): from
// kStandby or kTracking the next step() enters kHolding at that step's measured q with
// HoldReason::kEstop (an existing hold reason firing in the same step wins); kHolding, kFault,
// kIdle: no-op, ok; config mode: an error. reset() is the way out, as for every hold.
#include <string>

#include <gtest/gtest.h>

#include "manual_loop_util.hpp"

namespace cobot::pipeline::test {
namespace {

TEST(ManualEstop, EstopDuringAJointJogHoldsWithReasonEstop) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Host h = make_host(*setup, manual_config(*setup));
  ASSERT_TRUE(h.loop().jog_joint(0, +1, kSpeed).is_ok());
  for (int k = 0; k < 100; ++k) {
    h.step();
  }
  ASSERT_EQ(h.mode(), LoopMode::kTracking);
  ASSERT_TRUE(h.loop().estop().is_ok());
  h.step();
  EXPECT_EQ(h.rows.back().mode, LoopMode::kHolding);
  EXPECT_EQ(h.rows.back().hold_reason, HoldReason::kEstop);
  EXPECT_EQ(h.loop().hold_reason(), HoldReason::kEstop);
  // The hold target is the measured q of that step.
  EXPECT_LE((h.rows.back().q_ref - h.rows.back().q).cwiseAbs().maxCoeff(), 1e-12);
  for (int k = 0; k < 500; ++k) {
    h.step();
    ASSERT_EQ(h.mode(), LoopMode::kHolding);
  }
  EXPECT_EQ(h.rows.back().hold_reason, HoldReason::kEstop);
  // A second estop in holding: ok, nothing changes.
  EXPECT_TRUE(h.loop().estop().is_ok());
  EXPECT_EQ(h.loop().hold_reason(), HoldReason::kEstop);
  // Jogs are refused while holding.
  EXPECT_FALSE(h.loop().jog_joint(0, +1, kSpeed).is_ok());

  ASSERT_TRUE(h.loop().reset(h.s).is_ok());
  EXPECT_EQ(h.mode(), LoopMode::kStandby);
  for (int k = 0; k < 50; ++k) {
    h.step();
    ASSERT_EQ(h.mode(), LoopMode::kStandby);
  }
  EXPECT_TRUE(h.loop().jog_joint(0, -1, kSpeed).is_ok());
}

TEST(ManualEstop, EstopFromStandbyHolds) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Host h = make_host(*setup, manual_config(*setup));
  ASSERT_EQ(h.mode(), LoopMode::kStandby);
  ASSERT_TRUE(h.loop().estop().is_ok());
  h.step();
  EXPECT_EQ(h.mode(), LoopMode::kHolding);
  EXPECT_EQ(h.loop().hold_reason(), HoldReason::kEstop);
  EXPECT_EQ(h.rows.back().hold_reason, HoldReason::kEstop);
}

// Already holding for another reason (velocity spike): estop() is ok and the reason stays.
TEST(ManualEstop, EstopInHoldingKeepsTheReason) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Host h = make_host(*setup, manual_config(*setup), 10);
  const double qd2_max = qd_max_of(setup->model)(1);
  ASSERT_TRUE(h.rig->cycle(h.s, h.cmd, [&](State& x) { x.qd(1) = qd2_max + 0.1; }).is_ok());
  ASSERT_EQ(h.mode(), LoopMode::kHolding);
  ASSERT_EQ(h.loop().hold_reason(), HoldReason::kOverspeed);
  EXPECT_TRUE(h.loop().estop().is_ok());
  h.step();
  EXPECT_EQ(h.mode(), LoopMode::kHolding);
  EXPECT_EQ(h.loop().hold_reason(), HoldReason::kOverspeed);
  EXPECT_EQ(h.rows.back().hold_reason, HoldReason::kOverspeed);
}

// An existing hold reason firing in the same step wins over the estop.
TEST(ManualEstop, ExistingReasonInTheSameStepWins) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Host h = make_host(*setup, manual_config(*setup), 10);
  ASSERT_TRUE(h.loop().estop().is_ok());
  const double qd2_max = qd_max_of(setup->model)(1);
  ASSERT_TRUE(h.rig->cycle(h.s, h.cmd, [&](State& x) { x.qd(1) = qd2_max + 0.1; }).is_ok());
  EXPECT_EQ(h.mode(), LoopMode::kHolding);
  EXPECT_EQ(h.loop().hold_reason(), HoldReason::kOverspeed);
}

TEST(ManualEstop, ConfigModeEstopIsAnError) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Host h = make_host(*setup, setup->config, 10);
  ASSERT_EQ(h.mode(), LoopMode::kTracking);
  const common::Status st = h.loop().estop();
  ASSERT_FALSE(st.is_ok());
  EXPECT_TRUE(contains(st.reason(), "estop")) << st.reason();
  EXPECT_TRUE(contains(st.reason(), "config")) << st.reason();
  h.step();
  EXPECT_EQ(h.mode(), LoopMode::kTracking);
}

TEST(ManualEstop, Strings) {
  EXPECT_STREQ(to_string(HoldReason::kEstop), "estop");
  // Appended: the existing values keep their numbers.
  EXPECT_EQ(static_cast<int>(HoldReason::kOverspeed), 5);
  EXPECT_EQ(static_cast<int>(HoldReason::kEstop), 6);
  EXPECT_STREQ(to_string(ManualStop::kNone), "none");
  EXPECT_STREQ(to_string(ManualStop::kFinished), "finished");
  EXPECT_STREQ(to_string(ManualStop::kStopped), "stopped");
  EXPECT_STREQ(to_string(ManualStop::kDeadman), "deadman");
  EXPECT_STREQ(to_string(ManualStop::kSoftLimit), "soft_limit");
  EXPECT_STREQ(to_string(ManualStop::kSingularity), "singularity");
}

}  // namespace
}  // namespace cobot::pipeline::test
