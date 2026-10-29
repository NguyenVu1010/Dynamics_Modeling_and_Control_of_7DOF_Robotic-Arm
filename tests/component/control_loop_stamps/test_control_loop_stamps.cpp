#include <memory>

#include <gtest/gtest.h>

#include "cobot/common/timestamp.hpp"

#include "control_loop_test_util.hpp"

// HAL freshness (control_loop.hpp): the same stamp repeated for longer than hal_timeout at
// control_rate_hz -> hold while the HAL is still frozen; a stamp going backwards -> hold.
// config/default.yaml: 1 kHz, hal_timeout 5 ms, so the 6th repeat (6 ms > 5 ms) trips it.
namespace cobot::pipeline::test {
namespace {

TEST(ControlLoopStamps, FrozenStampsHoldAfterTheTimeoutWhileStillFrozen) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  ASSERT_NEAR(setup->config.control_rate_hz, 1000.0, 1e-12);
  ASSERT_NEAR(setup->config.safety.hal_timeout_s, 0.005, 1e-15);
  const std::unique_ptr<Rig> rig = make_rig(*setup, setup->config);
  ASSERT_TRUE(rig->start().is_ok());
  State s;
  Command cmd;
  for (int k = 0; k < 200; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  }
  const common::Timestamp t_frozen = s.t;
  int repeats_to_hold = 0;
  for (int k = 1; k <= 20; ++k) {
    rig->cycle(s, cmd, [&](State& x) { x.t = t_frozen; });
    EXPECT_TRUE(torque_ok(setup->model, cmd.tau));
    if (rig->loop.mode() == LoopMode::kHolding) {
      repeats_to_hold = k;
      break;
    }
    ASSERT_EQ(rig->loop.mode(), LoopMode::kTracking) << "repeat " << k;
  }
  // Not before 5 ms of repeats, and within one more cycle after the timeout.
  EXPECT_GE(repeats_to_hold, 5);
  EXPECT_LE(repeats_to_hold, 6);
}

TEST(ControlLoopStamps, StampGoingBackwardsHolds) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = make_rig(*setup, setup->config);
  ASSERT_TRUE(rig->start().is_ok());
  State s;
  Command cmd;
  for (int k = 0; k < 200; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  }
  ASSERT_EQ(rig->loop.mode(), LoopMode::kTracking);
  const common::Timestamp t_last = s.t;
  rig->cycle(s, cmd, [&](State& x) { x.t = common::Timestamp{t_last.ns - 1}; });
  EXPECT_EQ(rig->loop.mode(), LoopMode::kHolding);
  EXPECT_TRUE(torque_ok(setup->model, cmd.tau));
  // The hold is kept once fresh stamps come back.
  for (int k = 0; k < 10; ++k) {
    rig->cycle(s, cmd);
  }
  EXPECT_EQ(rig->loop.mode(), LoopMode::kHolding);
}

}  // namespace
}  // namespace cobot::pipeline::test
