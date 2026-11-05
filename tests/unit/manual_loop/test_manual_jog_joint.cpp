// ControlLoop::jog_joint (feature manual-commands, ADR-022 "Lệnh tay"): target velocity
// direction * speed * qd_max_j, acceleration speed * a_max_j (semi-implicit per step), other
// joints hold their q_ref exactly, deadman after deadman_s * control_rate_hz steps without a
// refresh, brake to rest at the jog limit q_max_j - (soft_margin + jog_margin) with kSoftLimit.
// External mode on the ideal plant, 1 ms steps, speed 0.1 (see manual_loop_util.hpp).
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "manual_loop_util.hpp"

namespace cobot::pipeline::test {
namespace {

// Joint 1 (index 0) +1 at 10 %: v -> 0.1 * 2.1 = 0.21 rad/s at 0.1 * 15 = 1.5 rad/s^2.
// Refreshed for 500 steps, then request_stop() (release):
//   ramp up 0.21 / 1.5 = 0.14 s: 0.5 * 1.5 * 0.14^2 = 0.0147 rad
//   cruise 0.5 - 0.14 = 0.36 s: 0.36 * 0.21 = 0.0756 rad
//   brake 0.14 s: 0.0147 rad                                   total 0.105 rad
// (discrete: sum_{k=1..140} 1.5e-3 k * 1e-3 + 360 * 0.21e-3 + sum_{k=1..139} 1.5e-3 k * 1e-3
//  = 0.014805 + 0.0756 + 0.014595 = 0.105 rad).
TEST(ManualJogJoint, RefreshedJogThenReleaseFollowsTheTrapezoid) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Host h = make_host(*setup, manual_config(*setup));
  ASSERT_EQ(h.mode(), LoopMode::kStandby);
  const Vec q0 = h.loop().joint_reference();

  ASSERT_TRUE(h.loop().jog_joint(0, +1, kSpeed).is_ok());
  step_refreshed(h, 500, [&] { EXPECT_TRUE(h.loop().jog_joint(0, +1, kSpeed).is_ok()); });
  EXPECT_EQ(h.mode(), LoopMode::kTracking);
  EXPECT_EQ(h.loop().manual_stop(), ManualStop::kNone);
  h.loop().request_stop();
  const int brake = step_until_standby(h, 1000);
  ASSERT_EQ(h.mode(), LoopMode::kStandby);
  EXPECT_LE(brake, 145);  // 140 steps of braking
  const std::size_t jog_rows = h.rows.size();
  for (int k = 0; k < 20; ++k) {
    h.step();
  }
  EXPECT_EQ(h.loop().manual_stop(), ManualStop::kStopped);
  EXPECT_EQ(h.loop().command_mode(), CommandMode::kJogJoint);

  double qd_max = 0.0;
  double qdd_max = 0.0;
  double q_last = q0(0);
  for (std::size_t i = 0; i < h.rows.size(); ++i) {
    const Record& r = h.rows[i];
    ASSERT_NE(r.mode, LoopMode::kHolding) << i;
    EXPECT_EQ(r.cmd_mode, CommandMode::kJogJoint) << i;  // standby rows after the jog too
    if (r.mode != LoopMode::kTracking) {
      continue;
    }
    qd_max = std::max(qd_max, std::abs(r.qd_ref(0)));
    qdd_max = std::max(qdd_max, std::abs(r.qdd_ref(0)));
    q_last = r.q_ref(0);
    for (int j = 1; j < kN; ++j) {
      ASSERT_EQ(r.q_ref(j), q0(j)) << "row " << i << " joint " << j + 1;  // bit-identical
      EXPECT_EQ(r.qd_ref(j), 0.0) << "row " << i << " joint " << j + 1;
    }
  }
  EXPECT_EQ(jog_rows, static_cast<std::size_t>(500 + brake));
  EXPECT_NEAR(qd_max, 0.21, 1e-9);
  EXPECT_LE(qdd_max, 1.5 + 1e-9);
  EXPECT_NEAR(q_last - q0(0), 0.105, 1e-3);
  EXPECT_NEAR(h.loop().joint_reference()(0) - q0(0), 0.105, 1e-3);
}

// One call, no refresh: the deadman (100 steps) ramps to 0. Velocity after 100 steps of 1.5e-3
// rad/s per step: 0.15 rad/s (below 0.21); brake 100 steps. Displacement
// 0.5 * 1.5 * 0.1^2 * 2 = 0.015 rad; standby after ~200 steps.
TEST(ManualJogJoint, DeadmanRampsToRestWithoutRefresh) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Host h = make_host(*setup, manual_config(*setup));
  const Vec q0 = h.loop().joint_reference();
  ASSERT_TRUE(h.loop().jog_joint(0, +1, kSpeed).is_ok());
  const int steps = step_until_standby(h, 400);
  ASSERT_EQ(h.mode(), LoopMode::kStandby);
  EXPECT_NEAR(steps, 200, 2);
  EXPECT_EQ(h.loop().manual_stop(), ManualStop::kDeadman);
  double q_last = q0(0);
  double qd_max = 0.0;
  for (const Record& r : h.rows) {
    ASSERT_NE(r.mode, LoopMode::kHolding);
    if (r.mode == LoopMode::kTracking) {
      q_last = r.q_ref(0);
      qd_max = std::max(qd_max, std::abs(r.qd_ref(0)));
    }
  }
  EXPECT_NEAR(qd_max, 0.15, 2e-3);
  EXPECT_NEAR(q_last - q0(0), 0.015, 5e-4);
  EXPECT_NEAR(h.loop().joint_reference()(0) - q0(0), 0.015, 5e-4);
}

// Joint 4 (index 3) +1 from home (q4 = -90 deg = -1.5708 rad) toward its upper limit -0.4 rad: the
// jog brakes so that q_ref comes to rest at -0.4 - (3 + 2) deg = -0.4 - 0.0872665 = -0.4872665
// rad, never trips the 3 deg hold band, and stops with kSoftLimit. Distance 1.0835 rad at 0.21
// rad/s: about 5.3 s.
TEST(ManualJogJoint, JogStopsAtTheJogBandWithSoftLimit) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Config<kN> cfg = manual_config(*setup);
  Host h = make_host(*setup, cfg);
  const double limit = q_max_of(setup->model)(3) - (cfg.safety.soft_margin_rad + 2.0 * kDegree);
  ASSERT_NEAR(limit, -0.4872665, 1e-6);
  ASSERT_NEAR(h.loop().joint_reference()(3), -0.5 * kPi, 1e-3);

  ASSERT_TRUE(h.loop().jog_joint(3, +1, kSpeed).is_ok());
  const int steps = step_until_standby(h, 6000, [&] { h.loop().jog_joint(3, +1, kSpeed); });
  ASSERT_EQ(h.mode(), LoopMode::kStandby) << "after " << steps << " steps";
  EXPECT_EQ(h.loop().manual_stop(), ManualStop::kSoftLimit);
  double q_max_seen = -10.0;
  for (const Record& r : h.rows) {
    ASSERT_NE(r.mode, LoopMode::kHolding);
    EXPECT_EQ(r.hold_reason, HoldReason::kNone);
    q_max_seen = std::max(q_max_seen, r.q_ref(3));
  }
  EXPECT_LE(q_max_seen, limit + 1e-3);
  EXPECT_NEAR(h.loop().joint_reference()(3), limit, 1e-3);
  for (int k = 0; k < 50; ++k) {
    h.step();
    ASSERT_EQ(h.mode(), LoopMode::kStandby);
  }

  const common::Status again = h.loop().jog_joint(3, +1, kSpeed);
  ASSERT_FALSE(again.is_ok());
  EXPECT_TRUE(contains(again.reason(), "soft limit")) << again.reason();
  EXPECT_EQ(h.mode(), LoopMode::kStandby);
  const double q_rest = h.loop().joint_reference()(3);
  EXPECT_TRUE(h.loop().jog_joint(3, -1, kSpeed).is_ok());
  for (int k = 0; k < 5; ++k) {
    h.step();
  }
  EXPECT_EQ(h.mode(), LoopMode::kTracking);
  EXPECT_LT(h.rows.back().q_ref(3), q_rest);  // moving away from the limit
}

TEST(ManualJogJoint, BadArgumentsAreRejected) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Host h = make_host(*setup, manual_config(*setup));
  EXPECT_FALSE(h.loop().jog_joint(7, +1, kSpeed).is_ok());   // out of range (0..6)
  EXPECT_FALSE(h.loop().jog_joint(-1, +1, kSpeed).is_ok());  // out of range
  EXPECT_FALSE(h.loop().jog_joint(0, 0, kSpeed).is_ok());    // direction 0
  EXPECT_FALSE(h.loop().jog_joint(0, 2, kSpeed).is_ok());    // direction not +-1
  EXPECT_FALSE(h.loop().jog_joint(0, +1, 0.0).is_ok());      // speed 0
  EXPECT_FALSE(h.loop().jog_joint(0, +1, 1.2).is_ok());      // speed > 1
  h.step();
  EXPECT_EQ(h.mode(), LoopMode::kStandby);
  EXPECT_EQ(h.loop().manual_stop(), ManualStop::kNone);
  EXPECT_EQ(h.loop().command_mode(), CommandMode::kWaypoints);  // nothing accepted yet
}

// A second, different jog while one runs: "release the running jog first".
TEST(ManualJogJoint, AnotherJogWhileOneRunsIsRejected) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Host h = make_host(*setup, manual_config(*setup));
  ASSERT_TRUE(h.loop().jog_joint(0, +1, kSpeed).is_ok());
  for (int k = 0; k < 10; ++k) {
    h.step();
  }
  const common::Status other = h.loop().jog_joint(1, +1, kSpeed);
  ASSERT_FALSE(other.is_ok());
  EXPECT_TRUE(contains(other.reason(), "release")) << other.reason();
  const common::Status tcp = h.loop().jog_tcp(JogAxis::kX, +1, JogFrame::kBase, kSpeed);
  ASSERT_FALSE(tcp.is_ok());
  EXPECT_TRUE(contains(tcp.reason(), "release")) << tcp.reason();
  // The running jog is unaffected and its refresh is accepted.
  EXPECT_TRUE(h.loop().jog_joint(0, +1, kSpeed).is_ok());
  h.step();
  EXPECT_EQ(h.mode(), LoopMode::kTracking);
  EXPECT_GT(h.rows.back().qd_ref(0), 0.0);
  EXPECT_EQ(h.rows.back().qd_ref(1), 0.0);
}

// A jog during a running move_joint: "not in standby".
TEST(ManualJogJoint, JogDuringAMoveIsRejected) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Config<kN> cfg = manual_config(*setup);
  Host h = make_host(*setup, cfg);
  const Vec q0 = h.loop().joint_reference();
  Vec goal = q0;
  goal(0) += 0.5;
  goal(1) -= 0.25;
  const auto plan = plan_joint_move<kN>(planner_model(*setup, cfg), cfg, q0, goal, kSpeed);
  ASSERT_TRUE(plan) << plan.reason();
  ASSERT_TRUE(h.loop().move_joint(plan.value()).is_ok());
  for (int k = 0; k < 10; ++k) {
    h.step();
  }
  ASSERT_EQ(h.mode(), LoopMode::kTracking);
  const common::Status st = h.loop().jog_joint(0, +1, kSpeed);
  ASSERT_FALSE(st.is_ok());
  EXPECT_TRUE(contains(st.reason(), "not in standby")) << st.reason();
  EXPECT_EQ(h.loop().command_mode(), CommandMode::kMoveJoint);
}

// Config mode: the manual commands are refused, rows log `config`.
TEST(ManualJogJoint, ConfigModeRefusesManualCommands) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Config<kN> cfg = setup->config;
  cfg.manual = default_manual();
  ASSERT_EQ(cfg.command_source, CommandSource::kConfig);
  Host h = make_host(*setup, cfg, 10);
  EXPECT_EQ(h.mode(), LoopMode::kTracking);
  EXPECT_FALSE(h.loop().jog_joint(0, +1, kSpeed).is_ok());
  EXPECT_FALSE(h.loop().jog_tcp(JogAxis::kX, +1, JogFrame::kBase, kSpeed).is_ok());
  h.step();
  EXPECT_EQ(h.rows.back().cmd_mode, CommandMode::kConfig);
  EXPECT_EQ(h.loop().command_mode(), CommandMode::kConfig);
}

// External mode before any command: `waypoints`.
TEST(ManualJogJoint, ExternalStandbyLogsWaypointsBeforeAnyCommand) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Host h = make_host(*setup, manual_config(*setup), 0);
  for (int k = 0; k < 5; ++k) {
    h.step();
    EXPECT_EQ(h.rows.back().cmd_mode, CommandMode::kWaypoints);
  }
  EXPECT_EQ(h.loop().command_mode(), CommandMode::kWaypoints);
  EXPECT_EQ(h.loop().manual_stop(), ManualStop::kNone);
}

}  // namespace
}  // namespace cobot::pipeline::test
