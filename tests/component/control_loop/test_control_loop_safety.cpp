#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/timestamp.hpp"

#include "control_loop_test_util.hpp"

// Safety logic of ControlLoop (design 5.4 step 6, E2E-04): soft joint limits, velocity
// limits, non-finite HAL data, HAL timeout, and the idle state.
namespace cobot::pipeline::test {
namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr int kJoint4 = 3;  // zero-based index of joint_4

void expect_zero(const Vec& tau) {
  for (int i = 0; i < kN; ++i) {
    EXPECT_EQ(tau(i), 0.0) << "joint " << i + 1;
  }
}

// ------------------------------------------------------------------ soft limit (E2E-04)

// Goal far outside the workspace, in front of the home tip (home tip p = (0.110, -0.807, 0.389)
// m): CLIK stretches the arm towards it and opens the elbow, so joint_4 runs into its upper
// limit -0.4 rad (URDF range [-3.1416, -0.4]). An offline run of the same law (exact model,
// 1 kHz) reaches |q4 - q4_max| < 3 deg at t = 4.59 s with qd4 = 1.24 rad/s; the hold then
// stops joint_4 at 0.048 rad from the limit. Torque saturates on the way (CLIK near the
// stretched singularity), which the clamp must absorb.
TEST(ControlLoopSafety, UnreachableGoalHoldsBeforeJoint4Limit) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Config<kN> cfg = setup->config;
  cfg.trajectory.goal_position_m = Vec3(-1.0, -1.2, 1.2);
  const Vec q_min = q_min_of(setup->model);
  const Vec q_max = q_max_of(setup->model);
  const double margin = cfg.safety.soft_margin_rad;
  ASSERT_NEAR(margin, 0.05235987755982988, 1e-12);  // 3 deg = 3 * pi / 180 rad
  ASSERT_EQ(q_max(kJoint4), -0.4);

  const std::unique_ptr<Rig> rig = make_rig(*setup, cfg);
  ASSERT_TRUE(rig->start().is_ok());
  constexpr int kSteps = 7000;  // 7 s
  int first_hold = -1;
  double min_dist = kInf;  // smallest distance of any joint to its limit, whole run
  std::vector<Record> records;
  records.reserve(kSteps);
  for (int k = 0; k < kSteps; ++k) {
    State s;
    Command cmd;
    EXPECT_TRUE(rig->cycle(s, cmd).is_ok()) << "step " << k;
    ASSERT_TRUE(torque_ok(setup->model, cmd.tau)) << "step " << k << ": " << cmd.tau.transpose();
    const Vec dist = (s.q - q_min).cwiseMin(q_max - s.q);
    min_dist = std::min(min_dist, dist.minCoeff());
    if (first_hold < 0 && rig->loop.mode() == LoopMode::kHolding) {
      first_hold = k;
      // The state that triggered the hold: joint_4 inside the soft margin, nothing else.
      EXPECT_LT(q_max(kJoint4) - s.q(kJoint4), margin);
      for (int i = 0; i < kN; ++i) {
        if (i != kJoint4) {
          EXPECT_GE(dist(i), margin) << "joint " << i + 1;
        }
      }
    }
    if (first_hold >= 0) {
      ASSERT_EQ(rig->loop.mode(), LoopMode::kHolding) << "hold never leaves, step " << k;
    }
    if ((k + 1) % 100 == 0) {
      drain(rig->loop, records);
    }
  }
  drain(rig->loop, records);
  ASSERT_GE(first_hold, 0) << "never reached kHolding";
  // Stopped before the limit: the plant's hard stop (distance 0) is never touched.
  EXPECT_GT(min_dist, 0.01);
  EXPECT_EQ(rig->loop.log().dropped(), 0U);
  ASSERT_EQ(records.size(), static_cast<std::size_t>(kSteps));
  for (std::size_t k = 0; k < records.size(); ++k) {
    EXPECT_TRUE(torque_ok(setup->model, records[k].tau)) << "record " << k;
    if (static_cast<int>(k) > first_hold) {
      EXPECT_EQ(records[k].mode, LoopMode::kHolding) << "record " << k;
    }
  }
}

// One synthetic state inside the soft margin of joint 1 (upper side) -> kHolding.
TEST(ControlLoopSafety, StateInsideSoftMarginHolds) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = make_rig(*setup, setup->config);
  ASSERT_TRUE(rig->start().is_ok());
  const double q1_max = q_max_of(setup->model)(0);
  const double margin = setup->config.safety.soft_margin_rad;
  State s;
  Command cmd;
  EXPECT_TRUE(rig->cycle(s, cmd).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kTracking);
  EXPECT_TRUE(rig->cycle(s, cmd, [&](State& x) { x.q(0) = q1_max - 0.5 * margin; }).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kHolding);
  EXPECT_TRUE(torque_ok(setup->model, cmd.tau));
}

// Twice the soft margin from the lower limit of joint 1 is still fine.
TEST(ControlLoopSafety, StateOutsideSoftMarginKeepsTracking) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = make_rig(*setup, setup->config);
  ASSERT_TRUE(rig->start().is_ok());
  const double q1_min = q_min_of(setup->model)(0);
  const double margin = setup->config.safety.soft_margin_rad;
  State s;
  Command cmd;
  EXPECT_TRUE(rig->cycle(s, cmd, [&](State& x) { x.q(0) = q1_min + 2.0 * margin; }).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kTracking);
  EXPECT_TRUE(torque_ok(setup->model, cmd.tau));
}

// |qd_2| above qd_max_2 (2.5 rad/s) -> kHolding; 0.9 qd_max does not trigger.
TEST(ControlLoopSafety, VelocityAboveLimitHolds) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = make_rig(*setup, setup->config);
  ASSERT_TRUE(rig->start().is_ok());
  const double qd2_max = qd_max_of(setup->model)(1);
  State s;
  Command cmd;
  EXPECT_TRUE(rig->cycle(s, cmd, [&](State& x) { x.qd(1) = -0.9 * qd2_max; }).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kTracking);
  EXPECT_TRUE(rig->cycle(s, cmd, [&](State& x) { x.qd(1) = -(qd2_max + 0.1); }).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kHolding);
  EXPECT_TRUE(torque_ok(setup->model, cmd.tau));
}

// ------------------------------------------------------------------ non-finite HAL data

// Tracks for 0.5 s, then one read carries NaN in q_3: kHolding with a finite command. The hold
// target is the last finite q (from the step before); the arm moves slowly at t = 0.5 s
// (quintic s'(0.05 T) = 30 * 0.05^2 / 10 s = 0.0075 / s of a ~1 rad move), so after 2 s of
// hold with real reads it sits within 0.01 rad of that q and the mode is still kHolding.
TEST(ControlLoopSafety, NanInReadHoldsAtLastFiniteQ) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = make_rig(*setup, setup->config);
  ASSERT_TRUE(rig->start().is_ok());
  State s;
  Command cmd;
  constexpr int kTrack = 500;
  for (int k = 0; k < kTrack; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  }
  ASSERT_EQ(rig->loop.mode(), LoopMode::kTracking);
  const Vec q_last_finite = s.q;

  EXPECT_TRUE(rig->cycle(s, cmd, [](State& x) { x.q(2) = kNan; }).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kHolding);
  EXPECT_TRUE(torque_ok(setup->model, cmd.tau)) << cmd.tau.transpose();

  constexpr int kHold = 2000;
  for (int k = 0; k < kHold; ++k) {
    EXPECT_TRUE(rig->cycle(s, cmd).is_ok());
    ASSERT_TRUE(torque_ok(setup->model, cmd.tau)) << "hold step " << k;
    ASSERT_EQ(rig->loop.mode(), LoopMode::kHolding) << "hold step " << k;
  }
  EXPECT_LT((s.q - q_last_finite).cwiseAbs().maxCoeff(), 0.01) << s.q.transpose();
}

TEST(ControlLoopSafety, InfiniteVelocityInReadHolds) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = make_rig(*setup, setup->config);
  ASSERT_TRUE(rig->start().is_ok());
  State s;
  Command cmd;
  for (int k = 0; k < 10; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  }
  EXPECT_TRUE(rig->cycle(s, cmd, [](State& x) { x.qd(5) = -kInf; }).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kHolding);
  EXPECT_TRUE(torque_ok(setup->model, cmd.tau)) << cmd.tau.transpose();
}

// ------------------------------------------------------------------ HAL timeout

// Runs 500 normal steps (last fresh stamp t = 499 ms), then `frozen` steps that repeat that
// stamp while the plant keeps moving, then one step with the true stamp (499 + frozen + 1) ms.
LoopMode run_with_frozen_stamps(const Setup& setup, int frozen) {
  const std::unique_ptr<Rig> rig = make_rig(setup, setup.config);
  EXPECT_TRUE(rig->start().is_ok());
  State s;
  Command cmd;
  for (int k = 0; k < 500; ++k) {
    EXPECT_TRUE(rig->cycle(s, cmd).is_ok());
  }
  const common::Timestamp t_fresh = s.t;
  EXPECT_EQ(t_fresh.ns, 499 * 1000000LL);
  for (int k = 0; k < frozen; ++k) {
    rig->cycle(s, cmd, [&](State& x) { x.t = t_fresh; });
    EXPECT_TRUE(torque_ok(setup.model, cmd.tau));
  }
  EXPECT_TRUE(rig->cycle(s, cmd).is_ok());
  EXPECT_EQ(s.t.ns, (499 + frozen + 1) * 1000000LL);
  EXPECT_TRUE(torque_ok(setup.model, cmd.tau));
  return rig->loop.mode();
}

// Stamps frozen for 10 ms: the next fresh stamp is 11 ms after the last one > 5 ms -> hold.
TEST(ControlLoopSafety, TimestampsFrozenFor10msHold) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  ASSERT_NEAR(setup->config.safety.hal_timeout_s, 0.005, 1e-15);
  EXPECT_EQ(run_with_frozen_stamps(*setup, 10), LoopMode::kHolding);
}

// Stamps frozen for 3 ms: gap 4 ms < 5 ms -> still tracking.
TEST(ControlLoopSafety, GapBelowTimeoutKeepsTracking) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  EXPECT_EQ(run_with_frozen_stamps(*setup, 3), LoopMode::kTracking);
}

// ------------------------------------------------------------------ idle state

TEST(ControlLoopIdle, StartWithNanFailsAndStaysIdle) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  ControlLoop<kN> loop(setup->model, setup->config);
  EXPECT_EQ(loop.mode(), LoopMode::kIdle);
  State bad;
  bad.q = setup->config.home_q_rad;
  bad.q(1) = kNan;
  EXPECT_FALSE(loop.start(bad).is_ok());
  EXPECT_EQ(loop.mode(), LoopMode::kIdle);

  State bad_qd;
  bad_qd.q = setup->config.home_q_rad;
  bad_qd.qd(4) = kInf;
  EXPECT_FALSE(loop.start(bad_qd).is_ok());
  EXPECT_EQ(loop.mode(), LoopMode::kIdle);

  // step() in kIdle: error, zero torque, still idle.
  State good;
  good.q = setup->config.home_q_rad;
  good.t = common::Timestamp{1000000};
  Command cmd;
  cmd.tau.setConstant(1.0);
  EXPECT_FALSE(loop.step(good, cmd).is_ok());
  expect_zero(cmd.tau);
  EXPECT_EQ(loop.mode(), LoopMode::kIdle);
}

TEST(ControlLoopIdle, StepBeforeStartFailsWithZeroTorque) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  ControlLoop<kN> loop(setup->model, setup->config);
  State s;
  s.q = setup->config.home_q_rad;
  Command cmd;
  cmd.tau.setConstant(-3.0);
  for (int k = 0; k < 3; ++k) {
    s.t = common::Timestamp{static_cast<std::int64_t>(k) * 1000000};
    EXPECT_FALSE(loop.step(s, cmd).is_ok());
    expect_zero(cmd.tau);
    EXPECT_EQ(loop.mode(), LoopMode::kIdle);
  }
}

TEST(ControlLoopIdle, StartWithFiniteStateTracks) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  ControlLoop<kN> loop(setup->model, setup->config);
  State s;
  s.q = setup->config.home_q_rad;
  EXPECT_TRUE(loop.start(s).is_ok());
  EXPECT_EQ(loop.mode(), LoopMode::kTracking);
  Command cmd;
  EXPECT_TRUE(loop.step(s, cmd).is_ok());
  EXPECT_TRUE(torque_ok(setup->model, cmd.tau));
  EXPECT_EQ(loop.mode(), LoopMode::kTracking);
}

}  // namespace
}  // namespace cobot::pipeline::test
