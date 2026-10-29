#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "cobot/control/hold.hpp"

#include "loop_external_util.hpp"

// reset(state) (feature `loop-external`, design 6.3): only from kHolding, only when every q is
// outside the soft margin, every value is finite and the stamp is fresh; then kStandby at
// FK(state.q) with CLIK reset. Real arm, ideal SimPlant.
namespace cobot::pipeline::test {
namespace {

constexpr int kJoint6 = 5;  // zero-based index of joint_6, URDF range [-0.0873, 3.8223] rad

// The arm powers up with joint_6 inside its soft margin (q6 = q6_min + margin / 2 =
// -0.0873 + 0.0262 = -0.0611 rad; margin = 3 deg = 0.0524 rad): the first step holds. reset()
// is rejected while the joint stays inside. Then something outside the loop moves the arm out
// (a joint PD of its own drives q6 to q6_min + 2 margin = -0.0873 + 0.1047 = 0.0174 rad while
// the loop keeps stepping in kHolding and its command is not applied): reset() with that state
// is accepted, the loop is in standby at FK(q) and holds that pose.
TEST(LoopExternalReset, ResetFromSoftMarginHoldRejectedInsideAcceptedOutside) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Config<kN> cfg = external_config(*setup);
  const double margin = cfg.safety.soft_margin_rad;
  const double q6_min = q_min_of(setup->model)(kJoint6);
  ASSERT_NEAR(q6_min, -0.0873, 1e-12);
  cfg.home_q_rad(kJoint6) = q6_min + 0.5 * margin;
  const std::unique_ptr<Rig> rig = make_rig(*setup, cfg);
  ASSERT_TRUE(rig->start().is_ok());

  State s;
  Command cmd;
  ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  ASSERT_EQ(rig->loop.mode(), LoopMode::kHolding);
  for (int k = 0; k < 300; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
    ASSERT_EQ(rig->loop.mode(), LoopMode::kHolding);
  }
  ASSERT_LT(s.q(kJoint6) - q6_min, margin);

  // Still inside: rejected, the cause is named.
  const common::Status inside = rig->loop.reset(s);
  ASSERT_FALSE(inside.is_ok());
  EXPECT_NE(inside.reason().find("reset"), std::string::npos) << inside.reason();
  EXPECT_NE(inside.reason().find("soft margin"), std::string::npos) << inside.reason();
  EXPECT_EQ(rig->loop.mode(), LoopMode::kHolding);

  // Move the plant out with an outside joint PD (the hold law at its own target).
  Vec q_out = cfg.home_q_rad;
  q_out(kJoint6) = q6_min + 2.0 * margin;
  control::HoldController<kN> mover(setup->model, cfg.computed_torque);
  mover.engage(q_out);
  for (int k = 0; k < 1500; ++k) {
    ASSERT_TRUE(rig->io.read(s).is_ok());
    Command ignored;
    ASSERT_TRUE(rig->loop.step(s, ignored).is_ok());
    ASSERT_EQ(rig->loop.mode(), LoopMode::kHolding) << "step " << k;
    ASSERT_TRUE(rig->io.write(mover.compute(s)).is_ok());
    rig->io.advance();
  }
  ASSERT_LT((s.q - q_out).cwiseAbs().maxCoeff(), 1e-3) << s.q.transpose();

  // s is the state of the last step: fresh, finite, every joint outside the margin.
  ASSERT_TRUE(rig->loop.reset(s).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kStandby);
  const Pose at_reset = fk_pose(setup->model, s.q);
  EXPECT_LT(pos_dist(rig->loop.reference_pose(), at_reset), 1e-12);
  EXPECT_LT(rot_dist(rig->loop.reference_pose(), at_reset), 1e-12);

  for (int k = 0; k < 1000; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
    ASSERT_EQ(rig->loop.mode(), LoopMode::kStandby) << "step " << k;
    ASSERT_TRUE(torque_ok(setup->model, cmd.tau)) << "step " << k;
  }
  EXPECT_LT(pos_dist(fk_pose(setup->model, s.q), at_reset), 1e-4);

  // And it accepts a trajectory from there.
  const Pose ref = rig->loop.reference_pose();
  const auto traj = make_trajectory(ref, {waypoint(short_goal(ref), 1.0, true)});
  ASSERT_TRUE(traj);
  EXPECT_TRUE(rig->loop.load(*traj).is_ok());
}

// Holding after a velocity spike at home (hold at the measured q, every joint far from its
// limits): a non-finite state and a stale stamp are rejected, the fresh real state is accepted.
TEST(LoopExternalReset, ResetNeedsAFiniteFreshState) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = make_rig(*setup, external_config(*setup));
  ASSERT_TRUE(rig->start().is_ok());
  State s;
  Command cmd;
  for (int k = 0; k < 10; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  }
  const double qd2_max = qd_max_of(setup->model)(1);
  ASSERT_TRUE(rig->cycle(s, cmd, [&](State& x) { x.qd(1) = qd2_max + 0.1; }).is_ok());
  ASSERT_EQ(rig->loop.mode(), LoopMode::kHolding);
  for (int k = 0; k < 200; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  }
  ASSERT_EQ(rig->loop.mode(), LoopMode::kHolding);

  State nan_q = s;
  nan_q.q(2) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(rig->loop.reset(nan_q).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kHolding);

  State inf_qd = s;
  inf_qd.qd(4) = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(rig->loop.reset(inf_qd).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kHolding);

  // 10 ms older than the newest stamp seen (hal_timeout 5 ms).
  State stale = s;
  stale.t.ns -= static_cast<std::int64_t>(10) * 1000000;
  EXPECT_FALSE(rig->loop.reset(stale).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kHolding);

  ASSERT_TRUE(rig->loop.reset(s).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kStandby);
  EXPECT_LT(pos_dist(rig->loop.reference_pose(), fk_pose(setup->model, s.q)), 1e-12);
  for (int k = 0; k < 200; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
    ASSERT_EQ(rig->loop.mode(), LoopMode::kStandby);
  }
}

// reset() is only for kHolding: rejected in kIdle, kStandby and kTracking, nothing changes.
TEST(LoopExternalReset, ResetOutsideHoldingIsRejected) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = make_rig(*setup, external_config(*setup));
  State s;
  ASSERT_TRUE(rig->io.read(s).is_ok());
  EXPECT_FALSE(rig->loop.reset(s).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kIdle);

  ASSERT_TRUE(rig->loop.start(s).is_ok());
  Command cmd;
  for (int k = 0; k < 10; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  }
  EXPECT_FALSE(rig->loop.reset(s).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kStandby);

  const Pose ref = rig->loop.reference_pose();
  const auto traj = make_trajectory(ref, {waypoint(short_goal(ref), 1.0, true)});
  ASSERT_TRUE(traj);
  ASSERT_TRUE(rig->loop.load(*traj).is_ok());
  for (int k = 0; k < 100; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  }
  ASSERT_EQ(rig->loop.mode(), LoopMode::kTracking);
  const double t_before = rig->loop.trajectory_time_s();
  EXPECT_FALSE(rig->loop.reset(s).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kTracking);
  ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  EXPECT_NEAR(rig->loop.trajectory_time_s() - t_before, kDt, 1e-9);
}

}  // namespace
}  // namespace cobot::pipeline::test
