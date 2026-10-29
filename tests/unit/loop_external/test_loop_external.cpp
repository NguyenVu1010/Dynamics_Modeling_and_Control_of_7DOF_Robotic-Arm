#include <algorithm>
#include <cstddef>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/lie.hpp"

#include "loop_external_util.hpp"

// ControlLoop external command mode (feature `loop-external`, design 6.3): config mode has no
// standby, external start holds the measured pose in kStandby, load() is accepted only in
// standby from the reference pose, and the end of an external trajectory returns to standby at
// the last waypoint. Real arm, ideal SimPlant (tests/e2e/configs/ideal_world.yaml).
namespace cobot::pipeline::test {
namespace {

// Two waypoints from the start pose to the MVP goal: a pass-through point halfway (position
// and rotation) after 5 s, then the goal after another 5 s, stopping there.
std::vector<Waypoint> mvp_two_waypoints(const Pose& start, const Pose& goal) {
  const Vec3 phi = common::so3_log(start.R.transpose() * goal.R);
  const Mat3 r_mid = start.R * common::so3_exp(0.5 * phi);
  const Pose mid{r_mid, 0.5 * (start.p + goal.p)};
  return {waypoint(mid, 5.0, false), waypoint(goal, 5.0, true)};
}

// ------------------------------------------------------------------ config mode

// Default command_source is config: start() tracks the configured trajectory, load() is an
// error, and the whole MVP run (10 s + 2 s hold) never shows a standby row.
TEST(LoopExternalConfigMode, ConfigModeHasNoStandby) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  ASSERT_EQ(setup->config.command_source, CommandSource::kConfig);
  const std::unique_ptr<Rig> rig = make_rig(*setup, setup->config);
  ASSERT_TRUE(rig->start().is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kTracking);

  const Pose home = fk_pose(setup->model, setup->config.home_q_rad);
  const auto traj = make_trajectory(home, {waypoint(short_goal(home), 1.0, true)});
  ASSERT_TRUE(traj);
  EXPECT_FALSE(rig->loop.load(*traj).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kTracking);

  // 12000 steps reach t = 12 s = duration + hold; 11 more make sure kFinished is reached.
  constexpr int kSteps = 12011;
  std::vector<Record> records;
  records.reserve(kSteps);
  for (int k = 0; k < kSteps; ++k) {
    State s;
    Command cmd;
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok()) << "step " << k;
    ASSERT_NE(rig->loop.mode(), LoopMode::kStandby) << "step " << k;
    if ((k + 1) % 100 == 0) {
      drain(rig->loop, records);
    }
  }
  drain(rig->loop, records);
  EXPECT_EQ(rig->loop.mode(), LoopMode::kFinished);
  ASSERT_EQ(records.size(), static_cast<std::size_t>(kSteps));
  for (std::size_t k = 0; k < records.size(); ++k) {
    ASSERT_NE(records[k].mode, LoopMode::kStandby) << "record " << k;
  }
}

// ------------------------------------------------------------------ external start / standby

// start() in external mode: kStandby at FK(initial.q); the arm holds that pose for 2 s within
// 1e-4 m, every row is a standby row whose desired pose is the start pose.
TEST(LoopExternalStandby, StartHoldsTheMeasuredPoseFor2s) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = make_rig(*setup, external_config(*setup));
  EXPECT_EQ(rig->loop.mode(), LoopMode::kIdle);
  ASSERT_TRUE(rig->start().is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kStandby);

  const Pose home = fk_pose(setup->model, setup->config.home_q_rad);
  EXPECT_LT(pos_dist(rig->loop.reference_pose(), home), 1e-12);
  EXPECT_LT(rot_dist(rig->loop.reference_pose(), home), 1e-12);
  EXPECT_EQ(rig->loop.segment(), 0U);

  constexpr int kSteps = 2000;  // 2 s at 1 kHz
  std::vector<Record> records;
  records.reserve(kSteps);
  double worst = 0.0;
  for (int k = 0; k < kSteps; ++k) {
    State s;
    Command cmd;
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok()) << "step " << k;
    ASSERT_EQ(rig->loop.mode(), LoopMode::kStandby) << "step " << k;
    ASSERT_TRUE(torque_ok(setup->model, cmd.tau)) << "step " << k;
    worst = std::max(worst, pos_dist(fk_pose(setup->model, s.q), home));
    if ((k + 1) % 100 == 0) {
      drain(rig->loop, records);
    }
  }
  drain(rig->loop, records);
  EXPECT_LT(worst, 1e-4);
  EXPECT_LT(pos_dist(rig->loop.reference_pose(), home), 1e-12);
  ASSERT_EQ(records.size(), static_cast<std::size_t>(kSteps));
  for (std::size_t k = 0; k < records.size(); ++k) {
    ASSERT_EQ(records[k].mode, LoopMode::kStandby) << "record " << k;
    ASSERT_LT(pos_dist(records[k].T_b_d, home), 1e-12) << "record " << k;
    ASSERT_TRUE(torque_ok(setup->model, records[k].tau)) << "record " << k;
  }
}

// Safety checks run in standby too: a state inside the soft margin -> kHolding, and load() is
// rejected there.
TEST(LoopExternalStandby, SoftMarginInStandbyHolds) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = make_rig(*setup, external_config(*setup));
  ASSERT_TRUE(rig->start().is_ok());
  State s;
  Command cmd;
  for (int k = 0; k < 10; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  }
  ASSERT_EQ(rig->loop.mode(), LoopMode::kStandby);
  const double q1_max = q_max_of(setup->model)(0);
  const double margin = setup->config.safety.soft_margin_rad;
  EXPECT_TRUE(rig->cycle(s, cmd, [&](State& x) { x.q(0) = q1_max - 0.5 * margin; }).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kHolding);
  EXPECT_TRUE(torque_ok(setup->model, cmd.tau));

  const Pose ref = rig->loop.reference_pose();
  const auto traj = make_trajectory(ref, {waypoint(short_goal(ref), 1.0, true)});
  ASSERT_TRUE(traj);
  EXPECT_FALSE(rig->loop.load(*traj).is_ok());
  for (int k = 0; k < 10; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
    ASSERT_EQ(rig->loop.mode(), LoopMode::kHolding);
  }
}

// ------------------------------------------------------------------ load

// load() before start (kIdle) is an error.
TEST(LoopExternalLoad, LoadBeforeStartIsRejected) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  ControlLoop<kN> loop(setup->model, external_config(*setup));
  const Pose home = fk_pose(setup->model, setup->config.home_q_rad);
  const auto traj = make_trajectory(home, {waypoint(short_goal(home), 1.0, true)});
  ASSERT_TRUE(traj);
  EXPECT_FALSE(loop.load(*traj).is_ok());
  EXPECT_EQ(loop.mode(), LoopMode::kIdle);
}

// From standby: accepted, tracking from the next step; a second load while tracking is
// rejected with "load: not in standby (mode tracking)" and the running trajectory goes on.
// After it ends the loop is in standby again and accepts the next trajectory.
TEST(LoopExternalLoad, LoadFromStandbyOkFromTrackingRejected) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = make_rig(*setup, external_config(*setup));
  ASSERT_TRUE(rig->start().is_ok());
  State s;
  Command cmd;
  for (int k = 0; k < 10; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  }
  const Pose ref = rig->loop.reference_pose();
  const Pose goal = short_goal(ref);
  const auto traj = make_trajectory(ref, {waypoint(goal, 1.0, true)});
  ASSERT_TRUE(traj);
  ASSERT_TRUE(rig->loop.load(*traj).is_ok());
  ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kTracking);

  const common::Status again = rig->loop.load(*traj);
  ASSERT_FALSE(again.is_ok());
  EXPECT_NE(again.reason().find("load: not in standby"), std::string::npos) << again.reason();
  EXPECT_NE(again.reason().find("tracking"), std::string::npos) << again.reason();
  EXPECT_EQ(rig->loop.mode(), LoopMode::kTracking);

  // 1 s trajectory: back in standby well within 1.2 s.
  int k = 0;
  for (; k < 1200 && rig->loop.mode() == LoopMode::kTracking; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  }
  ASSERT_EQ(rig->loop.mode(), LoopMode::kStandby) << "after " << k << " steps";
  EXPECT_LT(pos_dist(rig->loop.reference_pose(), goal), 1e-9);

  const auto back = make_trajectory(rig->loop.reference_pose(), {waypoint(ref, 1.0, true)});
  ASSERT_TRUE(back);
  EXPECT_TRUE(rig->loop.load(*back).is_ok());
  ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kTracking);
}

// The trajectory must start at the standby reference within 1e-4 m and 1.745e-4 rad (0.01 deg).
TEST(LoopExternalLoad, StartPoseOffTheReferenceIsRejected) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = make_rig(*setup, external_config(*setup));
  ASSERT_TRUE(rig->start().is_ok());
  State s;
  Command cmd;
  for (int k = 0; k < 10; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  }
  const Pose ref = rig->loop.reference_pose();

  // 1 mm off in x: 1e-3 m > 1e-4 m.
  Pose off_pos = ref;
  off_pos.p.x() += 1e-3;
  const auto t_pos = make_trajectory(off_pos, {waypoint(short_goal(ref), 1.0, true)});
  ASSERT_TRUE(t_pos);
  const common::Status st_pos = rig->loop.load(*t_pos);
  EXPECT_FALSE(st_pos.is_ok());
  EXPECT_NE(st_pos.reason().find("load"), std::string::npos) << st_pos.reason();

  // 3.5e-4 rad (0.02 deg) about z: > 1.745e-4 rad.
  Pose off_rot = ref;
  off_rot.R = ref.R * common::so3_exp(Vec3(0.0, 0.0, 3.5e-4));
  const auto t_rot = make_trajectory(off_rot, {waypoint(short_goal(ref), 1.0, true)});
  ASSERT_TRUE(t_rot);
  const common::Status st_rot = rig->loop.load(*t_rot);
  EXPECT_FALSE(st_rot.is_ok());
  EXPECT_NE(st_rot.reason().find("load"), std::string::npos) << st_rot.reason();

  // Rejected loads change nothing.
  for (int k = 0; k < 10; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
    ASSERT_EQ(rig->loop.mode(), LoopMode::kStandby);
  }
  EXPECT_LT(pos_dist(rig->loop.reference_pose(), ref), 1e-12);

  // Inside both tolerances: 5e-5 m < 1e-4 m and 8e-5 rad < 1.745e-4 rad -> accepted.
  Pose near = ref;
  near.p.x() += 5e-5;
  near.R = ref.R * common::so3_exp(Vec3(0.0, 8e-5, 0.0));
  const auto t_near = make_trajectory(near, {waypoint(short_goal(ref), 1.0, true)});
  ASSERT_TRUE(t_near);
  EXPECT_TRUE(rig->loop.load(*t_near).is_ok());
  ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kTracking);
}

// ------------------------------------------------------------------ end of the trajectory

// Two segments of 5 s from home to the MVP goal. While tracking: segment() follows
// trajectory.segment_at(trajectory_time_s()) and the time advances by 1 ms per step. At
// t = 10 s the loop goes to kStandby (no hold_s, never kFinished) with the reference exactly
// at the last waypoint, and the ideal plant is there within 1e-3 m.
TEST(LoopExternalEnd, EndOfTrajectoryIsStandbyAtTheLastWaypoint) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = make_rig(*setup, external_config(*setup));
  ASSERT_TRUE(rig->start().is_ok());
  State s;
  Command cmd;
  for (int k = 0; k < 5; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  }
  const Pose start = rig->loop.reference_pose();
  const Pose goal = goal_pose(setup->config);
  const auto traj = make_trajectory(start, mvp_two_waypoints(start, goal));
  ASSERT_TRUE(traj);
  ASSERT_NEAR(traj->duration_s(), 10.0, 1e-12);  // user durations 5 s + 5 s
  ASSERT_TRUE(rig->loop.load(*traj).is_ok());

  int tracking_steps = 0;
  double t_prev = -1.0;
  std::set<std::size_t> segments;
  for (int k = 0; k < 10500 && rig->loop.mode() != LoopMode::kStandby; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok()) << "step " << k;
    ASSERT_TRUE(torque_ok(setup->model, cmd.tau)) << "step " << k;
    const LoopMode m = rig->loop.mode();
    ASSERT_TRUE(m == LoopMode::kTracking || m == LoopMode::kStandby) << "step " << k;
    if (m == LoopMode::kTracking) {
      ++tracking_steps;
      const double t = rig->loop.trajectory_time_s();
      ASSERT_EQ(rig->loop.segment(), traj->segment_at(t)) << "t = " << t;
      if (t < traj->duration_s()) {
        segments.insert(rig->loop.segment());
      }
      if (t_prev >= 0.0) {
        ASSERT_NEAR(t - t_prev, kDt, 1e-9) << "t = " << t;
      }
      t_prev = t;
    }
  }
  ASSERT_EQ(rig->loop.mode(), LoopMode::kStandby);
  // 10 s at 1 ms per step = 10000 tracking steps, give or take the boundary step.
  EXPECT_GE(tracking_steps, 9998);
  EXPECT_LE(tracking_steps, 10002);
  EXPECT_EQ(segments, (std::set<std::size_t>{0, 1}));

  EXPECT_LT(pos_dist(rig->loop.reference_pose(), goal), 1e-9);
  EXPECT_LT(rot_dist(rig->loop.reference_pose(), goal), 1e-9);
  EXPECT_LT(pos_dist(fk_pose(setup->model, s.q), goal), 1e-3);

  // Stays there: 1 s more in standby.
  for (int k = 0; k < 1000; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
    ASSERT_EQ(rig->loop.mode(), LoopMode::kStandby) << "step " << k;
  }
  EXPECT_LT(pos_dist(rig->loop.reference_pose(), goal), 1e-9);
  EXPECT_LT(pos_dist(fk_pose(setup->model, s.q), goal), 1e-3);
}

}  // namespace
}  // namespace cobot::pipeline::test
