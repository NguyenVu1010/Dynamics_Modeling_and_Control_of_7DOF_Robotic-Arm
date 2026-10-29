#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "loop_external_util.hpp"

// request_stop() (feature `loop-external`, design 6.3): time scaling along the loaded path.
// tau (trajectory time) advances by r dt with r = 1 - smoothstep(t / stop_time_s),
// smoothstep(x) = 3x^2 - 2x^3, until r = 0 or tau reaches the end; then kStandby at
// sample(tau).pose. Real arm, ideal SimPlant, stop_time_s = 0.5 s (default).
namespace cobot::pipeline::test {
namespace {

constexpr double kStopTimeS = 0.5;  // Config default
constexpr int kStopSteps = 500;     // 0.5 s / 1 ms

// Loads the MVP goal as one stopping waypoint of 10 s (same shape as the config-mode
// CartesianTrajectory) from the standby reference.
std::optional<WaypointTrajectory> load_mvp(Rig& rig, const Setup& setup) {
  State s;
  Command cmd;
  for (int k = 0; k < 5; ++k) {
    EXPECT_TRUE(rig.cycle(s, cmd).is_ok());
  }
  auto traj =
      make_trajectory(rig.loop.reference_pose(), {waypoint(goal_pose(setup.config), 10.0, true)});
  if (!traj) {
    return std::nullopt;
  }
  EXPECT_NEAR(traj->duration_s(), 10.0, 1e-12);
  EXPECT_TRUE(rig.loop.load(*traj).is_ok());
  return traj;
}

// Steps until the loop is tracking with trajectory_time_s() >= t_s (at most max_steps).
void track_until(Rig& rig, double t_s, int max_steps) {
  State s;
  Command cmd;
  for (int k = 0; k < max_steps; ++k) {
    EXPECT_TRUE(rig.cycle(s, cmd).is_ok());
    if (rig.loop.mode() == LoopMode::kTracking && rig.loop.trajectory_time_s() >= t_s) {
      return;
    }
  }
  ADD_FAILURE() << "trajectory time " << t_s << " s not reached in " << max_steps << " steps";
}

// Stop at t = 3 s of the 10 s MVP move (tip speed there: quintic
// s'(0.3 T) = 30 * 0.3^2 * 0.7^2 / 10 s = 0.1323 / s, times |dp| = 0.822 m -> 0.109 m/s).
//  - standby within stop_time_s + 1 ms = 501 steps, and not before r reaches 0 (~500 steps);
//  - tau advances by int_0^0.5 r dt = 0.5 (1 - int_0^1 (3x^2 - 2x^3) dx) = 0.5 (1 - 0.5)
//    = 0.25 s;
//  - every torque within tau_max, the reference moves smoothly (no jump), the reference
//    velocity |J(q_ref) qd_ref| is ~0 at the end;
//  - the final reference is on the planned path: nearest 1 ms sample < 1e-4 m (samples are
//    at most 0.154 m/s * 1 ms = 1.54e-4 m apart, so the nearest is within 7.7e-5 m).
TEST(LoopExternalStop, RequestStopMidSegmentDeceleratesAlongThePath) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  ASSERT_DOUBLE_EQ(external_config(*setup).stop_time_s, kStopTimeS);
  const std::unique_ptr<Rig> rig = make_rig(*setup, external_config(*setup));
  ASSERT_TRUE(rig->start().is_ok());
  const auto traj = load_mvp(*rig, *setup);
  ASSERT_TRUE(traj);
  track_until(*rig, 3.0, 3100);
  ASSERT_EQ(rig->loop.mode(), LoopMode::kTracking);
  const double tau0 = rig->loop.trajectory_time_s();
  ASSERT_NEAR(tau0, 3.0, 1.5e-3);
  std::vector<Record> before;
  drain(rig->loop, before);

  rig->loop.request_stop();
  EXPECT_EQ(rig->loop.mode(), LoopMode::kTracking);
  State s;
  Command cmd;
  int steps = 0;
  double tau_last = tau0;
  while (steps < 1000 && rig->loop.mode() == LoopMode::kTracking) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok()) << "stop step " << steps;
    ASSERT_TRUE(torque_ok(setup->model, cmd.tau)) << "stop step " << steps;
    ++steps;
    if (rig->loop.mode() == LoopMode::kTracking) {
      tau_last = rig->loop.trajectory_time_s();
    }
  }
  std::vector<Record> recs;
  drain(rig->loop, recs);
  ASSERT_EQ(rig->loop.mode(), LoopMode::kStandby) << "after " << steps << " steps";
  EXPECT_LE(steps, kStopSteps + 1);
  EXPECT_GE(steps, kStopSteps - 10);
  EXPECT_NEAR(tau_last - tau0, 0.25, 2e-3);

  ASSERT_EQ(recs.size(), static_cast<std::size_t>(steps));
  for (std::size_t k = 0; k < recs.size(); ++k) {
    ASSERT_TRUE(torque_ok(setup->model, recs[k].tau)) << "record " << k;
    if (k > 0) {
      ASSERT_LT(pos_dist(recs[k].T_b_d, recs[k - 1].T_b_d), 2e-4) << "record " << k;
    }
  }
  ASSERT_GE(recs.size(), 2U);
  const Record& last_tracking = recs[recs.size() - 2];
  const Record& first_standby = recs.back();
  EXPECT_EQ(last_tracking.mode, LoopMode::kTracking);
  EXPECT_EQ(first_standby.mode, LoopMode::kStandby);
  EXPECT_LT(task_speed(setup->model, last_tracking.q_ref, last_tracking.qd_ref), 1e-3);
  EXPECT_LT(task_speed(setup->model, first_standby.q_ref, first_standby.qd_ref), 1e-3);

  const Pose end = rig->loop.reference_pose();
  double nearest = std::numeric_limits<double>::infinity();
  for (int i = 0; i <= 10000; ++i) {
    nearest = std::min(nearest, pos_dist(traj->sample(i * kDt).T_b_d, end));
  }
  EXPECT_LT(nearest, 1e-4);
  EXPECT_LT(pos_dist(traj->sample(tau_last).T_b_d, end), 1e-5);
  EXPECT_LT(pos_dist(first_standby.T_b_d, end), 1e-6);

  // Then it stays in standby at that pose.
  for (int k = 0; k < 500; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
    ASSERT_EQ(rig->loop.mode(), LoopMode::kStandby) << "step " << k;
  }
  EXPECT_LT(pos_dist(rig->loop.reference_pose(), end), 1e-12);
  EXPECT_LT(pos_dist(fk_pose(setup->model, s.q), end), 1e-3);
}

struct StopRun {
  std::vector<Vec> tau;
  std::vector<LoopMode> modes;
  Pose end;
};

// MVP move, request_stop() at t = 1.5 s, 800 steps after it; `extra` repeats the request
// 100 and 300 steps later (while still stopping).
StopRun run_stop(const Setup& setup, bool extra) {
  StopRun out;
  const std::unique_ptr<Rig> rig = make_rig(setup, external_config(setup));
  EXPECT_TRUE(rig->start().is_ok());
  const auto traj = load_mvp(*rig, setup);
  EXPECT_TRUE(traj);
  track_until(*rig, 1.5, 1600);
  rig->loop.request_stop();
  State s;
  Command cmd;
  for (int k = 0; k < 800; ++k) {
    if (extra && (k == 100 || k == 300)) {
      EXPECT_EQ(rig->loop.mode(), LoopMode::kTracking) << "step " << k;
      rig->loop.request_stop();
    }
    EXPECT_TRUE(rig->cycle(s, cmd).is_ok());
    out.tau.push_back(cmd.tau);
    out.modes.push_back(rig->loop.mode());
  }
  out.end = rig->loop.reference_pose();
  return out;
}

// A second request_stop() while stopping changes nothing: identical torques, modes and end
// pose (the loop is deterministic).
TEST(LoopExternalStop, RequestStopTwiceGivesTheSameResult) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const StopRun once = run_stop(*setup, false);
  const StopRun twice = run_stop(*setup, true);
  ASSERT_EQ(once.tau.size(), twice.tau.size());
  for (std::size_t k = 0; k < once.tau.size(); ++k) {
    ASSERT_EQ(once.modes[k], twice.modes[k]) << "step " << k;
    ASSERT_EQ(once.tau[k], twice.tau[k]) << "step " << k;
  }
  EXPECT_EQ(once.modes.back(), LoopMode::kStandby);
  EXPECT_EQ(once.end.p, twice.end.p);
  EXPECT_EQ(once.end.R, twice.end.R);
  // Standby within 501 steps of the first request.
  EXPECT_EQ(once.modes[kStopSteps], LoopMode::kStandby);
}

// Stop requested ~2 ms before the end of a 1 s move: tau reaches the end first (r is still
// ~1: 1 - smoothstep(0.003 / 0.5) = 1 - 1.08e-4), so the loop is in standby within a few steps
// exactly at the final pose.
TEST(LoopExternalStop, RequestStopJustBeforeTheEndEndsAtTheFinalPose) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = make_rig(*setup, external_config(*setup));
  ASSERT_TRUE(rig->start().is_ok());
  State s;
  Command cmd;
  for (int k = 0; k < 5; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  }
  const Pose goal = short_goal(rig->loop.reference_pose());
  const auto traj = make_trajectory(rig->loop.reference_pose(), {waypoint(goal, 1.0, true)});
  ASSERT_TRUE(traj);
  const double T = traj->duration_s();
  ASSERT_NEAR(T, 1.0, 1e-12);
  ASSERT_TRUE(rig->loop.load(*traj).is_ok());
  track_until(*rig, T - 0.0025, 1100);
  ASSERT_EQ(rig->loop.mode(), LoopMode::kTracking);
  // First step with tau >= 0.9975 s: tau in [0.9975, 0.9985) s, i.e. about 2 ms before the end.
  EXPECT_LT(rig->loop.trajectory_time_s(), T - 0.0014);

  rig->loop.request_stop();
  int steps = 0;
  while (steps < 50 && rig->loop.mode() == LoopMode::kTracking) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
    ASSERT_TRUE(torque_ok(setup->model, cmd.tau));
    ++steps;
  }
  ASSERT_EQ(rig->loop.mode(), LoopMode::kStandby);
  EXPECT_LE(steps, 5);
  EXPECT_LT(pos_dist(rig->loop.reference_pose(), goal), 1e-9);
  EXPECT_LT(rot_dist(rig->loop.reference_pose(), goal), 1e-9);
  for (int k = 0; k < 500; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
    ASSERT_EQ(rig->loop.mode(), LoopMode::kStandby);
  }
  EXPECT_LT(pos_dist(fk_pose(setup->model, s.q), goal), 1e-3);
}

// request_stop() outside tracking is ignored and not remembered: in standby nothing moves,
// and a trajectory loaded afterwards runs normally (no stop ramp at its start).
TEST(LoopExternalStop, RequestStopInStandbyIsIgnored) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = make_rig(*setup, external_config(*setup));
  ASSERT_TRUE(rig->start().is_ok());
  State s;
  Command cmd;
  for (int k = 0; k < 5; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
  }
  const Pose ref = rig->loop.reference_pose();
  rig->loop.request_stop();
  for (int k = 0; k < 100; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
    ASSERT_EQ(rig->loop.mode(), LoopMode::kStandby);
  }
  EXPECT_LT(pos_dist(rig->loop.reference_pose(), ref), 1e-12);

  const auto traj = make_trajectory(ref, {waypoint(short_goal(ref), 1.0, true)});
  ASSERT_TRUE(traj);
  ASSERT_TRUE(rig->loop.load(*traj).is_ok());
  // 800 steps (0.8 s) later the 1 s move is still running at full rate: tau advanced 1 ms per
  // step (a stop ramp would have ended it in standby after ~0.5 s with tau ~0.25 s).
  for (int k = 0; k < 800; ++k) {
    ASSERT_TRUE(rig->cycle(s, cmd).is_ok());
    ASSERT_EQ(rig->loop.mode(), LoopMode::kTracking) << "step " << k;
  }
  EXPECT_NEAR(rig->loop.trajectory_time_s(), 0.8, 2.5e-3);
}

}  // namespace
}  // namespace cobot::pipeline::test
