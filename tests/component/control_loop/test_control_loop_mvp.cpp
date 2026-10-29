#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/lie.hpp"
#include "cobot/common/timestamp.hpp"
#include "cobot/kinematics/kinematics.hpp"
#include "cobot/planning/cartesian_trajectory.hpp"
#include "cobot/planning/clik.hpp"

#include "control_loop_test_util.hpp"

// MVP trajectory (design G3) through the whole loop: ControlLoop + SimJointInterface + an
// ideal SimPlant with the same model as the controller, tests/e2e/configs/ideal_world.yaml
// (config/default.yaml without friction compensation) unchanged.
namespace cobot::pipeline::test {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::int64_t kStepNs = 1000000;  // 1 / 1000 Hz = 1 ms

// G3: final tip position < 2 mm, orientation < 1 deg, per-joint RMS(q_ref - q) < 0.01 rad.
constexpr double kFinalPosTolM = 0.002;
constexpr double kFinalRotTolRad = kPi / 180.0;  // 1 deg = 0.017453292519943295 rad
constexpr double kRmsTolRad = 0.01;

// Step k runs at t = k ms. duration 10 s + hold 2 s = 12 s, i.e. k = 12000; ten more steps
// make sure the loop has switched to kFinished.
constexpr int kFinishedStep = 12000;
constexpr int kMvpSteps = kFinishedStep + 11;
constexpr int kDrainEvery = 100;

kinematics::FkState<kN> fk_of(const model::Arm7& m, const Vec& q) {
  kinematics::FkState<kN> fk;
  kinematics::forward_kinematics(m, q, fk);
  return fk;
}

double rot_err(const Mat3& R_desired, const Mat3& R_actual) {
  const Mat3 R_err = R_desired * R_actual.transpose();
  return common::so3_log(R_err).norm();
}

void expect_mat_near(const Mat3& a, const Mat3& b, double tol, const char* what, int k) {
  EXPECT_LT((a - b).cwiseAbs().maxCoeff(), tol) << what << " at step " << k;
}

// Roll, pitch, yaw of R = Rz(yaw) Ry(pitch) Rx(roll) (|pitch| < pi / 2).
Vec3 rpy_of(const Mat3& R) {
  Vec3 rpy;
  rpy << std::atan2(R(2, 1), R(2, 2)), -std::asin(R(2, 0)), std::atan2(R(1, 0), R(0, 0));
  return rpy;
}

struct LoopRun {
  std::vector<Record> records;
  std::vector<State> states;
  std::vector<Command> commands;
  std::vector<bool> ok;
  std::size_t dropped = 0;
  LoopMode final_mode = LoopMode::kIdle;
};

// Runs `steps` host cycles from home, popping the ring every kDrainEvery steps.
LoopRun run_loop(const Setup& setup, const Config<kN>& config, int steps) {
  LoopRun run;
  run.records.reserve(static_cast<std::size_t>(steps));
  run.states.reserve(static_cast<std::size_t>(steps));
  run.commands.reserve(static_cast<std::size_t>(steps));
  const std::unique_ptr<Rig> rig = make_rig(setup, config);
  EXPECT_TRUE(rig->start().is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kTracking);
  for (int k = 0; k < steps; ++k) {
    State s;
    Command cmd;
    run.ok.push_back(rig->cycle(s, cmd).is_ok());
    run.states.push_back(s);
    run.commands.push_back(cmd);
    if ((k + 1) % kDrainEvery == 0) {
      drain(rig->loop, run.records);
    }
  }
  drain(rig->loop, run.records);
  run.dropped = rig->loop.log().dropped();
  run.final_mode = rig->loop.mode();
  return run;
}

TEST(ControlLoopMvp, TracksTheMvpTrajectoryWithinG3) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Config<kN>& cfg = setup->config;
  ASSERT_TRUE(cfg.deterministic);
  ASSERT_EQ(cfg.control_rate_hz, 1000.0);
  ASSERT_EQ(cfg.trajectory.duration_s, 10.0);
  ASSERT_EQ(cfg.trajectory.hold_s, 2.0);

  const LoopRun run = run_loop(*setup, cfg, kMvpSteps);
  EXPECT_EQ(run.final_mode, LoopMode::kFinished);
  EXPECT_EQ(run.dropped, 0U);
  ASSERT_EQ(run.records.size(), static_cast<std::size_t>(kMvpSteps));

  const Pose goal = goal_pose(cfg);
  const Vec tau_max = tau_max_of(setup->model);
  Vec sum_sq = Vec::Zero();
  for (int k = 0; k < kMvpSteps; ++k) {
    const auto ku = static_cast<std::size_t>(k);
    const Record& r = run.records[ku];
    EXPECT_TRUE(run.ok[ku]) << "step " << k;
    // Plant time k * 1 ms stamped in integer ns. 12000 additions of 0.001 s accumulate at most
    // 12000 * ulp(12) / 2 = 12000 * 8.9e-16 s = 1.1e-11 s << 0.5 ns, so the rounding is exact.
    ASSERT_EQ(r.t.ns, static_cast<std::int64_t>(k) * kStepNs) << "step " << k;
    if (k < kFinishedStep) {
      EXPECT_EQ(r.mode, LoopMode::kTracking) << "step " << k;
    } else if (k > kFinishedStep) {
      EXPECT_EQ(r.mode, LoopMode::kFinished) << "step " << k;
    }
    EXPECT_EQ(r.step_time_us, 0.0) << "deterministic: step_us is 0, step " << k;
    EXPECT_TRUE(r.tau.allFinite()) << "step " << k;
    EXPECT_TRUE((r.tau.cwiseAbs().array() <= tau_max.array()).all()) << "step " << k;
    const Vec e = r.q_ref - r.q;
    sum_sq += e.cwiseProduct(e);
  }
  // RMS over the whole run, per joint: sqrt(sum e^2 / n), n = kMvpSteps.
  const Vec rms = (sum_sq / static_cast<double>(kMvpSteps)).cwiseSqrt();
  for (int i = 0; i < kN; ++i) {
    EXPECT_LT(rms(i), kRmsTolRad) << "joint " << i + 1;
  }

  // Final tip error from FK of the last measured q against the configured goal.
  const kinematics::FkState<kN> fk = fk_of(setup->model, run.records.back().q);
  EXPECT_LT((goal.p - fk.T_b_e.p).norm(), kFinalPosTolM);
  EXPECT_LT(rot_err(goal.R, fk.T_b_e.R), kFinalRotTolRad);
}

// Every record carries the state it was computed from, the command that was written, FK of
// the measured q, the trajectory sample and the CLIK reference of that step. The reference
// is recomputed here with an independent CartesianTrajectory + ClikReference fed the same
// t = seconds_between(t_start, state.t) and dt = 1 / control_rate_hz.
TEST(ControlLoopMvp, RecordsMatchStateCommandFkAndReference) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Config<kN>& cfg = setup->config;
  constexpr int kSteps = 2000;  // 2 s, mid-trajectory
  const LoopRun run = run_loop(*setup, cfg, kSteps);
  ASSERT_EQ(run.records.size(), static_cast<std::size_t>(kSteps));

  const Vec q0 = run.states.front().q;
  EXPECT_EQ(q0, cfg.home_q_rad);
  const planning::CartesianTrajectory traj(
      fk_of(setup->model, q0).T_b_e, goal_pose(cfg), cfg.trajectory.duration_s);
  planning::ClikReference<kN> shadow(setup->model, cfg.clik);
  shadow.reset(q0);
  const double dt = 1.0 / cfg.control_rate_hz;
  const common::Timestamp t0 = run.states.front().t;

  constexpr double kTol = 1e-8;
  for (int k = 0; k < kSteps; ++k) {
    const auto ku = static_cast<std::size_t>(k);
    const Record& r = run.records[ku];
    const State& s = run.states[ku];
    ASSERT_EQ(r.t, s.t) << "step " << k;
    EXPECT_EQ(r.mode, LoopMode::kTracking) << "step " << k;
    EXPECT_EQ(r.q, s.q) << "step " << k;
    EXPECT_EQ(r.qd, s.qd) << "step " << k;
    EXPECT_EQ(r.tau, run.commands[ku].tau) << "step " << k;

    const kinematics::FkState<kN> fk = fk_of(setup->model, s.q);
    EXPECT_LT((r.T_b_e.p - fk.T_b_e.p).norm(), 1e-12) << "step " << k;
    expect_mat_near(r.T_b_e.R, fk.T_b_e.R, 1e-12, "T_b_e.R", k);

    const planning::CartesianSample x_d = traj.sample(common::seconds_between(t0, s.t));
    const planning::JointReference<kN>& ref = shadow.step(x_d, dt);
    EXPECT_LT((r.T_b_d.p - x_d.T_b_d.p).norm(), kTol) << "step " << k;
    expect_mat_near(r.T_b_d.R, x_d.T_b_d.R, kTol, "T_b_d.R", k);
    EXPECT_LT((r.q_ref - ref.q).cwiseAbs().maxCoeff(), kTol) << "step " << k;
    EXPECT_LT((r.qd_ref - ref.qd).cwiseAbs().maxCoeff(), kTol) << "step " << k;
    EXPECT_LT((r.task_error - ref.task_error).cwiseAbs().maxCoeff(), kTol) << "step " << k;
  }
}

// The first desired pose is FK(initial q); after duration_s it is the configured goal with
// R = Rz(yaw) Ry(pitch) Rx(roll). The goal rotates about all three axes so a wrong RPY order
// shows, and stays close to the start (2.4 cm, about 0.07 rad) so tracking is easy.
TEST(ControlLoopMvp, DesiredPoseGoesFromStartFkToTheConfiguredGoal) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Config<kN> cfg = setup->config;
  const kinematics::FkState<kN> fk0 = fk_of(setup->model, cfg.home_q_rad);
  cfg.trajectory.duration_s = 0.5;
  cfg.trajectory.hold_s = 0.1;
  cfg.trajectory.goal_position_m = fk0.T_b_e.p + Vec3(0.01, -0.01, 0.02);
  cfg.trajectory.goal_rpy_rad = rpy_of(fk0.T_b_e.R) + Vec3(0.04, -0.03, 0.05);
  constexpr int kSteps = 700;  // 0.7 s > 0.5 s + 0.1 s
  const LoopRun run = run_loop(*setup, cfg, kSteps);
  ASSERT_EQ(run.records.size(), static_cast<std::size_t>(kSteps));
  EXPECT_EQ(run.final_mode, LoopMode::kFinished);

  const Record& first = run.records.front();
  EXPECT_LT((first.T_b_d.p - fk0.T_b_e.p).norm(), 1e-12);
  expect_mat_near(first.T_b_d.R, fk0.T_b_e.R, 1e-12, "T_b_d.R", 0);
  const Pose goal = goal_pose(cfg);
  const Record& last = run.records.back();
  EXPECT_LT((last.T_b_d.p - goal.p).norm(), 1e-12);
  expect_mat_near(last.T_b_d.R, goal.R, 1e-12, "T_b_d.R", kSteps - 1);
}

template <typename M>
bool same_bits(const M& a, const M& b) {
  const auto n = static_cast<std::size_t>(a.size());
  return std::memcmp(a.data(), b.data(), n * sizeof(double)) == 0;
}

TEST(ControlLoopMvp, TwoRunsGiveBitIdenticalRecords) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  constexpr int kSteps = 3000;
  const LoopRun a = run_loop(*setup, setup->config, kSteps);
  const LoopRun b = run_loop(*setup, setup->config, kSteps);
  ASSERT_EQ(a.records.size(), static_cast<std::size_t>(kSteps));
  ASSERT_EQ(b.records.size(), a.records.size());
  for (std::size_t k = 0; k < a.records.size(); ++k) {
    const Record& x = a.records[k];
    const Record& y = b.records[k];
    ASSERT_EQ(x.t, y.t) << "step " << k;
    ASSERT_EQ(x.mode, y.mode) << "step " << k;
    ASSERT_TRUE(same_bits(x.q, y.q)) << "q, step " << k;
    ASSERT_TRUE(same_bits(x.qd, y.qd)) << "qd, step " << k;
    ASSERT_TRUE(same_bits(x.q_ref, y.q_ref)) << "q_ref, step " << k;
    ASSERT_TRUE(same_bits(x.qd_ref, y.qd_ref)) << "qd_ref, step " << k;
    ASSERT_TRUE(same_bits(x.tau, y.tau)) << "tau, step " << k;
    ASSERT_TRUE(same_bits(x.T_b_e.R, y.T_b_e.R)) << "T_b_e.R, step " << k;
    ASSERT_TRUE(same_bits(x.T_b_e.p, y.T_b_e.p)) << "T_b_e.p, step " << k;
    ASSERT_TRUE(same_bits(x.T_b_d.R, y.T_b_d.R)) << "T_b_d.R, step " << k;
    ASSERT_TRUE(same_bits(x.T_b_d.p, y.T_b_d.p)) << "T_b_d.p, step " << k;
    ASSERT_TRUE(same_bits(x.task_error, y.task_error)) << "task_error, step " << k;
    ASSERT_EQ(x.step_time_us, 0.0) << "step " << k;
    ASSERT_EQ(y.step_time_us, 0.0) << "step " << k;
  }
}

// deterministic: false measures step() with steady_clock: finite, non-negative, not all 0.
TEST(ControlLoopMvp, NonDeterministicModeMeasuresStepTime) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  Config<kN> cfg = setup->config;
  cfg.deterministic = false;
  constexpr int kSteps = 200;
  const LoopRun run = run_loop(*setup, cfg, kSteps);
  ASSERT_EQ(run.records.size(), static_cast<std::size_t>(kSteps));
  double max_us = 0.0;
  for (const Record& r : run.records) {
    EXPECT_TRUE(std::isfinite(r.step_time_us));
    EXPECT_GE(r.step_time_us, 0.0);
    max_us = std::max(max_us, r.step_time_us);
  }
  EXPECT_GT(max_us, 0.0);
}

}  // namespace
}  // namespace cobot::pipeline::test
