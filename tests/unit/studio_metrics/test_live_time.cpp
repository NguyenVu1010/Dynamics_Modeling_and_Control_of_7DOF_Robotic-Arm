// Studio metrics of a run whose rows do not start at t = 0 (feature studio-live-fixes, cobot_studio
// design 5.2, metrics.hpp). A live run's rows carry the host clock (tens of seconds in), while
// meta.goal_time_s and meta.window_end_s count from the first row (live_run.hpp), as they do for
// a bench run starting at 0. Pinned:
//   - eval_row = the last row with t <= first row + window_end_s (not t <= window_end_s);
//   - the goal pose is that row's T_b_d, so the goal error, the settle time and the stop windows
//     of a run shifted in time are those of the same run starting at t = 0 (time-shift
//     invariance; only t_s moves by the shift).
// Run: rows every 1 ms for 3.0 s (3001 rows) from t_first; desired position (0, y_d, 0) with y_d a
// ramp 0 -> 0.1 m over the first 1.0 s, desired rotation Rz(0.2 rad x the same ramp); actual pose
// = desired pose shifted by err(k) along x, so the path error of row k is err(k) exactly and the
// path rotation error is 0. err: 5 mm before 1.0 s, 5 mm exp(-(t - 1) / 0.2) after (as
// test_studio_metrics.cpp). goal_time 1.0 s, window_end 3.0 s.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "cobot/common/timestamp.hpp"
#include "cobot/common/types.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/studio/metrics.hpp"
#include "cobot/studio/studio_config.hpp"

#include "metrics.hpp"
#include "run_data.hpp"
#include "synthetic_run.hpp"

namespace cobot::studio {
namespace {

using test::kMm;
using test::kRowNs;
using test::kRows;
using test::t_of;

constexpr std::int64_t kLiveStartNs = 50'000'000'000;  // 50 s into the host clock
constexpr double kRampEndS = 1.0;
constexpr double kGoalY = 0.1;           // m
constexpr double kGoalYaw = 0.2;         // rad
constexpr std::size_t kRampRows = 1000;  // rows 0 .. 999 ramp, 1000 on rest at the goal

double ramp(std::size_t k) {  // exactly 1.0 from row 1000 on
  return k >= kRampRows ? 1.0 : t_of(k) / kRampEndS;
}

double err(std::size_t k) {  // m
  const double t = t_of(k);
  return t < 1.0 ? 5.0 * kMm : 5.0 * kMm * std::exp(-(t - 1.0) / 0.2);
}

common::Mat3 rz(double yaw) {
  return Eigen::AngleAxisd(yaw, common::Vec3::UnitZ()).toRotationMatrix();
}

bench::RunMeta live_meta(double window_end_s) {
  bench::RunMeta meta = test::synthetic_meta();
  meta.goal_time_s = 1.0;
  meta.window_end_s = window_end_s;
  return meta;
}

// Two stop waypoints on the ramp (node 0.5 s, 1.0 s) and the final one: their windows are found
// from the first row too.
bench::RunMeta stop_meta() {
  bench::RunMeta meta = live_meta(3.0);
  meta.waypoint_node_times_s = {0.5, 1.0, 2.0};
  meta.waypoint_stop = {true, true, true};
  meta.waypoint_positions_m = {common::Vec3(0.0, 0.5 * kGoalY, 0.0),
                               common::Vec3(0.0, kGoalY, 0.0),
                               common::Vec3(0.0, kGoalY, 0.0)};
  return meta;
}

bench::RunData moving_run(std::int64_t t_first_ns, const bench::RunMeta& meta) {
  bench::RunData run;
  run.meta = meta;
  run.records.resize(kRows);
  for (std::size_t k = 0; k < kRows; ++k) {
    pipeline::LogRecord<test::kJoints>& r = run.records[k];
    r.t = common::Timestamp{t_first_ns + static_cast<std::int64_t>(k) * kRowNs};
    r.mode = test::default_mode(k);
    r.hold_reason = pipeline::HoldReason::kNone;
    r.T_b_d = common::Pose{};
    r.T_b_d.p = common::Vec3(0.0, kGoalY * ramp(k), 0.0);
    r.T_b_d.R = rz(kGoalYaw * ramp(k));
    r.T_b_e = r.T_b_d;
    r.T_b_e.p.x() = err(k);
  }
  // The summary as make_live_entry computes it: t_eval = first row + window_end_s.
  const std::int64_t t_eval_ns = t_first_ns + std::llround(meta.window_end_s * 1e9);
  run.summary = bench::summarize(run.records, meta.tau_max, meta.rated, t_eval_ns);
  return run;
}

TEST(StudioMetricsLiveTime, EvalRowIsTheLastRowAtOrBeforeFirstRowPlusWindowEnd) {
  const bench::RunData run = moving_run(kLiveStartNs, live_meta(3.0));
  const ErrorTrace e = make_error_trace(run);
  ASSERT_EQ(e.t_s.size(), kRows);
  // first + window_end = 50 s + 3 s = 53 s = row 3000 exactly (50e9 + 3000 * 1e6 ns).
  EXPECT_EQ(e.eval_row, 3000);
  // Goal = T_b_d of row 3000 = (0, 0.1, 0), Rz(0.2). Row 3000: actual (err, 0.1, 0), Rz(0.2):
  // goal error = err(3000) = 5 mm exp(-10) = 2.27e-7 m, rotation ~0.
  EXPECT_NEAR(e.goal_pos_m[3000], err(3000), 1e-15);
  EXPECT_NEAR(e.goal_rot_rad[3000], 0.0, 1e-12);
  // Row 1000 (on the goal from 1.0 s): error err(1000) = 5 mm.
  EXPECT_NEAR(e.goal_pos_m[1000], 5.0 * kMm, 1e-15);
  // Row 0: actual (5 mm, 0, 0), identity: |(0, 0.1, 0) - (0.005, 0, 0)| = sqrt(0.1^2 + 0.005^2)
  // = sqrt(0.010025) = 0.1001249 m; rotation 0.2 rad. (With the start pose as the goal, the
  // bug of the brief, row 0 would read 5 mm and 0 rad.)
  EXPECT_NEAR(e.goal_pos_m[0], std::sqrt(kGoalY * kGoalY + 25e-6), 1e-12);
  EXPECT_NEAR(e.goal_rot_rad[0], kGoalYaw, 1e-12);
  // Path errors do not depend on the goal: err(k), rotation 0.
  EXPECT_NEAR(e.path_pos_m[500], 5.0 * kMm, 1e-15);
  EXPECT_NEAR(e.path_rot_rad[500], 0.0, 1e-12);
}

TEST(StudioMetricsLiveTime, EvalRowBetweenRowsIsTheEarlierOne) {
  // window_end 2.9995 s: first + 2.9995 s = 52.9995 s, between rows 2999 (52.999 s) and 3000
  // (53.000 s): row 2999.
  const bench::RunData run = moving_run(kLiveStartNs, live_meta(2.9995));
  const ErrorTrace e = make_error_trace(run);
  EXPECT_EQ(e.eval_row, 2999);
  EXPECT_NEAR(e.goal_pos_m[2999], err(2999), 1e-15);
}

TEST(StudioMetricsLiveTime, LiveRunSettles) {
  const bench::RunData run = moving_run(kLiveStartNs, live_meta(3.0));
  ASSERT_FALSE(run.summary.held);
  const TrackingMetrics m = compute_metrics(run, Tolerance{});
  // t0 = first + goal_time = 51.0 s; 5 mm exp(-s / 0.2) <= 1 mm  <=>  s >= 0.2 ln 5 = 0.32189 s,
  // the first row inside is 51.322 s (within one 1 ms row, as the t = 0 test).
  ASSERT_TRUE(m.settle_time_s.has_value())
      << "settle not reached on a run whose final error is " << err(3000) << " m";
  EXPECT_NEAR(*m.settle_time_s, 0.2 * std::log(5.0), 1.5e-3);
  EXPECT_NEAR(m.path_pos_err_max_m, 5.0 * kMm, 1e-15);
  EXPECT_TRUE(m.goal_ok);  // final error 2.27e-7 m from the summary
}

void expect_same_trace(const ErrorTrace& shifted, const ErrorTrace& at_zero, double shift_s) {
  ASSERT_EQ(shifted.t_s.size(), at_zero.t_s.size());
  EXPECT_EQ(shifted.eval_row, at_zero.eval_row);
  EXPECT_EQ(shifted.path_pos_m, at_zero.path_pos_m);
  EXPECT_EQ(shifted.path_rot_rad, at_zero.path_rot_rad);
  EXPECT_EQ(shifted.goal_pos_m, at_zero.goal_pos_m);
  EXPECT_EQ(shifted.goal_rot_rad, at_zero.goal_rot_rad);
  EXPECT_EQ(shifted.on_path, at_zero.on_path);
  for (std::size_t k = 0; k < shifted.t_s.size(); k += 250) {
    EXPECT_NEAR(shifted.t_s[k] - at_zero.t_s[k], shift_s, 1e-9) << k;
  }
  ASSERT_EQ(shifted.stops.size(), at_zero.stops.size());
  for (std::size_t i = 0; i < shifted.stops.size(); ++i) {
    const StopTrace& a = shifted.stops[i];
    const StopTrace& b = at_zero.stops[i];
    EXPECT_EQ(a.waypoint, b.waypoint) << i;
    EXPECT_NEAR(a.t_arrive_s - b.t_arrive_s, shift_s, 1e-9) << i;
    ASSERT_EQ(a.t_s.size(), b.t_s.size()) << i;
    EXPECT_EQ(a.pos_m, b.pos_m) << i;
    EXPECT_EQ(a.rot_rad, b.rot_rad) << i;
  }
}

void expect_same_metrics(const TrackingMetrics& a, const TrackingMetrics& b) {
  EXPECT_EQ(a.path_pos_err_max_m, b.path_pos_err_max_m);
  EXPECT_EQ(a.path_rot_err_max_rad, b.path_rot_err_max_rad);
  EXPECT_EQ(a.path_pos_err_rms_m, b.path_pos_err_rms_m);
  ASSERT_EQ(a.settle_time_s.has_value(), b.settle_time_s.has_value());
  if (a.settle_time_s) {
    EXPECT_NEAR(*a.settle_time_s, *b.settle_time_s, 1e-9);
  }
  EXPECT_EQ(a.goal_ok, b.goal_ok);
  EXPECT_EQ(a.path_ok, b.path_ok);
  ASSERT_EQ(a.stops.size(), b.stops.size());
  for (std::size_t i = 0; i < a.stops.size(); ++i) {
    EXPECT_EQ(a.stops[i].pos_err_m, b.stops[i].pos_err_m) << i;
    EXPECT_EQ(a.stops[i].rot_err_rad, b.stops[i].rot_err_rad) << i;
    ASSERT_EQ(a.stops[i].time_to_enter_s.has_value(), b.stops[i].time_to_enter_s.has_value()) << i;
    if (a.stops[i].time_to_enter_s) {
      EXPECT_NEAR(*a.stops[i].time_to_enter_s, *b.stops[i].time_to_enter_s, 1e-9) << i;
    }
  }
}

TEST(StudioMetricsLiveTime, ShiftedRunGivesTheTraceOfTheSameRunAtZero) {
  const bench::RunData shifted = moving_run(kLiveStartNs, live_meta(3.0));
  const bench::RunData at_zero = moving_run(0, live_meta(3.0));
  const ErrorTrace a = make_error_trace(shifted);
  const ErrorTrace b = make_error_trace(at_zero);
  EXPECT_EQ(b.eval_row, 3000);  // the bench case: window_end 3.0 s from t = 0
  expect_same_trace(a, b, 50.0);
  const TrackingMetrics ma = compute_metrics(shifted, Tolerance{});
  const TrackingMetrics mb = compute_metrics(at_zero, Tolerance{});
  ASSERT_TRUE(mb.settle_time_s.has_value());
  expect_same_metrics(ma, mb);
}

TEST(StudioMetricsLiveTime, ShiftedWaypointRunGivesTheStopsOfTheSameRunAtZero) {
  const bench::RunData shifted = moving_run(kLiveStartNs, stop_meta());
  const bench::RunData at_zero = moving_run(0, stop_meta());
  const ErrorTrace a = make_error_trace(shifted);
  const ErrorTrace b = make_error_trace(at_zero);
  // Stops 0 and 1 (the last waypoint is the final goal); arrivals at first + 0.5 s / 1.0 s.
  ASSERT_EQ(a.stops.size(), 2U);
  EXPECT_NEAR(a.stops[0].t_arrive_s, 50.5, 1e-9);
  EXPECT_NEAR(a.stops[1].t_arrive_s, 51.0, 1e-9);
  expect_same_trace(a, b, 50.0);
  expect_same_metrics(compute_metrics(shifted, Tolerance{}), compute_metrics(at_zero, Tolerance{}));
}

}  // namespace
}  // namespace cobot::studio
