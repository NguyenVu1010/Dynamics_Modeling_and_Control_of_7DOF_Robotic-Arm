// Studio tracking metrics on synthetic runs (feature studio-core, cobot_studio design 5.2,
// ADR-021, cobot/studio/metrics.hpp). Synthetic RunData (synthetic_run.hpp): dt 1 ms, rows t = 0 ..
// 3.0 s, goal_time 1.0 s, window_end 3.0 s, rotation error 0, position error err(k) on x.
// Definitions pinned here:
//   path error: |p_d - p| and the angle of R_d R^T over the tracking/finished rows only;
//   settle time: from t0 = first row + goal_time_s to the last entry into {pos <= goal_pos_m and
//     rot <= goal_rot_rad} after which the run stays inside until window_end_s; nullopt when not
//     inside at window_end_s or when the run held;
//   stop waypoints (not the last): t_arrive = first row + node time; window [t_arrive,
//     t_arrive + 0.2 s] (cobot_eval STOP_SETTLE_S); pos_err_m / rot_err_rad = the minimum error to
//     the waypoint in the window; time_to_enter_s = first time in the window, relative to
//     t_arrive, at which the error is within the goal tolerance, nullopt = "not entered";
//   goal_ok from the summary's final errors, path_ok from the path maxima.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/studio/metrics.hpp"
#include "cobot/studio/run_store.hpp"
#include "cobot/studio/studio_config.hpp"

#include "synthetic_run.hpp"

namespace cobot::studio {
namespace {

using test::kMm;
using test::synthetic_meta;
using test::synthetic_run;
using test::t_of;

void expect_same_summary(const bench::RunSummary& a, const bench::RunSummary& b) {
  EXPECT_EQ(a.final_pos_err_m, b.final_pos_err_m);
  EXPECT_EQ(a.final_rot_err_rad, b.final_rot_err_rad);
  EXPECT_EQ(a.rms_joint_err_rad, b.rms_joint_err_rad);
  EXPECT_EQ(a.peak_tau_ratio, b.peak_tau_ratio);
  EXPECT_EQ(a.sat_time_s, b.sat_time_s);
  EXPECT_EQ(a.rms_tau_ratio, b.rms_tau_ratio);
  EXPECT_EQ(a.held, b.held);
  EXPECT_EQ(a.hold_time_s, b.hold_time_s);
  EXPECT_EQ(a.hold_reason, b.hold_reason);
}

// --- errors from the exponential approach: 5 mm before t = 1 s, 5 mm exp(-(t - 1)/0.2) after ---

double exp_err(std::size_t k) {
  const double t = t_of(k);
  return t < 1.0 ? 5.0 * kMm : 5.0 * kMm * std::exp(-(t - 1.0) / 0.2);
}

TEST(StudioMetrics, ExponentialApproachSettlesAtPointTwoLnFive) {
  const bench::RunData run = synthetic_run(exp_err);
  const TrackingMetrics m = compute_metrics(run, Tolerance{});
  // 5 mm exp(-s / 0.2) <= 1 mm  <=>  s >= 0.2 ln 5 = 0.2 * 1.6094379 = 0.32189 s; the first row
  // inside is t = 1.322 s, 0.11 ms later.
  ASSERT_TRUE(m.settle_time_s.has_value());
  EXPECT_NEAR(*m.settle_time_s, 0.2 * std::log(5.0), 1.5e-3);
  EXPECT_NEAR(m.path_pos_err_max_m, 5.0 * kMm, 1e-15);
  EXPECT_EQ(m.path_rot_err_max_rad, 0.0);
  // Final error at 3.0 s: 5 mm exp(-10) = 2.27e-7 m <= 1 mm; path max 5 mm > 2 mm.
  EXPECT_TRUE(m.goal_ok);
  EXPECT_FALSE(m.path_ok);
  EXPECT_TRUE(m.stops.empty());  // config mode, no waypoints
  expect_same_summary(m.summary, run.summary);
}

// --- in and out: 1.5 mm, inside (0.5 mm) at 1.2 s, out at 1.5 s, back in at 1.8 s ---

double in_out_err(std::size_t k) {
  if (k < 1200) {
    return 1.5 * kMm;
  }
  if (k < 1500) {
    return 0.5 * kMm;
  }
  return k < 1800 ? 1.5 * kMm : 0.5 * kMm;
}

TEST(StudioMetrics, SettleIsTheLastEntryAfterWhichTheRunStaysInside) {
  const bench::RunData run = synthetic_run(in_out_err);
  const TrackingMetrics m = compute_metrics(run, Tolerance{});
  ASSERT_TRUE(m.settle_time_s.has_value());
  EXPECT_NEAR(*m.settle_time_s, 0.8, 1e-3);  // 1.8 s - t0 1.0 s
  EXPECT_NEAR(m.path_pos_err_max_m, 1.5 * kMm, 1e-15);
  // RMS over all 3001 tracking/finished rows: 1500 rows at 1.5 mm (k < 1200 and 1500 <= k < 1800),
  // 1501 rows at 0.5 mm: sqrt((1500 * 2.25 + 1501 * 0.25) / 3001) mm = 1.1182 mm.
  const double rms = std::sqrt((1500.0 * 1.5 * 1.5 + 1501.0 * 0.5 * 0.5) / 3001.0) * kMm;
  EXPECT_NEAR(m.path_pos_err_rms_m, rms, 1e-12);
  EXPECT_TRUE(m.goal_ok);  // final 0.5 mm <= 1 mm
  EXPECT_TRUE(m.path_ok);  // max 1.5 mm <= 2 mm, rotation 0
  expect_same_summary(m.summary, run.summary);
}

double never_err(std::size_t /*k*/) {
  return 1.8 * kMm;
}

TEST(StudioMetrics, NeverInsideHasNoSettleTime) {
  const bench::RunData run = synthetic_run(never_err);
  const TrackingMetrics m = compute_metrics(run, Tolerance{});
  EXPECT_FALSE(m.settle_time_s.has_value());
  EXPECT_FALSE(m.goal_ok);  // final 1.8 mm > 1 mm
  EXPECT_TRUE(m.path_ok);   // 1.8 mm <= 2 mm
}

// Inside from 1.2 s, holding from 2.0 s with a 50 mm error in the holding rows.
double held_err(std::size_t k) {
  if (k >= 2000) {
    return 50.0 * kMm;
  }
  return k < 1200 ? 1.5 * kMm : 0.5 * kMm;
}

pipeline::LoopMode held_mode(std::size_t k) {
  return k >= 2000 ? pipeline::LoopMode::kHolding : test::default_mode(k);
}

TEST(StudioMetrics, HeldRunHasNoSettleTimeAndPathMaximaIgnoreHoldingRows) {
  const bench::RunData run = synthetic_run(held_err, held_mode);
  ASSERT_TRUE(run.summary.held);
  const TrackingMetrics m = compute_metrics(run, Tolerance{});
  EXPECT_FALSE(m.settle_time_s.has_value());
  EXPECT_NEAR(m.path_pos_err_max_m, 1.5 * kMm, 1e-15);  // not the 50 mm of the holding rows
  // RMS over the 2000 rows before the hold: 1200 at 1.5 mm, 800 at 0.5 mm.
  const double rms = std::sqrt((1200.0 * 1.5 * 1.5 + 800.0 * 0.5 * 0.5) / 2000.0) * kMm;
  EXPECT_NEAR(m.path_pos_err_rms_m, rms, 1e-12);
  EXPECT_FALSE(m.goal_ok);  // final error from the summary: 50 mm at 3.0 s
  expect_same_summary(m.summary, run.summary);
}

// --- error trace: tolerance edits without the records ---

void expect_same_metrics(const TrackingMetrics& a, const TrackingMetrics& b) {
  expect_same_summary(a.summary, b.summary);
  EXPECT_EQ(a.path_pos_err_max_m, b.path_pos_err_max_m);
  EXPECT_EQ(a.path_rot_err_max_rad, b.path_rot_err_max_rad);
  EXPECT_EQ(a.path_pos_err_rms_m, b.path_pos_err_rms_m);
  EXPECT_EQ(a.settle_time_s, b.settle_time_s);
  EXPECT_EQ(a.goal_ok, b.goal_ok);
  EXPECT_EQ(a.path_ok, b.path_ok);
  ASSERT_EQ(a.stops.size(), b.stops.size());
  for (std::size_t i = 0; i < a.stops.size(); ++i) {
    EXPECT_EQ(a.stops[i].waypoint, b.stops[i].waypoint) << i;
    EXPECT_EQ(a.stops[i].t_arrive_s, b.stops[i].t_arrive_s) << i;
    EXPECT_EQ(a.stops[i].pos_err_m, b.stops[i].pos_err_m) << i;
    EXPECT_EQ(a.stops[i].rot_err_rad, b.stops[i].rot_err_rad) << i;
    EXPECT_EQ(a.stops[i].time_to_enter_s, b.stops[i].time_to_enter_s) << i;
  }
}

TEST(StudioMetrics, ErrorTraceHoldsThePerRowErrors) {
  const bench::RunData run = synthetic_run(held_err, held_mode);
  const ErrorTrace e = make_error_trace(run);
  ASSERT_EQ(e.t_s.size(), test::kRows);
  ASSERT_EQ(e.path_pos_m.size(), test::kRows);
  ASSERT_EQ(e.path_rot_rad.size(), test::kRows);
  ASSERT_EQ(e.goal_pos_m.size(), test::kRows);
  ASSERT_EQ(e.goal_rot_rad.size(), test::kRows);
  ASSERT_EQ(e.on_path.size(), test::kRows);
  for (const std::size_t k : {std::size_t{0}, std::size_t{1500}, std::size_t{2500}}) {
    EXPECT_NEAR(e.t_s[k], t_of(k), 1e-12) << k;
    EXPECT_NEAR(e.path_pos_m[k], held_err(k), 1e-15) << k;
    EXPECT_NEAR(e.goal_pos_m[k], held_err(k), 1e-15) << k;  // goal pose = origin too
    EXPECT_EQ(e.path_rot_rad[k], 0.0) << k;
  }
  EXPECT_NE(e.on_path[1500], 0) << "finished row";
  EXPECT_EQ(e.on_path[2500], 0) << "holding row";
}

TEST(StudioMetrics, TraceAndRunGiveTheSameMetricsAndToleranceEditsRecompute) {
  const bench::RunData run = synthetic_run(in_out_err);
  const ErrorTrace e = make_error_trace(run);
  expect_same_metrics(compute_metrics(e, run.meta, run.summary, Tolerance{}),
                      compute_metrics(run, Tolerance{}));
  // Tighter tolerance: the 0.5 mm level is never inside 0.4 mm; path 1.5 mm > 1 mm.
  Tolerance tight;
  tight.goal_pos_m = 0.4 * kMm;
  tight.path_pos_m = 1.0 * kMm;
  const TrackingMetrics m = compute_metrics(e, run.meta, run.summary, tight);
  EXPECT_FALSE(m.settle_time_s.has_value());
  EXPECT_FALSE(m.goal_ok);
  EXPECT_FALSE(m.path_ok);
  expect_same_metrics(m, compute_metrics(run, tight));
}

TEST(StudioMetrics, DefaultToleranceIsTheDesignValues) {
  const Tolerance t;
  constexpr double kDeg = 3.14159265358979323846 / 180.0;
  EXPECT_DOUBLE_EQ(t.goal_pos_m, 1e-3);
  EXPECT_DOUBLE_EQ(t.goal_rot_rad, 0.5 * kDeg);
  EXPECT_DOUBLE_EQ(t.path_pos_m, 2e-3);
  EXPECT_DOUBLE_EQ(t.path_rot_rad, 1.0 * kDeg);
}

// --- stop waypoints: three waypoints, node times 0.5 / 1.0 / 2.0 s, the middle one stops ---

bench::RunMeta stop_meta() {
  bench::RunMeta meta = synthetic_meta();
  meta.goal_time_s = 2.0;
  meta.waypoint_node_times_s = {0.5, 1.0, 2.0};
  meta.waypoint_stop = {false, true, true};
  // Waypoint 1 at the origin: its error is err(k) exactly (rotation 0 everywhere).
  meta.waypoint_positions_m = {
      common::Vec3(0.2, 0.0, 0.0), common::Vec3(0.0, 0.0, 0.0), common::Vec3(0.0, 0.0, 0.0)};
  return meta;
}

// Below 1 mm from 0.97 s: 2 mm before, then 0.5 mm + 10 mm/s |t - 1.0| (0.8 mm at 0.97 s, its
// minimum 0.5 mm at the node row t = 1.0 s, 2.5 mm at 1.2 s).
double stop_early_err(std::size_t k) {
  return k < 970 ? 2.0 * kMm : 0.5 * kMm + 10.0 * kMm * std::abs(t_of(k) - 1.0);
}

TEST(StudioMetrics, StopEnteredBeforeArrivalIsInsideAtTheWindowStart) {
  const bench::RunData run = synthetic_run(stop_early_err, test::default_mode, stop_meta());
  const TrackingMetrics m = compute_metrics(run, Tolerance{});
  ASSERT_EQ(m.stops.size(), 1U);  // the last waypoint is the final goal, not a stop entry
  const StopMetrics& s = m.stops[0];
  EXPECT_EQ(s.waypoint, 1U);
  EXPECT_NEAR(s.t_arrive_s, 1.0, 1e-9);
  EXPECT_NEAR(s.pos_err_m, stop_early_err(1000), 1e-12);  // the error of the row at 1.0 s
  EXPECT_EQ(s.rot_err_rad, 0.0);
  // Inside at t_arrive: the first time in [t_arrive, t_arrive + 0.2 s] within 1 mm is t_arrive.
  ASSERT_TRUE(s.time_to_enter_s.has_value());
  EXPECT_NEAR(*s.time_to_enter_s, 0.0, 1e-3);
}

double stop_late_err(std::size_t k) {  // 2 mm, 0.5 mm from 1.05 s on
  return k < 1050 ? 2.0 * kMm : 0.5 * kMm;
}

TEST(StudioMetrics, StopEnteredInsideTheWindowGivesTheTimeAfterArrival) {
  const bench::RunData run = synthetic_run(stop_late_err, test::default_mode, stop_meta());
  const TrackingMetrics m = compute_metrics(run, Tolerance{});
  ASSERT_EQ(m.stops.size(), 1U);
  const StopMetrics& s = m.stops[0];
  EXPECT_NEAR(s.pos_err_m, 0.5 * kMm, 1e-15);  // minimum over the window
  ASSERT_TRUE(s.time_to_enter_s.has_value());
  EXPECT_NEAR(*s.time_to_enter_s, 0.05, 1e-3);
}

double stop_missed_err(std::size_t k) {  // 2 mm, 0.5 mm only from 1.25 s, after the window
  return k < 1250 ? 2.0 * kMm : 0.5 * kMm;
}

TEST(StudioMetrics, StopNotEnteredInTheWindowIsNullopt) {
  const bench::RunData run = synthetic_run(stop_missed_err, test::default_mode, stop_meta());
  const TrackingMetrics m = compute_metrics(run, Tolerance{});
  ASSERT_EQ(m.stops.size(), 1U);
  EXPECT_FALSE(m.stops[0].time_to_enter_s.has_value());
  EXPECT_NEAR(m.stops[0].pos_err_m, 2.0 * kMm, 1e-15);
  // The trace alone gives the same stop numbers.
  const ErrorTrace e = make_error_trace(run);
  expect_same_metrics(compute_metrics(e, run.meta, run.summary, Tolerance{}), m);
}

// --- worst case over the seeds of one sweep level ---

RunEntry entry(const bench::RunData& run, std::uint64_t seed, bool keep_records) {
  RunEntry e{};
  e.kind = RunKind::kSweep;
  e.label = "seed " + std::to_string(seed);
  e.errors = std::make_shared<const ErrorTrace>(make_error_trace(run));
  e.meta = run.meta;
  e.summary = run.summary;
  e.sweep = SweepPoint{"sensor.q_noise_std_rad", 1e-4, seed};
  if (keep_records) {
    e.data = std::make_shared<const bench::RunData>(run);
  }
  return e;
}

TEST(StudioMetrics, WorstCaseTakesTheMaxOfEachErrorAndNoSettleIfAnySeedHasNone) {
  const bench::RunData a = synthetic_run(exp_err);     // settle 0.322 s, final 2.27e-7 m
  const bench::RunData b = synthetic_run(in_out_err);  // settle 0.8 s, final 0.5 mm, path 1.5 mm
  const bench::RunData c = synthetic_run(never_err);   // no settle, final 1.8 mm
  const RunEntry ea = entry(a, 1, true);
  const RunEntry eb = entry(b, 2, false);  // summary-only seeds: errors and meta, no records
  const RunEntry ec = entry(c, 3, false);
  const LevelRow row = worst_case({&ea, &eb, &ec}, Tolerance{});
  EXPECT_EQ(row.param, "sensor.q_noise_std_rad");
  EXPECT_EQ(row.level, 1e-4);
  EXPECT_EQ(row.seeds, 3U);
  EXPECT_EQ(row.held, 0U);
  EXPECT_EQ(row.worst.summary.final_pos_err_m, c.summary.final_pos_err_m);  // 1.8 mm
  EXPECT_NEAR(row.worst.path_pos_err_max_m, 5.0 * kMm, 1e-15);              // seed 1
  EXPECT_FALSE(row.worst.settle_time_s.has_value());                        // seed 3
  EXPECT_FALSE(row.worst.goal_ok);
  EXPECT_FALSE(row.worst.path_ok);

  const LevelRow settled = worst_case({&ea, &eb}, Tolerance{});
  ASSERT_TRUE(settled.worst.settle_time_s.has_value());
  EXPECT_NEAR(*settled.worst.settle_time_s, 0.8, 1e-3);  // max of 0.322 and 0.8
  EXPECT_EQ(settled.worst.summary.final_pos_err_m, b.summary.final_pos_err_m);
  EXPECT_NEAR(settled.worst.path_pos_err_max_m, 5.0 * kMm, 1e-15);
  EXPECT_NEAR(settled.worst.path_pos_err_rms_m,
              std::max(compute_metrics(a, Tolerance{}).path_pos_err_rms_m,
                       compute_metrics(b, Tolerance{}).path_pos_err_rms_m),
              1e-15);
  EXPECT_EQ(settled.seeds, 2U);
}

TEST(StudioMetrics, WorstCaseCountsHeldSeeds) {
  const bench::RunData a = synthetic_run(in_out_err);
  const bench::RunData h = synthetic_run(held_err, held_mode);
  const RunEntry ea = entry(a, 1, true);
  const RunEntry eh = entry(h, 2, false);
  const LevelRow row = worst_case({&ea, &eh}, Tolerance{});
  EXPECT_EQ(row.held, 1U);
  EXPECT_TRUE(row.worst.summary.held);
  EXPECT_FALSE(row.worst.settle_time_s.has_value());
  EXPECT_EQ(row.worst.summary.final_pos_err_m, h.summary.final_pos_err_m);  // 50 mm
}

}  // namespace
}  // namespace cobot::studio
