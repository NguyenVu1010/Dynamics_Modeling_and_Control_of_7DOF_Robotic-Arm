// One studio run (feature studio-core, ADR-021, cobot/studio/runner.hpp): run_once materialises
// the configuration under the work directory (<work_dir>/<run-id>/{config.yaml,
// imperfections.yaml, studio.yaml}, nothing else), runs the bench engine in process
// (bench::run_in_memory) and keeps the records. Same input, same numbers: the summary is
// bit-equal to bench::run_profile on the materialised files, and bench::write_run_files gives the
// bytes of `cobot_bench run` on them. compute_metrics copies the RunSummary and gives the same
// result from the stored ErrorTrace as from the records.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/overrides.hpp"
#include "cobot/studio/metrics.hpp"
#include "cobot/studio/run_store.hpp"
#include "cobot/studio/runner.hpp"
#include "cobot/studio/studio_config.hpp"

#include "engine.hpp"
#include "metrics.hpp"
#include "run.hpp"
#include "run_data.hpp"

namespace cobot::studio {
namespace {

namespace fs = std::filesystem;
using pipeline::KeyOverride;

const fs::path kRepo = COBOT_REPO_DIR;
constexpr double kRunS = 2.0;         // simulated s per run (ASan budget)
constexpr double kStopSettleS = 0.2;  // cobot_eval.metrics.STOP_SETTLE_S

fs::path work_dir(const std::string& name) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_run" / name;
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

std::string read_text(const fs::path& p) {
  std::ifstream in(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

bool same_file(const fs::path& a, const fs::path& b) {
  std::error_code ec;
  return fs::equivalent(a, b, ec) && !ec;
}

StudioConfig config_of(const std::string& name,
                       const fs::path& config,
                       const fs::path& imperfections,
                       std::optional<std::uint64_t> seed = std::nullopt) {
  StudioConfig c;
  c.base = BasePair{name, config, imperfections};
  c.duration_s = kRunS;
  c.seed = seed;
  return c;
}

StudioConfig default_ideal() {
  return config_of(
      "ideal", kRepo / "config/default.yaml", kRepo / "config/imperfections/ideal.yaml");
}

StudioConfig premium() {
  return config_of("premium_fr3",
                   kRepo / "config/premium_fr3.yaml",
                   kRepo / "config/imperfections/premium_fr3.yaml");
}

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

std::set<std::string> file_names(const fs::path& dir) {
  std::set<std::string> out;
  for (const fs::directory_entry& e : fs::directory_iterator(dir)) {
    out.insert(e.path().filename().string());
  }
  return out;
}

TEST(StudioRun, RunOnceMatchesTheBenchOnTheMaterialisedFiles) {
  const fs::path work = work_dir("default_ideal");
  const StudioConfig c = default_ideal();
  const common::Result<RunEntry> r = run_once(c, work);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const RunEntry& e = r.value();
  ASSERT_NE(e.data, nullptr);
  ASSERT_NE(e.errors, nullptr);
  EXPECT_EQ(e.kind, RunKind::kBench);
  EXPECT_FALSE(e.sweep.has_value());
  EXPECT_EQ(e.diff, describe_diff(c));
  EXPECT_EQ(e.data->records.size(), 2000U);  // 2 s at 1 kHz
  EXPECT_EQ(e.meta.seed, 1U);                // ideal.yaml's seed
  EXPECT_EQ(e.data->meta.seed, 1U);
  EXPECT_DOUBLE_EQ(e.meta.goal_time_s, 10.0);   // trajectory.duration_s
  EXPECT_DOUBLE_EQ(e.meta.window_end_s, 12.0);  // + hold_s
  EXPECT_DOUBLE_EQ(e.meta.period_s, 1e-3);
  expect_same_summary(e.summary, e.data->summary);

  // Layout: <work_dir>/<run-id>/{config.yaml, imperfections.yaml, studio.yaml}, nothing else.
  const fs::path run_dir = e.inputs.config_yaml.parent_path();
  EXPECT_TRUE(same_file(run_dir.parent_path(), work)) << run_dir;
  EXPECT_TRUE(same_file(e.inputs.imperfections_yaml.parent_path(), run_dir));
  EXPECT_EQ(file_names(run_dir),
            (std::set<std::string>{"config.yaml", "imperfections.yaml", "studio.yaml"}));
  const common::Result<StudioConfig> saved = load_studio_file(run_dir / "studio.yaml");
  ASSERT_TRUE(saved.is_ok()) << saved.reason();
  EXPECT_EQ(saved.value().duration_s, c.duration_s);

  // Same numbers as the bench engine on the materialised pair.
  common::Result<imperfect::ImperfectionConfig<7>> imp =
      pipeline::load_imperfections<7>(e.inputs.imperfections_yaml);
  ASSERT_TRUE(imp.is_ok()) << imp.reason();
  imp.value().seed = e.meta.seed;
  const common::Result<bench::RunSummary> ref =
      bench::run_profile(e.inputs.config_yaml, imp.value(), e.inputs.imperfections_yaml, "", kRunS);
  ASSERT_TRUE(ref.is_ok()) << ref.reason();
  expect_same_summary(e.summary, ref.value());

  // Run CSV and meta file: the bytes of `cobot_bench run` on the same files.
  const fs::path out = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_run" / "export";
  fs::create_directories(out);
  const common::Status w = bench::write_run_files(*e.data, out / "studio.csv");
  ASSERT_TRUE(w.is_ok()) << w.reason();
  const common::Status b = bench::run(
      e.inputs.config_yaml, e.inputs.imperfections_yaml, e.meta.seed, out / "bench.csv", kRunS);
  ASSERT_TRUE(b.is_ok()) << b.reason();
  const std::string bench_csv = read_text(out / "bench.csv");
  ASSERT_FALSE(bench_csv.empty());
  EXPECT_TRUE(read_text(out / "studio.csv") == bench_csv) << "Run CSV bytes differ";
  EXPECT_EQ(read_text(out / "studio.csv.meta.yaml"), read_text(out / "bench.csv.meta.yaml"));
}

TEST(StudioRun, OverrideEqualToTheFileGivesTheSameSummary) {
  // premium_fr3.yaml has kp 2500 = 50^2, kd 100 = 2 * 50: omega 50 changes nothing.
  StudioConfig with_omega = premium();
  with_omega.overrides = {KeyOverride{"config.computed_torque.omega", 50.0}};
  const common::Result<RunEntry> a = run_once(with_omega, work_dir("premium_omega"));
  const common::Result<RunEntry> b = run_once(premium(), work_dir("premium_plain"));
  ASSERT_TRUE(a.is_ok()) << a.reason();
  ASSERT_TRUE(b.is_ok()) << b.reason();
  expect_same_summary(a.value().summary, b.value().summary);
  EXPECT_EQ(a.value().diff, describe_diff(with_omega));
}

TEST(StudioRun, RunOnceRejectsAnInvalidConfigurationBeforeRunning) {
  StudioConfig c = default_ideal();
  c.overrides = {KeyOverride{"config.computed_torque.omega", 5.0}};  // below 10 rad/s
  const fs::path work = work_dir("invalid");
  const common::Result<RunEntry> r = run_once(c, work);
  ASSERT_FALSE(r.is_ok());
  EXPECT_NE(r.reason().find("config.computed_torque.omega"), std::string::npos) << r.reason();
}

// --- metrics on recorded runs ---

std::vector<Tolerance> tolerances() {
  const Tolerance base;
  Tolerance tenth = base;
  Tolerance ten = base;
  tenth.goal_pos_m *= 0.1;
  tenth.goal_rot_rad *= 0.1;
  tenth.path_pos_m *= 0.1;
  tenth.path_rot_rad *= 0.1;
  ten.goal_pos_m *= 10.0;
  ten.goal_rot_rad *= 10.0;
  ten.path_pos_m *= 10.0;
  ten.path_rot_rad *= 10.0;
  return {base, tenth, ten};
}

void check_metrics(const RunEntry& e) {
  ASSERT_NE(e.data, nullptr);
  ASSERT_NE(e.errors, nullptr);
  for (const Tolerance& tol : tolerances()) {
    const TrackingMetrics from_run = compute_metrics(*e.data, tol);
    expect_same_summary(from_run.summary, e.data->summary);
    expect_same_summary(from_run.summary, e.summary);
    expect_same_metrics(compute_metrics(*e.errors, e.meta, e.summary, tol), from_run);
    expect_same_metrics(compute_metrics(make_error_trace(*e.data), e.meta, e.summary, tol),
                        from_run);
  }
}

TEST(StudioRun, MetricsFromTheTraceEqualMetricsFromTheRecordsE2e01) {
  const StudioConfig c = config_of("e2e_01",
                                   kRepo / "tests/e2e/configs/ideal_world.yaml",
                                   kRepo / "config/imperfections/ideal.yaml",
                                   1);
  const common::Result<RunEntry> r = run_once(c, work_dir("e2e_01"));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  check_metrics(r.value());
}

TEST(StudioRun, MetricsFromTheTraceEqualMetricsFromTheRecordsE2e10) {
  StudioConfig c = config_of(
      "realistic", kRepo / "config/default.yaml", kRepo / "config/imperfections/realistic.yaml", 7);
  c.waypoints_yaml = kRepo / "tests/e2e/waypoints/e2e_10.yaml";
  const common::Result<RunEntry> r = run_once(c, work_dir("e2e_10"));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().meta.waypoint_node_times_s.size(), 4U);
  EXPECT_EQ(r.value().meta.waypoint_stop.size(), 4U);
  EXPECT_EQ(r.value().meta.waypoint_positions_m.size(), 4U);
  check_metrics(r.value());
}

// cobot_eval.metrics.stop_pos_err_m on the same rows: min |p - p_waypoint| over the rows nearest
// node time and node time + STOP_SETTLE_S, both included (first row t = 0 in bench runs).
double eval_stop_pos_err(const bench::RunData& run, std::size_t waypoint) {
  const double t_node = run.meta.waypoint_node_times_s.at(waypoint);
  const common::Vec3& target = run.meta.waypoint_positions_m.at(waypoint);
  const auto nearest = [&run](double t) {
    std::size_t best = 0;
    double gap = std::numeric_limits<double>::infinity();
    for (std::size_t k = 0; k < run.records.size(); ++k) {
      const double d = std::abs(static_cast<double>(run.records[k].t.ns) * 1e-9 - t);
      if (d < gap) {
        gap = d;
        best = k;
      }
    }
    return best;
  };
  const std::size_t k0 = nearest(t_node);
  const std::size_t k1 = nearest(t_node + kStopSettleS);
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t k = k0; k <= k1; ++k) {
    best = std::min(best, (run.records[k].T_b_e.p - target).norm());
  }
  return best;
}

TEST(StudioRun, StopWaypointMetricsMatchCobotEval) {
  // tests/data/studio/stops_short.yaml: waypoint 0 stops at node time 0.4 s, the run ends at 2 s.
  StudioConfig c = config_of(
      "realistic", kRepo / "config/default.yaml", kRepo / "config/imperfections/realistic.yaml", 7);
  c.waypoints_yaml = kRepo / "tests/data/studio/stops_short.yaml";
  const common::Result<RunEntry> r = run_once(c, work_dir("stops_short"));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const RunEntry& e = r.value();
  ASSERT_NE(e.data, nullptr);
  EXPECT_NEAR(e.meta.goal_time_s, 1.2, 1e-9);  // waypoint trajectory duration, 3 x 0.4 s
  check_metrics(e);
  const TrackingMetrics m = compute_metrics(*e.data, Tolerance{});
  ASSERT_EQ(m.stops.size(), 1U);  // waypoint 2 is the final goal
  EXPECT_EQ(m.stops[0].waypoint, 0U);
  EXPECT_NEAR(m.stops[0].t_arrive_s, 0.4, 1e-9);
  EXPECT_NEAR(m.stops[0].pos_err_m, eval_stop_pos_err(*e.data, 0), 1e-12);
}

}  // namespace
}  // namespace cobot::studio
