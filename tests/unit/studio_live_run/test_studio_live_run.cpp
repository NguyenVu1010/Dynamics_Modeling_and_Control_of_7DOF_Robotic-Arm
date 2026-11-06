// A live run as a Tracking-tab entry (feature studio-control, cobot/studio/live_run.hpp, design
// 5.2 settle_window_s, plan ambiguity 15), without a bridge: a 2 s bench run of the E2E-13
// waypoint list (config/default.yaml + ideal) written as a Run CSV stands for the segment the host
// exports. LiveRunInfo: t_start = first row, t_end = first standby row after tracking, segment
// durations from the run's node times. Pinned:
//   - the summary is bit-equal to bench::summarize on the same rows (read_run_csv) with the
//     session's tau_max and rated and t_eval = t_end + settle_window_s (first row + window_end);
//   - meta.goal_time_s = t_end - t_start, window_end_s = goal + 1 s, tau_max / rated from the
//     session, the waypoint fields from the info (node times = cumulative sums of the durations);
//   - errors = make_error_trace of the entry's data, so compute_metrics copies the summary;
//   - kind kGazebo, label "live waypoints #1"; truncated: label ends "(truncated)", note set.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/bridge/wire.hpp"
#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/studio/live_run.hpp"
#include "cobot/studio/metrics.hpp"
#include "cobot/studio/profile_catalog.hpp"
#include "cobot/studio/run_csv.hpp"
#include "cobot/studio/run_store.hpp"
#include "cobot/studio/runner.hpp"
#include "cobot/studio/studio_config.hpp"

#include "engine.hpp"
#include "metrics.hpp"
#include "run_data.hpp"

namespace cobot::studio {
namespace {

namespace fs = std::filesystem;
using Records = std::vector<pipeline::LogRecord<bench::kN>>;

const fs::path kRepo = COBOT_REPO_DIR;
constexpr std::int64_t kNsPerS = 1'000'000'000;

fs::path work_dir(const std::string& name) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_live_run" / name;
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

bool same_bits(double a, double b) {
  if (std::isnan(a) && std::isnan(b)) {
    return true;
  }
  std::uint64_t bits_a = 0;
  std::uint64_t bits_b = 0;
  std::memcpy(&bits_a, &a, sizeof(double));
  std::memcpy(&bits_b, &b, sizeof(double));
  return bits_a == bits_b;
}

void expect_same_summary(const bench::RunSummary& got, const bench::RunSummary& want) {
  EXPECT_TRUE(same_bits(got.final_pos_err_m, want.final_pos_err_m))
      << got.final_pos_err_m << " vs " << want.final_pos_err_m;
  EXPECT_TRUE(same_bits(got.final_rot_err_rad, want.final_rot_err_rad))
      << got.final_rot_err_rad << " vs " << want.final_rot_err_rad;
  EXPECT_TRUE(same_bits(got.rms_joint_err_rad, want.rms_joint_err_rad))
      << got.rms_joint_err_rad << " vs " << want.rms_joint_err_rad;
  EXPECT_TRUE(same_bits(got.peak_tau_ratio, want.peak_tau_ratio))
      << got.peak_tau_ratio << " vs " << want.peak_tau_ratio;
  EXPECT_TRUE(same_bits(got.sat_time_s, want.sat_time_s))
      << got.sat_time_s << " vs " << want.sat_time_s;
  EXPECT_TRUE(same_bits(got.rms_tau_ratio, want.rms_tau_ratio))
      << got.rms_tau_ratio << " vs " << want.rms_tau_ratio;
  EXPECT_EQ(got.held, want.held);
  EXPECT_TRUE(same_bits(got.hold_time_s, want.hold_time_s));
  EXPECT_EQ(got.hold_reason, want.hold_reason);
}

// The bench waypoint run standing for an exported segment, and what the bridge client would know
// about it.
struct Segment {
  StudioConfig config;
  bench::RunData run;  // in memory (meta: node times, tau_max, rated)
  fs::path csv;        // written by bench::write_run_files
  Records rows;        // read back from csv
  std::int64_t t_end_ns = 0;
};

// config/default.yaml + ideal with tests/e2e/waypoints/e2e_13.yaml for 2 s. The two auto
// segments of E2E-13 are about 0.053 m and 0.047 m long, a triangle profile at a_max 0.5 m/s^2
// each (2 sqrt(d / a) = 0.65 s and 0.61 s), so the loop is back in standby well before 2 s.
std::optional<Segment> waypoint_segment(const fs::path& dir) {
  Segment s;
  s.config.base =
      BasePair{"ideal", kRepo / "config/default.yaml", kRepo / "config/imperfections/ideal.yaml"};
  s.config.waypoints_yaml = kRepo / "tests/e2e/waypoints/e2e_13.yaml";
  s.config.duration_s = 2.0;
  common::Result<RunEntry> entry = run_once(s.config, dir / "work");
  if (!entry || !entry.value().data) {
    ADD_FAILURE() << (entry ? std::string("no records") : entry.reason());
    return std::nullopt;
  }
  s.run = *entry.value().data;
  s.csv = dir / "segment-1.csv";
  const common::Status written = bench::write_run_files(s.run, s.csv);
  if (!written) {
    ADD_FAILURE() << written.reason();
    return std::nullopt;
  }
  common::Result<Records> rows = read_run_csv(s.csv);
  if (!rows) {
    ADD_FAILURE() << rows.reason();
    return std::nullopt;
  }
  s.rows = rows.value();
  bool tracked = false;
  for (const auto& r : s.rows) {
    tracked = tracked || r.mode == pipeline::LoopMode::kTracking;
    if (tracked && r.mode == pipeline::LoopMode::kStandby) {
      s.t_end_ns = r.t.ns;
      return s;
    }
  }
  ADD_FAILURE() << "no standby row after tracking in 2 s (tracked: " << tracked << ")";
  return std::nullopt;
}

bridge::SessionInfo<bench::kN> session_of(const Segment& s) {
  bridge::SessionInfo<bench::kN> session{};
  session.generation = 1;
  bridge::copy_text(session.config_path, bridge::kPathChars, s.config.base.config_yaml.string());
  bridge::copy_text(
      session.imperfections_path, bridge::kPathChars, s.config.base.imperfections_yaml.string());
  session.command_source_external = 1;
  session.control_rate_hz = 1000.0;
  for (int j = 0; j < bench::kN; ++j) {
    const auto i = static_cast<std::size_t>(j);
    session.tau_max[i] = s.run.meta.tau_max(j);
    session.rated[i] = s.run.meta.rated(j);
  }
  return session;
}

LiveRunInfo info_of(const Segment& s) {
  LiveRunInfo info;
  info.cmd = pipeline::CommandMode::kWaypoints;
  info.command_id = 7;
  info.t_start_ns = s.rows.front().t.ns;
  info.t_end_ns = s.t_end_ns;
  info.settle_window_s = 1.0;
  double previous = 0.0;
  for (const double node : s.run.meta.waypoint_node_times_s) {
    info.segment_durations_s.push_back(node - previous);
    previous = node;
  }
  info.waypoint_stop = s.run.meta.waypoint_stop;
  info.waypoint_positions_m = s.run.meta.waypoint_positions_m;
  info.config = s.config;
  return info;
}

TEST(StudioLiveRun, WaypointRunEntryUsesBenchSummarizeOnTheSegmentRows) {
  const fs::path dir = work_dir("waypoints");
  const std::optional<Segment> s = waypoint_segment(dir);
  ASSERT_TRUE(s);
  ASSERT_EQ(s->run.meta.waypoint_node_times_s.size(), 2U) << "E2E-13 has two waypoints";
  const bridge::SessionInfo<bench::kN> session = session_of(*s);
  const LiveRunInfo info = info_of(*s);

  const common::Result<RunEntry> made = make_live_entry(s->csv, info, session, 1);
  ASSERT_TRUE(made.is_ok()) << made.reason();
  const RunEntry& e = made.value();

  EXPECT_EQ(e.kind, RunKind::kGazebo);
  EXPECT_EQ(e.label.rfind("live waypoints #1", 0), 0U) << e.label;
  EXPECT_EQ(e.label.find("truncated"), std::string::npos) << e.label;
  EXPECT_TRUE(e.note.empty()) << e.note;
  EXPECT_EQ(e.config.base.name, "ideal");

  ASSERT_TRUE(e.data);
  ASSERT_EQ(e.data->records.size(), s->rows.size());
  for (std::size_t k = 0; k < s->rows.size(); ++k) {
    ASSERT_EQ(e.data->records[k].t.ns, s->rows[k].t.ns) << "row " << k;
  }

  // goal = t_end - t_start; window_end = goal + settle_window_s (1 s).
  const double goal_s = static_cast<double>(s->t_end_ns - info.t_start_ns) / 1e9;
  EXPECT_NEAR(e.meta.goal_time_s, goal_s, 1e-12);
  EXPECT_NEAR(e.meta.window_end_s, goal_s + 1.0, 1e-12);
  EXPECT_NEAR(e.data->meta.goal_time_s, goal_s, 1e-12);
  for (int j = 0; j < bench::kN; ++j) {
    const auto i = static_cast<std::size_t>(j);
    EXPECT_EQ(e.meta.tau_max(j), session.tau_max[i]) << "joint " << j + 1;
    EXPECT_EQ(e.meta.rated(j), session.rated[i]) << "joint " << j + 1;
  }

  // Waypoint fields: node times are the cumulative sums of the segment durations.
  ASSERT_EQ(e.meta.waypoint_node_times_s.size(), s->run.meta.waypoint_node_times_s.size());
  for (std::size_t i = 0; i < e.meta.waypoint_node_times_s.size(); ++i) {
    EXPECT_NEAR(e.meta.waypoint_node_times_s[i], s->run.meta.waypoint_node_times_s[i], 1e-12)
        << "node " << i;
  }
  EXPECT_EQ(e.meta.waypoint_stop, s->run.meta.waypoint_stop);
  ASSERT_EQ(e.meta.waypoint_positions_m.size(), s->run.meta.waypoint_positions_m.size());
  for (std::size_t i = 0; i < e.meta.waypoint_positions_m.size(); ++i) {
    EXPECT_EQ(e.meta.waypoint_positions_m[i], s->run.meta.waypoint_positions_m[i]) << i;
  }

  // The same numbers as bench::summarize on the same rows: t_eval = first row + window_end =
  // t_end + 1 s, in integer nanoseconds (rows are 1 ms apart and t_end is a row stamp).
  const std::int64_t t_eval = s->t_end_ns + kNsPerS;
  const bench::RunSummary want =
      bench::summarize(s->rows, s->run.meta.tau_max, s->run.meta.rated, t_eval);
  expect_same_summary(e.summary, want);
  expect_same_summary(e.data->summary, want);
  EXPECT_FALSE(e.summary.held);

  // errors = make_error_trace(data), so the metrics copy the summary.
  ASSERT_TRUE(e.errors);
  const ErrorTrace trace = make_error_trace(*e.data);
  EXPECT_EQ(e.errors->eval_row, trace.eval_row);
  EXPECT_EQ(e.errors->t_s, trace.t_s);
  const TrackingMetrics metrics = compute_metrics(*e.errors, e.meta, e.summary, Tolerance{});
  expect_same_summary(metrics.summary, want);
}

TEST(StudioLiveRun, TruncatedSegmentIsLabelledAndNoted) {
  const fs::path dir = work_dir("truncated");
  const std::optional<Segment> s = waypoint_segment(dir);
  ASSERT_TRUE(s);
  LiveRunInfo info = info_of(*s);
  info.truncated = true;
  const common::Result<RunEntry> made = make_live_entry(s->csv, info, session_of(*s), 3);
  ASSERT_TRUE(made.is_ok()) << made.reason();
  const std::string& label = made.value().label;
  EXPECT_EQ(label.rfind("live waypoints #3", 0), 0U) << label;
  const std::string tail = "(truncated)";
  ASSERT_GE(label.size(), tail.size()) << label;
  EXPECT_EQ(label.substr(label.size() - tail.size()), tail) << label;
  EXPECT_FALSE(made.value().note.empty());
  EXPECT_EQ(made.value().kind, RunKind::kGazebo);
}

TEST(StudioLiveRun, CommandNameIsInTheLabel) {
  const fs::path dir = work_dir("label");
  const std::optional<Segment> s = waypoint_segment(dir);
  ASSERT_TRUE(s);
  LiveRunInfo info = info_of(*s);
  info.cmd = pipeline::CommandMode::kMoveJoint;
  info.segment_durations_s.clear();
  info.waypoint_stop.clear();
  info.waypoint_positions_m.clear();
  const common::Result<RunEntry> made = make_live_entry(s->csv, info, session_of(*s), 2);
  ASSERT_TRUE(made.is_ok()) << made.reason();
  EXPECT_EQ(made.value().label.rfind("live move_joint #2", 0), 0U) << made.value().label;
  EXPECT_TRUE(made.value().meta.waypoint_node_times_s.empty());
}

TEST(StudioLiveRun, MissingSegmentFileIsAnErrorNamingIt) {
  const fs::path dir = work_dir("missing");
  LiveRunInfo info;
  info.t_end_ns = 1;
  const common::Result<RunEntry> made =
      make_live_entry(dir / "segment-404.csv", info, bridge::SessionInfo<bench::kN>{}, 1);
  ASSERT_FALSE(made.is_ok());
  EXPECT_NE(made.reason().find("segment-404.csv"), std::string::npos) << made.reason();
}

}  // namespace
}  // namespace cobot::studio
