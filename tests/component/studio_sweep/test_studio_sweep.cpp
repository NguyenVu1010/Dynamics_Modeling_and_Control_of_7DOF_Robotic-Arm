// Studio sweeps (feature studio-core, ADR-021, cobot/studio/runner.hpp): SweepRunner runs one
// param over its levels and seeds with the bench engine in process. Same input, same numbers: each
// RunSummary equals the matching row of `cobot_bench sweep` (bench::sweep) on an equivalent sweep
// file, compared as Summary CSV text (%.17g, so bit-equal), including config.* params and the
// virtual key config.computed_torque.omega. Order level-major then seed; records kept only for
// seeds.front(); every level validated before the first run; cancel checked between runs.
// Base default + ideal, 1.5 s per run (ASan budget).
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/log_record.hpp"
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

const fs::path kRepo = COBOT_REPO_DIR;
constexpr double kRunS = 1.5;
const std::vector<std::uint64_t> kSeeds{1, 2};

fs::path out_dir(const std::string& name) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_sweep" / name;
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

StudioConfig default_ideal() {
  StudioConfig c;
  c.base =
      BasePair{"ideal", kRepo / "config/default.yaml", kRepo / "config/imperfections/ideal.yaml"};
  c.duration_s = kRunS;
  return c;
}

std::string number(double v) {  // as the Summary CSV writes doubles
  std::array<char, 32> text{};
  std::snprintf(text.data(), text.size(), "%.17g", v);
  return text.data();
}

// Summary CSV columns final_pos_err_m .. hold_reason of one run, as cobot_bench sweep writes them.
std::string metric_columns(const bench::RunSummary& s) {
  return number(s.final_pos_err_m) + ',' + number(s.final_rot_err_rad) + ',' +
         number(s.rms_joint_err_rad) + ',' + number(s.peak_tau_ratio) + ',' + number(s.sat_time_s) +
         ',' + number(s.rms_tau_ratio) + ',' + (s.held ? "1" : "0") + ',' +
         (s.held ? number(s.hold_time_s) : "") + ',' + pipeline::to_string(s.hold_reason);
}

using Key = std::tuple<std::string, std::string, std::string>;  // param, level, seed (CSV text)

std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> out;
  std::string cell;
  std::stringstream ss(line);
  while (std::getline(ss, cell, ',')) {
    out.push_back(cell);
  }
  if (!line.empty() && line.back() == ',') {
    out.emplace_back();
  }
  return out;
}

struct BenchSummary {
  std::string header;
  std::map<Key, std::string> rows;  // -> columns 4..12 joined
};

BenchSummary read_summary(const fs::path& csv) {
  BenchSummary out;
  std::ifstream in(csv, std::ios::binary);
  std::getline(in, out.header);
  std::string line;
  while (std::getline(in, line)) {
    const std::vector<std::string> c = split(line);
    EXPECT_EQ(c.size(), 14U) << line;
    if (c.size() != 14U) {
      continue;
    }
    std::string metrics = c[4];
    for (std::size_t i = 5; i <= 12; ++i) {
      metrics += ',' + c[i];
    }
    out.rows[Key{c[1], c[2], c[3]}] = metrics;
  }
  return out;
}

struct Collected {
  std::vector<RunEntry> entries;
  std::vector<std::pair<std::size_t, std::size_t>> progress;
};

common::Status run_sweep(SweepRunner& runner,
                         const SweepRequest& request,
                         Collected& out,
                         std::atomic<bool>* cancel_after_first = nullptr) {
  const std::atomic<bool> never{false};
  const std::atomic<bool>& cancel = cancel_after_first != nullptr ? *cancel_after_first : never;
  return runner.run(
      request,
      cancel,
      [&](RunEntry&& e) {
        out.entries.push_back(std::move(e));
        if (cancel_after_first != nullptr) {
          cancel_after_first->store(true);
        }
      },
      [&](std::size_t done, std::size_t total) { out.progress.emplace_back(done, total); });
}

SweepRequest request(const std::string& param, std::vector<double> levels) {
  SweepRequest r;
  r.base = default_ideal();
  r.param = param;
  r.levels = std::move(levels);
  r.seeds = kSeeds;
  return r;
}

TEST(StudioSweep, SummariesEqualCobotBenchSweepRows) {
  // Equivalent sweep file for cobot_bench sweep --config config/default.yaml.
  const fs::path dir = out_dir("cross_check");
  const fs::path sweep_yaml = dir / "sweep.yaml";
  {
    std::ofstream y(sweep_yaml, std::ios::binary);
    y << "base_profile: " << (kRepo / "config/imperfections/ideal.yaml").generic_string() << "\n"
      << "seeds: [1, 2]\n"
         "params:\n"
         "  - name: sensor.q_noise_std_rad\n"
         "    levels: [0, 1.0e-4]\n"
         "  - name: config.safety.hal_timeout_ms\n"
         "    levels: [5, 8]\n"
         "  - name: config.computed_torque.omega\n"
         "    levels: [40, 80]\n";
  }
  const common::Status bench_status =
      bench::sweep(kRepo / "config/default.yaml", sweep_yaml, dir / "bench", kRunS);
  ASSERT_TRUE(bench_status.is_ok()) << bench_status.reason();
  const BenchSummary bench_rows = read_summary(dir / "bench" / "summary.csv");
  ASSERT_EQ(bench_rows.rows.size(), 12U);  // 3 params x 2 levels x 2 seeds

  std::ostringstream header;
  bench::write_summary_header(header);
  EXPECT_EQ(header.str(), bench_rows.header + "\n");

  SweepRunner runner(dir / "studio", 1);
  std::size_t compared = 0;
  const std::vector<std::pair<std::string, std::vector<double>>> sweeps{
      {"sensor.q_noise_std_rad", {0.0, 1.0e-4}},
      {"config.safety.hal_timeout_ms", {5.0, 8.0}},
      {"config.computed_torque.omega", {40.0, 80.0}}};
  for (const auto& [param, levels] : sweeps) {
    Collected got;
    const common::Status s = run_sweep(runner, request(param, levels), got);
    ASSERT_TRUE(s.is_ok()) << param << ": " << s.reason();
    ASSERT_EQ(got.entries.size(), 4U) << param;
    for (const RunEntry& e : got.entries) {
      ASSERT_TRUE(e.sweep.has_value()) << param;
      EXPECT_EQ(e.kind, RunKind::kSweep);
      EXPECT_EQ(e.sweep->param, param);
      EXPECT_EQ(e.meta.seed, e.sweep->seed);
      const Key key{param, number(e.sweep->level), std::to_string(e.sweep->seed)};
      const auto row = bench_rows.rows.find(key);
      ASSERT_NE(row, bench_rows.rows.end())
          << param << " " << std::get<1>(key) << " seed " << std::get<2>(key);
      EXPECT_EQ(metric_columns(e.summary), row->second)
          << param << " " << std::get<1>(key) << " seed " << std::get<2>(key);
      ++compared;
    }
  }
  EXPECT_EQ(compared, 12U);
}

TEST(StudioSweep, OrderRecordsIdsAndProgress) {
  SweepRunner runner(out_dir("order"), 1);
  Collected got;
  const common::Status s = run_sweep(runner, request("sensor.q_noise_std_rad", {0.0, 1.0e-4}), got);
  ASSERT_TRUE(s.is_ok()) << s.reason();
  ASSERT_EQ(got.entries.size(), 4U);
  // Level-major, then seed.
  const std::vector<std::pair<double, std::uint64_t>> order{
      {0.0, 1}, {0.0, 2}, {1.0e-4, 1}, {1.0e-4, 2}};
  RunStore store;
  std::uint64_t last_id = 0;
  for (std::size_t i = 0; i < order.size(); ++i) {
    const RunEntry& e = got.entries[i];
    ASSERT_TRUE(e.sweep.has_value());
    EXPECT_EQ(e.sweep->level, order[i].first) << i;
    EXPECT_EQ(e.sweep->seed, order[i].second) << i;
    // Records only for seeds.front(); the error trace for every seed.
    EXPECT_EQ(e.data != nullptr, e.sweep->seed == 1U) << i;
    EXPECT_NE(e.errors, nullptr) << i;
    EXPECT_TRUE(fs::is_regular_file(e.inputs.config_yaml)) << e.inputs.config_yaml;
    EXPECT_TRUE(fs::is_regular_file(e.inputs.imperfections_yaml)) << e.inputs.imperfections_yaml;
    const std::uint64_t id = store.add(RunEntry(e));
    EXPECT_GT(id, last_id) << i;
    last_id = id;
  }
  ASSERT_EQ(got.progress.size(), 4U);
  for (std::size_t i = 0; i < got.progress.size(); ++i) {
    EXPECT_EQ(got.progress[i].first, i + 1) << i;
    EXPECT_EQ(got.progress[i].second, 4U) << i;
  }
  // The table row of a level: worst case over its two seeds.
  const LevelRow row = worst_case({&got.entries[2], &got.entries[3]}, Tolerance{});
  EXPECT_EQ(row.param, "sensor.q_noise_std_rad");
  EXPECT_EQ(row.level, 1.0e-4);
  EXPECT_EQ(row.seeds, 2U);
}

TEST(StudioSweep, CancelInsideTheFirstResultStopsAfterOneRun) {
  SweepRunner runner(out_dir("cancel"), 1);
  Collected got;
  std::atomic<bool> cancel{false};
  const common::Status s =
      run_sweep(runner, request("sensor.q_noise_std_rad", {0.0, 1.0e-4}), got, &cancel);
  EXPECT_TRUE(s.is_ok()) << s.reason();
  EXPECT_EQ(got.entries.size(), 1U);
}

TEST(StudioSweep, LevelOutsideTheCatalogLimitsFailsBeforeAnyRun) {
  SweepRunner runner(out_dir("limits"), 1);
  Collected got;
  const common::Status s =
      run_sweep(runner, request("config.computed_torque.omega", {40.0, 5.0}), got);
  ASSERT_FALSE(s.is_ok());
  EXPECT_NE(s.reason().find("config.computed_torque.omega"), std::string::npos) << s.reason();
  EXPECT_NE(s.reason().find("10"), std::string::npos) << s.reason();  // the lower limit
  EXPECT_TRUE(got.entries.empty());
}

TEST(StudioSweep, UnknownParamFailsBeforeAnyRun) {
  SweepRunner runner(out_dir("unknown"), 1);
  Collected got;
  const common::Status s = run_sweep(runner, request("sensor.q_noise_stdd", {0.0}), got);
  ASSERT_FALSE(s.is_ok());
  EXPECT_NE(s.reason().find("sensor.q_noise_stdd"), std::string::npos) << s.reason();
  EXPECT_TRUE(got.entries.empty());
}

TEST(StudioSweep, ParallelRunsGiveTheSameSummariesOrTheCtorRejectsThem) {
  std::unique_ptr<SweepRunner> parallel;
  try {
    parallel = std::make_unique<SweepRunner>(out_dir("parallel"), 2);
  } catch (const std::exception& e) {
    SUCCEED() << "max_parallel > 1 rejected: " << e.what();
    return;
  }
  SweepRunner serial(out_dir("serial"), 1);
  const SweepRequest r = request("sensor.q_noise_std_rad", {1.0e-4});
  Collected a;
  Collected b;
  ASSERT_TRUE(run_sweep(serial, r, a).is_ok());
  ASSERT_TRUE(run_sweep(*parallel, r, b).is_ok());
  ASSERT_EQ(a.entries.size(), 2U);
  ASSERT_EQ(b.entries.size(), 2U);
  std::map<std::uint64_t, std::string> by_seed;
  for (const RunEntry& e : a.entries) {
    by_seed[e.sweep->seed] = metric_columns(e.summary);
  }
  for (const RunEntry& e : b.entries) {
    EXPECT_EQ(metric_columns(e.summary), by_seed[e.sweep->seed]) << "seed " << e.sweep->seed;
  }
  EXPECT_EQ(b.progress.size(), 2U);
}

}  // namespace
}  // namespace cobot::studio
