#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/result.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/pipeline/config.hpp"

#include "engine.hpp"
#include "run.hpp"

// Bench side of feature comm-timing:
// - Summary CSV: `hold_time_s` (t of the first holding row minus t of the first row, s; empty when
//   the run never holds) and `hold_reason` (`none` when it never holds), right after `held`.
// - Run CSV: last column `hold`.
// - bench::run_profile(..., waypoints, config_override) and sweep params config.<section>.<key>
//   (config.safety.hal_timeout_ms) that override a scalar of the controller config.
// - The run engine passes timing.overrun_prob / overrun_cycles to SimJointInterface and still runs
//   round(duration * rate) cycles.
//
// Servo 250 Hz at 1 kHz with hal_timeout 3 ms. Read 0 (before start) and read 1 (row 0, before the
// first advance) are both at t = 0; read r >= 1 (row r - 1) is at (r - 1) ms. Packets on reads 0,
// 4, 8: reads 1..3 repeat stamp 0 (1, 2, 3 ms old; 3 ms is not > 3 ms), read 4 brings 3 ms (a 3 ms
// jump, not > 3 ms), reads 5..7 repeat it, read 8 brings 7 ms: a 4 ms jump > 3 ms -> hold at row 7
// (t = 7 ms). hold_time_s = 7 ms - 0 = 0.007. With 5 ms: at most 3 repeats and 4 ms jumps, never.
namespace cobot::bench {
namespace {

namespace fs = std::filesystem;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kConfig = kRepo / "config/default.yaml";
const fs::path kIdeal = kRepo / "config/imperfections/ideal.yaml";
constexpr double kDurationS = 0.1;  // 100 cycles
const std::string kTimeout = "config.safety.hal_timeout_ms";

const std::vector<std::string> kSummaryColumns = {"run_id",
                                                  "param",
                                                  "level",
                                                  "seed",
                                                  "final_pos_err_m",
                                                  "final_rot_err_rad",
                                                  "rms_joint_err_rad",
                                                  "peak_tau_ratio",
                                                  "sat_time_s",
                                                  "rms_tau_ratio",
                                                  "held",
                                                  "hold_time_s",
                                                  "hold_reason",
                                                  "passed"};

fs::path fresh_dir(const std::string& name) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "bench_comm_hold" / name;
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

std::string read_file(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

std::vector<std::string> split(const std::string& text, char sep) {
  std::vector<std::string> out;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, sep)) {
    out.push_back(item);
  }
  // getline drops a trailing empty field ("a,b," -> 2 items); keep it.
  if (!text.empty() && text.back() == sep) {
    out.emplace_back();
  }
  return out;
}

std::vector<std::vector<std::string>> read_csv(const fs::path& path) {
  std::vector<std::vector<std::string>> rows;
  for (const std::string& line : split(read_file(path), '\n')) {
    if (!line.empty()) {
      rows.push_back(split(line, ','));
    }
  }
  return rows;
}

imperfect::ImperfectionConfig<model::kArmDof> ideal_profile() {
  const auto r = pipeline::load_imperfections<model::kArmDof>(kIdeal);
  EXPECT_TRUE(r.is_ok()) << r.reason();
  return r.is_ok() ? r.value() : imperfect::ImperfectionConfig<model::kArmDof>{};
}

imperfect::ImperfectionConfig<model::kArmDof> servo_250() {
  imperfect::ImperfectionConfig<model::kArmDof> p = ideal_profile();
  p.comm.servo_rate_hz = 250.0;
  return p;
}

// ideal.yaml with its comm group replaced by `servo_rate_hz: 250` (all else 0).
fs::path servo_250_yaml(const fs::path& dir) {
  std::string text = read_file(kIdeal);
  const std::size_t start = text.find("\n  comm:");
  if (start != std::string::npos) {
    std::size_t end = text.find('\n', start + 1);
    while (end != std::string::npos && text.compare(end, 5, "\n    ") == 0) {
      end = text.find('\n', end + 1);
    }
    text.erase(start, (end == std::string::npos ? text.size() : end) - start);
  }
  while (!text.empty() && text.back() == '\n') {
    text.pop_back();
  }
  text +=
      "\n  comm:\n    source: \"test\"\n    servo_rate_hz: 250\n    packet_loss_prob: 0\n"
      "    latency_burst_prob: 0\n    burst_cycles: 0\n";
  const fs::path p = dir / "servo_250.yaml";
  std::ofstream(p, std::ios::binary) << text;
  return p;
}

TEST(BenchCommRun, ConfigOverrideChangesTheTimeout) {
  const fs::path dir = fresh_dir("override");
  const auto plain = run_profile(kConfig, servo_250(), "servo_250", "", kDurationS);
  ASSERT_TRUE(plain.is_ok()) << plain.reason();
  EXPECT_FALSE(plain.value().held) << "servo 250 Hz holds with hal_timeout 5 ms";

  const fs::path csv = dir / "run.csv";
  const auto tight = run_profile(
      kConfig, servo_250(), "servo_250", csv, kDurationS, nullptr, std::make_pair(kTimeout, 3.0));
  ASSERT_TRUE(tight.is_ok()) << tight.reason();
  EXPECT_TRUE(tight.value().held);

  // Run CSV: `hold` stays at index 56 (log channels follow it); none before the hold,
  // state_timeout from it on.
  const std::vector<std::vector<std::string>> rows = read_csv(csv);
  ASSERT_EQ(rows.size(), 101U);  // header + round(0.1 s * 1000 Hz) rows
  ASSERT_EQ(rows.front().size(), 114U);
  EXPECT_EQ(rows.front()[55], "step_us");
  EXPECT_EQ(rows.front()[56], "hold");
  for (std::size_t k = 1; k < rows.size(); ++k) {
    ASSERT_EQ(rows[k].size(), 114U) << "row " << k;
    const bool holding = rows[k][1] == "holding";
    EXPECT_EQ(holding, k >= 8) << "line " << k << " t_ns " << rows[k][0];
    EXPECT_EQ(rows[k][56], holding ? "state_timeout" : "none") << "line " << k;
  }
  EXPECT_EQ(rows[8][0], "7000000");  // line 8 = row 7
}

TEST(BenchCommRun, UnknownConfigOverrideIsAnError) {
  const auto r = run_profile(kConfig,
                             ideal_profile(),
                             "ideal",
                             "",
                             kDurationS,
                             nullptr,
                             std::make_pair(std::string("config.safety.no_such_key"), 1.0));
  ASSERT_FALSE(r.is_ok());
  EXPECT_NE(r.reason().find("config.safety.no_such_key"), std::string::npos) << r.reason();
}

// Overrun on every advance with one extra cycle: still round(0.1 s * 1000 Hz) = 100 rows. Read 0
// (before start) and row 0 see t = 0; each advance then adds 2 periods, so row r is at 2 r ms.
TEST(BenchCommRun, OverrunKeepsTheCycleCountAndStretchesPlantTime) {
  const fs::path dir = fresh_dir("overrun");
  imperfect::ImperfectionConfig<model::kArmDof> p = ideal_profile();
  p.timing.overrun_prob = 1.0;
  p.timing.overrun_cycles = 1;
  const fs::path csv = dir / "run.csv";
  const auto r = run_profile(kConfig, p, "overrun", csv, kDurationS);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const std::vector<std::vector<std::string>> rows = read_csv(csv);
  ASSERT_EQ(rows.size(), 101U);
  for (std::size_t k = 1; k < rows.size(); ++k) {
    EXPECT_EQ(std::stoll(rows[k][0]), static_cast<long long>(2 * (k - 1)) * 1'000'000LL)
        << "row " << k;
  }
}

using Row = std::map<std::string, std::string>;

std::vector<Row> read_summary(const fs::path& dir) {
  const std::vector<std::vector<std::string>> csv = read_csv(dir / "summary.csv");
  std::vector<Row> out;
  if (csv.empty()) {
    ADD_FAILURE() << "empty summary";
    return out;
  }
  EXPECT_EQ(csv.front(), kSummaryColumns);
  for (std::size_t k = 1; k < csv.size(); ++k) {
    EXPECT_EQ(csv[k].size(), kSummaryColumns.size()) << "row " << k;
    Row row;
    for (std::size_t c = 0; c < std::min(csv[k].size(), kSummaryColumns.size()); ++c) {
      row[kSummaryColumns[c]] = csv[k][c];
    }
    out.push_back(row);
  }
  return out;
}

// Two levels of the controller timeout over a servo-250 base: 3 ms holds at t = 7 ms with
// state_timeout, 5 ms never holds (hold_time_s empty, hold_reason none).
TEST(BenchCommSweep, HalTimeoutLevelsChangeTheTimeout) {
  const fs::path dir = fresh_dir("sweep");
  const fs::path base = servo_250_yaml(dir);
  const fs::path sweep_yaml = dir / "timeout_sweep.yaml";
  std::ofstream(sweep_yaml, std::ios::binary)
      << "base_profile: '" << base.string() << "'\nseeds: [1]\nparams:\n"
      << "  - name: " << kTimeout << "\n    levels: [3, 5]\n";
  const common::Status st = sweep(kConfig, sweep_yaml, dir / "out", kDurationS);
  ASSERT_TRUE(st.is_ok()) << st.reason();
  const std::vector<Row> rows = read_summary(dir / "out");
  ASSERT_EQ(rows.size(), 2U);
  const Row& at3 = rows[0];
  const Row& at5 = rows[1];
  EXPECT_EQ(at3.at("param"), kTimeout);
  EXPECT_EQ(std::stod(at3.at("level")), 3.0);
  EXPECT_EQ(at3.at("held"), "1");
  ASSERT_FALSE(at3.at("hold_time_s").empty());
  EXPECT_NEAR(std::stod(at3.at("hold_time_s")), 0.007, 1e-12);  // (7e6 ns - 0) * 1e-9
  EXPECT_EQ(at3.at("hold_reason"), "state_timeout");
  EXPECT_EQ(std::stod(at5.at("level")), 5.0);
  EXPECT_EQ(at5.at("held"), "0");
  EXPECT_EQ(at5.at("hold_time_s"), "");
  EXPECT_EQ(at5.at("hold_reason"), "none");
}

// Monte Carlo rows carry the new columns too; an ideal run never holds.
TEST(BenchCommSweep, MonteCarloSummaryHasTheHoldColumns) {
  const fs::path dir = fresh_dir("montecarlo");
  const common::Status st = montecarlo(kConfig, kIdeal, 2, 1, dir, false, kDurationS);
  ASSERT_TRUE(st.is_ok()) << st.reason();
  const std::vector<Row> rows = read_summary(dir);
  ASSERT_EQ(rows.size(), 2U);
  for (const Row& r : rows) {
    EXPECT_EQ(r.at("held"), "0");
    EXPECT_EQ(r.at("hold_time_s"), "");
    EXPECT_EQ(r.at("hold_reason"), "none");
  }
}

// An imperfection name keeps working next to the config params.
TEST(BenchCommSweep, ImperfectionParamsStillSweep) {
  const fs::path dir = fresh_dir("imperfection_param");
  const fs::path sweep_yaml = dir / "servo_sweep.yaml";
  std::ofstream(sweep_yaml, std::ios::binary)
      << "base_profile: '" << kIdeal.string() << "'\nseeds: [1]\nparams:\n"
      << "  - name: comm.servo_rate_hz\n    levels: [0, 250]\n"
      << "  - name: " << kTimeout << "\n    levels: [5]\n";
  const common::Status st = sweep(kConfig, sweep_yaml, dir / "out", kDurationS);
  ASSERT_TRUE(st.is_ok()) << st.reason();
  const std::vector<Row> rows = read_summary(dir / "out");
  ASSERT_EQ(rows.size(), 3U);
  EXPECT_EQ(rows[0].at("param"), "comm.servo_rate_hz");
  EXPECT_EQ(rows[1].at("param"), "comm.servo_rate_hz");
  EXPECT_EQ(rows[2].at("param"), kTimeout);
  for (const Row& r : rows) {
    EXPECT_EQ(r.at("held"), "0") << r.at("param") << " " << r.at("level");
  }
}

}  // namespace
}  // namespace cobot::bench
