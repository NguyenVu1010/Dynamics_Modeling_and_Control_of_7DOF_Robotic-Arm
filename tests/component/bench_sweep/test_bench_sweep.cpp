#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/lie.hpp"
#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"

#if __has_include("sweep.hpp")
#include "sweep.hpp"
#endif

// Monte Carlo and sweep modes of cobot_bench (feature sensitivity, design 11.2), declared in
// bench/sweep.hpp and built on the one-run engine bench::run (bench/run.hpp). Redeclared here so
// this test pins the signatures:
//
// montecarlo: runs seeds seed0 .. seed0 + runs - 1 on one imperfection profile and writes
//   `<out_dir>/summary.csv` (Summary CSV contract, docs/conventions.md; param = none,
//   level = 0). The per-run Run CSVs are written into out_dir only when keep_runs is true.
//   runs == 0 is an error.
// sweep: for each param and level of the sweep YAML (`base_profile`, `seeds`, `params` with
//   `name` = dotted key `<group>.<key>` of the imperfection profile and `levels`), the base
//   profile with that one key replaced (a scalar level is broadcast to every joint) is run
//   for each seed; one summary row per run in `<out_dir>/summary.csv`, rows in the order
//   param, level, seed of the YAML. An unknown param is an error whose reason names it.
// duration_s: as for bench::run (empty = trajectory.duration_s + hold_s).
// The summary metrics use the definitions of cobot_eval.metrics: final errors at the last row
// with t <= duration_s + hold_s, RMS of qref - q over rows in mode tracking or finished, the
// maximum over joints.
namespace cobot::bench {
common::Status montecarlo(const std::filesystem::path& config_yaml,
                          const std::filesystem::path& imperfections_yaml,
                          std::size_t runs,
                          std::uint64_t seed0,
                          const std::filesystem::path& out_dir,
                          bool keep_runs,
                          std::optional<double> duration_s);
common::Status sweep(const std::filesystem::path& config_yaml,
                     const std::filesystem::path& sweep_yaml,
                     const std::filesystem::path& out_dir,
                     std::optional<double> duration_s);
}  // namespace cobot::bench

namespace cobot::bench {
namespace {

namespace fs = std::filesystem;

using Row = std::map<std::string, std::string>;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kConfig = kRepo / "config/default.yaml";
const fs::path kIdeal = kRepo / "config/imperfections/ideal.yaml";
const fs::path kRealistic = kRepo / "config/imperfections/realistic.yaml";

const std::vector<std::string> kSummaryColumns = {"run_id",
                                                  "param",
                                                  "level",
                                                  "seed",
                                                  "final_pos_err_m",
                                                  "final_rot_err_rad",
                                                  "rms_joint_err_rad",
                                                  "peak_tau_ratio",
                                                  "sat_time_s",
                                                  "held",
                                                  "passed"};

// Fresh, empty output directory per test (re-runs must not see old files).
fs::path fresh_dir(const std::string& name) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "bench_sweep" / name;
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

std::string read_file(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

void write_file(const fs::path& path, const std::string& text) {
  std::ofstream out(path, std::ios::binary);
  out << text;
}

std::vector<std::string> split(const std::string& text, char sep) {
  std::vector<std::string> out;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, sep)) {
    out.push_back(item);
  }
  return out;
}

// Rows of a CSV (header first), each split into fields; empty lines dropped.
std::vector<std::vector<std::string>> read_csv(const fs::path& path) {
  std::vector<std::vector<std::string>> rows;
  for (const std::string& line : split(read_file(path), '\n')) {
    if (!line.empty()) {
      rows.push_back(split(line, ','));
    }
  }
  return rows;
}

// Column name -> value of one summary row; checks what every row must satisfy.
Row summary_row(const std::vector<std::string>& fields) {
  Row row;
  EXPECT_EQ(fields.size(), kSummaryColumns.size());
  for (std::size_t c = 0; c < std::min(fields.size(), kSummaryColumns.size()); ++c) {
    row[kSummaryColumns[c]] = fields[c];
  }
  EXPECT_FALSE(row["run_id"].empty());
  for (const char* name : {"level",
                           "final_pos_err_m",
                           "final_rot_err_rad",
                           "rms_joint_err_rad",
                           "peak_tau_ratio",
                           "sat_time_s"}) {
    const double v = std::stod(row[name]);
    EXPECT_TRUE(std::isfinite(v)) << name;
    EXPECT_GE(v, 0.0) << name;
  }
  // Gravity alone needs torque, so the peak torque ratio of any run is above 0.
  EXPECT_GT(std::stod(row["peak_tau_ratio"]), 0.0);
  for (const char* name : {"held", "passed"}) {
    EXPECT_TRUE(row[name] == "0" || row[name] == "1") << name << " = " << row[name];
  }
  return row;
}

std::vector<Row> read_summary(const fs::path& dir) {
  const fs::path path = dir / "summary.csv";
  EXPECT_TRUE(fs::is_regular_file(path)) << path;
  const std::vector<std::vector<std::string>> csv = read_csv(path);
  std::vector<Row> rows;
  if (csv.empty()) {
    ADD_FAILURE() << "empty " << path;
    return rows;
  }
  EXPECT_EQ(csv.front(), kSummaryColumns);
  for (std::size_t k = 1; k < csv.size(); ++k) {
    rows.push_back(summary_row(csv[k]));
  }
  return rows;
}

std::vector<fs::path> run_csvs(const fs::path& dir) {
  std::vector<fs::path> out;
  for (const fs::directory_entry& e : fs::recursive_directory_iterator(dir)) {
    if (e.is_regular_file() && e.path().extension() == ".csv" &&
        e.path().filename() != "summary.csv") {
      out.push_back(e.path());
    }
  }
  return out;
}

// Sweep YAML with an absolute base_profile, valid whatever base the loader resolves against.
fs::path write_sweep(const fs::path& dir, const std::string& params_yaml) {
  const fs::path path = dir / "sweep.yaml";
  write_file(path, "base_profile: " + kIdeal.string() + "\nseeds: [1]\nparams:\n" + params_yaml);
  return path;
}

TEST(BenchSweep, TwoParamsTwoLevelsOneSeedGiveFourContractRows) {
  const fs::path dir = fresh_dir("two_by_two");
  const fs::path sweep_yaml = write_sweep(dir,
                                          "  - name: sensor.encoder_bits\n"
                                          "    levels: [12, 16]\n"
                                          "  - name: actuator.command_delay_cycles\n"
                                          "    levels: [0, 2]\n");
  const fs::path out = dir / "out";
  const common::Status status = sweep(kConfig, sweep_yaml, out, 1.0);
  ASSERT_TRUE(status.is_ok()) << status.reason();

  std::vector<Row> rows = read_summary(out);
  ASSERT_EQ(rows.size(), 4U);  // 2 params * 2 levels * 1 seed
  const std::vector<std::string> params = {"sensor.encoder_bits",
                                           "sensor.encoder_bits",
                                           "actuator.command_delay_cycles",
                                           "actuator.command_delay_cycles"};
  const std::vector<double> levels = {12.0, 16.0, 0.0, 2.0};
  std::set<std::string> ids;
  for (std::size_t k = 0; k < rows.size(); ++k) {
    Row& row = rows[k];
    EXPECT_EQ(row["param"], params[k]) << "row " << k;
    EXPECT_DOUBLE_EQ(std::stod(row["level"]), levels[k]) << "row " << k;
    EXPECT_EQ(row["seed"], "1") << "row " << k;
    // 1 s run = 1000 cycles of 1 ms: saturation cannot last longer than the run.
    EXPECT_LE(std::stod(row["sat_time_s"]), 1.0 + 1e-9) << "row " << k;
    ids.insert(row["run_id"]);
  }
  EXPECT_EQ(ids.size(), 4U) << "run_id must be unique";

  // The level really reaches the run: a 12-bit encoder (step 2 pi / 4096 = 1.5e-3 rad) and a
  // 16-bit one (2 pi / 65536 = 9.6e-5 rad) cannot give the same measured joint error; neither
  // can 0 and 2 cycles of command delay.
  EXPECT_NE(rows[0]["rms_joint_err_rad"], rows[1]["rms_joint_err_rad"]);
  EXPECT_NE(rows[2]["rms_joint_err_rad"], rows[3]["rms_joint_err_rad"]);
}

TEST(BenchSweep, UnknownParamIsAnErrorNamingIt) {
  const fs::path dir = fresh_dir("unknown_param");
  const fs::path sweep_yaml = write_sweep(dir,
                                          "  - name: sensor.no_such_key\n"
                                          "    levels: [1, 2]\n");
  const common::Status status = sweep(kConfig, sweep_yaml, dir / "out", 0.1);
  EXPECT_FALSE(status.is_ok());
  EXPECT_NE(status.reason().find("sensor.no_such_key"), std::string::npos) << status.reason();
}

TEST(BenchSweep, UnknownGroupIsAnErrorNamingIt) {
  const fs::path dir = fresh_dir("unknown_group");
  const fs::path sweep_yaml = write_sweep(dir,
                                          "  - name: gearbox.backlash_rad\n"
                                          "    levels: [0.0]\n");
  const common::Status status = sweep(kConfig, sweep_yaml, dir / "out", 0.1);
  EXPECT_FALSE(status.is_ok());
  EXPECT_NE(status.reason().find("gearbox.backlash_rad"), std::string::npos) << status.reason();
}

TEST(BenchSweep, MissingSweepFileIsAnErrorNamingIt) {
  const fs::path dir = fresh_dir("missing_sweep");
  const common::Status status = sweep(kConfig, dir / "does_not_exist.yaml", dir / "out", 0.1);
  EXPECT_FALSE(status.is_ok());
  EXPECT_NE(status.reason().find("does_not_exist.yaml"), std::string::npos) << status.reason();
}

TEST(BenchMonteCarlo, RunsConsecutiveSeedsAndKeepsNoRunCsvByDefault) {
  const fs::path out = fresh_dir("mc_two_runs");
  const common::Status status = montecarlo(kConfig, kRealistic, 2, 5, out, false, 0.5);
  ASSERT_TRUE(status.is_ok()) << status.reason();

  std::vector<Row> rows = read_summary(out);
  ASSERT_EQ(rows.size(), 2U);
  for (std::size_t k = 0; k < rows.size(); ++k) {
    EXPECT_EQ(rows[k]["param"], "none") << "row " << k;
    EXPECT_DOUBLE_EQ(std::stod(rows[k]["level"]), 0.0) << "row " << k;
  }
  // seed0 = 5, runs = 2 -> seeds 5 and 6.
  EXPECT_EQ(rows[0]["seed"], "5");
  EXPECT_EQ(rows[1]["seed"], "6");
  EXPECT_NE(rows[0]["run_id"], rows[1]["run_id"]);
  // realistic draws encoder offsets and noise from the seed, so the two runs differ.
  EXPECT_NE(rows[0]["rms_joint_err_rad"], rows[1]["rms_joint_err_rad"]);
  EXPECT_TRUE(run_csvs(out).empty()) << "per-run CSVs are written only with keep_runs";
}

TEST(BenchMonteCarlo, ZeroRunsIsAnError) {
  const fs::path out = fresh_dir("mc_zero_runs");
  EXPECT_FALSE(montecarlo(kConfig, kIdeal, 0, 1, out, false, 0.1).is_ok());
}

bool near(double actual, double expected) {
  return std::abs(actual - expected) <= 1e-9 + 1e-6 * std::abs(expected);
}

// The C++ summary duplicates the metric definitions of cobot_eval.metrics; recompute them here
// from the kept Run CSV of one run.
TEST(BenchMonteCarlo, SummaryMetricsMatchTheKeptRunCsv) {
  const fs::path out = fresh_dir("mc_keep");
  const common::Status status = montecarlo(kConfig, kRealistic, 1, 3, out, true, 0.5);
  ASSERT_TRUE(status.is_ok()) << status.reason();
  std::vector<Row> rows = read_summary(out);
  ASSERT_EQ(rows.size(), 1U);
  Row& row = rows.front();
  EXPECT_EQ(row["seed"], "3");

  const std::vector<fs::path> kept = run_csvs(out);
  ASSERT_EQ(kept.size(), 1U);
  const std::vector<std::vector<std::string>> csv = read_csv(kept.front());
  ASSERT_GE(csv.size(), 2U);
  std::map<std::string, std::size_t> col;
  for (std::size_t c = 0; c < csv.front().size(); ++c) {
    col[csv.front()[c]] = c;
  }
  ASSERT_EQ(col.size(), 56U);
  const auto value = [&col](const std::vector<std::string>& r, const std::string& name) {
    return std::stod(r.at(col.at(name)));
  };

  // 0.5 s < duration_s + hold_s = 10 + 2 = 12 s, so the final errors are the last row's.
  const std::vector<std::string>& last = csv.back();
  common::Vec3 p;
  common::Vec3 pd;
  common::Vec3 r;
  common::Vec3 rd;
  const std::vector<std::string> axes = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    p(i) = value(last, "p" + axes[i]);
    pd(i) = value(last, "pd" + axes[i]);
    r(i) = value(last, "r" + axes[i]);
    rd(i) = value(last, "rd" + axes[i]);
  }
  const double pos_err = (p - pd).norm();
  // Angle of R(rd) R(r)^T, as cobot_eval.metrics.rotation_angle.
  const double rot_err =
      common::so3_log(common::so3_exp(rd) * common::so3_exp(r).transpose()).norm();

  // RMS of qref - q per joint over rows in tracking or finished, then the maximum over joints.
  std::vector<double> sum_sq(7, 0.0);
  std::size_t n = 0;
  bool held = false;
  for (std::size_t k = 1; k < csv.size(); ++k) {
    const std::string& mode = csv[k].at(col.at("mode"));
    held = held || mode == "holding";
    if (mode != "tracking" && mode != "finished") {
      continue;
    }
    ++n;
    for (std::size_t j = 0; j < sum_sq.size(); ++j) {
      const std::string idx = std::to_string(j + 1);
      const double e = value(csv[k], "qref" + idx) - value(csv[k], "q" + idx);
      sum_sq[j] += e * e;
    }
  }
  ASSERT_GT(n, 0U);
  double rms = 0.0;
  for (const double s : sum_sq) {
    rms = std::max(rms, std::sqrt(s / static_cast<double>(n)));
  }

  EXPECT_PRED2(near, std::stod(row["final_pos_err_m"]), pos_err);
  EXPECT_PRED2(near, std::stod(row["final_rot_err_rad"]), rot_err);
  EXPECT_PRED2(near, std::stod(row["rms_joint_err_rad"]), rms);
  EXPECT_EQ(row["held"], held ? "1" : "0");
}

}  // namespace
}  // namespace cobot::bench
