#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "cobot/common/result.hpp"

#if __has_include("run.hpp")
#include "run.hpp"
#endif

// `cobot_bench run ... --waypoints FILE` (feature bench-waypoints, design 2026-10-29 sections
// 6.6 and 7). Redeclared here so this test pins the signature of the one-run engine: bench::run
// gains `waypoints_yaml` as its last parameter. With a value the run
//   - forces command_source external, starts the loop (-> standby at the home pose),
//   - loads the file (pipeline::load_waypoints; a bad file is an error naming the file),
//   - runs pipeline::validate_path from the standby reference pose and the measured q; when it
//     is not ok the run fails with one line per issue, "waypoint <i>: <issue> ..." (i is the
//     0-based index of the file's waypoints[i], as WaypointIssue::waypoint, and <issue> is
//     to_string(PathIssue)), and writes neither the Run CSV nor its meta file,
//   - otherwise loads the validated trajectory and runs trajectory duration + trajectory.hold_s
//     (or duration_s when set), one row per cycle,
//   - adds to `<out_csv>.meta.yaml`: waypoints_file, waypoint_node_times_s [n],
//     waypoint_stop [n], waypoint_positions_m [[x, y, z] x n] (tip positions in base_link).
// The binary: exit 2 on a bad waypoint file (reason naming it on stderr, as for other bad
// input), exit 1 when validation fails (each issue printed), 0 on a completed run.
namespace cobot::bench {
// NOLINTNEXTLINE(readability-redundant-declaration): pins the signature declared in run.hpp
common::Status run(const std::filesystem::path& config_yaml,
                   const std::filesystem::path& imperfections_yaml,
                   std::optional<std::uint64_t> seed,
                   const std::filesystem::path& out_csv,
                   std::optional<double> duration_s,
                   const std::optional<std::filesystem::path>& waypoints_yaml);
}  // namespace cobot::bench

namespace cobot::bench {
namespace {

namespace fs = std::filesystem;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kConfig = kRepo / "config/default.yaml";
const fs::path kIdeal = kRepo / "config/imperfections/ideal.yaml";
const fs::path kE2e10 = kRepo / "tests/e2e/waypoints/e2e_10.yaml";
const fs::path kUnreachable = kRepo / "tests/data/waypoints/unreachable_last.yaml";
const fs::path kBadPosition = kRepo / "tests/data/waypoints/bad_position.yaml";
const fs::path kMissing = kRepo / "tests/data/waypoints/does_not_exist.yaml";

// Tip positions of tests/e2e/waypoints/e2e_10.yaml (frame base_link, tcp_offset identity).
const std::array<std::array<double, 3>, 4> kE2e10Positions = {{{-0.0730, -0.7449, 0.5424},
                                                               {-0.2255, -0.6931, 0.6701},
                                                               {-0.3780, -0.6414, 0.7978},
                                                               {-0.5000, -0.6000, 0.9000}}};
constexpr double kHoldS = 2.0;  // config/default.yaml trajectory.hold_s
constexpr double kRateHz = 1000.0;
// Run CSV columns (docs/conventions.md): t_ns, mode, 5 x 7 joint columns, then px, py, pz.
// 2 + 5 * 7 + 12 + 6 + 1 = 56 columns.
constexpr std::size_t kPxColumn = 2 + 5 * 7;
constexpr std::size_t kColumns = 56;

fs::path meta_of(const fs::path& csv) {
  return csv.string() + ".meta.yaml";
}

fs::path out_path(const std::string& name) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "bench_waypoints";
  fs::create_directories(dir);
  const fs::path path = dir / name;
  fs::remove(path);
  fs::remove(meta_of(path));
  return path;
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
  return out;
}

// Data rows of a Run CSV, each split into its fields.
std::vector<std::vector<std::string>> rows_of(const fs::path& csv) {
  const std::vector<std::string> lines = split(read_file(csv), '\n');
  std::vector<std::vector<std::string>> rows;
  for (std::size_t k = 1; k < lines.size(); ++k) {
    rows.push_back(split(lines[k], ','));
  }
  return rows;
}

struct Cli {
  int exit_code = -1;
  std::string output;  // stdout and stderr
};

Cli run_bench(const std::string& args) {
  const std::string cmd = std::string("'") + COBOT_BENCH_EXE + "' " + args + " 2>&1";
  Cli cli;
  FILE* pipe = popen(cmd.c_str(), "r");
  if (pipe == nullptr) {
    return cli;
  }
  std::array<char, 4096> buf{};
  std::size_t n = 0;
  while ((n = std::fread(buf.data(), 1, buf.size(), pipe)) > 0) {
    cli.output.append(buf.data(), n);
  }
  const int status = pclose(pipe);
  cli.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  return cli;
}

std::string quoted(const fs::path& path) {
  return "'" + path.string() + "'";
}

std::string run_args(const fs::path& waypoints, const fs::path& out) {
  return "run --config " + quoted(kConfig) + " --imperfections " + quoted(kIdeal) +
         " --seed 1 --out " + quoted(out) + " --waypoints " + quoted(waypoints);
}

TEST(BenchWaypointsCli, InvalidWaypointFileExitsTwoNamingTheFile) {
  const fs::path out = out_path("cli_bad.csv");
  const Cli cli = run_bench(run_args(kBadPosition, out) + " --duration-s 0.1");
  EXPECT_EQ(cli.exit_code, 2) << cli.output;
  EXPECT_NE(cli.output.find("bad_position.yaml"), std::string::npos) << cli.output;
  EXPECT_FALSE(fs::exists(out));
}

TEST(BenchWaypointsCli, MissingWaypointFileExitsTwoNamingTheFile) {
  const fs::path out = out_path("cli_missing.csv");
  const Cli cli = run_bench(run_args(kMissing, out) + " --duration-s 0.1");
  EXPECT_EQ(cli.exit_code, 2) << cli.output;
  EXPECT_NE(cli.output.find("does_not_exist.yaml"), std::string::npos) << cli.output;
  EXPECT_FALSE(fs::exists(out));
}

TEST(BenchWaypointsCli, UnreachableListExitsOneNamingTheWaypointAndWritesNoCsv) {
  const fs::path out = out_path("cli_unreachable.csv");
  const Cli cli = run_bench(run_args(kUnreachable, out));
  EXPECT_EQ(cli.exit_code, 1) << cli.output;
  // waypoints[2] is ~2.8 m from the base: its segment and node fail. Waypoint 1 stops, so the
  // segments into waypoints 0 and 1 do not depend on waypoint 2 and stay clean.
  EXPECT_NE(cli.output.find("waypoint 2: unreachable"), std::string::npos) << cli.output;
  EXPECT_EQ(cli.output.find("waypoint 0:"), std::string::npos) << cli.output;
  EXPECT_EQ(cli.output.find("waypoint 1:"), std::string::npos) << cli.output;
  EXPECT_FALSE(fs::exists(out));
  EXPECT_FALSE(fs::exists(meta_of(out)));
}

TEST(BenchWaypointsCli, ValidListWithDurationExitsZeroWithOneRowPerCycle) {
  const fs::path out = out_path("cli_valid.csv");
  const Cli cli = run_bench(run_args(kE2e10, out) + " --duration-s 0.5");
  ASSERT_EQ(cli.exit_code, 0) << cli.output;
  // 0.5 s * 1000 Hz = 500 cycles (+-1 for the end-point rule).
  const std::size_t rows = rows_of(out).size();
  EXPECT_GE(rows, 499U);
  EXPECT_LE(rows, 501U);
}

TEST(BenchWaypoints, UnreachableListIsAnErrorNamingTheWaypointAndWritesNoCsv) {
  const fs::path out = out_path("api_unreachable.csv");
  const common::Status status =
      run(kConfig, kIdeal, std::uint64_t{1}, out, std::nullopt, kUnreachable);
  ASSERT_FALSE(status.is_ok());
  EXPECT_NE(status.reason().find("waypoint 2: unreachable"), std::string::npos) << status.reason();
  EXPECT_FALSE(fs::exists(out));
  EXPECT_FALSE(fs::exists(meta_of(out)));
}

TEST(BenchWaypoints, BadWaypointFileIsAnErrorNamingTheFile) {
  const fs::path out = out_path("api_bad.csv");
  const common::Status status = run(kConfig, kIdeal, std::uint64_t{1}, out, 0.1, kBadPosition);
  ASSERT_FALSE(status.is_ok());
  EXPECT_NE(status.reason().find("bad_position.yaml"), std::string::npos) << status.reason();
  EXPECT_FALSE(fs::exists(out));
}

TEST(BenchWaypoints, ValidListRowsCoverTrajectoryPlusHoldAndMetaHasWaypointKeys) {
  const fs::path out = out_path("api_valid.csv");
  const common::Status status = run(kConfig, kIdeal, std::uint64_t{1}, out, std::nullopt, kE2e10);
  ASSERT_TRUE(status.is_ok()) << status.reason();

  const YAML::Node meta = YAML::LoadFile(meta_of(out).string());
  ASSERT_TRUE(meta["waypoints_file"]) << read_file(meta_of(out));
  EXPECT_NE(meta["waypoints_file"].as<std::string>().find("e2e_10.yaml"), std::string::npos);
  // The keys written without waypoints stay.
  EXPECT_TRUE(meta["config"] && meta["imperfections"] && meta["seed"]);

  const auto times = meta["waypoint_node_times_s"].as<std::vector<double>>();
  ASSERT_EQ(times.size(), 4U);
  EXPECT_GT(times[0], 0.0);
  for (std::size_t i = 1; i < times.size(); ++i) {
    EXPECT_GT(times[i], times[i - 1]) << "node " << i;
  }
  // User durations are used as given (meta written with full precision): 3.0 s into
  // waypoint 1, 3.5 s into waypoint 3.
  EXPECT_NEAR(times[1] - times[0], 3.0, 1e-9);
  EXPECT_NEAR(times[3] - times[2], 3.5, 1e-9);

  const auto stop = meta["waypoint_stop"].as<std::vector<bool>>();
  EXPECT_EQ(stop, (std::vector<bool>{false, true, false, true}));

  const auto positions = meta["waypoint_positions_m"].as<std::vector<std::vector<double>>>();
  ASSERT_EQ(positions.size(), 4U);
  for (std::size_t i = 0; i < positions.size(); ++i) {
    ASSERT_EQ(positions[i].size(), 3U) << "waypoint " << i;
    for (std::size_t a = 0; a < 3; ++a) {
      EXPECT_NEAR(positions[i][a], kE2e10Positions.at(i).at(a), 1e-9) << "waypoint " << i;
    }
  }

  // Duration = trajectory duration (last node time) + hold_s 2.0, one row per 1 ms cycle.
  const std::vector<std::vector<std::string>> rows = rows_of(out);
  const auto expected = static_cast<double>(std::llround((times.back() + kHoldS) * kRateHz));
  EXPECT_NEAR(static_cast<double>(rows.size()), expected, 1.0);

  // External mode: only standby and tracking rows; the run ends in standby at the last
  // waypoint (ideal plant: within 1 mm).
  bool tracked = false;
  for (std::size_t k = 0; k < rows.size(); ++k) {
    ASSERT_EQ(rows[k].size(), kColumns) << "row " << k;
    const std::string& mode = rows[k][1];
    EXPECT_TRUE(mode == "standby" || mode == "tracking") << "row " << k << ": " << mode;
    tracked = tracked || mode == "tracking";
  }
  EXPECT_TRUE(tracked);
  ASSERT_FALSE(rows.empty());
  EXPECT_EQ(rows.back()[1], "standby");
  double err_sq = 0.0;
  for (std::size_t a = 0; a < 3; ++a) {
    const double d = std::stod(rows.back()[kPxColumn + a]) - kE2e10Positions[3].at(a);
    err_sq += d * d;
  }
  EXPECT_LT(std::sqrt(err_sq), 1e-3);
}

TEST(BenchWaypoints, WithoutWaypointsTheMetaHasNoWaypointKeys) {
  const fs::path out = out_path("api_config_mode.csv");
  const common::Status status = run(kConfig, kIdeal, std::uint64_t{1}, out, 0.1, std::nullopt);
  ASSERT_TRUE(status.is_ok()) << status.reason();
  const YAML::Node meta = YAML::LoadFile(meta_of(out).string());
  for (const char* key :
       {"waypoints_file", "waypoint_node_times_s", "waypoint_stop", "waypoint_positions_m"}) {
    EXPECT_FALSE(meta[key]) << key;
  }
  // Config mode is unchanged: the MVP trajectory is still tracking at 0.1 s.
  const std::vector<std::vector<std::string>> rows = rows_of(out);
  ASSERT_FALSE(rows.empty());
  EXPECT_EQ(rows.back()[1], "tracking");
}

}  // namespace
}  // namespace cobot::bench
