#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/result.hpp"

#if __has_include("run.hpp")
#include "run.hpp"
#endif

// One-run engine of `cobot_bench run` (feature bench-e2e), declared in bench/run.hpp and reused
// by the montecarlo/sweep modes. Redeclared here so this test pins the signature:
//   config_yaml, imperfections_yaml: as for load_config / load_imperfections;
//   seed: overrides the profile's seed when set;
//   out_csv: Run CSV (contract in docs/conventions.md); `<out_csv>.meta.yaml` is written next
//            to it with at least the keys `config`, `imperfections` and `seed`;
//   duration_s: total simulated time in s; when empty, trajectory.duration_s + hold_s. The loop
//               runs round(duration_s * control_rate_hz) cycles and logs one row per cycle.
// Errors (bad paths, bad YAML) come back as a Status whose reason names the offending file.
namespace cobot::bench {
common::Status run(const std::filesystem::path& config_yaml,
                   const std::filesystem::path& imperfections_yaml,
                   std::optional<std::uint64_t> seed,
                   const std::filesystem::path& out_csv,
                   std::optional<double> duration_s);
}  // namespace cobot::bench

namespace cobot::bench {
namespace {

namespace fs = std::filesystem;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kConfig = kRepo / "config/default.yaml";
const fs::path kIdeal = kRepo / "config/imperfections/ideal.yaml";
const fs::path kRealistic = kRepo / "config/imperfections/realistic.yaml";

fs::path out_path(const std::string& name) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "bench_run";
  fs::create_directories(dir);
  return dir / name;
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

// Built from the conventions contract, independently of pipeline::write_csv_header.
// 2 + 5 * 7 + 12 + 6 + 1 = 56 columns.
std::string contract_header() {
  std::string h = "t_ns,mode";
  for (const char* prefix : {"q", "qd", "qref", "qdref", "tau"}) {
    for (int j = 1; j <= 7; ++j) {
      h += "," + std::string(prefix) + std::to_string(j);
    }
  }
  return h + ",px,py,pz,rx,ry,rz,pdx,pdy,pdz,rdx,rdy,rdz,e1,e2,e3,e4,e5,e6,step_us";
}

TEST(BenchRun, IdealTwoSecondsWritesContractCsvWithOneRowPerCycle) {
  const fs::path out = out_path("ideal_2s.csv");
  const common::Status status = run(kConfig, kIdeal, std::uint64_t{1}, out, 2.0);
  ASSERT_TRUE(status.is_ok()) << status.reason();

  const std::vector<std::string> lines = split(read_file(out), '\n');
  ASSERT_FALSE(lines.empty());
  EXPECT_EQ(lines.front(), contract_header());
  // 2 s * control_rate_hz 1000 = 2000 cycles, one row each (+-1 for the end-point rule).
  const std::size_t rows = lines.size() - 1;
  EXPECT_GE(rows, 1999U);
  EXPECT_LE(rows, 2001U);

  long long last_t = -1;
  for (std::size_t k = 1; k < lines.size(); ++k) {
    const std::vector<std::string> f = split(lines[k], ',');
    ASSERT_EQ(f.size(), 56U) << "row " << k;
    const long long t = std::stoll(f[0]);
    EXPECT_GT(t, last_t) << "row " << k;
    last_t = t;
    // 2 s is inside the 10 s MVP trajectory: the ideal arm is still tracking.
    EXPECT_EQ(f[1], "tracking") << "row " << k;
    // default.yaml has deterministic: true -> step_us written as 0.
    EXPECT_EQ(f.back(), "0") << "row " << k;
  }
}

TEST(BenchRun, SameSeedGivesIdenticalBytesOtherSeedDiffers) {
  const fs::path a = out_path("realistic_a.csv");
  const fs::path b = out_path("realistic_b.csv");
  const fs::path c = out_path("realistic_c.csv");
  ASSERT_TRUE(run(kConfig, kRealistic, std::uint64_t{7}, a, 0.5).is_ok());
  ASSERT_TRUE(run(kConfig, kRealistic, std::uint64_t{7}, b, 0.5).is_ok());
  ASSERT_TRUE(run(kConfig, kRealistic, std::uint64_t{8}, c, 0.5).is_ok());
  const std::string bytes_a = read_file(a);
  ASSERT_FALSE(bytes_a.empty());
  EXPECT_EQ(bytes_a, read_file(b));
  // realistic draws encoder offsets and noise from the seed, so the measured q differs.
  EXPECT_NE(bytes_a, read_file(c));

  // The CLI seed overrides the profile's `seed: 1` and is recorded in the meta file.
  const std::string meta = read_file(a.string() + ".meta.yaml");
  EXPECT_NE(meta.find("config:"), std::string::npos) << meta;
  EXPECT_NE(meta.find("imperfections:"), std::string::npos) << meta;
  EXPECT_NE(meta.find("seed: 7"), std::string::npos) << meta;
}

TEST(BenchRun, MissingConfigIsAnErrorNamingTheFile) {
  const fs::path missing = kRepo / "config/does_not_exist.yaml";
  const common::Status status = run(missing, kIdeal, std::nullopt, out_path("missing.csv"), 0.1);
  EXPECT_FALSE(status.is_ok());
  EXPECT_NE(status.reason().find("does_not_exist.yaml"), std::string::npos) << status.reason();
}

}  // namespace
}  // namespace cobot::bench
