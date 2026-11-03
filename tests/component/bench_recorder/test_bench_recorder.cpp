// The bench run engine on a Recorder (feature pipeline-recorder, cobot_studio design 6.3): no
// writer thread, the loop records into HeapStorage(cycles, 1) and the Run CSV is written after
// the loop. Same bytes for the same seed, one row per cycle; a duration whose in-memory log would
// exceed 2 GiB is refused before anything is allocated or written; timing() runs with log.mode
// off (no recorder).
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/result.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/pipeline/config.hpp"

#include "engine.hpp"
#include "run.hpp"

namespace cobot::bench {
namespace {

namespace fs = std::filesystem;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kConfig = kRepo / "config/default.yaml";
const fs::path kRealistic = kRepo / "config/imperfections/realistic.yaml";
const fs::path kUrdf = kRepo / "src/sim/urdf/cobot30_5.urdf";

std::string read_file(const fs::path& p) {
  const std::ifstream in(p, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

fs::path out_dir() {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "bench_recorder";
  fs::create_directories(dir);
  return dir;
}

// A Run CSV path in out_dir() with any earlier output (CSV and meta file) removed.
fs::path fresh_csv(const std::string& name) {
  const fs::path p = out_dir() / name;
  fs::remove(p);
  fs::remove(p.string() + ".meta.yaml");
  return p;
}

std::vector<std::string> lines_of(const std::string& text) {
  std::vector<std::string> out;
  std::stringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    out.push_back(line);
  }
  return out;
}

imperfect::ImperfectionConfig<7> realistic_seed(std::uint64_t seed) {
  auto r = pipeline::load_imperfections<7>(kRealistic);
  EXPECT_TRUE(r.is_ok()) << r.reason();
  imperfect::ImperfectionConfig<7> profile =
      r.is_ok() ? r.value() : imperfect::ImperfectionConfig<7>{};
  profile.seed = seed;
  return profile;
}

// config/default.yaml with an absolute URDF path and `  log:` set to `body` (lines indented by
// four spaces), written to out_dir() / name.
fs::path default_with_log(const std::string& name, const std::string& body) {
  std::string text = read_file(kConfig);
  const std::string rel = "../src/sim/urdf/cobot30_5.urdf";
  const auto pos = text.find(rel);
  EXPECT_NE(pos, std::string::npos) << "fixture text not found: " << rel;
  if (pos != std::string::npos) {
    text.replace(pos, rel.size(), kUrdf.string());
  }
  std::istringstream in(text);
  std::string out;
  std::string line;
  bool inside = false;
  bool replaced = false;
  while (std::getline(in, line)) {
    if (line.rfind("  log:", 0) == 0) {
      inside = true;
      replaced = true;
      out += "  log:\n" + body;
      continue;
    }
    if (inside && (line.rfind("    ", 0) == 0 || line.empty())) {
      continue;
    }
    inside = false;
    out += line + "\n";
  }
  EXPECT_TRUE(replaced) << "no log section in config/default.yaml";
  const fs::path p = out_dir() / name;
  std::ofstream(p, std::ios::binary) << out;
  return p;
}

// default + realistic, seed 3, 2 s at 1000 Hz = 2000 cycles: the same bytes twice, header plus
// 2000 rows.
TEST(BenchRecorder, SameSeedTwiceGivesTheSameBytesWithOneRowPerCycle) {
  const fs::path a = fresh_csv("realistic_seed3_a.csv");
  const fs::path b = fresh_csv("realistic_seed3_b.csv");
  const auto ra = run_profile(kConfig, realistic_seed(3), kRealistic, a, 2.0);
  ASSERT_TRUE(ra.is_ok()) << ra.reason();
  const auto rb = run_profile(kConfig, realistic_seed(3), kRealistic, b, 2.0);
  ASSERT_TRUE(rb.is_ok()) << rb.reason();
  const std::string bytes = read_file(a);
  ASSERT_FALSE(bytes.empty());
  EXPECT_EQ(bytes, read_file(b));
  EXPECT_EQ(bytes.back(), '\n');
  const std::vector<std::string> lines = lines_of(bytes);
  EXPECT_EQ(lines.size(), 1U + 2000U);
  EXPECT_EQ(lines.front().rfind("t_ns,mode,", 0), 0U) << lines.front();
  EXPECT_TRUE(fs::exists(a.string() + ".meta.yaml"));
  // The run summary does not depend on the recorder either.
  EXPECT_EQ(ra.value().final_pos_err_m, rb.value().final_pos_err_m);
  EXPECT_EQ(ra.value().rms_joint_err_rad, rb.value().rms_joint_err_rad);
}

// 1e6 s at 1000 Hz = 1e9 cycles; at sizeof(LogRecord<7>) > 500 bytes that is > 5e11 bytes, far
// above the 2 GiB = 2147483648 bytes limit: an error naming the in-memory log and duration_s,
// and neither the Run CSV nor its meta file is written.
TEST(BenchRecorder, DurationBeyondTheInMemoryLogLimitIsRefused) {
  static_assert(sizeof(pipeline::LogRecord<7>) > 500, "the arithmetic above assumes > 500 B");
  const fs::path p = fresh_csv("too_long.csv");
  const auto r = run_profile(kConfig, realistic_seed(3), kRealistic, p, 1e6);
  ASSERT_FALSE(r.is_ok());
  EXPECT_NE(r.reason().find("in-memory log"), std::string::npos) << r.reason();
  EXPECT_NE(r.reason().find("duration_s"), std::string::npos) << r.reason();
  EXPECT_NE(r.reason().find("2048 MB"), std::string::npos) << r.reason();
  EXPECT_FALSE(fs::exists(p));
  EXPECT_FALSE(fs::exists(p.string() + ".meta.yaml"));
  // bench::run (the CLI path) reports the same.
  const common::Status s = run(kConfig, kRealistic, std::uint64_t{3}, p, 1e6);
  ASSERT_FALSE(s.is_ok());
  EXPECT_NE(s.reason().find("in-memory log"), std::string::npos) << s.reason();
  EXPECT_FALSE(fs::exists(p));
}

void expect_finite_stats(const common::Result<TimingStats>& r) {
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const TimingStats& t = r.value();
  EXPECT_TRUE(std::isfinite(t.p50_us));
  EXPECT_TRUE(std::isfinite(t.p95_us));
  EXPECT_TRUE(std::isfinite(t.max_us));
  EXPECT_GE(t.p50_us, 0.0);
  EXPECT_LE(t.p50_us, t.p95_us);
  EXPECT_LE(t.p95_us, t.max_us);
}

TEST(BenchRecorder, TimingWithLogModeOffReturnsFiniteStats) {
  const fs::path off =
      default_with_log("log_off.yaml", "    output_dir: ../results\n    mode: off\n");
  const auto cfg = pipeline::load_config<7>(off);
  ASSERT_TRUE(cfg.is_ok()) << cfg.reason();
  ASSERT_EQ(cfg.value().log_mode, pipeline::LogMode::kOff);
  expect_finite_stats(timing(off, 200));
}

TEST(BenchRecorder, TimingWithTheRecorderReturnsFiniteStats) {
  const auto cfg = pipeline::load_config<7>(kConfig);
  ASSERT_TRUE(cfg.is_ok()) << cfg.reason();
  ASSERT_EQ(cfg.value().log_mode, pipeline::LogMode::kRecorder);
  expect_finite_stats(timing(kConfig, 200));
}

}  // namespace
}  // namespace cobot::bench
