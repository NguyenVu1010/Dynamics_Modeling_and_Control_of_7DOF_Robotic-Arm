// PF-02 (feature shm-bridge, cobot_studio design 7, ADR-022 "PF-02"): `cobot_bench timing --paced`
// steps the loop at 1 kHz with clock_nanosleep and times step() (plus to_sample and publish with
// log.mode recorder_live, a reader thread draining the bridge state ring). CI gate: p99 < 1 ms and
// no allocation per step; the real numbers are measured by hand. This binary replaces the global
// operator new (count_new.hpp) and runs alone (RUN_SERIAL).
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <sched.h>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/result.hpp"
#include "cobot/pipeline/config.hpp"

#include "count_new.hpp"
#include "run.hpp"

namespace cobot::bench {
namespace {

namespace fs = std::filesystem;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kConfig = kRepo / "config/default.yaml";
const char* const kBenchExe = COBOT_BENCH_EXE;

constexpr std::int64_t kSteps = 2000;
constexpr double kP99GateUs = 1000.0;  // ADR-022: loose CI gate, one control period

struct Mode {
  pipeline::LogMode mode;
  const char* name;
};
const Mode kModes[] = {
    {pipeline::LogMode::kOff, "off"},
    {pipeline::LogMode::kRecorder, "recorder"},
    {pipeline::LogMode::kRecorderLive, "recorder_live"},
};

TimingOptions paced(pipeline::LogMode mode) {
  TimingOptions o;
  o.paced = true;
  o.log_mode = mode;
  return o;
}

std::size_t allowed_cpus() {
  cpu_set_t set;
  CPU_ZERO(&set);
  if (::sched_getaffinity(0, sizeof(set), &set) != 0) {
    return 1;
  }
  return static_cast<std::size_t>(CPU_COUNT(&set));
}

// The bench's own region, "/cobot_timing_<pid>" under /dev/shm.
fs::path timing_region() {
  return fs::path("/dev/shm") / ("cobot_timing_" + std::to_string(static_cast<long>(::getpid())));
}

void record(const std::string& prefix, const TimingStats& s) {
  const auto prop = [&prefix](const char* key, const std::string& value) {
    ::testing::Test::RecordProperty(prefix + "_" + key, value);
  };
  prop("p50_us", std::to_string(s.p50_us));
  prop("p95_us", std::to_string(s.p95_us));
  prop("p99_us", std::to_string(s.p99_us));
  prop("max_us", std::to_string(s.max_us));
  prop("late_cycles", std::to_string(s.late_cycles));
  prop("late_p99_us", std::to_string(s.late_p99_us));
  prop("reader_samples", std::to_string(s.reader_samples));
  prop("reader_lost", std::to_string(s.reader_lost));
  std::printf(
      "%s: p50_us=%.3f p95_us=%.3f p99_us=%.3f max_us=%.3f late_cycles=%llu late_p99_us=%.3f "
      "reader_samples=%llu reader_lost=%llu reader_cpu=%s\n",
      prefix.c_str(),
      s.p50_us,
      s.p95_us,
      s.p99_us,
      s.max_us,
      static_cast<unsigned long long>(s.late_cycles),
      s.late_p99_us,
      static_cast<unsigned long long>(s.reader_samples),
      static_cast<unsigned long long>(s.reader_lost),
      s.reader_on_other_cpu ? "other" : "shared");
}

TEST(BenchPacedTiming, EachLogModeMeetsTheLooseGate) {
  for (const Mode& m : kModes) {
    const common::Result<TimingStats> r = timing(kConfig, kSteps, paced(m.mode));
    ASSERT_TRUE(r.is_ok()) << m.name << ": " << r.reason();
    const TimingStats& s = r.value();
    record(m.name, s);
    EXPECT_GT(s.p50_us, 0.0) << m.name;
    EXPECT_LE(s.p50_us, s.p95_us) << m.name;
    EXPECT_LE(s.p95_us, s.p99_us) << m.name;
    EXPECT_LE(s.p99_us, s.max_us) << m.name;
    EXPECT_LT(s.p99_us, kP99GateUs) << m.name;
    EXPECT_LE(s.late_cycles, static_cast<std::uint64_t>(kSteps)) << m.name;
    if (m.mode == pipeline::LogMode::kRecorderLive) {
      EXPECT_GT(s.reader_samples, 0U);
      EXPECT_EQ(s.reader_samples + s.reader_lost, static_cast<std::uint64_t>(kSteps))
          << "the reader drains once more after the last step";
      EXPECT_EQ(s.reader_on_other_cpu, allowed_cpus() >= 2);
    } else {
      EXPECT_EQ(s.reader_samples, 0U) << m.name << ": no bridge";
      EXPECT_EQ(s.reader_lost, 0U) << m.name;
      EXPECT_FALSE(s.reader_on_other_cpu) << m.name;
    }
    EXPECT_FALSE(fs::exists(timing_region())) << m.name << ": region removed at the end";
  }
}

// Unpaced recorder_live: the reader still accounts for every step; nothing is ever late.
TEST(BenchPacedTiming, UnpacedRecorderLiveAccountsForEveryStep) {
  TimingOptions o;
  o.log_mode = pipeline::LogMode::kRecorderLive;
  const common::Result<TimingStats> r = timing(kConfig, kSteps, o);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  record("unpaced_recorder_live", r.value());
  EXPECT_EQ(r.value().late_cycles, 0U);
  EXPECT_EQ(r.value().reader_samples + r.value().reader_lost, static_cast<std::uint64_t>(kSteps));
  EXPECT_FALSE(fs::exists(timing_region()));
}

TEST(BenchPacedTiming, ErrorsRemoveTheRegionToo) {
  const common::Result<TimingStats> r = timing(kConfig, 0, paced(pipeline::LogMode::kRecorderLive));
  EXPECT_FALSE(r.is_ok());
  EXPECT_FALSE(fs::exists(timing_region()));
  const common::Result<TimingStats> missing =
      timing(kRepo / "config/no_such_file.yaml", kSteps, paced(pipeline::LogMode::kRecorderLive));
  EXPECT_FALSE(missing.is_ok());
  EXPECT_FALSE(fs::exists(timing_region()));
}

// No allocation per step: a paced run of 4000 steps allocates exactly as often as one of 2000
// (set-up, thread start and the step-time buffer are counted once in both).
TEST(BenchPacedTiming, AllocationCountDoesNotGrowWithSteps) {
  // Warm-up outside any count: yaml-cpp builds its tables lazily on the first parse in the process
  // (about 460 extra allocations once), which is not a per-step cost.
  static_cast<void>(timing(kConfig, 1, paced(kModes[0].mode)));
  for (const Mode& m : kModes) {
    std::size_t short_run = 0;
    std::size_t long_run = 0;
    {
      const count_new::Scope scope;
      const common::Result<TimingStats> r = timing(kConfig, kSteps, paced(m.mode));
      short_run = count_new::Scope::count();
      ASSERT_TRUE(r.is_ok()) << m.name << ": " << r.reason();
    }
    {
      const count_new::Scope scope;
      const common::Result<TimingStats> r = timing(kConfig, 2 * kSteps, paced(m.mode));
      long_run = count_new::Scope::count();
      ASSERT_TRUE(r.is_ok()) << m.name << ": " << r.reason();
    }
    std::printf("%s: %zu allocations for %lld steps, %zu for %lld\n",
                m.name,
                short_run,
                static_cast<long long>(kSteps),
                long_run,
                2 * static_cast<long long>(kSteps));
    EXPECT_EQ(long_run, short_run) << m.name;
  }
}

// ---------------------------------------------------------------- CLI

struct Cli {
  int exit_code = -1;
  std::string out;
};

Cli run_bench(const std::string& args) {
  Cli cli;
  if (std::string(kBenchExe).empty()) {
    ADD_FAILURE() << "cobot_bench is not built (no target cobot_bench)";
    return cli;
  }
  const std::string cmd = "'" + std::string(kBenchExe) + "' " + args;
  FILE* pipe = ::popen(cmd.c_str(), "r");
  if (pipe == nullptr) {
    ADD_FAILURE() << "popen failed: " << cmd;
    return cli;
  }
  std::array<char, 4096> buf{};
  std::size_t n = 0;
  while ((n = std::fread(buf.data(), 1, buf.size(), pipe)) > 0) {
    cli.out.append(buf.data(), n);
  }
  const int status = ::pclose(pipe);
  cli.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  return cli;
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

bool is_digits(const std::string& s) {
  if (s.empty()) {
    return false;
  }
  for (const char c : s) {
    if (c < '0' || c > '9') {
      return false;
    }
  }
  return true;
}

// "%.3f" of a non-negative number: digits, '.', exactly three digits.
bool is_fixed3(const std::string& s) {
  const std::size_t dot = s.find('.');
  return dot != std::string::npos && is_digits(s.substr(0, dot)) && s.size() == dot + 4 &&
         is_digits(s.substr(dot + 1));
}

// Splits "key=value" tokens; empty when a token has no '='.
std::vector<std::pair<std::string, std::string>> fields(const std::string& line) {
  std::vector<std::pair<std::string, std::string>> out;
  for (const std::string& token : split(line, ' ')) {
    const std::size_t eq = token.find('=');
    if (eq == std::string::npos) {
      return {};
    }
    out.emplace_back(token.substr(0, eq), token.substr(eq + 1));
  }
  return out;
}

std::string strip_newline(std::string s) {
  if (!s.empty() && s.back() == '\n') {
    s.pop_back();
  }
  return s;
}

// p50_us=%.3f p95_us=%.3f p99_us=%.3f max_us=%.3f log_mode=<m> paced=<0|1> late_cycles=<n>
// late_p99_us=%.3f reader_samples=<n> reader_lost=<n> reader_cpu=<other|shared>
void expect_long_line(const std::string& line,
                      const std::string& log_mode,
                      const std::string& paced_flag,
                      std::uint64_t steps) {
  const auto f = fields(line);
  ASSERT_EQ(f.size(), 11U) << line;
  const char* const keys[] = {"p50_us",
                              "p95_us",
                              "p99_us",
                              "max_us",
                              "log_mode",
                              "paced",
                              "late_cycles",
                              "late_p99_us",
                              "reader_samples",
                              "reader_lost",
                              "reader_cpu"};
  for (std::size_t i = 0; i < f.size(); ++i) {
    EXPECT_EQ(f[i].first, keys[i]) << line;
  }
  for (const std::size_t i : {0U, 1U, 2U, 3U, 7U}) {
    EXPECT_TRUE(is_fixed3(f[i].second)) << f[i].first << " in " << line;
  }
  for (const std::size_t i : {6U, 8U, 9U}) {
    EXPECT_TRUE(is_digits(f[i].second)) << f[i].first << " in " << line;
  }
  EXPECT_EQ(f[4].second, log_mode) << line;
  EXPECT_EQ(f[5].second, paced_flag) << line;
  EXPECT_TRUE(f[10].second == "other" || f[10].second == "shared") << line;
  if (log_mode == "recorder_live" && is_digits(f[8].second) && is_digits(f[9].second)) {
    EXPECT_EQ(std::stoull(f[8].second) + std::stoull(f[9].second), steps) << line;
  }
}

TEST(BenchPacedTimingCli, PacedRecorderLivePrintsTheLongLine) {
  const Cli cli = run_bench("timing --config '" + kConfig.string() +
                            "' --steps 500 --paced --log-mode recorder_live");
  ASSERT_EQ(cli.exit_code, 0) << cli.out;
  ASSERT_EQ(split(cli.out, '\n').size(), 1U) << "one line: " << cli.out;
  expect_long_line(strip_newline(cli.out), "recorder_live", "1", 500);
  EXPECT_FALSE(fs::exists(timing_region()));
}

TEST(BenchPacedTimingCli, LogModeAlonePrintsTheLongLineUnpaced) {
  const Cli cli =
      run_bench("timing --config '" + kConfig.string() + "' --steps 500 --log-mode recorder");
  ASSERT_EQ(cli.exit_code, 0) << cli.out;
  expect_long_line(strip_newline(cli.out), "recorder", "0", 500);
}

// Without the two flags the line is the PF-01 one, byte for byte in format: three fields.
TEST(BenchPacedTimingCli, WithoutFlagsThePf01LineIsUnchanged) {
  const Cli cli = run_bench("timing --config '" + kConfig.string() + "' --steps 500");
  ASSERT_EQ(cli.exit_code, 0) << cli.out;
  ASSERT_FALSE(cli.out.empty());
  EXPECT_EQ(cli.out.back(), '\n');
  const auto f = fields(strip_newline(cli.out));
  ASSERT_EQ(f.size(), 3U) << cli.out;
  EXPECT_EQ(f[0].first, "p50_us");
  EXPECT_EQ(f[1].first, "p95_us");
  EXPECT_EQ(f[2].first, "max_us");
  for (const auto& kv : f) {
    EXPECT_TRUE(is_fixed3(kv.second)) << kv.first << " in " << cli.out;
  }
}

TEST(BenchPacedTimingCli, UnknownLogModeIsBadInput) {
  const Cli cli = run_bench("timing --config '" + kConfig.string() +
                            "' --steps 500 --log-mode sometimes 2>/dev/null");
  EXPECT_EQ(cli.exit_code, 2) << cli.out;
}

}  // namespace
}  // namespace cobot::bench
