#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/task.hpp"

#include "tune.hpp"

// Helpers of test_bench_tune_runs: the reference files, one output directory per test, and options.
namespace cobot::bench::tune_test {

namespace fs = std::filesystem;

inline const fs::path kRepo = COBOT_REPO_DIR;

#if defined(__OPTIMIZE__)
inline constexpr bool kOptimised = true;
#else
inline constexpr bool kOptimised = false;
#endif

inline bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

inline std::string read_text(const fs::path& p) {
  std::ifstream in(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

inline fs::path model_path(const std::string& name) {
  return kRepo / "config/hardware" / (name + ".yaml");
}

inline fs::path task_path(const std::string& name) {
  return kRepo / "config/tasks" / (name + ".yaml");
}

inline pipeline::HardwareModel load_model(const std::string& name) {
  const auto m = pipeline::load_hardware_model(model_path(name));
  EXPECT_TRUE(m.is_ok()) << m.reason();
  return m ? m.value() : pipeline::HardwareModel{};
}

inline pipeline::Task load_task(const std::string& name) {
  const auto t = pipeline::load_task(task_path(name));
  EXPECT_TRUE(t.is_ok()) << t.reason();
  return t ? t.value() : pipeline::Task{};
}

// An empty directory of its own for the running test (ctest runs tests in parallel processes).
inline fs::path test_dir() {
  const ::testing::TestInfo* info = ::testing::UnitTest::GetInstance()->current_test_info();
  std::string name = std::string(info->test_suite_name()) + "." + info->name();
  for (char& c : name) {
    c = c == '/' ? '_' : c;
  }
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "bench_tune_runs" / name;
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

inline TuneOptions options(std::vector<double> levels,
                           std::vector<std::uint64_t> seeds,
                           unsigned jobs) {
  TuneOptions o{};
  o.levels = std::move(levels);
  o.seeds = std::move(seeds);
  o.jobs = jobs;
  return o;
}

}  // namespace cobot::bench::tune_test
