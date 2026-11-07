// A task through the run engine (brief hardware-catalog, run check): pick_place_3kg on premium_fr3
// with computed torque, one seed, through bench::run on the files written from run_overrides: the
// path is accepted and no row holds. This proves the waypoints and the payload are feasible; when
// it fails, fix the waypoints, not this test. The run takes seconds on an elastic plant: optimised
// builds only; the file round trip runs in every build.
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/overrides.hpp"
#include "cobot/pipeline/task.hpp"

#include "hw_test_util.hpp"
#include "run.hpp"

namespace cobot::pipeline {
namespace {

using namespace hw_test;
using common::Vec3;

#if defined(__OPTIMIZE__)
constexpr bool kOptimised = true;
#else
constexpr bool kOptimised = false;
#endif

constexpr std::uint64_t kSeed = 1;

struct RunFiles {
  fs::path config;
  fs::path imperfections;
  std::optional<fs::path> waypoints;
};

// config_for-style files of pick_place_3kg on premium_fr3 with computed torque.
std::optional<RunFiles> write_pick_place_files() {
  const auto m = load_hardware_model(kRepo / "config/hardware/premium_fr3.yaml");
  if (!m.is_ok()) {
    ADD_FAILURE() << m.reason();
    return std::nullopt;
  }
  const auto t = load_task(kRepo / "config/tasks/pick_place_3kg.yaml");
  if (!t.is_ok()) {
    ADD_FAILURE() << t.reason();
    return std::nullopt;
  }
  const RunOverrides ro = run_overrides(m.value(), control::Algorithm::kComputedTorque, t.value());
  RunFiles files{test_dir() / "config.yaml", test_dir() / "imperfections.yaml", ro.waypoints};
  const common::Status wc = write_config_with(m.value().controller, ro.config, files.config);
  if (!wc.is_ok()) {
    ADD_FAILURE() << wc.reason();
    return std::nullopt;
  }
  const common::Status wi =
      write_imperfections_with(m.value().imperfections, ro.imperfections, files.imperfections);
  if (!wi.is_ok()) {
    ADD_FAILURE() << wi.reason();
    return std::nullopt;
  }
  return files;
}

std::vector<std::string> split_csv(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    out.push_back(cell);
  }
  return out;
}

std::size_t column(const std::vector<std::string>& header, const std::string& name) {
  for (std::size_t i = 0; i < header.size(); ++i) {
    if (header[i] == name) {
      return i;
    }
  }
  ADD_FAILURE() << "no column " << name;
  return 0;
}

// The written pair carries the payload: the plant through imperfections, the controller through
// cobot.payload.
TEST(TaskRun, WrittenFilesCarryThePayload) {
  const std::optional<RunFiles> files = write_pick_place_files();
  ASSERT_TRUE(files.has_value());
  ASSERT_TRUE(files->waypoints.has_value());
  const auto c = load_config<kN>(files->config);
  ASSERT_TRUE(c.is_ok()) << c.reason();
  EXPECT_EQ(c.value().payload.kg, 3.0);
  EXPECT_TRUE(c.value().payload.offset_m == Vec3(0.0, 0.0, 0.05));
  EXPECT_EQ(c.value().payload.radius_m, 0.05);
  const auto imp = load_imperfections<kN>(files->imperfections);
  ASSERT_TRUE(imp.is_ok()) << imp.reason();
  EXPECT_EQ(imp.value().model.payload_kg, 3.0);
  EXPECT_TRUE(imp.value().model.payload_offset_m == Vec3(0.0, 0.0, 0.05));
  EXPECT_EQ(imp.value().model.payload_radius_m, 0.05);
}

TEST(TaskRun, PickPlaceOnPremiumIsAcceptedAndNeverHolds) {
  if (!kOptimised) {
    GTEST_SKIP() << "elastic 7-DOF closed loop: release builds only";
  }
  const std::optional<RunFiles> files = write_pick_place_files();
  ASSERT_TRUE(files.has_value());
  const fs::path csv = test_dir() / "run.csv";
  const common::Status st =
      bench::run(files->config, files->imperfections, kSeed, csv, std::nullopt, files->waypoints);
  ASSERT_TRUE(st.is_ok()) << st.reason();

  std::ifstream in(csv);
  std::string line;
  ASSERT_TRUE(static_cast<bool>(std::getline(in, line))) << csv;
  const std::vector<std::string> header = split_csv(line);
  const std::size_t hold = column(header, "hold");
  const std::size_t mode = column(header, "mode");
  std::size_t rows = 0;
  while (std::getline(in, line)) {
    const std::vector<std::string> row = split_csv(line);
    ASSERT_GT(row.size(), hold) << line;
    ASSERT_GT(row.size(), mode) << line;
    ASSERT_EQ(row[hold], "none") << "t row " << rows;
    ASSERT_NE(row[mode], "holding") << "t row " << rows;
    ++rows;
  }
  EXPECT_GT(rows, 0U);
}

}  // namespace
}  // namespace cobot::pipeline
