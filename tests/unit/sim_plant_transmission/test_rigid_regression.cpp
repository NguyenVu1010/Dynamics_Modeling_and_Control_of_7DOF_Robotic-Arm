// All k3 = 0 is the rigid joint of today (realism-2 design 6.2 and 8, brief "transmission"):
// 3 s of the E2E-01 closed loop must equal the run recorded before this feature, bit for bit on
// x86-64 (release and ASan builds gave the same bits; other targets get 1e-9 relative as in
// loop_external), whatever the other transmission values are, with q_motor == q and qd_motor == qd.
// The JointState extras filled by SimJointInterface do not change any controller output (tau_cmd is
// in the file).
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "rigid_run.hpp"
#include "transmission_test_util.hpp"

namespace cobot::dynamics::test {
namespace {

namespace fs = std::filesystem;
constexpr int kCycles = 3000;  // 3 s at 1 kHz

std::vector<std::vector<double>> parse(const std::string& csv) {
  std::vector<std::vector<double>> rows;
  std::istringstream lines(csv);
  std::string line;
  while (std::getline(lines, line)) {
    std::vector<double> row;
    std::istringstream cells(line);
    std::string cell;
    while (std::getline(cells, cell, ',')) {
      row.push_back(std::strtod(cell.c_str(), nullptr));
    }
    rows.push_back(row);
  }
  return rows;
}

void expect_matches_reference(const std::string& actual, const char* what) {
  const fs::path out = fs::path(COBOT_TEST_OUTPUT_DIR) / "sim_plant_transmission";
  fs::create_directories(out);
  std::ofstream(out / (std::string(what) + ".csv"), std::ios::binary) << actual;
  const std::ifstream in(
      fs::path(COBOT_REPO_DIR) / "tests/data/transmission/rigid_e2e01_reference.csv",
      std::ios::binary);
  ASSERT_TRUE(in.good());
  std::stringstream ss;
  ss << in.rdbuf();
  const auto want = parse(ss.str());
  const auto got = parse(actual);
  ASSERT_EQ(want.size(), 30U);
  ASSERT_EQ(got.size(), want.size()) << what;
  for (std::size_t r = 0; r < want.size(); ++r) {
    ASSERT_EQ(got[r].size(), 29U) << what << " row " << r;  // k + 4 * 7
    for (std::size_t c = 0; c < want[r].size(); ++c) {
#if defined(__x86_64__)
      EXPECT_EQ(got[r][c], want[r][c]) << what << " row " << r << " column " << c;
#else
      EXPECT_LE(std::abs(got[r][c] - want[r][c]), 1e-9 * std::max(1.0, std::abs(want[r][c])))
          << what << " row " << r << " column " << c;
#endif
    }
  }
}

TEST(TransmissionRigid, DefaultPlantEffectsReproduceTheRecordedRun) {
  const auto setup = load_e2e01();
  ASSERT_TRUE(setup);
  int mismatches = 0;
  const std::string csv = run_e2e01(
      *setup,
      PlantEffects<7>{},
      kCycles,
      [&](int /*k*/,
          const SimPlant<7>& p,
          const hal::JointState<7>& s,
          const hal::JointCommand<7>&) {
        mismatches += (p.q_motor() == p.q() && p.qd_motor() == p.qd() && s.motor_valid) ? 0 : 1;
      });
  EXPECT_EQ(mismatches, 0);
  expect_matches_reference(csv, "default");
}

TEST(TransmissionRigid, ZeroK3IgnoresEveryOtherTransmissionValue) {
  const auto setup = load_e2e01();
  ASSERT_TRUE(setup);
  PlantEffects<7> fx;
  const std::array<Gear, 7> gears = {kShg25, kShg25, kShg20, kShg20, kShg14, kShg14, kShg14};
  for (int j = 0; j < 7; ++j) {
    Gear g = gears[static_cast<std::size_t>(j)];
    g.k3 = 0.0;  // rigid, while k1, k2, t1, t2 and the rest are set
    set_elastic(fx, j, g, 0.05, 8.7e-4, 8.7e-4, 120.0, 0.3 + (0.7 * j));
  }
  int mismatches = 0;
  const std::string csv =
      run_e2e01(*setup, fx, kCycles, [&](int, const SimPlant<7>& p, const auto&, const auto&) {
        mismatches += (p.q_motor() == p.q() && p.qd_motor() == p.qd()) ? 0 : 1;
      });
  EXPECT_EQ(mismatches, 0);
  expect_matches_reference(csv, "zero_k3");
}

}  // namespace
}  // namespace cobot::dynamics::test
