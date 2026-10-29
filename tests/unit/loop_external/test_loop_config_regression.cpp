#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "control_loop_test_util.hpp"

// Config mode must not change with feature `loop-external` (design 6.3: "command_source:
// config keeps today's code path"). tests/data/loop_external/config_mode_reference.csv was
// recorded with the ControlLoop of main before the feature (commit 733feeb) by this test, and
// every later build must reproduce it: mode exactly, numbers within 1e-9 (relative above 1).
// Two runs of 2 s on the ideal rig, the MVP trajectory from config, one with friction
// feed-forward off (ideal_world.yaml) and one with it on (the nominal values of
// config/default.yaml); a NaN read at step 1500 exercises the hold path. Rows every 50 steps
// and around the hold.
namespace cobot::pipeline::test {
namespace {

namespace fs = std::filesystem;

constexpr int kSteps = 2000;
constexpr int kNanStep = 1500;

bool sampled(int k) {
  return k % 50 == 0 || (k >= kNanStep - 1 && k <= kNanStep + 3) || k == kSteps - 1;
}

std::string fmt(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return buf;
}

// One line per sampled step: case,k,t_ns,mode,tau1..7,qref1..7,qdref1..7,pdx,pdy,pdz
void run_case(const Setup& setup, const Config<kN>& cfg, const char* name, std::string& out) {
  const std::unique_ptr<Rig> rig = make_rig(setup, cfg);
  ASSERT_TRUE(rig->start().is_ok());
  std::vector<Record> records;
  records.reserve(kSteps);
  for (int k = 0; k < kSteps; ++k) {
    State s;
    Command cmd;
    if (k == kNanStep) {
      EXPECT_TRUE(
          rig->cycle(s, cmd, [](State& x) { x.q(2) = std::numeric_limits<double>::quiet_NaN(); })
              .is_ok());
    } else {
      EXPECT_TRUE(rig->cycle(s, cmd).is_ok());
    }
    if ((k + 1) % 100 == 0) {
      drain(rig->loop, records);
    }
  }
  drain(rig->loop, records);
  ASSERT_EQ(records.size(), static_cast<std::size_t>(kSteps));
  for (int k = 0; k < kSteps; ++k) {
    if (!sampled(k)) {
      continue;
    }
    const Record& r = records[static_cast<std::size_t>(k)];
    std::string line = std::string(name) + "," + std::to_string(k) + "," + std::to_string(r.t.ns) +
                       "," + to_string(r.mode);
    for (const Vec* v : {&r.tau, &r.q_ref, &r.qd_ref}) {
      for (int i = 0; i < kN; ++i) {
        line += "," + fmt((*v)(i));
      }
    }
    for (int i = 0; i < 3; ++i) {
      line += "," + fmt(r.T_b_d.p(i));
    }
    out += line + "\n";
  }
}

std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> cells;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    cells.push_back(cell);
  }
  return cells;
}

std::vector<std::string> lines_of(const std::string& text) {
  std::vector<std::string> lines;
  std::stringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    lines.push_back(line);
  }
  return lines;
}

TEST(LoopConfigRegression, ConfigModeMatchesTheRecordedReference) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  std::string actual;
  run_case(*setup, setup->config, "ideal", actual);
  Config<kN> with_ff = setup->config;
  with_ff.friction_compensation.enabled = true;
  run_case(*setup, with_ff, "friction_ff", actual);

  const fs::path out = fs::path(COBOT_TEST_OUTPUT_DIR) / "loop_external";
  fs::create_directories(out);
  std::ofstream(out / "config_mode_reference.csv", std::ios::binary) << actual;

  const fs::path ref_path =
      fs::path(COBOT_REPO_DIR) / "tests/data/loop_external/config_mode_reference.csv";
  const std::ifstream in(ref_path, std::ios::binary);
  ASSERT_TRUE(in.good()) << "missing " << ref_path << "; this run wrote "
                         << out / "config_mode_reference.csv";
  std::stringstream ss;
  ss << in.rdbuf();
  const std::vector<std::string> want = lines_of(ss.str());
  const std::vector<std::string> got = lines_of(actual);
  ASSERT_EQ(got.size(), want.size());
  for (std::size_t l = 0; l < want.size(); ++l) {
    const std::vector<std::string> w = split(want[l]);
    const std::vector<std::string> g = split(got[l]);
    ASSERT_EQ(g.size(), w.size()) << "line " << l;
    ASSERT_GE(w.size(), 4U);
    for (std::size_t c = 0; c < 4; ++c) {  // case, k, t_ns, mode
      ASSERT_EQ(g[c], w[c]) << "line " << l << " column " << c;
    }
    for (std::size_t c = 4; c < w.size(); ++c) {
      const double a = std::strtod(g[c].c_str(), nullptr);
      const double b = std::strtod(w[c].c_str(), nullptr);
      EXPECT_LE(std::abs(a - b), 1e-9 * std::max(1.0, std::abs(b)))
          << "line " << l << " column " << c << ": " << g[c] << " vs " << w[c];
    }
  }
}

}  // namespace
}  // namespace cobot::pipeline::test
