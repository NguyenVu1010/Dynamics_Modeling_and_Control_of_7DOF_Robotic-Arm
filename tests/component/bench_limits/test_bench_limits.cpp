// Bench Summary metrics against the config torque limits (feature joint-limits, design
// docs/design/2026-11-02-realism-2-design.md section 5): peak_tau_ratio and sat_time_s use the
// peak torque P (cobot.limits.peak_torque_nm, the controller model's tau_max) and the new
// RunSummary::rms_tau_ratio = max_j RMS(tau_j over tracking/finished rows) / R_j uses the rated
// torque R (cobot.limits.rated_torque_nm).
//
// Two runs of the same short ideal trajectory: one without the limits section (P = R = URDF
// effort M), one with P = M / 2 and R = M / 4. While no torque reaches M / 2 the controller never
// clamps, both runs log the same torques, and the ratios scale by exactly 2 and 4 (division by a
// power-of-two multiple of M is exact in binary floating point).
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"

#include "engine.hpp"

namespace cobot::bench {
namespace {

namespace fs = std::filesystem;
using Vec7 = common::VecN<7>;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kUrdf = kRepo / "src/sim/urdf/cobot30_5.urdf";
constexpr double kDurationS = 0.5;

std::string read_file(const fs::path& p) {
  const std::ifstream in(p, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

fs::path write_temp(const std::string& name, const std::string& text) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "bench_limits";
  fs::create_directories(dir);
  const fs::path p = dir / name;
  std::ofstream(p, std::ios::binary) << text;
  return p;
}

bool starts_with(const std::string& line, const std::string& prefix) {
  return line.rfind(prefix, 0) == 0;
}

// config/default.yaml with an absolute URDF path and without its `  limits:` block.
std::string default_without_limits() {
  std::string text = read_file(kRepo / "config/default.yaml");
  const std::string rel = "../src/sim/urdf/cobot30_5.urdf";
  const auto pos = text.find(rel);
  EXPECT_NE(pos, std::string::npos);
  if (pos != std::string::npos) {
    text.replace(pos, rel.size(), kUrdf.string());
  }
  std::istringstream in(text);
  std::string out;
  std::string line;
  bool inside = false;
  while (std::getline(in, line)) {
    if (starts_with(line, "  limits:")) {
      inside = true;
      continue;
    }
    if (inside && (starts_with(line, "   ") || line.empty())) {
      continue;
    }
    inside = false;
    out += line + "\n";
  }
  return out;
}

std::string yaml_list(const Vec7& v) {
  std::ostringstream os;
  os.precision(std::numeric_limits<double>::max_digits10);
  os << "[";
  for (int i = 0; i < 7; ++i) {
    os << (i > 0 ? ", " : "") << v[i];
  }
  os << "]";
  return os.str();
}

Vec7 urdf_effort() {
  const auto chain = model::parse_urdf_chain(kUrdf, "base_link", "link_7");
  EXPECT_TRUE(chain.is_ok()) << chain.reason();
  Vec7 effort = Vec7::Zero();
  if (chain.is_ok()) {
    for (int i = 0; i < 7; ++i) {
      effort[i] = chain.value().joints[static_cast<std::size_t>(i)].tau_max;
    }
  }
  return effort;
}

RunSummary run(const fs::path& config) {
  const auto ideal = pipeline::load_imperfections<7>(kRepo / "config/imperfections/ideal.yaml");
  EXPECT_TRUE(ideal.is_ok()) << ideal.reason();
  const common::Result<RunSummary> r =
      run_profile(config, ideal.value(), "ideal", fs::path(), kDurationS);
  EXPECT_TRUE(r.is_ok()) << r.reason();
  return r.is_ok() ? r.value() : RunSummary{};
}

TEST(BenchLimits, RatiosUsePeakAndRatedTorqueOfTheConfig) {
  const std::string base = default_without_limits();
  const Vec7 effort = urdf_effort();
  const fs::path urdf_only = write_temp("urdf_only.yaml", base);
  const fs::path scaled =
      write_temp("scaled.yaml",
                 base + "  limits:\n    rated_torque_nm: " + yaml_list(0.25 * effort) +
                     "\n    peak_torque_nm: " + yaml_list(0.5 * effort) + "\n");
  const RunSummary m = run(urdf_only);  // P = R = M
  const RunSummary s = run(scaled);     // P = M / 2, R = M / 4

  // Set-up: no torque reaches M / 2, so the controller clamp is idle in both runs.
  ASSERT_GT(s.peak_tau_ratio, 0.0);
  ASSERT_LT(s.peak_tau_ratio, 0.999);
  EXPECT_EQ(s.sat_time_s, 0.0);
  EXPECT_EQ(s.final_pos_err_m, m.final_pos_err_m) << "same torques, same run";

  // max |tau| / (M / 2) = 2 max |tau| / M; RMS / (M / 4) = 4 RMS / M.
  EXPECT_DOUBLE_EQ(s.peak_tau_ratio, 2.0 * m.peak_tau_ratio);
  ASSERT_TRUE(std::isfinite(m.rms_tau_ratio));
  ASSERT_GT(m.rms_tau_ratio, 0.0);  // gravity alone needs torque
  EXPECT_DOUBLE_EQ(s.rms_tau_ratio, 4.0 * m.rms_tau_ratio);
  // RMS never exceeds the peak: RMS / M <= max |tau| / M.
  EXPECT_LE(m.rms_tau_ratio, m.peak_tau_ratio);
}

}  // namespace
}  // namespace cobot::bench
