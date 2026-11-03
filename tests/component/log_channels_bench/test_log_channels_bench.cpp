// The log channels in bench runs (feature log-channels, cobot_studio design 5.1 and 7): the run
// engine passes SimTruthProbe::sample() to ControlLoop::step, so every row carries the plant truth
// of its instant. Alignment (docs/conventions.md): taum, tauf and qddt of row k describe the plant
// interval (t_{k-1}, t_k]; taum of row k+1 is the plant's response to tau of row k.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/dynamics/dynamics.hpp"
#include "cobot/dynamics/harmonic_spring.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"

#include "engine.hpp"

namespace cobot::bench {
namespace {

namespace fs = std::filesystem;
constexpr int kN = 7;
using Vec = common::VecN<kN>;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kDefault = kRepo / "config/default.yaml";
const fs::path kMid = kRepo / "config/mid_kr1205.yaml";
constexpr double kDt = 1e-3;  // s, control_rate_hz 1000 of both configs
constexpr double kRunS = 3.0;

// A Run CSV read back: column index by name, rows as cells.
struct Csv {
  std::map<std::string, std::size_t> col;
  std::vector<std::vector<std::string>> rows;

  const std::string& cell(std::size_t k, const std::string& name) const {
    return rows.at(k).at(col.at(name));
  }
  double num(std::size_t k, const std::string& prefix, int j) const {  // j 0-based
    return std::strtod(cell(k, prefix + std::to_string(j + 1)).c_str(), nullptr);
  }
  double t_s(std::size_t k) const { return std::stod(cell(k, "t_ns")) * 1e-9; }
};

std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    out.push_back(cell);
  }
  return out;
}

Csv read_csv(const fs::path& p) {
  Csv csv;
  std::ifstream in(p, std::ios::binary);
  std::string line;
  EXPECT_TRUE(std::getline(in, line)) << p;
  const std::vector<std::string> head = split(line);
  EXPECT_EQ(head.size(), 114U);
  for (std::size_t i = 0; i < head.size(); ++i) {
    csv.col[head[i]] = i;
  }
  while (std::getline(in, line)) {
    csv.rows.push_back(split(line));
    EXPECT_EQ(csv.rows.back().size(), 114U) << "row " << csv.rows.size() - 1;
  }
  return csv;
}

imperfect::ImperfectionConfig<kN> profile(const std::string& name, std::uint64_t seed) {
  auto r = pipeline::load_imperfections<kN>(kRepo / "config/imperfections" / name);
  EXPECT_TRUE(r.is_ok()) << r.reason();
  imperfect::ImperfectionConfig<kN> p = r.is_ok() ? r.value() : imperfect::ImperfectionConfig<kN>{};
  p.seed = seed;
  return p;
}

// Runs `config` with `imp` for kRunS (or the waypoint run) and reads the Run CSV back.
std::optional<Csv> run_csv(const fs::path& config,
                           const imperfect::ImperfectionConfig<kN>& imp,
                           const std::string& name,
                           WaypointInput* waypoints = nullptr) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "log_channels_bench";
  fs::create_directories(dir);
  const fs::path out = dir / name;
  fs::remove(out);
  const auto r = run_profile(config, imp, name, out, kRunS, waypoints);
  if (!r.is_ok()) {
    ADD_FAILURE() << r.reason();
    return std::nullopt;
  }
  Csv csv = read_csv(out);
  EXPECT_EQ(csv.rows.size(), 3000U);  // 3 s at 1000 Hz
  return csv;
}

TEST(LogChannelsBench, IdealProfileTruthEqualsTheMeasurementAndTaumIsThePreviousTau) {
  const auto csv = run_csv(kDefault, profile("ideal.yaml", 1), "ideal.csv");
  ASSERT_TRUE(csv);
  for (std::size_t k = 0; k < csv->rows.size(); ++k) {
    EXPECT_EQ(csv->cell(k, "cmd"), "config") << k;
    for (int j = 0; j < kN; ++j) {
      ASSERT_EQ(csv->num(k, "qt", j), csv->num(k, "q", j)) << "row " << k << " joint " << j;
      ASSERT_EQ(csv->num(k, "qdt", j), csv->num(k, "qd", j)) << "row " << k << " joint " << j;
      ASSERT_EQ(csv->num(k, "qraw", j), csv->num(k, "q", j)) << "row " << k << " joint " << j;
      ASSERT_EQ(csv->num(k, "qmt", j), csv->num(k, "qt", j)) << "row " << k << " joint " << j;
      ASSERT_EQ(csv->num(k, "tauf", j), 0.0) << "row " << k << " joint " << j;
      // No motor lag, |tau| <= P < M: the plant applied exactly the previous row's command.
      if (k > 0) {
        ASSERT_EQ(csv->num(k, "taum", j), csv->num(k - 1, "tau", j)) << "row " << k << " " << j;
      }
    }
  }
}

TEST(LogChannelsBench, IdealProfileAccelerationsAreTheVelocityDifferences) {
  const auto csv = run_csv(kDefault, profile("ideal.yaml", 1), "ideal_acc.csv");
  ASSERT_TRUE(csv);
  std::size_t tracking_pairs = 0;
  for (int j = 0; j < kN; ++j) {
    EXPECT_EQ(csv->num(0, "qddt", j), 0.0) << "first sample";
  }
  for (std::size_t k = 1; k < csv->rows.size(); ++k) {
    const bool tracking =
        csv->cell(k, "mode") == "tracking" && csv->cell(k - 1, "mode") == "tracking";
    tracking_pairs += tracking ? 1 : 0;
    for (int j = 0; j < kN; ++j) {
      // qddt over (t_{k-1}, t_k] on the plant clock, 1 ms per cycle without jitter.
      const double qddt = (csv->num(k, "qdt", j) - csv->num(k - 1, "qdt", j)) / kDt;
      EXPECT_NEAR(csv->num(k, "qddt", j), qddt, 1e-12 * std::max(1.0, std::abs(qddt)))
          << "row " << k << " joint " << j;
      if (tracking) {
        // CLIK: qd_ref += qdd_ref dt, so the logged qdd_ref is the difference of qd_ref.
        const double qddref = csv->num(k, "qddref", j);
        const double diff = (csv->num(k, "qdref", j) - csv->num(k - 1, "qdref", j)) / kDt;
        EXPECT_NEAR(qddref, diff, 1e-9 * std::max(1.0, std::abs(qddref)))
            << "row " << k << " joint " << j;
      }
    }
  }
  EXPECT_GT(tracking_pairs, 2000U);  // 3 s inside the 10 s trajectory
}

TEST(LogChannelsBench, RealisticProfileFrictionIsBoundedAndTheMeasurementIsNoisy) {
  const imperfect::ImperfectionConfig<kN> imp = profile("realistic.yaml", 1);
  const auto csv = run_csv(kDefault, imp, "realistic.csv");
  ASSERT_TRUE(csv);
  bool friction_seen = false;
  bool error_seen = false;
  for (std::size_t k = 0; k < csv->rows.size(); ++k) {
    for (int j = 0; j < kN; ++j) {
      const double tauf = csv->num(k, "tauf", j);
      const double bound = imp.friction.coulomb_nm[j] +
                           (imp.friction.viscous_nm_s_rad[j] * std::abs(csv->num(k, "qdt", j)));
      ASSERT_TRUE(std::isfinite(tauf)) << "row " << k << " joint " << j;
      EXPECT_LE(std::abs(tauf), bound + 1e-9) << "row " << k << " joint " << j;
      friction_seen = friction_seen || tauf != 0.0;
      const double err = csv->num(k, "q", j) - csv->num(k, "qt", j);
      ASSERT_TRUE(std::isfinite(err)) << "row " << k << " joint " << j;
      error_seen = error_seen || err != 0.0;
    }
  }
  EXPECT_TRUE(friction_seen);
  EXPECT_TRUE(error_seen);
}

TEST(LogChannelsBench, MidTierMotorEncoderShowsDeflectionAndTheCompensation) {
  const auto cfg = pipeline::load_config<kN>(kMid);
  ASSERT_TRUE(cfg.is_ok()) << cfg.reason();
  const pipeline::Config<kN>& c = cfg.value();
  ASSERT_TRUE(c.deflection_comp.enabled);
  const auto chain = model::parse_urdf_chain(c.urdf_path, c.base_link, c.tip_link);
  ASSERT_TRUE(chain.is_ok()) << chain.reason();
  const auto arm = model::make_model<kN>(chain.value(), c.gravity_b);
  ASSERT_TRUE(arm.is_ok()) << arm.reason();
  const model::Arm7 controller = pipeline::controller_model<kN>(arm.value(), c);

  const auto csv = run_csv(kMid, profile("mid_kr1205.yaml", 1), "mid_kr1205.csv");
  ASSERT_TRUE(csv);
  const pipeline::DeflectionCompConfig<kN>& d = c.deflection_comp;
  for (std::size_t k = 0; k < csv->rows.size(); ++k) {
    Vec q_raw;
    for (int j = 0; j < kN; ++j) {
      q_raw(j) = csv->num(k, "qraw", j);
    }
    // q (q_hat) = q_raw - harmonic_spring_deflection(g(q_raw)) with the controller's model.
    const Vec g = dynamics::gravity_torque(controller, q_raw);
    bool compensated = false;
    for (int j = 0; j < kN; ++j) {
      const double comp = -dynamics::harmonic_spring_deflection(g(j),
                                                                d.stiffness_k1_nm_rad(j),
                                                                d.stiffness_k2_nm_rad(j),
                                                                d.stiffness_k3_nm_rad(j),
                                                                d.torque_t1_nm(j),
                                                                d.torque_t2_nm(j));
      EXPECT_NEAR(csv->num(k, "q", j) - q_raw(j), comp, 1e-12) << "row " << k << " joint " << j;
      compensated = compensated || csv->num(k, "q", j) != q_raw(j);
      if (csv->t_s(k) >= 0.5 && j < 4) {
        EXPECT_NE(csv->num(k, "qmt", j), csv->num(k, "qt", j)) << "row " << k << " joint " << j;
      }
    }
    EXPECT_TRUE(compensated) << "row " << k;
  }
}

TEST(LogChannelsBench, WaypointRunIsLoggedAsWaypointsCommandMode) {
  WaypointInput waypoints{kRepo / "tests/e2e/waypoints/e2e_10.yaml"};
  const auto csv = run_csv(kDefault, profile("realistic.yaml", 7), "e2e_10.csv", &waypoints);
  ASSERT_TRUE(csv);
  for (std::size_t k = 0; k < csv->rows.size(); ++k) {
    EXPECT_EQ(csv->cell(k, "cmd"), "waypoints") << "row " << k;
  }
}

}  // namespace
}  // namespace cobot::bench
