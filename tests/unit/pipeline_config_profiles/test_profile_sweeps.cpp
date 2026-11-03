#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/imperfect/config.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/sweep_config.hpp"

// Realism-2 sweep files (brief "profiles", design 4.6 and 7):
//   config/sweeps/realism2_budget.yaml: base budget_xarm7.yaml, seeds [1, 2, 3],
//     config.safety.hal_timeout_ms [5, 8, 12, 20], comm.burst_cycles [1, 2, 5, 10],
//     timing.overrun_cycles [1, 2, 3, 5], comm.packet_loss_prob [0, 1e-4, 1e-3, 1e-2];
//   config/sweeps/realism2_mid.yaml: base mid_kr1205.yaml,
//     transmission.lost_motion_rad [0, 2.9e-4, 8.7e-4, 1.7e-3],
//     transmission.error_amp_rad [0, 4.4e-4, 8.7e-4, 1.7e-3],
//     actuator.torque_slew_nm_s [1e4, 2e4, 5e4, 1e5], model.payload_kg [0, 1, 3, 5]
//     (sensor.position_source is not sweepable).
// Each file loads (pipeline::load_sweep) and every param name resolves at every level: an
// imperfection key through load_imperfections_with on the base profile, a config.<section>.<key>
// through load_config_with on config/budget_xarm7.yaml (the controller config the budget sweep
// runs with: cobot_bench sweep --config config/budget_xarm7.yaml).
namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;
constexpr int kN = 7;
using Levels = std::vector<double>;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kConfig = kRepo / "config/budget_xarm7.yaml";

// True when both paths name the same existing file (false, not a throw, when either is missing).
bool same_file(const fs::path& a, const fs::path& b) {
  std::error_code ec;
  return fs::equivalent(a, b, ec) && !ec;
}

// The value a resolved param took in the loaded profile (joint 0 for per-joint keys); empty for a
// key this test does not map.
std::optional<double> value_of(const imperfect::ImperfectionConfig<kN>& c,
                               const std::string& name) {
  if (name == "comm.burst_cycles") {
    return static_cast<double>(c.comm.burst_cycles);
  }
  if (name == "comm.packet_loss_prob") {
    return c.comm.packet_loss_prob;
  }
  if (name == "timing.overrun_cycles") {
    return static_cast<double>(c.timing.overrun_cycles);
  }
  if (name == "transmission.lost_motion_rad") {
    return c.transmission.lost_motion_rad(0);
  }
  if (name == "transmission.error_amp_rad") {
    return c.transmission.error_amp_rad(0);
  }
  if (name == "actuator.torque_slew_nm_s") {
    return c.actuator.torque_slew_nm_s(0);
  }
  if (name == "model.payload_kg") {
    return c.model.payload_kg;
  }
  return std::nullopt;
}

void expect_every_param_resolves(const SweepSpec& spec) {
  for (const SweepParam& p : spec.params) {
    ASSERT_FALSE(p.levels.empty()) << p.name;
    for (const double level : p.levels) {
      if (p.name.rfind("config.", 0) == 0) {
        const auto r = load_config_with<kN>(kConfig, p.name, level);
        EXPECT_TRUE(r.is_ok()) << p.name << " " << level << ": " << r.reason();
        if (r.is_ok() && p.name == "config.safety.hal_timeout_ms") {
          // level ms * 1e-3 s/ms, the product load_config forms
          EXPECT_EQ(r.value().safety.hal_timeout_s, level * 1.0e-3) << level;
        }
        continue;
      }
      const auto r = load_imperfections_with<kN>(spec.base_profile, p.name, level);
      ASSERT_TRUE(r.is_ok()) << p.name << " " << level << ": " << r.reason();
      const std::optional<double> v = value_of(r.value(), p.name);
      if (v) {
        EXPECT_DOUBLE_EQ(*v, level) << p.name;
      }
    }
  }
}

void expect_param(const SweepSpec& spec, const std::string& name, const Levels& levels) {
  const SweepParam* found = nullptr;
  for (const SweepParam& p : spec.params) {
    found = p.name == name ? &p : found;
  }
  ASSERT_NE(found, nullptr) << name;
  ASSERT_EQ(found->levels.size(), levels.size()) << name;
  for (std::size_t i = 0; i < levels.size(); ++i) {
    EXPECT_DOUBLE_EQ(found->levels[i], levels[i]) << name << " level " << i;
  }
}

TEST(ProfileSweeps, BudgetSweepLoadsAndEveryParamResolves) {
  const auto r = load_sweep(kRepo / "config/sweeps/realism2_budget.yaml");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const SweepSpec& s = r.value();
  EXPECT_TRUE(same_file(s.base_profile, kRepo / "config/imperfections/budget_xarm7.yaml"))
      << s.base_profile;
  EXPECT_EQ(s.seeds, (std::vector<std::uint64_t>{1, 2, 3}));
  expect_param(s, "config.safety.hal_timeout_ms", {5, 8, 12, 20});
  expect_param(s, "comm.burst_cycles", {1, 2, 5, 10});
  expect_param(s, "timing.overrun_cycles", {1, 2, 3, 5});
  expect_param(s, "comm.packet_loss_prob", {0, 1e-4, 1e-3, 1e-2});
  expect_every_param_resolves(s);
}

TEST(ProfileSweeps, MidSweepLoadsAndEveryParamResolves) {
  const auto r = load_sweep(kRepo / "config/sweeps/realism2_mid.yaml");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const SweepSpec& s = r.value();
  EXPECT_TRUE(same_file(s.base_profile, kRepo / "config/imperfections/mid_kr1205.yaml"))
      << s.base_profile;
  EXPECT_FALSE(s.seeds.empty());
  expect_param(s, "transmission.lost_motion_rad", {0, 2.9e-4, 8.7e-4, 1.7e-3});
  expect_param(s, "transmission.error_amp_rad", {0, 4.4e-4, 8.7e-4, 1.7e-3});
  expect_param(s, "actuator.torque_slew_nm_s", {1e4, 2e4, 5e4, 1e5});
  expect_param(s, "model.payload_kg", {0, 1, 3, 5});
  for (const SweepParam& p : s.params) {
    EXPECT_NE(p.name, "sensor.position_source") << "not sweepable";
  }
  expect_every_param_resolves(s);
}

}  // namespace
}  // namespace cobot::pipeline
