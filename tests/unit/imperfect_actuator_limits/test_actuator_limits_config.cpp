#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/sweep_config.hpp"

// Loader and sweep side of feature actuator-limits: the new `actuator.*` keys are read from a
// scalar or a 7-list, validated, and swept by config/sweeps/hardware.yaml.
namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;
using Vec = common::VecN<7>;

std::string ideal_text() {
  const std::ifstream in(fs::path(COBOT_REPO_DIR) / "config/imperfections/ideal.yaml",
                         std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// ideal.yaml with `actuator.<key>: <value>` set: replaces the key (and any nested block under it)
// when the profile already has it, else inserts it at the top of the actuator group.
std::string with_actuator(std::string text, const std::string& key, const std::string& value) {
  const std::string group = "\n  actuator:";
  const std::size_t start = text.find(group);
  EXPECT_NE(start, std::string::npos);
  const std::size_t body = text.find('\n', start + 1);
  std::size_t end = body;
  while (end != std::string::npos && text.compare(end, 5, "\n    ") == 0) {
    end = text.find('\n', end + 1);
  }
  const std::string line = "\n    " + key + ": " + value;
  const std::size_t at = text.find("\n    " + key + ":", body);
  if (at == std::string::npos || at >= end) {
    text.insert(body, line);
    return text;
  }
  std::size_t stop = text.find('\n', at + 1);
  while (stop != std::string::npos && text.compare(stop, 7, "\n      ") == 0) {
    stop = text.find('\n', stop + 1);
  }
  text.replace(at, (stop == std::string::npos ? text.size() : stop) - at, line);
  return text;
}

Result<imperfect::ImperfectionConfig<7>> load_text(const std::string& name,
                                                   const std::string& text) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "imperfect_actuator_limits";
  fs::create_directories(dir);
  const fs::path p = dir / name;
  std::ofstream(p, std::ios::binary) << text;
  return load_imperfections<7>(p);
}

void expect_rejected(const std::string& name, const std::string& text, const std::string& key) {
  const auto r = load_text(name, text);
  ASSERT_FALSE(r.is_ok()) << name;
  EXPECT_NE(r.reason().find(key), std::string::npos) << r.reason();
}

TEST(ActuatorLimitsConfig, IdealProfileHasEveryNewEffectOff) {
  const auto r = load_text("ideal.yaml", ideal_text());
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const imperfect::ActuatorConfig<7>& a = r.value().actuator;
  EXPECT_TRUE(a.torque_scale_asym_frac.isZero(0.0));
  EXPECT_TRUE(a.deadband_nm.isZero(0.0));
  EXPECT_TRUE(a.torque_slew_nm_s.isZero(0.0));
  EXPECT_TRUE(a.knee_speed_rad_s.isZero(0.0));
  EXPECT_TRUE(a.zero_torque_speed_rad_s.isZero(0.0));
  EXPECT_EQ(a.bus.v_nom_v, 0.0);
}

TEST(ActuatorLimitsConfig, NewKeysFromScalars) {
  std::string t = ideal_text();
  t = with_actuator(t, "torque_scale_asym_frac", "0.1");
  t = with_actuator(t, "deadband_nm", "0.5");
  t = with_actuator(t, "torque_slew_nm_s", "1000");
  t = with_actuator(t, "knee_speed_rad_s", "2.2");
  t = with_actuator(t, "zero_torque_speed_rad_s", "3.6");
  t = with_actuator(t, "bus", "{v_nom_v: 48, r_source_ohm: 0.25, v_min_v: 40}");
  const auto r = load_text("scalars.yaml", t);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const imperfect::ActuatorConfig<7>& a = r.value().actuator;
  EXPECT_EQ(a.torque_scale_asym_frac, Vec::Constant(0.1));
  EXPECT_EQ(a.deadband_nm, Vec::Constant(0.5));
  EXPECT_EQ(a.torque_slew_nm_s, Vec::Constant(1000.0));
  EXPECT_EQ(a.knee_speed_rad_s, Vec::Constant(2.2));
  EXPECT_EQ(a.zero_torque_speed_rad_s, Vec::Constant(3.6));
  EXPECT_EQ(a.bus.v_nom_v, 48.0);
  EXPECT_EQ(a.bus.r_source_ohm, 0.25);
  EXPECT_EQ(a.bus.v_min_v, 40.0);
}

TEST(ActuatorLimitsConfig, NewKeysFromLists) {
  std::string t = ideal_text();
  t = with_actuator(t, "torque_scale_asym_frac", "[0, 0.01, 0.03, 0.1, 0.2, 0.3, 0.5]");
  t = with_actuator(t, "deadband_nm", "[0, 0.01, 0.1, 0.5, 1, 2, 3]");
  t = with_actuator(t, "torque_slew_nm_s", "[0, 1e5, 2e4, 5e3, 1, 2, 3]");
  t = with_actuator(t, "knee_speed_rad_s", "[1.2, 1.2, 1.3, 1.3, 0, 2.2, 2.2]");
  t = with_actuator(t, "zero_torque_speed_rad_s", "[2.1, 2.1, 2.1, 2.1, 0, 3.6, 3.6]");
  const auto r = load_text("lists.yaml", t);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const imperfect::ActuatorConfig<7>& a = r.value().actuator;
  EXPECT_EQ(a.torque_scale_asym_frac, (Vec() << 0, 0.01, 0.03, 0.1, 0.2, 0.3, 0.5).finished());
  EXPECT_EQ(a.deadband_nm, (Vec() << 0, 0.01, 0.1, 0.5, 1, 2, 3).finished());
  EXPECT_EQ(a.torque_slew_nm_s, (Vec() << 0, 1e5, 2e4, 5e3, 1, 2, 3).finished());
  EXPECT_EQ(a.knee_speed_rad_s, (Vec() << 1.2, 1.2, 1.3, 1.3, 0, 2.2, 2.2).finished());
  EXPECT_EQ(a.zero_torque_speed_rad_s, (Vec() << 2.1, 2.1, 2.1, 2.1, 0, 3.6, 3.6).finished());
}

TEST(ActuatorLimitsConfig, InvalidValuesRejectedNamingTheKey) {
  const std::string t = ideal_text();
  const std::string curve = with_actuator(t, "zero_torque_speed_rad_s", "3.6");
  expect_rejected("knee_eq.yaml", with_actuator(curve, "knee_speed_rad_s", "3.6"), "knee_speed");
  expect_rejected("knee_gt.yaml",
                  with_actuator(curve, "knee_speed_rad_s", "[2, 2, 2, 4, 2, 2, 2]"),
                  "knee_speed");
  expect_rejected("bus_vmin_eq.yaml",
                  with_actuator(t, "bus", "{v_nom_v: 48, r_source_ohm: 0.25, v_min_v: 48}"),
                  "actuator.bus");
  expect_rejected("bus_vmin_gt.yaml",
                  with_actuator(t, "bus", "{v_nom_v: 48, r_source_ohm: 0.25, v_min_v: 50}"),
                  "actuator.bus");
  expect_rejected("bus_r_neg.yaml",
                  with_actuator(t, "bus", "{v_nom_v: 48, r_source_ohm: -0.1, v_min_v: 40}"),
                  "r_source_ohm");
  expect_rejected(
      "asym_one.yaml", with_actuator(t, "torque_scale_asym_frac", "1.0"), "torque_scale_asym_frac");
  expect_rejected("asym_neg.yaml",
                  with_actuator(t, "torque_scale_asym_frac", "-0.1"),
                  "torque_scale_asym_frac");
  expect_rejected("deadband_neg.yaml", with_actuator(t, "deadband_nm", "-0.1"), "deadband_nm");
  expect_rejected("slew_neg.yaml", with_actuator(t, "torque_slew_nm_s", "-1"), "torque_slew_nm_s");
}

// design 8: one-at-a-time levels for the three single-key effects, applicable to the base profile.
TEST(ActuatorLimitsConfig, HardwareSweepCoversTheSingleKeyEffects) {
  const auto sweep = load_sweep(fs::path(COBOT_REPO_DIR) / "config/sweeps/hardware.yaml");
  ASSERT_TRUE(sweep.is_ok()) << sweep.reason();
  const std::vector<std::pair<std::string, std::vector<double>>> want = {
      {"actuator.torque_scale_asym_frac", {0, 0.01, 0.03, 0.1}},
      {"actuator.deadband_nm", {0, 0.01, 0.1, 0.5}},
      {"actuator.torque_slew_nm_s", {0, 1e5, 2e4, 5e3}}};
  for (const auto& [name, levels] : want) {
    const SweepParam* found = nullptr;
    for (const SweepParam& p : sweep.value().params) {
      found = p.name == name ? &p : found;
    }
    ASSERT_NE(found, nullptr) << name;
    EXPECT_EQ(found->levels, levels) << name;
    for (const double level : levels) {
      const auto r = load_imperfections_with<7>(sweep.value().base_profile, name, level);
      ASSERT_TRUE(r.is_ok()) << name << " " << level << ": " << r.reason();
      const imperfect::ActuatorConfig<7>& a = r.value().actuator;
      const Vec& got = name == "actuator.deadband_nm"        ? a.deadband_nm
                       : name == "actuator.torque_slew_nm_s" ? a.torque_slew_nm_s
                                                             : a.torque_scale_asym_frac;
      EXPECT_EQ(got, Vec::Constant(level)) << name;
    }
  }
}

}  // namespace
}  // namespace cobot::pipeline
