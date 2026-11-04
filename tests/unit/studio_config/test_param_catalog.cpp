// The studio's parameter catalog (feature studio-core, cobot_studio design 9, ADR-021,
// cobot/studio/param_catalog.hpp): hand-listed, key names exactly as the YAML files. Every leaf key
// of the six imperfection profiles (except `source` and `seed`) is in it; every entry loads on
// default + ideal through the typed-override loaders with its base_value; base_value reads values
// in YAML units (an override of 0.123 reads back as 0.123).
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/overrides.hpp"
#include "cobot/studio/param_catalog.hpp"
#include "cobot/studio/profile_catalog.hpp"

namespace cobot::studio {
namespace {

namespace fs = std::filesystem;
using pipeline::KeyOverride;
constexpr int kN = 7;

const fs::path kRepo = COBOT_REPO_DIR;
const std::vector<std::string> kProfiles{
    "ideal", "realistic", "pessimistic", "premium_fr3", "mid_kr1205", "budget_xarm7"};
// Integer keys of the imperfection schema: their read-back check uses 1 instead of 0.123.
const std::set<std::string> kIntegerKeys{"sensor.sensor_delay_cycles",
                                         "actuator.command_delay_cycles",
                                         "timing.overrun_cycles",
                                         "comm.burst_cycles"};

BasePair default_ideal() {
  return BasePair{
      "ideal", kRepo / "config/default.yaml", kRepo / "config/imperfections/ideal.yaml"};
}

bool is_config_key(const std::string& key) {
  return key.rfind("config.", 0) == 0;
}

// Dotted paths of the leaves under `node` (a list is one leaf).
void leaves(const YAML::Node& node, const std::string& prefix, std::set<std::string>& out) {
  for (const auto& kv : node) {
    const auto key = kv.first.as<std::string>();
    std::string path = prefix;
    if (!path.empty()) {
      path += '.';
    }
    path += key;
    if (kv.second.IsMap()) {
      leaves(kv.second, path, out);
    } else if (key != "source" && path != "seed") {
      out.insert(path);
    }
  }
}

TEST(ParamCatalog, KeysAreUnique) {
  std::set<std::string> seen;
  for (const ParamInfo& p : param_catalog()) {
    EXPECT_TRUE(seen.insert(p.key).second) << "duplicate " << p.key;
    EXPECT_FALSE(p.group.empty()) << p.key;
  }
}

TEST(ParamCatalog, EveryProfileKeyIsListed) {
  std::set<std::string> keys;
  for (const std::string& name : kProfiles) {
    const YAML::Node root =
        YAML::LoadFile((kRepo / "config/imperfections" / (name + ".yaml")).string());
    leaves(root["imperfections"], "", keys);
  }
  EXPECT_TRUE(keys.count("actuator.bus.v_nom_v") == 1U);  // nested groups are walked
  EXPECT_TRUE(keys.count("transmission.lost_motion_rad") == 1U);
  for (const std::string& key : keys) {
    EXPECT_NE(find_param(key), nullptr) << key << " is not in param_catalog()";
  }
}

TEST(ParamCatalog, ControllerEntriesAndLimits) {
  const ParamInfo* omega = find_param("config.computed_torque.omega");
  ASSERT_NE(omega, nullptr);
  EXPECT_EQ(omega->kind, ParamKind::kNumber);
  EXPECT_EQ(omega->min, 10.0);
  EXPECT_EQ(omega->max, 200.0);
  EXPECT_TRUE(omega->sweepable);
  const ParamInfo* hal = find_param("config.safety.hal_timeout_ms");
  ASSERT_NE(hal, nullptr);
  EXPECT_EQ(hal->kind, ParamKind::kNumber);
  EXPECT_EQ(hal->min, 1.0);
  EXPECT_EQ(hal->max, 50.0);
  EXPECT_TRUE(hal->sweepable);
  for (const char* key : {"config.computed_torque.kp", "config.computed_torque.kd"}) {
    const ParamInfo* p = find_param(key);
    ASSERT_NE(p, nullptr) << key;
    EXPECT_EQ(p->kind, ParamKind::kPerJoint) << key;
  }
  EXPECT_EQ(find_param("config.no_such.key"), nullptr);
  EXPECT_EQ(find_param("sensor.q_noise_stdd"), nullptr);
}

TEST(ParamCatalog, KindsAndNoiseSources) {
  const ParamInfo* noise = find_param("sensor.q_noise_std_rad");
  ASSERT_NE(noise, nullptr);
  EXPECT_EQ(noise->kind, ParamKind::kPerJoint);
  EXPECT_EQ(noise->noise, NoiseSource::kSensor);
  EXPECT_TRUE(noise->sweepable);
  const ParamInfo* source = find_param("sensor.position_source");
  ASSERT_NE(source, nullptr);
  EXPECT_EQ(source->kind, ParamKind::kChoice);
  EXPECT_EQ(source->choices, (std::vector<std::string>{"link", "motor"}));
  EXPECT_FALSE(source->sweepable);
  const ParamInfo* diff = find_param("sensor.qd_from_diff");
  ASSERT_NE(diff, nullptr);
  EXPECT_EQ(diff->kind, ParamKind::kBool);
  ASSERT_NE(find_param("actuator.torque_ripple_frac"), nullptr);
  EXPECT_EQ(find_param("actuator.torque_ripple_frac")->noise, NoiseSource::kActuator);
  ASSERT_NE(find_param("friction.coulomb_nm"), nullptr);
  EXPECT_EQ(find_param("friction.coulomb_nm")->noise, NoiseSource::kFriction);
  ASSERT_NE(find_param("model.payload_kg"), nullptr);
  EXPECT_EQ(find_param("model.payload_kg")->noise, NoiseSource::kNone);
  EXPECT_EQ(find_param("model.payload_kg")->kind, ParamKind::kNumber);
  ASSERT_NE(find_param("comm.packet_loss_prob"), nullptr);
  EXPECT_EQ(find_param("comm.packet_loss_prob")->noise, NoiseSource::kNone);
}

TEST(ParamCatalog, EveryEntryLoadsOnDefaultIdealWithItsBaseValue) {
  const BasePair base = default_ideal();
  for (const ParamInfo& p : param_catalog()) {
    const common::Result<KeyOverride> v = base_value(base, p.key);
    ASSERT_TRUE(v.is_ok()) << p.key << ": " << v.reason();
    EXPECT_EQ(v.value().key, p.key);
    const std::vector<KeyOverride> ov{v.value()};
    if (is_config_key(p.key)) {
      const auto r = pipeline::load_config_with<kN>(base.config_yaml, ov);
      EXPECT_TRUE(r.is_ok()) << p.key << ": " << r.reason();
    } else {
      const auto r = pipeline::load_imperfections_with<kN>(base.imperfections_yaml, ov);
      EXPECT_TRUE(r.is_ok()) << p.key << ": " << r.reason();
    }
  }
}

TEST(ParamCatalog, BaseOmegaIsThePairsGain) {
  // omega = sqrt(kp_1): default.yaml 6400 -> 80; premium 2500 -> 50; mid 1600 -> 40; budget
  // 400 -> 20 (config/<profile>.yaml, owner decision 2026-11-03).
  const std::vector<std::pair<std::string, double>> expected{
      {"realistic", 80.0}, {"premium_fr3", 50.0}, {"mid_kr1205", 40.0}, {"budget_xarm7", 20.0}};
  const std::vector<BasePair> pairs = discover_base_pairs(kRepo / "config");
  for (const auto& [name, omega] : expected) {
    const BasePair* pair = nullptr;
    for (const BasePair& p : pairs) {
      pair = p.name == name ? &p : pair;
    }
    ASSERT_NE(pair, nullptr) << name;
    const common::Result<KeyOverride> v = base_value(*pair, "config.computed_torque.omega");
    ASSERT_TRUE(v.is_ok()) << v.reason();
    ASSERT_TRUE(std::holds_alternative<double>(v.value().value)) << name;
    EXPECT_DOUBLE_EQ(std::get<double>(v.value().value), omega) << name;
  }
}

TEST(ParamCatalog, NumberKeysReadBackInYamlUnits) {
  const BasePair base = default_ideal();
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_param_readback";
  fs::create_directories(dir);
  std::size_t checked = 0;
  for (const ParamInfo& p : param_catalog()) {
    if (p.kind != ParamKind::kNumber) {
      continue;
    }
    const double level = kIntegerKeys.count(p.key) == 1U ? 1.0 : 0.123;
    const std::vector<KeyOverride> ov{KeyOverride{p.key, level}};
    BasePair edited = base;
    common::Status written = common::Status::ok();
    if (is_config_key(p.key)) {
      edited.config_yaml = dir / (p.key + ".config.yaml");
      written = pipeline::write_config_with(base.config_yaml, ov, edited.config_yaml);
    } else {
      edited.imperfections_yaml = dir / (p.key + ".imperfections.yaml");
      written = pipeline::write_imperfections_with(
          base.imperfections_yaml, ov, edited.imperfections_yaml);
    }
    // omega, hal_timeout_ms and every imperfection number take 0.123 (or 1); another controller
    // number may reject it by its own range rule, never as an unknown key.
    const bool must_accept = !is_config_key(p.key) || p.key == "config.computed_torque.omega" ||
                             p.key == "config.safety.hal_timeout_ms";
    if (!written.is_ok()) {
      EXPECT_FALSE(must_accept) << p.key << ": " << written.reason();
      EXPECT_EQ(written.reason().find("unknown key"), std::string::npos) << written.reason();
      continue;
    }
    const common::Result<KeyOverride> v = base_value(edited, p.key);
    ASSERT_TRUE(v.is_ok()) << p.key << ": " << v.reason();
    ASSERT_TRUE(std::holds_alternative<double>(v.value().value)) << p.key;
    // omega reads back as sqrt(kp_1) = sqrt(0.123^2): equal to rounding.
    EXPECT_NEAR(std::get<double>(v.value().value), level, 1e-15) << p.key;
    ++checked;
  }
  EXPECT_GE(checked, 3U);
}

}  // namespace
}  // namespace cobot::studio
