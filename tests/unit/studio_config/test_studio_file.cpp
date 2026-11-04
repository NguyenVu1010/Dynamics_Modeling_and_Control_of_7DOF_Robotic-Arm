// The studio configuration (feature studio-core, ADR-021, cobot/studio/studio_config.hpp): the
// studio file (root `studio:`, paths relative to the file, tolerance in mm and degrees), the input
// limits of validate (omega [10, 200] rad/s, safety.hal_timeout_ms [1, 50] ms, speed 1-100 %),
// materialize and describe_diff.
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/imperfect/config.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/overrides.hpp"
#include "cobot/studio/param_catalog.hpp"
#include "cobot/studio/profile_catalog.hpp"
#include "cobot/studio/series.hpp"
#include "cobot/studio/studio_config.hpp"

namespace cobot::studio {
namespace {

namespace fs = std::filesystem;
using pipeline::KeyOverride;

const fs::path kRepo = COBOT_REPO_DIR;
constexpr double kDeg = 3.14159265358979323846 / 180.0;

fs::path out_dir(const std::string& name) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_config" / name;
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

bool same_file(const fs::path& a, const fs::path& b) {
  std::error_code ec;
  return fs::equivalent(a, b, ec) && !ec;
}

bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

BasePair default_ideal() {
  return BasePair{
      "ideal", kRepo / "config/default.yaml", kRepo / "config/imperfections/ideal.yaml"};
}

BasePair premium() {
  return BasePair{"premium_fr3",
                  kRepo / "config/premium_fr3.yaml",
                  kRepo / "config/imperfections/premium_fr3.yaml"};
}

StudioConfig with(std::vector<KeyOverride> overrides) {
  StudioConfig c;
  c.base = default_ideal();
  c.overrides = std::move(overrides);
  return c;
}

void expect_same_overrides(const std::vector<KeyOverride>& a, const std::vector<KeyOverride>& b) {
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].key, b[i].key) << i;
    EXPECT_TRUE(a[i].value == b[i].value) << a[i].key;
  }
}

void expect_round_trip(const StudioConfig& c, const fs::path& file) {
  const common::Status saved = save_studio_file(c, file);
  ASSERT_TRUE(saved.is_ok()) << saved.reason();
  const common::Result<StudioConfig> r = load_studio_file(file);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const StudioConfig& l = r.value();
  EXPECT_EQ(l.base.name, c.base.name);
  EXPECT_TRUE(same_file(l.base.config_yaml, c.base.config_yaml)) << l.base.config_yaml;
  EXPECT_TRUE(same_file(l.base.imperfections_yaml, c.base.imperfections_yaml))
      << l.base.imperfections_yaml;
  expect_same_overrides(l.overrides, c.overrides);
  EXPECT_EQ(l.seed, c.seed);
  // Saved in mm and degrees: equal to rounding.
  EXPECT_NEAR(l.tolerance.goal_pos_m, c.tolerance.goal_pos_m, 1e-15);
  EXPECT_NEAR(l.tolerance.goal_rot_rad, c.tolerance.goal_rot_rad, 1e-15);
  EXPECT_NEAR(l.tolerance.path_pos_m, c.tolerance.path_pos_m, 1e-15);
  EXPECT_NEAR(l.tolerance.path_rot_rad, c.tolerance.path_rot_rad, 1e-15);
  EXPECT_EQ(l.speed_pct, c.speed_pct);
  ASSERT_EQ(l.waypoints_yaml.has_value(), c.waypoints_yaml.has_value());
  if (c.waypoints_yaml) {
    EXPECT_TRUE(same_file(*l.waypoints_yaml, *c.waypoints_yaml)) << *l.waypoints_yaml;
  }
  EXPECT_EQ(l.duration_s, c.duration_s);
}

TEST(StudioFile, RoundTripKeepsEveryFieldAndTheOverrideOrder) {
  StudioConfig c;
  c.base = premium();
  c.overrides = {
      KeyOverride{"sensor.q_noise_std_rad", 1.0e-4},
      KeyOverride{"config.computed_torque.omega", 50.0},
      KeyOverride{"friction.coulomb_nm", std::vector<double>{1.3, 1.3, 0.8, 0.8, 0.3, 0.3, 0.3}},
      KeyOverride{"config.deflection_comp.enabled", false},
      KeyOverride{"sensor.position_source", std::string("link")},
      KeyOverride{"config.safety.hal_timeout_ms", 0.1 + 0.2}};
  c.seed = 7;
  c.tolerance.goal_pos_m = 1.5e-3;
  c.tolerance.goal_rot_rad = 0.25 * kDeg;
  c.tolerance.path_pos_m = 3.0e-3;
  c.tolerance.path_rot_rad = 2.0 * kDeg;
  c.speed_pct = 25.0;
  c.waypoints_yaml = kRepo / "tests/e2e/waypoints/e2e_10.yaml";
  c.duration_s = 2.5;
  expect_round_trip(c, out_dir("round_trip") / "sub" / "studio.yaml");
}

TEST(StudioFile, RoundTripOfEmptyOptionals) {
  StudioConfig c;
  c.base = default_ideal();
  expect_round_trip(c, out_dir("round_trip_empty") / "studio.yaml");
}

// The example of docs/conventions.md "Studio file", base paths relative to the file.
TEST(StudioFile, LoadsTheDocumentedExample) {
  const fs::path dir = out_dir("example");
  const std::string config = fs::relative(kRepo / "config/premium_fr3.yaml", dir).generic_string();
  const std::string imperfections =
      fs::relative(kRepo / "config/imperfections/premium_fr3.yaml", dir).generic_string();
  const fs::path file = dir / "studio.yaml";
  {
    std::ofstream out(file, std::ios::binary);
    out << "studio:\n"
           "  version: 1\n"
           "  base: {name: premium_fr3, config: "
        << config << ", imperfections: " << imperfections
        << "}\n"
           "  overrides:\n"
           "    config.computed_torque.omega: 50\n"
           "    sensor.q_noise_std_rad: 1.0e-4\n"
           "    friction.coulomb_nm: [1.3, 1.3, 0.8, 0.8, 0.3, 0.3, 0.3]\n"
           "  seed: 1\n"
           "  tolerance: {goal_pos_mm: 1.0, goal_rot_deg: 0.5, path_pos_mm: 2.0, path_rot_deg: "
           "1.0}\n"
           "  speed_pct: 10\n"
           "  waypoints: null\n"
           "  duration_s: null\n";
  }
  const common::Result<StudioConfig> r = load_studio_file(file);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const StudioConfig& c = r.value();
  EXPECT_EQ(c.base.name, "premium_fr3");
  EXPECT_TRUE(same_file(c.base.config_yaml, kRepo / "config/premium_fr3.yaml"));
  EXPECT_TRUE(
      same_file(c.base.imperfections_yaml, kRepo / "config/imperfections/premium_fr3.yaml"));
  expect_same_overrides(
      c.overrides,
      {KeyOverride{"config.computed_torque.omega", 50.0},
       KeyOverride{"sensor.q_noise_std_rad", 1.0e-4},
       KeyOverride{"friction.coulomb_nm", std::vector<double>{1.3, 1.3, 0.8, 0.8, 0.3, 0.3, 0.3}}});
  EXPECT_EQ(c.seed, std::optional<std::uint64_t>(1));
  EXPECT_NEAR(c.tolerance.goal_pos_m, 1e-3, 1e-15);
  EXPECT_NEAR(c.tolerance.goal_rot_rad, 0.5 * kDeg, 1e-15);
  EXPECT_NEAR(c.tolerance.path_pos_m, 2e-3, 1e-15);
  EXPECT_NEAR(c.tolerance.path_rot_rad, 1.0 * kDeg, 1e-15);
  EXPECT_EQ(c.speed_pct, 10.0);
  EXPECT_FALSE(c.waypoints_yaml.has_value());
  EXPECT_FALSE(c.duration_s.has_value());
  EXPECT_TRUE(validate(c).empty());
}

TEST(StudioFile, UnknownKeyAndMissingFileAreErrors) {
  const fs::path dir = out_dir("bad");
  const fs::path file = dir / "studio.yaml";
  {
    std::ofstream out(file, std::ios::binary);
    // Valid base paths: the only problem is the unknown key.
    out << "studio:\n  version: 1\n  base: {name: ideal, config: "
        << (kRepo / "config/default.yaml").generic_string()
        << ", imperfections: " << (kRepo / "config/imperfections/ideal.yaml").generic_string()
        << "}\n  bogus_key: 3\n";
  }
  const common::Result<StudioConfig> r = load_studio_file(file);
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "bogus_key")) << r.reason();
  EXPECT_FALSE(load_studio_file(dir / "missing.yaml").is_ok());
}

// --- validate: input limits ---

void expect_one_issue_naming(const StudioConfig& c, const std::string& key) {
  const std::vector<std::string> issues = validate(c);
  ASSERT_EQ(issues.size(), 1U) << key;
  EXPECT_TRUE(contains(issues[0], key)) << issues[0];
}

TEST(StudioValidate, OutOfRangeValuesGiveOneMessageNamingTheKey) {
  for (const double omega : {9.9, 200.1}) {
    expect_one_issue_naming(with({KeyOverride{"config.computed_torque.omega", omega}}),
                            "config.computed_torque.omega");
  }
  for (const double ms : {0.5, 51.0}) {
    expect_one_issue_naming(with({KeyOverride{"config.safety.hal_timeout_ms", ms}}),
                            "config.safety.hal_timeout_ms");
  }
  for (const double pct : {0.0, 101.0}) {
    StudioConfig c = with({});
    c.speed_pct = pct;
    expect_one_issue_naming(c, "speed_pct");
  }
}

TEST(StudioValidate, LimitsAreInclusive) {
  for (const double omega : {10.0, 200.0}) {
    EXPECT_TRUE(validate(with({KeyOverride{"config.computed_torque.omega", omega}})).empty())
        << omega;
  }
  for (const double ms : {1.0, 50.0}) {
    EXPECT_TRUE(validate(with({KeyOverride{"config.safety.hal_timeout_ms", ms}})).empty()) << ms;
  }
  for (const double pct : {1.0, 100.0}) {
    StudioConfig c = with({});
    c.speed_pct = pct;
    EXPECT_TRUE(validate(c).empty()) << pct;
  }
}

TEST(StudioValidate, UnknownKeyGivesAMessage) {
  const std::vector<std::string> issues = validate(with({KeyOverride{"sensor.bogus_key", 1.0}}));
  ASSERT_FALSE(issues.empty());
  bool named = false;
  for (const std::string& line : issues) {
    named = named || contains(line, "sensor.bogus_key");
  }
  EXPECT_TRUE(named) << issues[0];
}

// --- materialize, describe_diff, noise source ---

TEST(StudioMaterialize, WritesAPairThePlainLoadersRead) {
  const fs::path dir = out_dir("materialize");
  const StudioConfig c = with({KeyOverride{"config.computed_torque.omega", 50.0},
                               KeyOverride{"sensor.q_noise_std_rad", 1.0e-4}});
  const common::Result<Materialized> m = materialize(c, dir);
  ASSERT_TRUE(m.is_ok()) << m.reason();
  for (const fs::path& p : {m.value().config_yaml, m.value().imperfections_yaml}) {
    EXPECT_TRUE(fs::is_regular_file(p)) << p;
    const fs::path rel = fs::relative(p, dir);
    EXPECT_FALSE(rel.empty() || *rel.begin() == "..") << p << " is not under " << dir;
  }
  const auto config = pipeline::load_config<7>(m.value().config_yaml);
  ASSERT_TRUE(config.is_ok()) << config.reason();
  EXPECT_EQ(config.value().computed_torque.kp(0), 2500.0);  // 50^2
  EXPECT_EQ(config.value().computed_torque.kd(6), 100.0);   // 2 * 50
  const auto imp = pipeline::load_imperfections<7>(m.value().imperfections_yaml);
  ASSERT_TRUE(imp.is_ok()) << imp.reason();
  EXPECT_EQ(imp.value().sensor.q_noise_std_rad(3), 1.0e-4);
}

TEST(StudioDiff, NamesTheChangedKeysInOrder) {
  EXPECT_EQ(describe_diff(with({})), "");
  const std::string d = describe_diff(with({KeyOverride{"config.computed_torque.omega", 50.0},
                                            KeyOverride{"sensor.q_noise_std_rad", 1.0e-4}}));
  EXPECT_EQ(d.rfind("omega=50", 0), 0U) << d;
  const std::size_t noise = d.find(", sensor.q_noise_std_rad=");
  EXPECT_NE(noise, std::string::npos) << d;
}

TEST(StudioNoiseSource, FollowsTheChangedKeysCatalogEntries) {
  EXPECT_EQ(noise_source_of(with({})), NoiseSource::kNone);
  EXPECT_EQ(noise_source_of(with({KeyOverride{"sensor.q_noise_std_rad", 1.0e-4}})),
            NoiseSource::kSensor);
  EXPECT_EQ(noise_source_of(with({KeyOverride{"actuator.torque_ripple_frac", 0.02}})),
            NoiseSource::kActuator);
  EXPECT_EQ(noise_source_of(with({KeyOverride{"friction.coulomb_nm", 1.0}})),
            NoiseSource::kFriction);
  EXPECT_EQ(noise_source_of(with({KeyOverride{"model.payload_kg", 1.0}})), NoiseSource::kNone);
  EXPECT_EQ(noise_source_of(with({KeyOverride{"config.computed_torque.omega", 50.0}})),
            NoiseSource::kNone);
}

}  // namespace
}  // namespace cobot::studio
