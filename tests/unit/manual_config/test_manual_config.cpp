// cobot.manual in load_config (feature manual-commands, ADR-022 "Lệnh tay", plan ambiguity 2):
//   cobot:
//     manual:
//       joint_a_max_rad_s2: [7 values] or a scalar   # > 0, at 100 % speed
//       tcp_v_max_m_s, tcp_w_max_rad_s, tcp_a_max_m_s2, tcp_alpha_max_rad_s2   # > 0
//       deadman_ms       # > 0, stored as deadman_s
//       jog_margin_deg   # >= 0, stored as jog_margin_rad
//       joint_speed_fraction   # in (0, 1], default 0.9 (feature jog-headroom): joint jog and
//                              # joint move use at most this fraction of qd_max at 100 % speed
// The section and each of its sub-keys are optional: a missing one keeps the ManualConfig default.
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/pipeline/config.hpp"

namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;
using Vec7 = common::VecN<7>;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegree = kPi / 180.0;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kUrdf = kRepo / "src/sim/urdf/cobot30_5.urdf";

std::string read_file(const fs::path& p) {
  const std::ifstream in(p, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

fs::path write_temp(const std::string& name, const std::string& text) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "manual_config";
  fs::create_directories(dir);
  const fs::path p = dir / name;
  std::ofstream(p, std::ios::binary) << text;
  return p;
}

// Replaces the first occurrence of `from`; fails the test if the fixture text is absent.
std::string replace(std::string text, const std::string& from, const std::string& to) {
  const auto pos = text.find(from);
  EXPECT_NE(pos, std::string::npos) << "fixture text not found: " << from;
  if (pos != std::string::npos) {
    text.replace(pos, from.size(), to);
  }
  return text;
}

bool starts_with(const std::string& line, const std::string& prefix) {
  return line.rfind(prefix, 0) == 0;
}

// config/default.yaml with an absolute URDF path and without its `  manual:` block (the key line
// and every following line indented by more than two spaces, or empty), so a test can append its
// own block at the end of the cobot map.
std::string default_without_manual() {
  const std::string text = replace(
      read_file(kRepo / "config/default.yaml"), "../src/sim/urdf/cobot30_5.urdf", kUrdf.string());
  std::istringstream in(text);
  std::string out;
  std::string line;
  bool inside = false;
  while (std::getline(in, line)) {
    if (starts_with(line, "  manual:")) {
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

// default.yaml with `  manual:` + `body` (lines indented by four spaces) at the end of the cobot
// map.
Result<Config<7>> load_with_manual(const std::string& name, const std::string& body) {
  return load_config<7>(write_temp(name, default_without_manual() + "  manual:\n" + body));
}

void expect_vec_eq(const Vec7& actual, const Vec7& expected) {
  for (int j = 0; j < 7; ++j) {
    EXPECT_DOUBLE_EQ(actual(j), expected(j)) << "joint " << j + 1;
  }
}

// The values of the YAML block of the brief (URDF <limit acceleration> per joint, ADR-022 TCP
// limits).
TEST(ManualConfig, DefaultYamlLoadsTheManualSection) {
  const auto cfg = load_config<7>(kRepo / "config/default.yaml");
  ASSERT_TRUE(cfg) << cfg.reason();
  const ManualConfig<7>& m = cfg.value().manual;
  Vec7 a;
  a << 15.0, 7.5, 12.5, 12.5, 15.0, 20.0, 20.0;
  expect_vec_eq(m.joint_a_max_rad_s2, a);
  EXPECT_DOUBLE_EQ(m.tcp_v_max_m_s, 0.25);
  EXPECT_DOUBLE_EQ(m.tcp_w_max_rad_s, 1.0);
  EXPECT_DOUBLE_EQ(m.tcp_a_max_m_s2, 0.5);
  EXPECT_DOUBLE_EQ(m.tcp_alpha_max_rad_s2, 2.0);
  EXPECT_NEAR(m.deadman_s, 0.1, 1e-15);                 // 100 ms
  EXPECT_NEAR(m.jog_margin_rad, 2.0 * kDegree, 1e-15);  // 2 deg = 0.0349066 rad
  EXPECT_DOUBLE_EQ(m.joint_speed_fraction, 0.9);
  EXPECT_NE(read_file(kRepo / "config/default.yaml").find("joint_speed_fraction: 0.9"),
            std::string::npos);
}

// The struct defaults (no section): a_max 5 rad/s^2 per joint, TCP 0.25 m/s, 1 rad/s, 0.5 m/s^2,
// 2 rad/s^2, deadman 0.1 s, jog margin 2 deg.
TEST(ManualConfig, MissingSectionKeepsTheStructDefaults) {
  const std::string text = default_without_manual();
  ASSERT_EQ(text.find("\n  manual:"), std::string::npos);
  const auto cfg = load_config<7>(write_temp("no_manual.yaml", text));
  ASSERT_TRUE(cfg) << cfg.reason();
  const ManualConfig<7>& m = cfg.value().manual;
  const ManualConfig<7> d;
  expect_vec_eq(m.joint_a_max_rad_s2, Vec7::Constant(5.0));
  expect_vec_eq(d.joint_a_max_rad_s2, Vec7::Constant(5.0));
  EXPECT_DOUBLE_EQ(m.tcp_v_max_m_s, 0.25);
  EXPECT_DOUBLE_EQ(m.tcp_w_max_rad_s, 1.0);
  EXPECT_DOUBLE_EQ(m.tcp_a_max_m_s2, 0.5);
  EXPECT_DOUBLE_EQ(m.tcp_alpha_max_rad_s2, 2.0);
  EXPECT_DOUBLE_EQ(m.deadman_s, 0.1);
  EXPECT_NEAR(m.jog_margin_rad, 2.0 * kDegree, 1e-15);
  EXPECT_DOUBLE_EQ(m.joint_speed_fraction, 0.9);
  EXPECT_DOUBLE_EQ(d.joint_speed_fraction, 0.9);
}

// A partial section keeps the defaults of the keys it leaves out.
TEST(ManualConfig, PartialSectionKeepsTheOtherDefaults) {
  const auto cfg = load_with_manual("partial.yaml",
                                    "    tcp_v_max_m_s: 0.1\n"
                                    "    deadman_ms: 250.0\n");
  ASSERT_TRUE(cfg) << cfg.reason();
  const ManualConfig<7>& m = cfg.value().manual;
  EXPECT_DOUBLE_EQ(m.tcp_v_max_m_s, 0.1);
  EXPECT_NEAR(m.deadman_s, 0.25, 1e-15);  // 250 ms
  expect_vec_eq(m.joint_a_max_rad_s2, Vec7::Constant(5.0));
  EXPECT_DOUBLE_EQ(m.tcp_w_max_rad_s, 1.0);
  EXPECT_DOUBLE_EQ(m.tcp_a_max_m_s2, 0.5);
  EXPECT_DOUBLE_EQ(m.tcp_alpha_max_rad_s2, 2.0);
  EXPECT_NEAR(m.jog_margin_rad, 2.0 * kDegree, 1e-15);
  EXPECT_DOUBLE_EQ(m.joint_speed_fraction, 0.9);  // key missing
}

// jog_margin_deg 0 is allowed (>= 0): 0 rad.
TEST(ManualConfig, JogMarginZeroIsAllowed) {
  const auto cfg = load_with_manual("jog_margin_zero.yaml", "    jog_margin_deg: 0.0\n");
  ASSERT_TRUE(cfg) << cfg.reason();
  EXPECT_EQ(cfg.value().manual.jog_margin_rad, 0.0);
}

TEST(ManualConfig, DeadmanZeroIsAnErrorNamingTheFullKey) {
  const auto cfg = load_with_manual("deadman_zero.yaml", "    deadman_ms: 0\n");
  ASSERT_FALSE(cfg);
  EXPECT_NE(cfg.reason().find("cobot.manual.deadman_ms"), std::string::npos) << cfg.reason();
}

TEST(ManualConfig, NonPositiveLimitsAreErrorsNamingTheFullKey) {
  const std::vector<std::string> keys = {
      "tcp_v_max_m_s", "tcp_w_max_rad_s", "tcp_a_max_m_s2", "tcp_alpha_max_rad_s2"};
  for (const std::string& key : keys) {
    const auto cfg = load_with_manual(key + "_zero.yaml", "    " + key + ": 0.0\n");
    ASSERT_FALSE(cfg) << key;
    EXPECT_NE(cfg.reason().find("cobot.manual." + key), std::string::npos) << cfg.reason();
  }
  const auto neg_a = load_with_manual("a_negative.yaml",
                                      "    joint_a_max_rad_s2: [15, 7.5, 12.5, -1, 15, 20, 20]\n");
  ASSERT_FALSE(neg_a);
  EXPECT_NE(neg_a.reason().find("cobot.manual.joint_a_max_rad_s2"), std::string::npos)
      << neg_a.reason();
  const auto neg_margin = load_with_manual("margin_negative.yaml", "    jog_margin_deg: -1.0\n");
  ASSERT_FALSE(neg_margin);
  EXPECT_NE(neg_margin.reason().find("cobot.manual.jog_margin_deg"), std::string::npos)
      << neg_margin.reason();
}

TEST(ManualConfig, SixAccelerationValuesIsAnErrorNamingBothCounts) {
  const auto cfg = load_with_manual(
      "a_six.yaml", "    joint_a_max_rad_s2: [15.0, 7.5, 12.5, 12.5, 15.0, 20.0]\n");
  ASSERT_FALSE(cfg);
  const std::string reason = cfg.reason();
  EXPECT_NE(reason.find("cobot.manual.joint_a_max_rad_s2"), std::string::npos) << reason;
  EXPECT_NE(reason.find('7'), std::string::npos) << reason;
  EXPECT_NE(reason.find('6'), std::string::npos) << reason;
}

TEST(ManualConfig, ScalarAccelerationBroadcasts) {
  const auto cfg = load_with_manual("a_scalar.yaml", "    joint_a_max_rad_s2: 8.0\n");
  ASSERT_TRUE(cfg) << cfg.reason();
  expect_vec_eq(cfg.value().manual.joint_a_max_rad_s2, Vec7::Constant(8.0));
}

TEST(ManualConfig, UnknownKeyInTheSectionIsAnError) {
  const auto cfg = load_with_manual("unknown_key.yaml", "    jog_speed: 1.0\n");
  ASSERT_FALSE(cfg);
  EXPECT_NE(cfg.reason().find("cobot.manual.jog_speed"), std::string::npos) << cfg.reason();
}

// The four controller configs differ only in the gains: their manual blocks are identical
// (tests/python/test_profile_controllers.py pins the text; this pins the loaded values).
TEST(ManualConfig, TheFourControllerConfigsCarryIdenticalManualBlocks) {
  const auto ref = load_config<7>(kRepo / "config/default.yaml");
  ASSERT_TRUE(ref) << ref.reason();
  const ManualConfig<7>& r = ref.value().manual;
  for (const char* name : {"premium_fr3.yaml", "mid_kr1205.yaml", "budget_xarm7.yaml"}) {
    const std::string text = read_file(kRepo / "config" / name);
    EXPECT_NE(text.find("\n  manual:"), std::string::npos) << name << ": no manual block";
    const auto cfg = load_config<7>(kRepo / "config" / name);
    ASSERT_TRUE(cfg) << name << ": " << cfg.reason();
    const ManualConfig<7>& m = cfg.value().manual;
    for (int j = 0; j < 7; ++j) {
      EXPECT_EQ(m.joint_a_max_rad_s2(j), r.joint_a_max_rad_s2(j)) << name << " joint " << j + 1;
    }
    EXPECT_EQ(m.tcp_v_max_m_s, r.tcp_v_max_m_s) << name;
    EXPECT_EQ(m.tcp_w_max_rad_s, r.tcp_w_max_rad_s) << name;
    EXPECT_EQ(m.tcp_a_max_m_s2, r.tcp_a_max_m_s2) << name;
    EXPECT_EQ(m.tcp_alpha_max_rad_s2, r.tcp_alpha_max_rad_s2) << name;
    EXPECT_EQ(m.deadman_s, r.deadman_s) << name;
    EXPECT_EQ(m.jog_margin_rad, r.jog_margin_rad) << name;
    EXPECT_EQ(m.joint_speed_fraction, r.joint_speed_fraction) << name;
    EXPECT_NE(text.find("joint_speed_fraction: 0.9"), std::string::npos) << name;
  }
  // And default.yaml itself has the block (not the struct defaults: a_max differs from 5).
  EXPECT_NE(read_file(kRepo / "config/default.yaml").find("\n  manual:"), std::string::npos);
  EXPECT_DOUBLE_EQ(r.joint_a_max_rad_s2(1), 7.5);
}

// Feature jog-headroom: cobot.manual.joint_speed_fraction.
TEST(ManualConfig, JointSpeedFractionKeyMissingIsZeroPointNine) {
  const auto cfg = load_with_manual("fraction_missing.yaml", "    deadman_ms: 100.0\n");
  ASSERT_TRUE(cfg) << cfg.reason();
  EXPECT_DOUBLE_EQ(cfg.value().manual.joint_speed_fraction, 0.9);
}

// Inside (0, 1], the upper end included: stored as written.
TEST(ManualConfig, JointSpeedFractionInsideTheUnitIntervalIsStored) {
  const std::vector<std::pair<std::string, double>> cases = {
      {"1.0", 1.0}, {"0.5", 0.5}, {"0.9", 0.9}};
  for (const auto& [text, value] : cases) {
    const auto cfg =
        load_with_manual("fraction_ok.yaml", "    joint_speed_fraction: " + text + "\n");
    ASSERT_TRUE(cfg) << text << ": " << cfg.reason();
    EXPECT_DOUBLE_EQ(cfg.value().manual.joint_speed_fraction, value) << text;
  }
}

// Outside (0, 1]: an error naming the key and the value.
TEST(ManualConfig, JointSpeedFractionOutsideTheUnitIntervalIsAnErrorNamingKeyAndValue) {
  for (const std::string value : {"0", "-0.1", "1.5"}) {
    const auto cfg =
        load_with_manual("fraction_bad.yaml", "    joint_speed_fraction: " + value + "\n");
    ASSERT_FALSE(cfg) << value;
    const std::string reason = cfg.reason();
    EXPECT_NE(reason.find("cobot.manual.joint_speed_fraction"), std::string::npos) << reason;
    EXPECT_NE(reason.find(value), std::string::npos) << reason;
  }
}

// The two e2e configs that mirror default.yaml carry the key in their manual block too.
TEST(ManualConfig, TheMirroringE2eConfigsCarryTheJointSpeedFraction) {
  for (const char* name : {"ideal_world.yaml", "e2e_04_safety_limit.yaml"}) {
    const fs::path path = kRepo / "tests/e2e/configs" / name;
    EXPECT_NE(read_file(path).find("joint_speed_fraction: 0.9"), std::string::npos) << name;
    const auto cfg = load_config<7>(path);
    ASSERT_TRUE(cfg) << name << ": " << cfg.reason();
    EXPECT_DOUBLE_EQ(cfg.value().manual.joint_speed_fraction, 0.9) << name;
  }
}

}  // namespace
}  // namespace cobot::pipeline
