#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/control/friction_compensation.hpp"
#include "cobot/pipeline/config.hpp"

// cobot.friction_compensation: {enabled, coulomb_nm, viscous_nm_s_rad, velocity_eps_rad_s},
// optional (missing -> disabled defaults), per-joint values as a scalar or a 7-list,
// coulomb/viscous >= 0, eps > 0, errors name the full key.
namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;
using Vec7 = common::VecN<7>;

const fs::path kRepo = COBOT_REPO_DIR;

std::string read_file(const fs::path& p) {
  const std::ifstream in(p, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

fs::path write_temp(const std::string& name, const std::string& text) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "pipeline_config_friction";
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

// config/default.yaml with an absolute URDF path, so the copy is valid in any directory.
std::string default_text() {
  return replace(read_file(kRepo / "config/default.yaml"),
                 "../sim/urdf/cobot30_5.urdf",
                 (kRepo / "sim/urdf/cobot30_5.urdf").string());
}

// Removes the `  friction_compensation:` block (the key line and every following line indented
// by more than two spaces, or empty) wherever it sits in the cobot map.
std::string without_section(const std::string& text) {
  std::istringstream in(text);
  std::string out;
  std::string line;
  bool inside = false;
  while (std::getline(in, line)) {
    if (line.rfind("  friction_compensation:", 0) == 0) {
      inside = true;
      continue;
    }
    if (inside && (line.rfind("   ", 0) == 0 || line.empty())) {
      continue;
    }
    inside = false;
    out += line + "\n";
  }
  return out;
}

// default.yaml without its own section, plus the given section body appended at the end of the
// cobot map (every body line is indented by four spaces).
std::string with_section(const std::string& body) {
  return without_section(default_text()) + "  friction_compensation:\n" + body;
}

const char* const kValidBody =
    "    enabled: true\n"
    "    coulomb_nm: [1.0, 1.0, 1.0, 1.0, 0.1, 0.1, 0.1]\n"
    "    viscous_nm_s_rad: 0.2\n"
    "    velocity_eps_rad_s: 0.02\n";

TEST(LoadConfigFriction, DefaultFileHasTheNominalIdentifiedValues) {
  const auto r = load_config<7>(kRepo / "config/default.yaml");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const control::FrictionCompensationConfig<7>& f = r.value().friction_compensation;
  EXPECT_TRUE(f.enabled);
  // 80 % of the realistic profile's plant friction: 0.8 * 1.7 = 1.36, 0.8 * 0.24 = 0.192,
  // 0.8 * 0.5 = 0.4, 0.8 * 0.1 = 0.08.
  Vec7 coulomb;
  coulomb << 1.36, 1.36, 1.36, 1.36, 0.192, 0.192, 0.192;
  Vec7 viscous;
  viscous << 0.4, 0.4, 0.4, 0.4, 0.08, 0.08, 0.08;
  for (int i = 0; i < 7; ++i) {
    EXPECT_DOUBLE_EQ(f.coulomb_nm[i], coulomb[i]) << "joint " << i + 1;
    EXPECT_DOUBLE_EQ(f.viscous_nm_s_rad[i], viscous[i]) << "joint " << i + 1;
  }
  EXPECT_DOUBLE_EQ(f.velocity_eps_rad_s, 0.01);
}

TEST(LoadConfigFriction, MissingSectionMeansDisabledDefaults) {
  const std::string text = without_section(default_text());
  ASSERT_EQ(text.find("friction_compensation"), std::string::npos);
  const auto r = load_config<7>(write_temp("missing.yaml", text));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const control::FrictionCompensationConfig<7>& f = r.value().friction_compensation;
  EXPECT_FALSE(f.enabled);
  EXPECT_TRUE(f.coulomb_nm.isZero(0.0));
  EXPECT_TRUE(f.viscous_nm_s_rad.isZero(0.0));
  EXPECT_DOUBLE_EQ(f.velocity_eps_rad_s, 0.01);
  // The rest of the file is unaffected.
  EXPECT_DOUBLE_EQ(r.value().computed_torque.kp[0], 6400.0);
}

TEST(LoadConfigFriction, ScalarBroadcastAndPerJointList) {
  const auto r = load_config<7>(write_temp("valid.yaml", with_section(kValidBody)));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const control::FrictionCompensationConfig<7>& f = r.value().friction_compensation;
  EXPECT_TRUE(f.enabled);
  const double coulomb[] = {1.0, 1.0, 1.0, 1.0, 0.1, 0.1, 0.1};
  for (int i = 0; i < 7; ++i) {
    EXPECT_DOUBLE_EQ(f.coulomb_nm[i], coulomb[i]) << "joint " << i + 1;
    EXPECT_DOUBLE_EQ(f.viscous_nm_s_rad[i], 0.2) << "joint " << i + 1;
  }
  EXPECT_DOUBLE_EQ(f.velocity_eps_rad_s, 0.02);
}

TEST(LoadConfigFriction, DisabledKeepsTheGivenValues) {
  const auto r = load_config<7>(write_temp(
      "disabled.yaml", with_section(replace(kValidBody, "enabled: true", "enabled: false"))));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_FALSE(r.value().friction_compensation.enabled);
  EXPECT_DOUBLE_EQ(r.value().friction_compensation.coulomb_nm[0], 1.0);
}

TEST(LoadConfigFriction, ZeroFrictionIsABoundaryValue) {
  std::string body = replace(kValidBody, "[1.0, 1.0, 1.0, 1.0, 0.1, 0.1, 0.1]", "0.0");
  body = replace(body, "viscous_nm_s_rad: 0.2", "viscous_nm_s_rad: 0.0");
  const auto r = load_config<7>(write_temp("zero.yaml", with_section(body)));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_TRUE(r.value().friction_compensation.coulomb_nm.isZero(0.0));
  EXPECT_TRUE(r.value().friction_compensation.viscous_nm_s_rad.isZero(0.0));
}

// One edit of kValidBody and the key path the error message must name.
struct BadCase {
  const char* from;
  const char* to;
  const char* key;
};

TEST(LoadConfigFriction, EachBadValueNamesItsKey) {
  const std::vector<BadCase> cases = {
      {"[1.0, 1.0, 1.0, 1.0, 0.1, 0.1, 0.1]",
       "[1.0, -1.0, 1.0, 1.0, 0.1, 0.1, 0.1]",
       "cobot.friction_compensation.coulomb_nm"},
      {"[1.0, 1.0, 1.0, 1.0, 0.1, 0.1, 0.1]", "-0.5", "cobot.friction_compensation.coulomb_nm"},
      {"[1.0, 1.0, 1.0, 1.0, 0.1, 0.1, 0.1]",
       "[1.0, 1.0, 1.0, 1.0, 0.1, 0.1]",
       "cobot.friction_compensation.coulomb_nm"},
      {"viscous_nm_s_rad: 0.2",
       "viscous_nm_s_rad: -0.2",
       "cobot.friction_compensation.viscous_nm_s_rad"},
      {"velocity_eps_rad_s: 0.02",
       "velocity_eps_rad_s: 0.0",
       "cobot.friction_compensation.velocity_eps_rad_s"},
      {"velocity_eps_rad_s: 0.02",
       "velocity_eps_rad_s: -0.01",
       "cobot.friction_compensation.velocity_eps_rad_s"},
      {"velocity_eps_rad_s: 0.02",
       "velocity_eps_rad_s: wide",
       "cobot.friction_compensation.velocity_eps_rad_s"},
      {"enabled: true", "enabled: [1]", "cobot.friction_compensation.enabled"},
      {"    enabled: true\n",
       "    enabled: true\n    stiction_nm: 1.0\n",
       "cobot.friction_compensation.stiction_nm"},
  };
  int n = 0;
  for (const auto& bad : cases) {
    const auto r = load_config<7>(write_temp("bad" + std::to_string(n++) + ".yaml",
                                             with_section(replace(kValidBody, bad.from, bad.to))));
    ASSERT_FALSE(r.is_ok()) << bad.key << " (" << bad.to << ")";
    EXPECT_NE(r.reason().find(bad.key), std::string::npos) << r.reason();
  }
}

// tests/e2e/configs/ideal_world.yaml: config/default.yaml with the compensation switched off,
// for the scenarios on the frictionless ideal profile.
TEST(LoadConfigFriction, IdealWorldConfigIsDefaultWithCompensationOff) {
  const auto def = load_config<7>(kRepo / "config/default.yaml");
  const auto ideal = load_config<7>(kRepo / "tests/e2e/configs/ideal_world.yaml");
  ASSERT_TRUE(def.is_ok()) << def.reason();
  ASSERT_TRUE(ideal.is_ok()) << ideal.reason();
  const Config<7>& d = def.value();
  const Config<7>& c = ideal.value();
  EXPECT_FALSE(c.friction_compensation.enabled);
  EXPECT_EQ(c.urdf_path, d.urdf_path);
  EXPECT_EQ(c.log_output_dir, d.log_output_dir);
  EXPECT_EQ(c.home_q_rad, d.home_q_rad);
  EXPECT_EQ(c.trajectory.goal_position_m, d.trajectory.goal_position_m);
  EXPECT_EQ(c.trajectory.goal_rpy_rad, d.trajectory.goal_rpy_rad);
  EXPECT_EQ(c.trajectory.duration_s, d.trajectory.duration_s);
  EXPECT_EQ(c.trajectory.hold_s, d.trajectory.hold_s);
  EXPECT_EQ(c.computed_torque.kp, d.computed_torque.kp);
  EXPECT_EQ(c.computed_torque.kd, d.computed_torque.kd);
  EXPECT_EQ(c.clik.kp, d.clik.kp);
  EXPECT_EQ(c.safety.soft_margin_rad, d.safety.soft_margin_rad);
  EXPECT_EQ(c.safety.hal_timeout_s, d.safety.hal_timeout_s);
}

}  // namespace
}  // namespace cobot::pipeline
