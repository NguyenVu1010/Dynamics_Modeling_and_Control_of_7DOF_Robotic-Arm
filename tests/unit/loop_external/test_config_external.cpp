#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/path_validation.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"

// New cobot.* keys of feature `loop-external` (design 5, 6.3): command_source,
// cartesian_limits, stop_time_s, tcp_offset, path_validation. Every one is optional with the
// default of its struct; errors name the full key.
namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;
using common::Mat3;
using common::Vec3;

const fs::path kRepo = COBOT_REPO_DIR;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

std::string read_file(const fs::path& p) {
  const std::ifstream in(p, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

fs::path write_temp(const std::string& name, const std::string& text) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "loop_external_config";
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
                 "../src/sim/urdf/cobot30_5.urdf",
                 (kRepo / "src/sim/urdf/cobot30_5.urdf").string());
}

// Removes each `  <key>:` entry of the cobot map (its line and every following line indented by
// more than two spaces, or empty), wherever it sits.
std::string without_keys(const std::string& text) {
  const std::initializer_list<const char*> keys = {"  command_source:",
                                                   "  cartesian_limits:",
                                                   "  stop_time_s:",
                                                   "  tcp_offset:",
                                                   "  path_validation:"};
  std::istringstream in(text);
  std::string out;
  std::string line;
  bool inside = false;
  while (std::getline(in, line)) {
    bool starts = false;
    for (const char* k : keys) {
      starts = starts || line.rfind(k, 0) == 0;
    }
    if (starts) {
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

// Every new key with a non-default value, appended at the end of the cobot map.
const char* const kValidBody =
    "  command_source: external\n"
    "  cartesian_limits:\n"
    "    v_max_m_s: 0.3\n"
    "    a_max_m_s2: 0.6\n"
    "    w_max_rad_s: 0.9\n"
    "    alpha_max_rad_s2: 1.2\n"
    "  stop_time_s: 0.8\n"
    "  tcp_offset:\n"
    "    position_m: [0.0, 0.0, 0.1]\n"
    "    rpy_deg: [0.0, 0.0, 90.0]\n"
    "  path_validation:\n"
    "    reach_pos_m: 0.002\n"
    "    reach_rot_deg: 1.0\n"
    "    joint_margin_deg: 3.0\n"
    "    torque_ratio: 0.8\n"
    "    max_stretch: 2.0\n";

std::string with_body(const std::string& body) {
  return without_keys(default_text()) + body;
}

void expect_defaults(const Config<7>& c) {
  EXPECT_EQ(c.command_source, CommandSource::kConfig);
  EXPECT_DOUBLE_EQ(c.cartesian_limits.v_max_m_s, 0.25);
  EXPECT_DOUBLE_EQ(c.cartesian_limits.a_max_m_s2, 0.5);
  EXPECT_DOUBLE_EQ(c.cartesian_limits.w_max_rad_s, 0.8);
  EXPECT_DOUBLE_EQ(c.cartesian_limits.alpha_max_rad_s2, 1.6);
  EXPECT_DOUBLE_EQ(c.stop_time_s, 0.5);
  EXPECT_TRUE(c.tcp_offset.T_tip_tcp.R.isIdentity(0.0));
  EXPECT_TRUE(c.tcp_offset.T_tip_tcp.p.isZero(0.0));
  const ValidationLimits defaults;
  EXPECT_DOUBLE_EQ(c.path_validation.reach_pos_m, 1e-3);
  EXPECT_NEAR(c.path_validation.reach_rot_rad, 0.5 * kDeg, 1e-15);     // 0.008726646259971648 rad
  EXPECT_NEAR(c.path_validation.joint_margin_rad, 2.0 * kDeg, 1e-15);  // 0.03490658503988659 rad
  EXPECT_DOUBLE_EQ(c.path_validation.torque_ratio, 0.9);
  EXPECT_DOUBLE_EQ(c.path_validation.max_stretch, 3.0);
  EXPECT_DOUBLE_EQ(c.path_validation.dt_s, defaults.dt_s);
  EXPECT_DOUBLE_EQ(c.path_validation.path_sample_dt_s, defaults.path_sample_dt_s);
}

TEST(LoadConfigExternal, DefaultFileHasTheDefaultsAndConfigMode) {
  const std::string text = read_file(kRepo / "config/default.yaml");
  // The sections are written out in the default file, command_source stays config.
  EXPECT_NE(text.find("\n  command_source: config"), std::string::npos);
  EXPECT_NE(text.find("\n  cartesian_limits:"), std::string::npos);
  EXPECT_NE(text.find("\n  stop_time_s:"), std::string::npos);
  EXPECT_NE(text.find("\n  tcp_offset:"), std::string::npos);
  EXPECT_NE(text.find("\n  path_validation:"), std::string::npos);
  const auto r = load_config<7>(kRepo / "config/default.yaml");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  expect_defaults(r.value());
}

TEST(LoadConfigExternal, MissingSectionsMeanDefaults) {
  const std::string text = without_keys(default_text());
  for (const char* key :
       {"command_source", "cartesian_limits", "stop_time_s", "tcp_offset", "path_validation"}) {
    ASSERT_EQ(text.find(std::string("  ") + key + ":"), std::string::npos) << key;
  }
  const auto r = load_config<7>(write_temp("missing.yaml", text));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  expect_defaults(r.value());
  // The rest of the file is unaffected.
  EXPECT_DOUBLE_EQ(r.value().computed_torque.kp[0], 6400.0);
  EXPECT_DOUBLE_EQ(r.value().trajectory.duration_s, 10.0);
}

TEST(LoadConfigExternal, EveryKeyIsRead) {
  const auto r = load_config<7>(write_temp("valid.yaml", with_body(kValidBody)));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const Config<7>& c = r.value();
  EXPECT_EQ(c.command_source, CommandSource::kExternal);
  EXPECT_DOUBLE_EQ(c.cartesian_limits.v_max_m_s, 0.3);
  EXPECT_DOUBLE_EQ(c.cartesian_limits.a_max_m_s2, 0.6);
  EXPECT_DOUBLE_EQ(c.cartesian_limits.w_max_rad_s, 0.9);
  EXPECT_DOUBLE_EQ(c.cartesian_limits.alpha_max_rad_s2, 1.2);
  EXPECT_DOUBLE_EQ(c.stop_time_s, 0.8);
  // rpy (0, 0, 90 deg): R = Rz(90 deg) = [0 -1 0; 1 0 0; 0 0 1].
  Mat3 rz90;
  rz90 << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  EXPECT_LT((c.tcp_offset.T_tip_tcp.R - rz90).cwiseAbs().maxCoeff(), 1e-15);
  EXPECT_LT((c.tcp_offset.T_tip_tcp.p - Vec3(0.0, 0.0, 0.1)).cwiseAbs().maxCoeff(), 1e-15);
  EXPECT_DOUBLE_EQ(c.path_validation.reach_pos_m, 0.002);
  EXPECT_NEAR(c.path_validation.reach_rot_rad, 1.0 * kDeg, 1e-15);     // 0.017453292519943295
  EXPECT_NEAR(c.path_validation.joint_margin_rad, 3.0 * kDeg, 1e-15);  // 0.05235987755982988
  EXPECT_DOUBLE_EQ(c.path_validation.torque_ratio, 0.8);
  EXPECT_DOUBLE_EQ(c.path_validation.max_stretch, 2.0);
}

TEST(LoadConfigExternal, ConfigValueOfCommandSource) {
  const auto r = load_config<7>(write_temp(
      "config_mode.yaml",
      with_body(replace(kValidBody, "command_source: external", "command_source: config"))));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().command_source, CommandSource::kConfig);
}

TEST(LoadConfigExternal, CommandSourceMessage) {
  const auto r = load_config<7>(write_temp(
      "manual.yaml",
      with_body(replace(kValidBody, "command_source: external", "command_source: manual"))));
  ASSERT_FALSE(r.is_ok());
  EXPECT_NE(r.reason().find("cobot.command_source: expected config or external"), std::string::npos)
      << r.reason();
}

// One edit of kValidBody and the key path the error message must name.
struct BadCase {
  const char* from;
  const char* to;
  const char* key;
};

TEST(LoadConfigExternal, EachBadValueNamesItsKey) {
  const std::vector<BadCase> cases = {
      {"command_source: external", "command_source: manual", "cobot.command_source"},
      {"command_source: external", "command_source: [external]", "cobot.command_source"},
      {"stop_time_s: 0.8", "stop_time_s: 0.0", "cobot.stop_time_s"},
      {"stop_time_s: 0.8", "stop_time_s: -1.0", "cobot.stop_time_s"},
      {"stop_time_s: 0.8", "stop_time_s: soon", "cobot.stop_time_s"},
      {"v_max_m_s: 0.3", "v_max_m_s: 0.0", "cobot.cartesian_limits.v_max_m_s"},
      {"a_max_m_s2: 0.6", "a_max_m_s2: -0.6", "cobot.cartesian_limits.a_max_m_s2"},
      {"w_max_rad_s: 0.9", "w_max_rad_s: fast", "cobot.cartesian_limits.w_max_rad_s"},
      {"alpha_max_rad_s2: 1.2", "alpha_max_rad_s2: 0.0", "cobot.cartesian_limits.alpha_max_rad_s2"},
      {"alpha_max_rad_s2: 1.2\n",
       "alpha_max_rad_s2: 1.2\n    j_max_m_s3: 5.0\n",
       "cobot.cartesian_limits.j_max_m_s3"},
      {"position_m: [0.0, 0.0, 0.1]", "position_m: [0.0, 0.1]", "cobot.tcp_offset.position_m"},
      {"position_m: [0.0, 0.0, 0.1]",
       "position_m: [0.0, 0.0, .nan]",
       "cobot.tcp_offset.position_m"},
      {"    rpy_deg: [0.0, 0.0, 90.0]", "    rpy_deg: [0.0, 90.0]", "cobot.tcp_offset.rpy_deg"},
      {"    rpy_deg: [0.0, 0.0, 90.0]\n",
       "    rpy_deg: [0.0, 0.0, 90.0]\n    mass_kg: 1.0\n",
       "cobot.tcp_offset.mass_kg"},
      {"reach_pos_m: 0.002", "reach_pos_m: 0.0", "cobot.path_validation.reach_pos_m"},
      {"reach_rot_deg: 1.0", "reach_rot_deg: -1.0", "cobot.path_validation.reach_rot_deg"},
      {"joint_margin_deg: 3.0", "joint_margin_deg: 0.0", "cobot.path_validation.joint_margin_deg"},
      {"torque_ratio: 0.8", "torque_ratio: 0.0", "cobot.path_validation.torque_ratio"},
      {"max_stretch: 2.0", "max_stretch: 0.0", "cobot.path_validation.max_stretch"},
      {"max_stretch: 2.0", "max_stretch: long", "cobot.path_validation.max_stretch"},
      {"max_stretch: 2.0\n", "max_stretch: 2.0\n    dt_s: 0.001\n", "cobot.path_validation.dt_s"},
  };
  int n = 0;
  for (const auto& bad : cases) {
    const auto r = load_config<7>(write_temp("bad" + std::to_string(n++) + ".yaml",
                                             with_body(replace(kValidBody, bad.from, bad.to))));
    ASSERT_FALSE(r.is_ok()) << bad.key << " (" << bad.to << ")";
    EXPECT_NE(r.reason().find(bad.key), std::string::npos) << r.reason();
  }
}

}  // namespace
}  // namespace cobot::pipeline
