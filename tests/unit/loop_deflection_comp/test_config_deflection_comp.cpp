// cobot.deflection_comp (brief "deflection-comp" contract 3): {enabled, stiffness_k1_nm_rad,
// stiffness_k2_nm_rad, stiffness_k3_nm_rad, torque_t1_nm, torque_t2_nm}, optional (missing ->
// disabled), unknown keys rejected. When enabled every value > 0 and t1 < t2 per joint; errors
// name the full key. config/default.yaml: enabled with the catalog values (SHG-25 joints 1-2,
// SHG-20 joints 3-4, SHG-14 joints 5-7).
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/config.hpp"

#include "deflection_test_util.hpp"

namespace cobot::pipeline::test {
namespace {

namespace fs = std::filesystem;

const fs::path kRepo = COBOT_REPO_DIR;

std::string read_file(const fs::path& p) {
  const std::ifstream in(p, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

fs::path write_temp(const std::string& name, const std::string& text) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "loop_deflection_comp";
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

// Removes the `  deflection_comp:` block (the key line and every following line indented by
// more than two spaces, or empty) wherever it sits in the cobot map.
std::string without_section(const std::string& text) {
  std::istringstream in(text);
  std::string out;
  std::string line;
  bool inside = false;
  while (std::getline(in, line)) {
    if (line.rfind("  deflection_comp:", 0) == 0) {
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

// default.yaml without its own section, plus the given body appended at the end of the cobot
// map (every body line is indented by four spaces).
std::string with_section(const std::string& body) {
  return without_section(default_text()) + "  deflection_comp:\n" + body;
}

const char* const kValidBody =
    "    enabled: true\n"
    "    stiffness_k1_nm_rad: [31000, 31000, 13000, 13000, 4700, 4700, 4701]\n"
    "    stiffness_k2_nm_rad: [50000, 50000, 25000, 25000, 6100, 6100, 6101]\n"
    "    stiffness_k3_nm_rad: [57000, 57000, 29000, 29000, 7100, 7100, 7101]\n"
    "    torque_t1_nm: [14, 14, 7, 7, 2, 2, 2.5]\n"
    "    torque_t2_nm: [48, 48, 25, 25, 6.9, 6.9, 7.5]\n";

Vec column(double Gear::*field) {
  Vec v;
  for (int j = 0; j < kN; ++j) {
    v(j) = gear_of(j).*field;
  }
  return v;
}

TEST(LoadConfigDeflectionComp, DefaultFileEnablesTheCatalogValues) {
  const auto r = load_config<kN>(kRepo / "config/default.yaml");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const auto& d = r.value().deflection_comp;
  EXPECT_TRUE(d.enabled);
  EXPECT_EQ(d.stiffness_k1_nm_rad, column(&Gear::k1));
  EXPECT_EQ(d.stiffness_k2_nm_rad, column(&Gear::k2));
  EXPECT_EQ(d.stiffness_k3_nm_rad, column(&Gear::k3));
  EXPECT_EQ(d.torque_t1_nm, column(&Gear::t1));
  EXPECT_EQ(d.torque_t2_nm, column(&Gear::t2));
}

TEST(LoadConfigDeflectionComp, MissingSectionMeansDisabled) {
  const std::string text = without_section(default_text());
  ASSERT_EQ(text.find("deflection_comp"), std::string::npos);
  const auto r = load_config<kN>(write_temp("missing.yaml", text));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_FALSE(r.value().deflection_comp.enabled);
  // The rest of the file is unaffected.
  EXPECT_DOUBLE_EQ(r.value().computed_torque.kp[0], 6400.0);
}

TEST(LoadConfigDeflectionComp, StructDefaultIsDisabled) {
  const Config<kN> cfg;
  EXPECT_FALSE(cfg.deflection_comp.enabled);
}

TEST(LoadConfigDeflectionComp, ParsesEveryKeyPerJoint) {
  const auto r = load_config<kN>(write_temp("valid.yaml", with_section(kValidBody)));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const auto& d = r.value().deflection_comp;
  EXPECT_TRUE(d.enabled);
  EXPECT_EQ(d.stiffness_k1_nm_rad,
            (Vec() << 31000, 31000, 13000, 13000, 4700, 4700, 4701).finished());
  EXPECT_EQ(d.stiffness_k2_nm_rad,
            (Vec() << 50000, 50000, 25000, 25000, 6100, 6100, 6101).finished());
  EXPECT_EQ(d.stiffness_k3_nm_rad,
            (Vec() << 57000, 57000, 29000, 29000, 7100, 7100, 7101).finished());
  EXPECT_EQ(d.torque_t1_nm, (Vec() << 14, 14, 7, 7, 2, 2, 2.5).finished());
  EXPECT_EQ(d.torque_t2_nm, (Vec() << 48, 48, 25, 25, 6.9, 6.9, 7.5).finished());
}

TEST(LoadConfigDeflectionComp, DisabledSectionKeepsTheValues) {
  const auto r = load_config<kN>(write_temp(
      "disabled.yaml", with_section(replace(kValidBody, "enabled: true", "enabled: false"))));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_FALSE(r.value().deflection_comp.enabled);
  EXPECT_EQ(r.value().deflection_comp.torque_t2_nm(6), 7.5);
}

// One edit of kValidBody and the key path the error message must name.
struct BadCase {
  const char* from;
  const char* to;
  const char* key;
};

TEST(LoadConfigDeflectionComp, EachBadValueNamesItsKey) {
  const std::string k = "cobot.deflection_comp.";
  const std::vector<BadCase> cases = {
      {"[31000, 31000, 13000", "[0, 31000, 13000", "stiffness_k1_nm_rad"},
      {"[31000, 31000, 13000", "[31000, -1, 13000", "stiffness_k1_nm_rad"},
      {"[50000, 50000, 25000", "[50000, 50000, 0", "stiffness_k2_nm_rad"},
      {"[50000, 50000, 25000", "[-50000, 50000, 25000", "stiffness_k2_nm_rad"},
      {"[57000, 57000, 29000", "[57000, 57000, 29000, 0", "stiffness_k3_nm_rad"},  // 8 values
      {"7100, 7100, 7101]", "7100, 7100, 0]", "stiffness_k3_nm_rad"},
      {"7100, 7100, 7101]", "7100, 7100, -7101]", "stiffness_k3_nm_rad"},
      {"[14, 14, 7, 7, 2, 2, 2.5]", "[14, 14, 7, 7, 2, 0, 2.5]", "torque_t1_nm"},
      {"[14, 14, 7, 7, 2, 2, 2.5]", "[-14, 14, 7, 7, 2, 2, 2.5]", "torque_t1_nm"},
      {"[48, 48, 25, 25, 6.9, 6.9, 7.5]", "[48, 48, 25, 25, 6.9, 6.9, 0]", "torque_t"},
      // t1 >= t2 on one joint: equal (joints 7 and 2), then greater (joint 4).
      {"[48, 48, 25, 25, 6.9, 6.9, 7.5]", "[48, 48, 25, 25, 6.9, 6.9, 2.5]", "torque_t"},
      {"[48, 48, 25, 25, 6.9, 6.9, 7.5]", "[48, 14, 25, 25, 6.9, 6.9, 7.5]", "torque_t"},
      {"[14, 14, 7, 7, 2, 2, 2.5]", "[14, 14, 7, 30, 2, 2, 2.5]", "torque_t"},
      {"[14, 14, 7, 7, 2, 2, 2.5]", "[14, 14, 7, 7, 2, 2]", "torque_t1_nm"},  // 6 values
      {"stiffness_k1_nm_rad: [31000, 31000, 13000, 13000, 4700, 4700, 4701]",
       "stiffness_k1_nm_rad: stiff",
       "stiffness_k1_nm_rad"},
      {"enabled: true", "enabled: [1]", "enabled"},
      {"    enabled: true\n", "    enabled: true\n    stiffness_k4_nm_rad: 1.0\n", "stiffness_k4"},
  };
  int n = 0;
  for (const auto& bad : cases) {
    const auto r = load_config<kN>(write_temp("bad" + std::to_string(n++) + ".yaml",
                                              with_section(replace(kValidBody, bad.from, bad.to))));
    ASSERT_FALSE(r.is_ok()) << bad.key << " (" << bad.to << ")";
    EXPECT_NE(r.reason().find(k + bad.key), std::string::npos) << bad.to << ": " << r.reason();
  }
}

}  // namespace
}  // namespace cobot::pipeline::test
