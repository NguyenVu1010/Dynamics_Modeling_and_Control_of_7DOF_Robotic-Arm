// cobot.control.algorithm and cobot.pd_gravity in load_config (brief controller-registry 4,
// ADR-023, ADR-009 key checks). `control` is optional with the single optional key `algorithm`
// (computed_torque when missing); a name outside control::kAlgorithms is an error listing the
// registered names. `pd_gravity: {kp, kd}` is optional, both keys required in it, > 0, N values or
// a scalar broadcast; it is required when pd_gravity is selected and parsed and checked (unused)
// otherwise. The override path creates both sections when absent.
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"
#include "cobot/control/pd_gravity.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/overrides.hpp"

namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;
using control::Algorithm;

constexpr int kN = 7;
const fs::path kRepo = COBOT_REPO_DIR;

std::string read_text(const fs::path& p) {
  std::ifstream in(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

fs::path write_temp(const std::string& name, const std::string& text) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "control_registry_config";
  fs::create_directories(dir);
  const fs::path p = dir / name;
  std::ofstream(p, std::ios::binary) << text;
  return p;
}

// Removes every `  <name>:` block of the cobot map (the key line and each following line indented
// by more than two spaces, or empty); a file without the block is returned unchanged.
std::string without_section(const std::string& text, const std::string& name) {
  std::istringstream in(text);
  std::string out;
  std::string line;
  bool inside = false;
  while (std::getline(in, line)) {
    if (line.rfind("  " + name + ":", 0) == 0) {
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

// config/default.yaml with an absolute URDF path and without `control` and `pd_gravity`, plus
// `extra` (lines indented by two spaces) at the end of the cobot map.
std::string default_with(const std::string& extra) {
  std::string text = read_text(kRepo / "config/default.yaml");
  const std::string rel = "../src/sim/urdf/cobot30_5.urdf";
  const auto pos = text.find(rel);
  EXPECT_NE(pos, std::string::npos);
  if (pos != std::string::npos) {
    text.replace(pos, rel.size(), (kRepo / "src/sim/urdf/cobot30_5.urdf").string());
  }
  return without_section(without_section(text, "control"), "pd_gravity") + extra;
}

Result<Config<kN>> load_text(const std::string& name, const std::string& extra) {
  return load_config<kN>(write_temp(name, default_with(extra)));
}

void expect_error(const std::string& name, const std::string& extra, const std::string& part) {
  const auto r = load_text(name, extra);
  ASSERT_FALSE(r.is_ok()) << name << " loaded";
  EXPECT_TRUE(contains(r.reason(), part)) << name << ": " << r.reason();
}

TEST(ConfigAlgorithm, ControllerConfigsSelectComputedTorque) {
  for (const char* file : {"default", "premium_fr3", "mid_kr1205", "budget_xarm7"}) {
    const auto r = load_config<kN>(kRepo / "config" / (std::string(file) + ".yaml"));
    ASSERT_TRUE(r.is_ok()) << file << ": " << r.reason();
    EXPECT_EQ(r.value().algorithm, Algorithm::kComputedTorque) << file;
  }
}

TEST(ConfigAlgorithm, MissingSectionOrKeyMeansComputedTorque) {
  const std::string bare = default_with("");
  ASSERT_FALSE(contains(bare, "  control:"));
  ASSERT_FALSE(contains(bare, "  pd_gravity:"));
  const auto missing = load_text("missing.yaml", "");
  ASSERT_TRUE(missing.is_ok()) << missing.reason();
  EXPECT_EQ(missing.value().algorithm, Algorithm::kComputedTorque);
  const auto empty = load_text("empty.yaml", "  control: {}\n");
  ASSERT_TRUE(empty.is_ok()) << empty.reason();
  EXPECT_EQ(empty.value().algorithm, Algorithm::kComputedTorque);
}

TEST(ConfigAlgorithm, PdGravitySelectedWithScalarKdBroadcast) {
  const auto r = load_text("pd.yaml",
                           "  control:\n    algorithm: pd_gravity\n  pd_gravity:\n"
                           "    kp: [100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 700.0]\n"
                           "    kd: 40\n");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().algorithm, Algorithm::kPdGravity);
  for (int j = 0; j < kN; ++j) {
    EXPECT_EQ(r.value().pd_gravity.kp(j), 100.0 * (j + 1)) << "joint " << j;
    EXPECT_EQ(r.value().pd_gravity.kd(j), 40.0) << "joint " << j;
  }
  EXPECT_EQ(r.value().computed_torque.kp(0), 6400.0);  // the rest of the file unaffected
}

TEST(ConfigAlgorithm, ErrorsNameTheFullKey) {
  expect_error("unknown.yaml",
               "  control:\n    algorithm: foo\n",
               "cobot.control.algorithm: unknown algorithm 'foo' (registered: computed_torque, "
               "pd_gravity, drive_position)");
  expect_error("number.yaml", "  control:\n    algorithm: 3\n", "cobot.control.algorithm");
  expect_error("case.yaml", "  control:\n    algorithm: PD_GRAVITY\n", "registered: ");
  expect_error("key.yaml", "  control:\n    algo: x\n", "cobot.control.algo: unknown key");
  expect_error("required.yaml",
               "  control:\n    algorithm: pd_gravity\n",
               "cobot.pd_gravity: required when cobot.control.algorithm is pd_gravity");
  const std::string pd = "  control:\n    algorithm: pd_gravity\n  pd_gravity:\n";
  // The broadcast reader's wording ("expected a scalar or 7 values, got 6", as coulomb_nm) or the
  // plain one: both carry the key and the two lengths.
  const std::string six = pd + "    kp: [1, 1, 1, 1, 1, 1]\n    kd: 40\n";
  expect_error("six.yaml", six, "cobot.pd_gravity.kp: expected ");
  expect_error("six.yaml", six, "7 values, got 6");
  expect_error("kd_zero.yaml", pd + "    kp: 400\n    kd: 0\n", "cobot.pd_gravity.kd");
  expect_error("kp_negative.yaml", pd + "    kp: -1\n    kd: 40\n", "cobot.pd_gravity.kp");
  expect_error("kd_missing.yaml", pd + "    kp: 1\n", "cobot.pd_gravity.kd");
  expect_error("extra_key.yaml", pd + "    kp: 1\n    kd: 1\n    ki: 1\n", "cobot.pd_gravity.ki");
  // Checked even when another algorithm is selected.
  expect_error("unused_bad.yaml", "  pd_gravity:\n    kp: 400\n    kd: 0\n", "cobot.pd_gravity.kd");
}

TEST(ConfigAlgorithm, PdGravityPresentWithComputedTorqueIsParsedAndUnused) {
  const auto r = load_text("unused.yaml",
                           "  control:\n    algorithm: computed_torque\n"
                           "  pd_gravity:\n    kp: 900\n    kd: 60\n");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().algorithm, Algorithm::kComputedTorque);
  for (int j = 0; j < kN; ++j) {
    EXPECT_EQ(r.value().pd_gravity.kp(j), 900.0) << "joint " << j;
    EXPECT_EQ(r.value().pd_gravity.kd(j), 60.0) << "joint " << j;
  }
}

TEST(ConfigAlgorithm, OverrideCreatesTheAbsentSection) {
  const std::vector<KeyOverride> overrides{
      {"config.control.algorithm", std::string("pd_gravity")},
      {"config.pd_gravity.kp", 400.0},
      {"config.pd_gravity.kd", 40.0},
  };
  const auto r = load_config_with<kN>(kRepo / "config/default.yaml", overrides);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().algorithm, Algorithm::kPdGravity);
  for (int j = 0; j < kN; ++j) {
    EXPECT_EQ(r.value().pd_gravity.kp(j), 400.0) << "joint " << j;
    EXPECT_EQ(r.value().pd_gravity.kd(j), 40.0) << "joint " << j;
  }
  // Same through a temp file that has no `control` section either.
  const fs::path bare = write_temp("bare.yaml", default_with(""));
  const auto b = load_config_with<kN>(bare, overrides);
  ASSERT_TRUE(b.is_ok()) << b.reason();
  EXPECT_EQ(b.value().algorithm, Algorithm::kPdGravity);
  // An unknown name through the override is the loader's error.
  const auto bad = load_config_with<kN>(kRepo / "config/default.yaml",
                                        {{"config.control.algorithm", std::string("pid")}});
  ASSERT_FALSE(bad.is_ok());
  EXPECT_TRUE(contains(bad.reason(), "cobot.control.algorithm: unknown algorithm 'pid'"))
      << bad.reason();
}

}  // namespace
}  // namespace cobot::pipeline
