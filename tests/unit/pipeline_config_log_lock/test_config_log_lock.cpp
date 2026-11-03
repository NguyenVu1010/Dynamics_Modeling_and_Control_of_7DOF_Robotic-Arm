// cobot.log.lock_memory in load_config (feature blackbox-log, cobot_studio design 6.1, ADR-020):
//   cobot:
//     log:
//       lock_memory: false   # true on the Pi 5 (mlock the black-box region)
// Optional, default false (Config::log_lock_memory). A value that is not a boolean is an error
// naming the key. The four controller configs write the key as false with the comment
// "true on the Pi 5".
#include <array>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "cobot/pipeline/config.hpp"

namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kUrdf = kRepo / "src/sim/urdf/cobot30_5.urdf";
const std::array<const char*, 4> kConfigs = {
    "default.yaml", "premium_fr3.yaml", "mid_kr1205.yaml", "budget_xarm7.yaml"};

std::string read_file(const fs::path& p) {
  const std::ifstream in(p, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

fs::path write_temp(const std::string& name, const std::string& text) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "pipeline_config_log_lock";
  fs::create_directories(dir);
  const fs::path p = dir / name;
  std::ofstream(p, std::ios::binary) << text;
  return p;
}

bool starts_with(const std::string& line, const std::string& prefix) {
  return line.rfind(prefix, 0) == 0;
}

// The `  log:` block of a cobot config: its key line and every following line indented by four
// spaces or more, or empty.
std::string log_block(const std::string& text) {
  std::istringstream in(text);
  std::string out;
  std::string line;
  bool inside = false;
  while (std::getline(in, line)) {
    if (starts_with(line, "  log:")) {
      inside = true;
      out += line + "\n";
      continue;
    }
    if (inside && (starts_with(line, "    ") || line.empty())) {
      out += line + "\n";
      continue;
    }
    inside = false;
  }
  return out;
}

// config/default.yaml with an absolute URDF path and its log section replaced by `  log:` and
// `body` (lines indented by four spaces).
std::string default_with_log(const std::string& body) {
  std::string text = read_file(kRepo / "config/default.yaml");
  const std::string rel = "../src/sim/urdf/cobot30_5.urdf";
  const auto pos = text.find(rel);
  EXPECT_NE(pos, std::string::npos) << "fixture text not found: " << rel;
  if (pos != std::string::npos) {
    text.replace(pos, rel.size(), kUrdf.string());
  }
  const std::string block = log_block(text);
  const auto at = text.find(block);
  EXPECT_FALSE(block.empty()) << "no log section in config/default.yaml";
  if (!block.empty() && at != std::string::npos) {
    text.replace(at, block.size(), "  log:\n" + body);
  }
  return text;
}

common::Result<Config<7>> load_with_log(const std::string& name, const std::string& body) {
  return load_config<7>(write_temp(name, default_with_log(body)));
}

TEST(ConfigLogLock, ShippedConfigsWriteFalseWithThePiComment) {
  for (const char* name : kConfigs) {
    const auto r = load_config<7>(kRepo / "config" / name);
    ASSERT_TRUE(r.is_ok()) << name << ": " << r.reason();
    EXPECT_FALSE(r.value().log_lock_memory) << name;
    const YAML::Node log = YAML::LoadFile((kRepo / "config" / name).string())["cobot"]["log"];
    ASSERT_TRUE(log["lock_memory"].IsDefined()) << name << ": key written out";
    EXPECT_FALSE(log["lock_memory"].as<bool>()) << name;
    // The key's line carries the comment.
    std::istringstream block(log_block(read_file(kRepo / "config" / name)));
    std::string line;
    bool found = false;
    while (std::getline(block, line)) {
      if (line.find("lock_memory:") != std::string::npos) {
        found = true;
        EXPECT_NE(line.find("true on the Pi 5"), std::string::npos) << name << ": " << line;
      }
    }
    EXPECT_TRUE(found) << name;
  }
}

TEST(ConfigLogLock, DefaultIsFalse) {
  const auto r = load_with_log("no_lock.yaml", "    output_dir: ../results\n");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_FALSE(r.value().log_lock_memory);
  const Config<7> defaults;
  EXPECT_FALSE(defaults.log_lock_memory);
}

TEST(ConfigLogLock, TrueAndFalseParse) {
  const auto on =
      load_with_log("lock_true.yaml", "    output_dir: ../results\n    lock_memory: true\n");
  ASSERT_TRUE(on.is_ok()) << on.reason();
  EXPECT_TRUE(on.value().log_lock_memory);
  const auto off =
      load_with_log("lock_false.yaml", "    output_dir: ../results\n    lock_memory: false\n");
  ASSERT_TRUE(off.is_ok()) << off.reason();
  EXPECT_FALSE(off.value().log_lock_memory);
  // The other log keys are unaffected.
  EXPECT_EQ(on.value().log_mode, LogMode::kRecorder);
  EXPECT_DOUBLE_EQ(on.value().log_capacity_s, 60.0);
}

TEST(ConfigLogLock, NonBooleanNamesTheKey) {
  const std::array<const char*, 3> bodies = {
      "    lock_memory: maybe\n", "    lock_memory: [true]\n", "    lock_memory: 2\n"};
  int i = 0;
  for (const char* body : bodies) {
    const auto r = load_with_log("bad_lock_" + std::to_string(i++) + ".yaml",
                                 std::string("    output_dir: ../results\n") + body);
    ASSERT_FALSE(r.is_ok()) << body;
    EXPECT_NE(r.reason().find("cobot.log.lock_memory"), std::string::npos)
        << body << ": " << r.reason();
  }
}

}  // namespace
}  // namespace cobot::pipeline
