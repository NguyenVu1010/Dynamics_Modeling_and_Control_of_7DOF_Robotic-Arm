// cobot.log.mode and cobot.log.capacity_s in load_config (feature pipeline-recorder, cobot_studio
// design 5.1 and 6.1, ADR-020):
//   cobot:
//     log:
//       output_dir: ../results
//       mode: recorder        # off | recorder | recorder_live (recorder_live behaves as recorder
//                             # until the shared-memory bridge exists)
//       capacity_s: 60.0      # > 0, s of records the ROS node keeps per buffer
// Both keys are optional (defaults kRecorder and 60.0). A wrong value is an error naming the key.
// config/default.yaml and the three realism-2 controller configs carry both keys with the same
// text in their log sections.
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
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "pipeline_config_log";
  fs::create_directories(dir);
  const fs::path p = dir / name;
  std::ofstream(p, std::ios::binary) << text;
  return p;
}

bool starts_with(const std::string& line, const std::string& prefix) {
  return line.rfind(prefix, 0) == 0;
}

// The `  log:` block of a cobot config: its key line and every following line indented by four
// spaces or more (keys and comments), or empty.
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
// `body` (lines indented by four spaces), in the same place.
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

TEST(ConfigLog, ShippedConfigsRecordWithSixtySeconds) {
  for (const char* name : kConfigs) {
    const auto r = load_config<7>(kRepo / "config" / name);
    ASSERT_TRUE(r.is_ok()) << name << ": " << r.reason();
    EXPECT_EQ(r.value().log_mode, LogMode::kRecorder) << name;
    EXPECT_DOUBLE_EQ(r.value().log_capacity_s, 60.0) << name;
  }
}

// The keys are written out (not left to the defaults), and the log section is the same text in
// the four files.
TEST(ConfigLog, ShippedConfigsWriteBothKeysInTheSameLogSection) {
  const std::string reference = log_block(read_file(kRepo / "config/default.yaml"));
  ASSERT_FALSE(reference.empty());
  for (const char* name : kConfigs) {
    const YAML::Node root = YAML::LoadFile((kRepo / "config" / name).string());
    const YAML::Node log = root["cobot"]["log"];
    ASSERT_TRUE(log.IsMap()) << name;
    ASSERT_TRUE(log["mode"].IsDefined()) << name;
    EXPECT_EQ(log["mode"].as<std::string>(), "recorder") << name;
    ASSERT_TRUE(log["capacity_s"].IsDefined()) << name;
    EXPECT_DOUBLE_EQ(log["capacity_s"].as<double>(), 60.0) << name;
    EXPECT_EQ(log_block(read_file(kRepo / "config" / name)), reference) << name;
  }
}

TEST(ConfigLog, EachModeParses) {
  struct Case {
    const char* text;
    LogMode mode;
  };
  const std::array<Case, 3> cases = {Case{"off", LogMode::kOff},
                                     Case{"recorder", LogMode::kRecorder},
                                     Case{"recorder_live", LogMode::kRecorderLive}};
  for (const Case& c : cases) {
    const auto r = load_with_log(
        std::string("mode_") + c.text + ".yaml",
        "    output_dir: ../results\n    mode: " + std::string(c.text) + "\n    capacity_s: 2.5\n");
    ASSERT_TRUE(r.is_ok()) << c.text << ": " << r.reason();
    EXPECT_EQ(r.value().log_mode, c.mode) << c.text;
    EXPECT_DOUBLE_EQ(r.value().log_capacity_s, 2.5) << c.text;
  }
}

TEST(ConfigLog, SectionWithoutTheKeysKeepsTheDefaults) {
  const auto r = load_with_log("no_keys.yaml", "    output_dir: ../results\n");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().log_mode, LogMode::kRecorder);
  EXPECT_DOUBLE_EQ(r.value().log_capacity_s, 60.0);
  // The defaults of the struct are the same values.
  const Config<7> defaults;
  EXPECT_EQ(defaults.log_mode, LogMode::kRecorder);
  EXPECT_DOUBLE_EQ(defaults.log_capacity_s, 60.0);
}

TEST(ConfigLog, OnlyOneKeyKeepsTheOtherDefault) {
  const auto mode_only =
      load_with_log("mode_only.yaml", "    output_dir: ../results\n    mode: off\n");
  ASSERT_TRUE(mode_only.is_ok()) << mode_only.reason();
  EXPECT_EQ(mode_only.value().log_mode, LogMode::kOff);
  EXPECT_DOUBLE_EQ(mode_only.value().log_capacity_s, 60.0);
  const auto capacity_only =
      load_with_log("capacity_only.yaml", "    output_dir: ../results\n    capacity_s: 0.5\n");
  ASSERT_TRUE(capacity_only.is_ok()) << capacity_only.reason();
  EXPECT_EQ(capacity_only.value().log_mode, LogMode::kRecorder);
  EXPECT_DOUBLE_EQ(capacity_only.value().log_capacity_s, 0.5);
}

TEST(ConfigLog, EachBadValueNamesItsKey) {
  struct Case {
    const char* name;
    const char* body;
    const char* key;
  };
  const std::array<Case, 6> cases = {
      Case{"bad_mode.yaml", "    mode: fast\n", "cobot.log.mode"},
      Case{"list_mode.yaml", "    mode: [recorder]\n", "cobot.log.mode"},
      Case{"zero_capacity.yaml", "    capacity_s: 0\n", "cobot.log.capacity_s"},
      Case{"negative_capacity.yaml", "    capacity_s: -1\n", "cobot.log.capacity_s"},
      Case{"text_capacity.yaml", "    capacity_s: long\n", "cobot.log.capacity_s"},
      Case{"unknown_key.yaml", "    foo: 1\n", "cobot.log.foo"},
  };
  for (const Case& c : cases) {
    const auto r = load_with_log(c.name, std::string("    output_dir: ../results\n") + c.body);
    ASSERT_FALSE(r.is_ok()) << c.name;
    EXPECT_NE(r.reason().find(c.key), std::string::npos) << c.name << ": " << r.reason();
  }
}

TEST(ConfigLog, OutputDirIsStillRead) {
  const fs::path p =
      write_temp("output_dir.yaml", default_with_log("    output_dir: elsewhere\n    mode: off\n"));
  const auto r = load_config<7>(p);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().log_output_dir,
            (fs::absolute(p).parent_path() / "elsewhere").lexically_normal());
}

}  // namespace
}  // namespace cobot::pipeline
