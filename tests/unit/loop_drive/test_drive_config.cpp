// cobot.drive in load_config (brief drive-loop 3, ADR-009 key checks): {rate_hz, omega_rad_s,
// zeta, velocity_filter_hz, torque_feedforward}, every key required in the section; rate_hz > 0
// and rate_hz / control_rate_hz an integer >= 1 (1e-9 relative); omega_rad_s > 0, N values or a
// scalar; zeta > 0; velocity_filter_hz > 0; torque_feedforward none | gravity | inverse_dynamics.
// The section is required when cobot.control.algorithm is drive_position.
// config/budget_xarm7_drive.yaml is budget_xarm7.yaml with the drive algorithm, the drive block
// and safety.hal_timeout_ms 20 (owner question C2).
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"
#include "cobot/control/drive_position.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/overrides.hpp"

namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;
using control::Algorithm;
using control::DriveFeedForward;

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
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "loop_drive_config";
  fs::create_directories(dir);
  const fs::path p = dir / name;
  std::ofstream(p, std::ios::binary) << text;
  return p;
}

// Removes the `  <name>:` block of the cobot map (the key line and the following lines indented by
// more than two spaces, or empty).
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

// config/default.yaml with an absolute URDF path, without `control` and `drive`, plus `extra`
// (lines indented by two spaces) at the end of the cobot map.
std::string default_with(const std::string& extra) {
  std::string text = read_text(kRepo / "config/default.yaml");
  const std::string rel = "../src/sim/urdf/cobot30_5.urdf";
  const auto pos = text.find(rel);
  EXPECT_NE(pos, std::string::npos);
  if (pos != std::string::npos) {
    text.replace(pos, rel.size(), (kRepo / "src/sim/urdf/cobot30_5.urdf").string());
  }
  return without_section(without_section(text, "control"), "drive") + extra;
}

Result<Config<kN>> load_text(const std::string& name, const std::string& extra) {
  return load_config<kN>(write_temp(name, default_with(extra)));
}

void expect_error(const std::string& name, const std::string& extra, const std::string& part) {
  const auto r = load_text(name, extra);
  ASSERT_FALSE(r.is_ok()) << name << " loaded";
  EXPECT_TRUE(contains(r.reason(), part)) << name << ": " << r.reason();
}

const std::string kAlgo = "  control:\n    algorithm: drive_position\n";

// The drive block with one key replaced (`key: value`) or removed (empty value).
std::string drive_block(const std::string& key = "", const std::string& value = "") {
  const std::vector<std::pair<std::string, std::string>> keys{
      {"rate_hz", "4000"},
      {"omega_rad_s", "60"},
      {"zeta", "1.0"},
      {"velocity_filter_hz", "500"},
      {"torque_feedforward", "inverse_dynamics"},
  };
  std::string out = "  drive:\n";
  for (const auto& [k, v] : keys) {
    const std::string& text = k == key ? value : v;
    if (!text.empty()) {
      out.append("    ").append(k).append(": ").append(text).append("\n");
    }
  }
  return out;
}

TEST(DriveConfig, Defaults) {
  const DriveConfig<kN> d;
  EXPECT_EQ(d.rate_hz, 4000.0);
  EXPECT_EQ(d.omega_rad_s, common::VecN<kN>::Constant(60.0));
  EXPECT_EQ(d.zeta, 1.0);
  EXPECT_EQ(d.velocity_filter_hz, 500.0);
  EXPECT_EQ(d.torque_feedforward, DriveFeedForward::kInverseDynamics);
}

TEST(DriveConfig, SectionParsed) {
  const auto r = load_text("drive.yaml", kAlgo + drive_block());
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const Config<kN>& c = r.value();
  EXPECT_EQ(c.algorithm, Algorithm::kDrivePosition);
  EXPECT_EQ(c.drive.rate_hz, 4000.0);
  EXPECT_EQ(c.drive.omega_rad_s, common::VecN<kN>::Constant(60.0)) << "scalar broadcast";
  EXPECT_EQ(c.drive.zeta, 1.0);
  EXPECT_EQ(c.drive.velocity_filter_hz, 500.0);
  EXPECT_EQ(c.drive.torque_feedforward, DriveFeedForward::kInverseDynamics);

  const auto list =
      load_text("list.yaml", kAlgo + drive_block("omega_rad_s", "[10, 20, 30, 40, 50, 60, 70]"));
  ASSERT_TRUE(list.is_ok()) << list.reason();
  for (int j = 0; j < kN; ++j) {
    EXPECT_EQ(list.value().drive.omega_rad_s(j), 10.0 * (j + 1)) << "joint " << j;
  }
  const struct {
    const char* text;
    DriveFeedForward ff;
  } names[] = {{"none", DriveFeedForward::kNone},
               {"gravity", DriveFeedForward::kGravity},
               {"inverse_dynamics", DriveFeedForward::kInverseDynamics}};
  for (const auto& n : names) {
    const auto f = load_text(std::string("ff_") + n.text + ".yaml",
                             kAlgo + drive_block("torque_feedforward", n.text));
    ASSERT_TRUE(f.is_ok()) << n.text << ": " << f.reason();
    EXPECT_EQ(f.value().drive.torque_feedforward, n.ff) << n.text;
  }
  // rate 1000 Hz = control rate: ratio 1.
  const auto one = load_text("rate_1000.yaml", kAlgo + drive_block("rate_hz", "1000"));
  ASSERT_TRUE(one.is_ok()) << one.reason();
  EXPECT_EQ(one.value().drive.rate_hz, 1000.0);
}

TEST(DriveConfig, Rejections) {
  expect_error("required.yaml",
               kAlgo,
               "cobot.drive: required when cobot.control.algorithm is drive_position");
  expect_error("rate_2500.yaml",
               kAlgo + drive_block("rate_hz", "2500"),
               "cobot.drive.rate_hz: must be an integer multiple of control_rate_hz (1000), got "
               "2500");
  expect_error("rate_500.yaml", kAlgo + drive_block("rate_hz", "500"), "integer multiple");
  expect_error("rate_0.yaml", kAlgo + drive_block("rate_hz", "0"), "cobot.drive.rate_hz");
  expect_error("ff_x.yaml",
               kAlgo + drive_block("torque_feedforward", "x"),
               "cobot.drive.torque_feedforward: expected none, gravity or inverse_dynamics, got "
               "'x'");
  expect_error("omega_0.yaml", kAlgo + drive_block("omega_rad_s", "0"), "cobot.drive.omega_rad_s");
  const std::string six = kAlgo + drive_block("omega_rad_s", "[60, 60, 60, 60, 60, 60]");
  expect_error("omega_six.yaml", six, "cobot.drive.omega_rad_s: expected ");
  expect_error("omega_six.yaml", six, "7 values, got 6");
  expect_error("zeta_0.yaml", kAlgo + drive_block("zeta", "0"), "cobot.drive.zeta");
  expect_error("filter_neg.yaml",
               kAlgo + drive_block("velocity_filter_hz", "-5"),
               "cobot.drive.velocity_filter_hz");
  for (const char* key :
       {"rate_hz", "omega_rad_s", "zeta", "velocity_filter_hz", "torque_feedforward"}) {
    expect_error(std::string("missing_") + key + ".yaml",
                 kAlgo + drive_block(key),
                 std::string("cobot.drive.") + key);
  }
  expect_error("extra.yaml", kAlgo + drive_block() + "    ki: 1\n", "cobot.drive.ki");
}

TEST(DriveConfig, BudgetDriveFile) {
  const auto r = load_config<kN>(kRepo / "config/budget_xarm7_drive.yaml");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const Config<kN>& c = r.value();
  EXPECT_EQ(c.algorithm, Algorithm::kDrivePosition);
  EXPECT_DOUBLE_EQ(c.safety.hal_timeout_s, 0.020);  // 20 ms = 5 servo periods at 250 Hz
  EXPECT_EQ(c.drive.rate_hz, 4000.0);
  EXPECT_EQ(c.drive.torque_feedforward, DriveFeedForward::kInverseDynamics);  // C1 default
  EXPECT_TRUE((c.drive.omega_rad_s.array() >= 20.0).all() &&
              (c.drive.omega_rad_s.array() <= 150.0).all())
      << "the swept range 20..150 rad/s: " << c.drive.omega_rad_s.transpose();
  EXPECT_GT(c.drive.zeta, 0.0);
  EXPECT_GT(c.drive.velocity_filter_hz, 0.0);
  EXPECT_TRUE(c.deflection_comp.enabled) << "as budget_xarm7.yaml";

  const auto base = load_config<kN>(kRepo / "config/budget_xarm7.yaml");
  ASSERT_TRUE(base.is_ok()) << base.reason();
  EXPECT_EQ(base.value().algorithm, Algorithm::kComputedTorque) << "E2E-12 keeps its finding";
  EXPECT_EQ(c.computed_torque.kp, base.value().computed_torque.kp);
  EXPECT_EQ(c.limits.peak_torque_nm, base.value().limits.peak_torque_nm);
}

// The gain family of drive_position for auto-tune (plan data contracts): the scalar
// config.drive.omega_rad_s broadcast to every joint through the override path.
TEST(DriveConfig, OmegaOverride) {
  const auto r = load_config_with<kN>(kRepo / "config/budget_xarm7_drive.yaml",
                                      {{"config.drive.omega_rad_s", 80.0}});
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().drive.omega_rad_s, common::VecN<kN>::Constant(80.0));

  // The section created by the override path on a file without it (default.yaml).
  const std::vector<KeyOverride> drive{
      {"config.control.algorithm", std::string("drive_position")},
      {"config.drive.rate_hz", 4000.0},
      {"config.drive.omega_rad_s", 70.0},
      {"config.drive.zeta", 0.9},
      {"config.drive.velocity_filter_hz", 400.0},
      {"config.drive.torque_feedforward", std::string("gravity")},
  };
  const auto d = load_config_with<kN>(kRepo / "config/default.yaml", drive);
  ASSERT_TRUE(d.is_ok()) << d.reason();
  EXPECT_EQ(d.value().algorithm, Algorithm::kDrivePosition);
  EXPECT_EQ(d.value().drive.omega_rad_s, common::VecN<kN>::Constant(70.0));
  EXPECT_EQ(d.value().drive.zeta, 0.9);
  EXPECT_EQ(d.value().drive.velocity_filter_hz, 400.0);
  EXPECT_EQ(d.value().drive.torque_feedforward, DriveFeedForward::kGravity);
}

}  // namespace
}  // namespace cobot::pipeline
