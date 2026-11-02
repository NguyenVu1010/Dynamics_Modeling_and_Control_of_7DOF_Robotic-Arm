// cobot.limits in load_config and pipeline::controller_model (feature joint-limits, design
// docs/design/2026-11-02-realism-2-design.md G1 and section 5).
//
// Three torque levels per joint (eRob ratio 100): rated R (continuous), peak P (start/stop),
// momentary M (URDF <limit effort>, enforced by the plant). The config carries R and P:
//   cobot:
//     limits:
//       rated_torque_nm: [7 values]   # > 0 each
//       peak_torque_nm: [7 values]    # > 0 each and <= URDF effort (M) of that joint
// The section is optional: without it both default to the URDF effort. Joint indices in error
// messages are 0-based, as in VecN: "limits.peak_torque_nm[2]" is the third joint (joint_3).
// controller_model(urdf_model, config) is the URDF model with tau_max = peak, nothing else.
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"

namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;
using Vec7 = common::VecN<7>;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kUrdf = kRepo / "src/sim/urdf/cobot30_5.urdf";

std::string read_file(const fs::path& p) {
  const std::ifstream in(p, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

fs::path write_temp(const std::string& name, const std::string& text) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "pipeline_config_limits";
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
  return replace(
      read_file(kRepo / "config/default.yaml"), "../src/sim/urdf/cobot30_5.urdf", kUrdf.string());
}

bool starts_with(const std::string& line, const std::string& prefix) {
  return line.rfind(prefix, 0) == 0;
}

// Removes the `  limits:` block of the cobot map (the key line and every following line indented
// by more than two spaces, or empty). `  cartesian_limits:` does not match the prefix.
std::string without_limits(const std::string& text) {
  std::istringstream in(text);
  std::string out;
  std::string line;
  bool inside = false;
  while (std::getline(in, line)) {
    if (starts_with(line, "  limits:")) {
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

bool has_limits_section(const std::string& text) {
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    if (starts_with(line, "  limits:")) {
      return true;
    }
  }
  return false;
}

// "[a, b, ...]" with round-trip precision.
std::string yaml_list(const Vec7& v) {
  std::ostringstream os;
  os.precision(std::numeric_limits<double>::max_digits10);
  os << "[";
  for (int i = 0; i < 7; ++i) {
    os << (i > 0 ? ", " : "") << v[i];
  }
  os << "]";
  return os.str();
}

// default.yaml without its own section, plus a limits section (body lines indented by four
// spaces) at the end of the cobot map.
std::string with_limits(const std::string& body) {
  return without_limits(default_text()) + "  limits:\n" + body;
}

std::string limits_body(const Vec7& rated, const Vec7& peak) {
  return "    rated_torque_nm: " + yaml_list(rated) + "\n    peak_torque_nm: " + yaml_list(peak) +
         "\n";
}

model::Arm7 urdf_model() {
  const auto chain = model::parse_urdf_chain(kUrdf, "base_link", "link_7");
  EXPECT_TRUE(chain.is_ok()) << chain.reason();
  const auto arm = model::make_model<7>(chain.value(), common::Vec3(0.0, 0.0, -9.80665));
  EXPECT_TRUE(arm.is_ok()) << arm.reason();
  return arm.value();
}

// URDF <limit effort> (M) per joint, read through the model parser.
Vec7 urdf_effort() {
  const model::Arm7 arm = urdf_model();
  Vec7 effort;
  for (int i = 0; i < 7; ++i) {
    effort[i] = arm.joints[static_cast<std::size_t>(i)].tau_max;
  }
  return effort;
}

// A valid section relative to the URDF: rated = effort / 4, peak = effort / 2.
Vec7 valid_rated() {
  return 0.25 * urdf_effort();
}
Vec7 valid_peak() {
  return 0.5 * urdf_effort();
}

TEST(LoadConfigLimits, DefaultFileHasTheErobTable) {
  const auto r = load_config<7>(kRepo / "config/default.yaml");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const JointLimitsConfig<7>& l = r.value().limits;
  // Design section 5: joints 1-2 eRob110H, 3-4 eRob90H, 5-7 eRob70H (ratio 100).
  Vec7 rated;
  rated << 87.0, 87.0, 52.0, 52.0, 10.0, 10.0, 10.0;
  Vec7 peak;
  peak << 204.0, 204.0, 107.0, 107.0, 36.0, 36.0, 36.0;
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(l.rated_torque_nm[i], rated[i]) << "joint " << i + 1;
    EXPECT_EQ(l.peak_torque_nm[i], peak[i]) << "joint " << i + 1;
  }
}

TEST(LoadConfigLimits, DefaultFilePeakIsWithinTheUrdfEffort) {
  // P <= M joint by joint: 204 <= 369, 107 <= 191, 36 <= 70.
  const auto r = load_config<7>(kRepo / "config/default.yaml");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const Vec7 effort = urdf_effort();
  for (int i = 0; i < 7; ++i) {
    EXPECT_LE(r.value().limits.peak_torque_nm[i], effort[i]) << "joint " << i + 1;
    EXPECT_LE(r.value().limits.rated_torque_nm[i], r.value().limits.peak_torque_nm[i])
        << "joint " << i + 1;
  }
}

TEST(LoadConfigLimits, MissingSectionUsesUrdfEffortForBoth) {
  const std::string text = without_limits(default_text());
  ASSERT_FALSE(has_limits_section(text));
  const auto r = load_config<7>(write_temp("missing.yaml", text));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const Vec7 effort = urdf_effort();
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(r.value().limits.rated_torque_nm[i], effort[i]) << "joint " << i + 1;
    EXPECT_EQ(r.value().limits.peak_torque_nm[i], effort[i]) << "joint " << i + 1;
  }
  // The rest of the file is unaffected.
  EXPECT_DOUBLE_EQ(r.value().computed_torque.kp[0], 6400.0);
}

TEST(LoadConfigLimits, GivenValuesAreParsed) {
  const Vec7 rated = valid_rated();
  const Vec7 peak = valid_peak();
  const auto r = load_config<7>(write_temp("valid.yaml", with_limits(limits_body(rated, peak))));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(r.value().limits.rated_torque_nm[i], rated[i]) << "joint " << i + 1;
    EXPECT_EQ(r.value().limits.peak_torque_nm[i], peak[i]) << "joint " << i + 1;
  }
}

TEST(LoadConfigLimits, PeakEqualToUrdfEffortIsABoundaryValue) {
  const Vec7 effort = urdf_effort();
  const auto r =
      load_config<7>(write_temp("equal.yaml", with_limits(limits_body(valid_rated(), effort))));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(r.value().limits.peak_torque_nm[i], effort[i]) << "joint " << i + 1;
  }
}

TEST(LoadConfigLimits, PeakAboveUrdfEffortNamesTheJoint) {
  const Vec7 effort = urdf_effort();
  for (const int j : {0, 2, 6}) {
    Vec7 peak = valid_peak();
    peak[j] = effort[j] * 1.01;  // 1 % above M
    const auto r = load_config<7>(write_temp("peak_above" + std::to_string(j) + ".yaml",
                                             with_limits(limits_body(valid_rated(), peak))));
    ASSERT_FALSE(r.is_ok()) << "joint index " << j;
    const std::string key = "limits.peak_torque_nm[" + std::to_string(j) + "]";
    EXPECT_NE(r.reason().find(key), std::string::npos) << key << " not in: " << r.reason();
  }
}

// One change to a valid section and the key the error message must name.
struct BadCase {
  bool rated;  // edit rated_torque_nm (else peak_torque_nm)
  int joint;   // 0-based
  double value;
  const char* key;
};

TEST(LoadConfigLimits, NonPositiveValueIsRejectedNamingTheKey) {
  const std::vector<BadCase> cases = {
      {true, 4, 0.0, "limits.rated_torque_nm"},
      {true, 1, -5.0, "limits.rated_torque_nm"},
      {false, 3, 0.0, "limits.peak_torque_nm"},
      {false, 0, -1.0, "limits.peak_torque_nm"},
  };
  int n = 0;
  for (const BadCase& bad : cases) {
    Vec7 rated = valid_rated();
    Vec7 peak = valid_peak();
    (bad.rated ? rated : peak)[bad.joint] = bad.value;
    const auto r = load_config<7>(write_temp("non_positive" + std::to_string(n++) + ".yaml",
                                             with_limits(limits_body(rated, peak))));
    ASSERT_FALSE(r.is_ok()) << bad.key << "[" << bad.joint << "] = " << bad.value;
    EXPECT_NE(r.reason().find(bad.key), std::string::npos) << r.reason();
  }
}

TEST(LoadConfigLimits, MalformedSectionIsRejectedNamingTheKey) {
  const std::string valid = limits_body(valid_rated(), valid_peak());
  const std::string rated_line = "    rated_torque_nm: " + yaml_list(valid_rated()) + "\n";
  struct Edit {
    std::string to;
    const char* key;
  };
  const std::vector<Edit> edits = {
      {"    rated_torque_nm: [10, 10, 10, 10, 10, 10]\n", "limits.rated_torque_nm"},
      {"    rated_torque_nm: strong\n", "limits.rated_torque_nm"},
      {rated_line + "    continuous_torque_nm: 5.0\n", "limits.continuous_torque_nm"},
  };
  int n = 0;
  for (const Edit& e : edits) {
    const auto r = load_config<7>(write_temp("malformed" + std::to_string(n++) + ".yaml",
                                             with_limits(replace(valid, rated_line, e.to))));
    ASSERT_FALSE(r.is_ok()) << e.key;
    EXPECT_NE(r.reason().find(e.key), std::string::npos) << r.reason();
  }
}

void expect_same_except_tau_max(const model::Arm7& got, const model::Arm7& urdf) {
  for (std::size_t k = 0; k < 7; ++k) {
    const model::JointParams& a = got.joints[k];
    const model::JointParams& b = urdf.joints[k];
    EXPECT_EQ(a.name, b.name) << "joint " << k + 1;
    EXPECT_EQ(a.T_parent_joint.R, b.T_parent_joint.R) << "joint " << k + 1;
    EXPECT_EQ(a.T_parent_joint.p, b.T_parent_joint.p) << "joint " << k + 1;
    EXPECT_EQ(a.axis_joint, b.axis_joint) << "joint " << k + 1;
    EXPECT_EQ(a.q_min, b.q_min) << "joint " << k + 1;
    EXPECT_EQ(a.q_max, b.q_max) << "joint " << k + 1;
    EXPECT_EQ(a.qd_max, b.qd_max) << "joint " << k + 1;
    EXPECT_EQ(a.damping, b.damping) << "joint " << k + 1;
    EXPECT_EQ(a.armature, b.armature) << "joint " << k + 1;
    EXPECT_EQ(got.links[k].mass_kg, urdf.links[k].mass_kg) << "link " << k + 1;
    EXPECT_EQ(got.links[k].com_i, urdf.links[k].com_i) << "link " << k + 1;
    EXPECT_EQ(got.links[k].I_com_i, urdf.links[k].I_com_i) << "link " << k + 1;
  }
  EXPECT_EQ(got.T_last_tip.R, urdf.T_last_tip.R);
  EXPECT_EQ(got.T_last_tip.p, urdf.T_last_tip.p);
  EXPECT_EQ(got.gravity_b, urdf.gravity_b);
}

TEST(ControllerModel, SetsTauMaxToPeakAndNothingElse) {
  const model::Arm7 urdf = urdf_model();
  Config<7> config;
  config.limits.rated_torque_nm << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0;
  config.limits.peak_torque_nm << 11.5, 12.5, 13.5, 14.5, 15.5, 16.5, 17.5;
  const model::Arm7 got = controller_model<7>(urdf, config);
  for (std::size_t k = 0; k < 7; ++k) {
    EXPECT_EQ(got.joints[k].tau_max, config.limits.peak_torque_nm[static_cast<int>(k)])
        << "joint " << k + 1;
  }
  expect_same_except_tau_max(got, urdf);
}

TEST(ControllerModel, KeepsAModelGravityThatDiffersFromTheUrdfDefault) {
  model::Arm7 urdf = urdf_model();
  urdf.gravity_b = common::Vec3(0.1, -0.2, -9.7);
  Config<7> config;
  config.limits.rated_torque_nm = Vec7::Constant(5.0);
  config.limits.peak_torque_nm = Vec7::Constant(9.0);
  const model::Arm7 got = controller_model<7>(urdf, config);
  EXPECT_EQ(got.gravity_b, urdf.gravity_b);
  for (std::size_t k = 0; k < 7; ++k) {
    EXPECT_EQ(got.joints[k].tau_max, 9.0) << "joint " << k + 1;
  }
}

TEST(ControllerModel, DefaultFileGivesPeakTable) {
  const auto r = load_config<7>(kRepo / "config/default.yaml");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const model::Arm7 urdf = urdf_model();
  const model::Arm7 got = controller_model<7>(urdf, r.value());
  const double peak[] = {204.0, 204.0, 107.0, 107.0, 36.0, 36.0, 36.0};
  for (std::size_t k = 0; k < 7; ++k) {
    EXPECT_EQ(got.joints[k].tau_max, peak[k]) << "joint " << k + 1;
  }
  expect_same_except_tau_max(got, urdf);
}

TEST(ControllerModel, MissingSectionGivesTheUrdfModel) {
  const auto r = load_config<7>(write_temp("missing_model.yaml", without_limits(default_text())));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const model::Arm7 urdf = urdf_model();
  const model::Arm7 got = controller_model<7>(urdf, r.value());
  for (std::size_t k = 0; k < 7; ++k) {
    EXPECT_EQ(got.joints[k].tau_max, urdf.joints[k].tau_max) << "joint " << k + 1;
  }
  expect_same_except_tau_max(got, urdf);
}

}  // namespace
}  // namespace cobot::pipeline
