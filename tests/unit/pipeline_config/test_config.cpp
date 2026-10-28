#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/config.hpp"

namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

const fs::path kRepo = COBOT_REPO_DIR;

std::string read_file(const fs::path& p) {
  const std::ifstream in(p, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
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

fs::path write_temp(const std::string& name, const std::string& text) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "pipeline_config";
  fs::create_directories(dir);
  const fs::path p = dir / name;
  std::ofstream(p, std::ios::binary) << text;
  return p;
}

// config/default.yaml with an absolute URDF path, so the copy is valid in any directory.
std::string default_text() {
  return replace(read_file(kRepo / "config/default.yaml"),
                 "../sim/urdf/cobot30_5.urdf",
                 (kRepo / "sim/urdf/cobot30_5.urdf").string());
}
std::string ideal_text() {
  return read_file(kRepo / "config/imperfections/ideal.yaml");
}

// One edit of a valid file and the key path the error message must name.
struct BadCase {
  const char* from;
  const char* to;
  const char* key;
};

TEST(LoadConfig, DefaultFile) {
  const auto r = load_config<7>(kRepo / "config/default.yaml");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const Config<7>& c = r.value();
  EXPECT_NEAR(c.home_q_rad[3], -kPi / 2.0, 1e-15);
  EXPECT_NEAR(c.home_q_rad[1], 30.0 * kDeg, 1e-15);
  EXPECT_NEAR(c.trajectory.goal_rpy_rad[2], kPi / 2.0, 1e-15);
  EXPECT_DOUBLE_EQ(c.trajectory.goal_position_m[0], -0.5);
  EXPECT_DOUBLE_EQ(c.trajectory.duration_s, 10.0);
  EXPECT_DOUBLE_EQ(c.trajectory.hold_s, 2.0);
  EXPECT_TRUE(c.urdf_path.is_absolute());
  EXPECT_TRUE(fs::exists(c.urdf_path)) << c.urdf_path;
  EXPECT_EQ(c.urdf_path, (kRepo / "sim/urdf/cobot30_5.urdf").lexically_normal());
  EXPECT_EQ(c.log_output_dir, (kRepo / "results").lexically_normal());
  EXPECT_EQ(c.base_link, "base_link");
  EXPECT_EQ(c.tip_link, "link_7");
  EXPECT_DOUBLE_EQ(c.gravity_b[2], -9.80665);
  EXPECT_DOUBLE_EQ(c.control_rate_hz, 1000.0);
  EXPECT_TRUE(c.deterministic);
  EXPECT_TRUE((c.clik.kp.array() == 100.0).all());
  EXPECT_TRUE((c.clik.kd.array() == 20.0).all());
  EXPECT_DOUBLE_EQ(c.clik.k_null, 1.0);
  EXPECT_DOUBLE_EQ(c.clik.k_damp, 2.0);
  EXPECT_DOUBLE_EQ(c.clik.lambda_max, 0.05);
  EXPECT_DOUBLE_EQ(c.clik.cond_threshold, 100.0);
  EXPECT_DOUBLE_EQ(c.computed_torque.kp[1], 6400.0);
  EXPECT_DOUBLE_EQ(c.computed_torque.kd[6], 160.0);
  EXPECT_NEAR(c.safety.soft_margin_rad, 3.0 * kDeg, 1e-15);
  EXPECT_NEAR(c.safety.hal_timeout_s, 0.005, 1e-15);
}

TEST(LoadConfig, RelativePathsFollowTheYamlDirectory) {
  const fs::path p = write_temp(
      "relative.yaml",
      replace(
          read_file(kRepo / "config/default.yaml"), "../sim/urdf/cobot30_5.urdf", "robot.urdf"));
  const auto r = load_config<7>(p);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const fs::path dir = fs::absolute(p).parent_path();
  EXPECT_EQ(r.value().urdf_path, (dir / "robot.urdf").lexically_normal());
  EXPECT_EQ(r.value().log_output_dir, (dir / "../results").lexically_normal());
}

TEST(LoadConfig, EachBadValueNamesItsKey) {
  const std::vector<BadCase> cases = {
      {"  clik:", "  bogus: 1\n  clik:", "cobot.bogus"},   // unknown key
      {"    hold_s: 2.0", "", "cobot.trajectory.hold_s"},  // missing key
      {"kp: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]",
       "kp: [100.0, 100.0, 100.0, 100.0, 100.0]",
       "cobot.clik.kp: expected 6 values, got 5"},
      {"[0.0, 30.0, 0.0, -90.0, 0.0, 60.0, 0.0]", "[0.0, 30.0]", "cobot.home_q_deg"},
      {"[0.0, 0.0, -9.80665]", "[0.0, -9.80665]", "cobot.gravity_b"},
      {"[-0.5, -0.6, 0.9]", "[-0.5, -0.6, 0.9, 1.0]", "cobot.trajectory.goal_position_m"},
      {"kp: [6400.0, 6400.0,", "kp: [6400.0, -6400.0,", "cobot.computed_torque.kp"},
      {"k_null: 1.0", "k_null: 0.0", "cobot.clik.k_null"},
      {"control_rate_hz: 1000.0", "control_rate_hz: fast", "cobot.control_rate_hz"},
      {"control_rate_hz: 1000.0", "control_rate_hz: 0.0", "cobot.control_rate_hz"},
      {"deterministic: true", "deterministic: [1]", "cobot.deterministic"},
      {"duration_s: 10.0", "duration_s: 0.0", "cobot.trajectory.duration_s"},
      {"hold_s: 2.0", "hold_s: -1.0", "cobot.trajectory.hold_s"},
      {"lambda_max: 0.05", "lambda_max: -0.1", "cobot.clik.lambda_max"},
      {"cond_threshold: 100.0", "cond_threshold: 1.0", "cobot.clik.cond_threshold"},
      {"soft_margin_deg: 3.0", "soft_margin_deg: -3.0", "cobot.safety.soft_margin_deg"},
  };
  int n = 0;
  for (const auto& bad : cases) {
    const auto r = load_config<7>(write_temp("bad" + std::to_string(n++) + ".yaml",
                                             replace(default_text(), bad.from, bad.to)));
    ASSERT_FALSE(r.is_ok()) << bad.key;
    EXPECT_NE(r.reason().find(bad.key), std::string::npos) << r.reason();
  }
}

TEST(LoadConfig, BoundaryValuesAccepted) {
  std::string text = replace(default_text(), "hold_s: 2.0", "hold_s: 0.0");
  text = replace(text, "lambda_max: 0.05", "lambda_max: 0.0");
  text = replace(text, "soft_margin_deg: 3.0", "soft_margin_deg: 0.0");
  const auto r = load_config<7>(write_temp("boundary.yaml", text));
  EXPECT_TRUE(r.is_ok()) << r.reason();
}

TEST(LoadConfig, TwoErrorsReportedTogetherOnePerLine) {
  std::string text = replace(default_text(), "control_rate_hz: 1000.0", "control_rate_hz: -1.0");
  text = replace(text, "kd: [20.0, 20.0, 20.0, 20.0, 20.0, 20.0]", "kd: [20.0, 20.0]");
  const auto r = load_config<7>(write_temp("two.yaml", text));
  ASSERT_FALSE(r.is_ok());
  const std::string msg = r.reason();
  const auto a = msg.find("cobot.control_rate_hz");
  const auto b = msg.find("cobot.clik.kd");
  ASSERT_NE(a, std::string::npos) << msg;
  ASSERT_NE(b, std::string::npos) << msg;
  const auto first = std::min(a, b);
  EXPECT_NE(msg.substr(first, std::max(a, b) - first).find('\n'), std::string::npos) << msg;
}

TEST(LoadConfig, MissingFileNamesThePath) {
  const auto r = load_config<7>(fs::path(COBOT_TEST_OUTPUT_DIR) / "does_not_exist.yaml");
  ASSERT_FALSE(r.is_ok());
  EXPECT_NE(r.reason().find("does_not_exist.yaml"), std::string::npos) << r.reason();
  EXPECT_FALSE(load_imperfections<7>(fs::path(COBOT_TEST_OUTPUT_DIR) / "nope.yaml").is_ok());
}

TEST(LoadImperfections, Ideal) {
  const auto r = load_imperfections<7>(kRepo / "config/imperfections/ideal.yaml");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const auto& c = r.value();
  EXPECT_TRUE(c.sensor.encoder_bits.isZero());
  EXPECT_FALSE(c.sensor.qd_from_diff);
  EXPECT_TRUE((c.sensor.qd_filter_hz.array() == 200.0).all());
  EXPECT_EQ(c.sensor.sensor_delay_cycles, 0);
  EXPECT_TRUE((c.actuator.ripple_periods_per_rev.array() == 6.0).all());
  EXPECT_EQ(c.actuator.command_delay_cycles, 0);
  EXPECT_TRUE(c.friction.coulomb_nm.isZero());
  EXPECT_DOUBLE_EQ(c.model.payload_kg, 0.0);
  EXPECT_DOUBLE_EQ(c.timing.period_jitter_std_us, 0.0);
  EXPECT_EQ(c.seed, 1U);
}

TEST(LoadImperfections, RealisticAndPessimisticProfiles) {
  const auto r = load_imperfections<7>(kRepo / "config/imperfections/realistic.yaml");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const auto& c = r.value();
  EXPECT_TRUE((c.sensor.encoder_bits.array() == 19.0).all());
  EXPECT_DOUBLE_EQ(c.friction.coulomb_nm[4], 0.24);
  EXPECT_DOUBLE_EQ(c.friction.coulomb_nm[0], 1.7);
  EXPECT_DOUBLE_EQ(c.actuator.torque_cmd_resolution_nm[0], 0.087);
  EXPECT_DOUBLE_EQ(c.actuator.torque_cmd_resolution_nm[6], 0.012);
  EXPECT_TRUE(c.sensor.qd_from_diff);
  EXPECT_EQ(c.sensor.sensor_delay_cycles, 1);
  EXPECT_EQ(c.actuator.command_delay_cycles, 1);
  EXPECT_DOUBLE_EQ(c.timing.period_jitter_std_us, 20.0);

  const auto p = load_imperfections<7>(kRepo / "config/imperfections/pessimistic.yaml");
  ASSERT_TRUE(p.is_ok()) << p.reason();
  EXPECT_TRUE((p.value().sensor.encoder_bits.array() == 14.0).all());
  EXPECT_EQ(p.value().sensor.sensor_delay_cycles, 2);
  EXPECT_EQ(p.value().actuator.command_delay_cycles, 2);
  EXPECT_DOUBLE_EQ(p.value().model.payload_kg, 1.0);
  EXPECT_DOUBLE_EQ(p.value().friction.viscous_nm_s_rad[5], 0.3);
}

TEST(LoadImperfections, ScalarBroadcastAndPerJointList) {
  std::string text = replace(ideal_text(), "q_noise_std_rad: 0.0", "q_noise_std_rad: 0.001");
  text = replace(text, "coulomb_nm: 0.0", "coulomb_nm: [1, 2, 3, 4, 5, 6, 7]");
  text = replace(text, "sensor_delay_cycles: 0", "sensor_delay_cycles: 32");  // upper bound
  text = replace(text, "encoder_bits: 0 ", "encoder_bits: 8 ");               // lower bound
  const auto r = load_imperfections<7>(write_temp("broadcast.yaml", text));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_TRUE((r.value().sensor.q_noise_std_rad.array() == 0.001).all());
  EXPECT_TRUE((r.value().sensor.encoder_bits.array() == 8.0).all());
  EXPECT_EQ(r.value().sensor.sensor_delay_cycles, 32);
  for (int i = 0; i < 7; ++i) {
    EXPECT_DOUBLE_EQ(r.value().friction.coulomb_nm[i], i + 1.0);
  }
}

TEST(LoadImperfections, EachBadValueNamesItsKey) {
  const std::vector<BadCase> cases = {
      {"coulomb_nm: 0.0", "coulomb_nm: [1, 2, 3]", "imperfections.friction.coulomb_nm"},
      {"sensor_delay_cycles: 0",
       "sensor_delay_cycles: 40",
       "imperfections.sensor.sensor_delay_cycles"},
      {"command_delay_cycles: 0",
       "command_delay_cycles: -1",
       "imperfections.actuator.command_delay_cycles"},
      {"command_delay_cycles: 0",
       "command_delay_cycles: 1.5",
       "imperfections.actuator.command_delay_cycles"},
      {"q_noise_std_rad: 0.0", "q_noise_std_rad: -0.1", "imperfections.sensor.q_noise_std_rad"},
      {"link_mass_scale_std: 0.0",
       "link_mass_scale_std: -1.0",
       "imperfections.model.link_mass_scale_std"},
      {"encoder_bits: 0 ", "encoder_bits: 4 ", "imperfections.sensor.encoder_bits"},
      {"encoder_bits: 0 ", "encoder_bits: 33 ", "imperfections.sensor.encoder_bits"},
      {"  friction:\n    source: \"ideal\"\n", "  friction:\n", "imperfections.friction.source"},
      {"  timing:", "  extra: 1\n  timing:", "imperfections.extra"},
      {"    payload_kg: 0.0", "", "imperfections.model.payload_kg"},
  };
  int n = 0;
  for (const auto& bad : cases) {
    const auto r = load_imperfections<7>(write_temp("imp_bad" + std::to_string(n++) + ".yaml",
                                                    replace(ideal_text(), bad.from, bad.to)));
    ASSERT_FALSE(r.is_ok()) << bad.key;
    EXPECT_NE(r.reason().find(bad.key), std::string::npos) << r.reason();
  }
}

}  // namespace
}  // namespace cobot::pipeline
