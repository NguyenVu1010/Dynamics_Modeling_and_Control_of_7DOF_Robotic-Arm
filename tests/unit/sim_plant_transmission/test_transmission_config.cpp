// Loader side of feature transmission: the optional `imperfections.transmission` group (scalar or
// 7-list per key, `source` like every group) and `imperfections.sensor.position_source`
// (link | motor). Ranges: stiffness >= 0, k3 == 0 is a rigid joint; 0 < t1 < t2 when elastic;
// damping_ratio >= 0; lost_motion_rad >= 0; ratio > 0. Errors name the full key.
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/sweep_config.hpp"

namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;
using Vec = common::VecN<7>;
using imperfect::PositionSource;

std::string ideal_text() {
  const std::ifstream in(fs::path(COBOT_REPO_DIR) / "config/imperfections/ideal.yaml",
                         std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// ideal.yaml plus a transmission group made of the starting values (brief table), with `extra`
// lines (each "    key: value\n") replacing the defaults of the same keys.
std::string with_transmission(const std::string& extra) {
  std::string text = ideal_text();
  if (!text.empty() && text.back() != '\n') {
    text += '\n';
  }
  std::string group =
      "  transmission:\n"
      "    source: \"test\"\n"
      "    stiffness_k1_nm_rad: [3.1e4, 3.1e4, 1.3e4, 1.3e4, 4700, 4700, 4700]\n"
      "    stiffness_k2_nm_rad: [5.0e4, 5.0e4, 2.5e4, 2.5e4, 6100, 6100, 6100]\n"
      "    stiffness_k3_nm_rad: [5.7e4, 5.7e4, 2.9e4, 2.9e4, 7100, 7100, 7100]\n"
      "    torque_t1_nm: [14, 14, 7, 7, 2, 2, 2]\n"
      "    torque_t2_nm: [48, 48, 25, 25, 6.9, 6.9, 6.9]\n"
      "    damping_ratio: 0.02\n"
      "    error_amp_rad: 4.4e-4\n"
      "    lost_motion_rad: 2.9e-4\n"
      "    ratio: 120\n";
  std::istringstream lines(extra);
  std::string line;
  while (std::getline(lines, line)) {
    const std::string key = line.substr(0, line.find(':') + 1);
    const std::size_t at = group.find(key);
    if (at == std::string::npos) {
      group += line + "\n";
    } else {
      group.replace(at, group.find('\n', at) - at, line);
    }
  }
  return text + group;
}

std::string with_position_source(const std::string& value) {
  std::string text = ideal_text();
  const std::string anchor = "    sensor_delay_cycles: 0\n";
  const std::size_t at = text.find(anchor);
  EXPECT_NE(at, std::string::npos);
  text.insert(at + anchor.size(), "    position_source: " + value + "\n");
  return text;
}

fs::path write_temp(const std::string& name, const std::string& text) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "sim_plant_transmission";
  fs::create_directories(dir);
  const fs::path p = dir / name;
  std::ofstream(p, std::ios::binary) << text;
  return p;
}

void expect_rejected(const std::string& name, const std::string& text, const std::string& key) {
  const auto r = load_imperfections<7>(write_temp(name, text));
  ASSERT_FALSE(r.is_ok()) << name;
  EXPECT_NE(r.reason().find(key), std::string::npos) << name << ": " << r.reason();
}

TEST(TransmissionConfig, IdealProfileIsRigidAndReadsTheLink) {
  const auto r = load_imperfections<7>(write_temp("ideal.yaml", ideal_text()));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_TRUE(r.value().transmission.stiffness_k3_nm_rad.isZero(0.0));
  EXPECT_EQ(r.value().sensor.position_source, PositionSource::kLink);
}

TEST(TransmissionConfig, ParsesListsAndScalars) {
  const auto r = load_imperfections<7>(write_temp("full.yaml", with_transmission("")));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const imperfect::TransmissionConfig<7>& t = r.value().transmission;
  EXPECT_EQ(t.stiffness_k1_nm_rad,
            (Vec() << 3.1e4, 3.1e4, 1.3e4, 1.3e4, 4700, 4700, 4700).finished());
  EXPECT_EQ(t.stiffness_k2_nm_rad,
            (Vec() << 5.0e4, 5.0e4, 2.5e4, 2.5e4, 6100, 6100, 6100).finished());
  EXPECT_EQ(t.stiffness_k3_nm_rad,
            (Vec() << 5.7e4, 5.7e4, 2.9e4, 2.9e4, 7100, 7100, 7100).finished());
  EXPECT_EQ(t.torque_t1_nm, (Vec() << 14, 14, 7, 7, 2, 2, 2).finished());
  EXPECT_EQ(t.torque_t2_nm, (Vec() << 48, 48, 25, 25, 6.9, 6.9, 6.9).finished());
  EXPECT_EQ(t.damping_ratio, Vec::Constant(0.02));
  EXPECT_EQ(t.error_amp_rad, Vec::Constant(4.4e-4));
  EXPECT_EQ(t.lost_motion_rad, Vec::Constant(2.9e-4));
  EXPECT_EQ(t.ratio, Vec::Constant(120.0));
}

TEST(TransmissionConfig, RigidJointsNeedNoBreakpoints) {
  const auto r = load_imperfections<7>(
      write_temp("rigid.yaml",
                 with_transmission("    stiffness_k3_nm_rad: [5.7e4, 0, 0, 0, 0, 0, 0]\n"
                                   "    torque_t1_nm: [14, 0, 0, 0, 0, 0, 0]\n"
                                   "    torque_t2_nm: [48, 0, 0, 0, 0, 0, 0]\n")));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().transmission.stiffness_k3_nm_rad(0), 5.7e4);
  EXPECT_EQ(r.value().transmission.stiffness_k3_nm_rad(1), 0.0);
}

TEST(TransmissionConfig, PositionSource) {
  const auto motor = load_imperfections<7>(write_temp("motor.yaml", with_position_source("motor")));
  ASSERT_TRUE(motor.is_ok()) << motor.reason();
  EXPECT_EQ(motor.value().sensor.position_source, PositionSource::kMotor);
  const auto link = load_imperfections<7>(write_temp("link.yaml", with_position_source("link")));
  ASSERT_TRUE(link.is_ok()) << link.reason();
  EXPECT_EQ(link.value().sensor.position_source, PositionSource::kLink);
  expect_rejected(
      "enc.yaml", with_position_source("encoder"), "imperfections.sensor.position_source");
}

TEST(TransmissionConfig, OutOfRangeValuesAreRejectedNamingTheKey) {
  const std::string k = "imperfections.transmission.";
  expect_rejected("t1_zero.yaml", with_transmission("    torque_t1_nm: 0\n"), k + "torque_t1_nm");
  expect_rejected("t1_t2.yaml", with_transmission("    torque_t1_nm: 50\n"), k + "torque_t");
  expect_rejected("damp.yaml", with_transmission("    damping_ratio: -0.1\n"), k + "damping_ratio");
  expect_rejected(
      "lost.yaml", with_transmission("    lost_motion_rad: -1e-4\n"), k + "lost_motion_rad");
  expect_rejected("ratio.yaml", with_transmission("    ratio: 0\n"), k + "ratio");
  expect_rejected(
      "k3.yaml", with_transmission("    stiffness_k3_nm_rad: -1\n"), k + "stiffness_k3");
  expect_rejected(
      "unknown.yaml", with_transmission("    stiffness_k4_nm_rad: 1\n"), "stiffness_k4");
}

TEST(TransmissionConfig, LostMotionAndErrorAmplitudeAreSweepable) {
  const fs::path p = write_temp("sweep.yaml", with_transmission(""));
  const auto lost = load_imperfections_with<7>(p, "transmission.lost_motion_rad", 8.7e-4);
  ASSERT_TRUE(lost.is_ok()) << lost.reason();
  EXPECT_EQ(lost.value().transmission.lost_motion_rad, Vec::Constant(8.7e-4));
  const auto err = load_imperfections_with<7>(p, "transmission.error_amp_rad", 1.7e-3);
  ASSERT_TRUE(err.is_ok()) << err.reason();
  EXPECT_EQ(err.value().transmission.error_amp_rad, Vec::Constant(1.7e-3));
}

}  // namespace
}  // namespace cobot::pipeline
