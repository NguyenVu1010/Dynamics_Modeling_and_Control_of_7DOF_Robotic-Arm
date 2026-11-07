#pragma once

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include <gtest/gtest.h>

#include "cobot/pipeline/config.hpp"

// Helpers shared by tests/unit/pipeline_hardware_model and tests/unit/pipeline_task: line edits on
// YAML fixture text, one output directory per test, and a field-for-field Config comparison.
namespace cobot::pipeline::hw_test {

namespace fs = std::filesystem;

inline constexpr int kN = 7;
inline const fs::path kRepo = COBOT_REPO_DIR;

inline std::string read_text(const fs::path& p) {
  std::ifstream in(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

inline bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

// A directory of its own for the running test (ctest runs tests in parallel processes).
inline fs::path test_dir() {
  const ::testing::TestInfo* info = ::testing::UnitTest::GetInstance()->current_test_info();
  std::string name = std::string(info->test_suite_name()) + "." + info->name();
  for (char& c : name) {
    if (c == '/') {
      c = '_';
    }
  }
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "hardware_catalog" / name;
  fs::create_directories(dir);
  return dir;
}

inline fs::path write_text(const fs::path& p, const std::string& text) {
  fs::create_directories(p.parent_path());
  std::ofstream(p, std::ios::binary) << text;
  return p;
}

// Start of the first line of `text` that begins with `prefix`; npos when there is none.
inline std::size_t line_start(const std::string& text, const std::string& prefix) {
  std::size_t begin = 0;
  while (begin < text.size()) {
    if (text.compare(begin, prefix.size(), prefix) == 0) {
      return begin;
    }
    const std::size_t end = text.find('\n', begin);
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1;
  }
  return std::string::npos;
}

inline std::size_t line_end(const std::string& text, std::size_t begin) {
  const std::size_t end = text.find('\n', begin);
  return end == std::string::npos ? text.size() : end + 1;
}

// The whole first line starting with `prefix` replaced by `line`.
inline std::string set_line(std::string text, const std::string& prefix, const std::string& line) {
  const std::size_t begin = line_start(text, prefix);
  if (begin == std::string::npos) {
    ADD_FAILURE() << "fixture line not found: " << prefix;
    return text;
  }
  text.replace(begin, line_end(text, begin) - begin, line + "\n");
  return text;
}

// `line` added below the first line starting with `prefix`.
inline std::string insert_after(std::string text,
                                const std::string& prefix,
                                const std::string& line) {
  const std::size_t begin = line_start(text, prefix);
  if (begin == std::string::npos) {
    ADD_FAILURE() << "fixture line not found: " << prefix;
    return text;
  }
  text.insert(line_end(text, begin), line + "\n");
  return text;
}

// The first line starting with `prefix` and the deeper-indented lines below it replaced by `line`
// (empty: removed).
inline std::string set_block(std::string text, const std::string& prefix, const std::string& line) {
  const std::size_t begin = line_start(text, prefix);
  if (begin == std::string::npos) {
    ADD_FAILURE() << "fixture line not found: " << prefix;
    return text;
  }
  const std::size_t indent = prefix.find_first_not_of(' ');
  std::size_t end = line_end(text, begin);
  while (end < text.size()) {
    const std::size_t first = text.find_first_not_of(' ', end);
    if (first == std::string::npos || first - end <= indent) {
      break;
    }
    end = line_end(text, end);
  }
  text.replace(begin, end - begin, line.empty() ? line : line + "\n");
  return text;
}

inline std::string drop_block(const std::string& text, const std::string& prefix) {
  return set_block(text, prefix, "");
}

inline bool same_path(const fs::path& a, const fs::path& b) {
  return fs::weakly_canonical(a) == fs::weakly_canonical(b);
}

// Every field of Config, exact. Paths compare canonically: a model reaches its controller file as
// config/hardware/../<file>, so the URDF path differs lexically but names the same file.
template <int N>
void expect_same_config(const Config<N>& a, const Config<N>& b) {
  EXPECT_TRUE(same_path(a.urdf_path, b.urdf_path)) << a.urdf_path << " / " << b.urdf_path;
  EXPECT_EQ(a.base_link, b.base_link);
  EXPECT_EQ(a.tip_link, b.tip_link);
  EXPECT_TRUE(a.gravity_b == b.gravity_b);
  EXPECT_EQ(a.control_rate_hz, b.control_rate_hz);
  EXPECT_EQ(a.deterministic, b.deterministic);
  EXPECT_TRUE(a.home_q_rad == b.home_q_rad);

  EXPECT_TRUE(a.trajectory.goal_position_m == b.trajectory.goal_position_m);
  EXPECT_TRUE(a.trajectory.goal_rpy_rad == b.trajectory.goal_rpy_rad);
  EXPECT_EQ(a.trajectory.duration_s, b.trajectory.duration_s);
  EXPECT_EQ(a.trajectory.hold_s, b.trajectory.hold_s);

  EXPECT_TRUE(a.clik.kp == b.clik.kp);
  EXPECT_TRUE(a.clik.kd == b.clik.kd);
  EXPECT_EQ(a.clik.k_null, b.clik.k_null);
  EXPECT_EQ(a.clik.k_damp, b.clik.k_damp);
  EXPECT_EQ(a.clik.lambda_max, b.clik.lambda_max);
  EXPECT_EQ(a.clik.cond_threshold, b.clik.cond_threshold);

  EXPECT_TRUE(a.computed_torque.kp == b.computed_torque.kp);
  EXPECT_TRUE(a.computed_torque.kd == b.computed_torque.kd);
  EXPECT_TRUE(a.algorithm == b.algorithm);
  EXPECT_TRUE(a.pd_gravity.kp == b.pd_gravity.kp);
  EXPECT_TRUE(a.pd_gravity.kd == b.pd_gravity.kd);

  EXPECT_EQ(a.drive.rate_hz, b.drive.rate_hz);
  EXPECT_TRUE(a.drive.omega_rad_s == b.drive.omega_rad_s);
  EXPECT_EQ(a.drive.zeta, b.drive.zeta);
  EXPECT_EQ(a.drive.velocity_filter_hz, b.drive.velocity_filter_hz);
  EXPECT_TRUE(a.drive.torque_feedforward == b.drive.torque_feedforward);

  EXPECT_EQ(a.friction_compensation.enabled, b.friction_compensation.enabled);
  EXPECT_TRUE(a.friction_compensation.coulomb_nm == b.friction_compensation.coulomb_nm);
  EXPECT_TRUE(a.friction_compensation.viscous_nm_s_rad == b.friction_compensation.viscous_nm_s_rad);
  EXPECT_EQ(a.friction_compensation.velocity_eps_rad_s, b.friction_compensation.velocity_eps_rad_s);

  EXPECT_EQ(a.safety.soft_margin_rad, b.safety.soft_margin_rad);
  EXPECT_EQ(a.safety.hal_timeout_s, b.safety.hal_timeout_s);

  EXPECT_TRUE(same_path(a.log_output_dir, b.log_output_dir));
  EXPECT_TRUE(a.log_mode == b.log_mode);
  EXPECT_EQ(a.log_capacity_s, b.log_capacity_s);
  EXPECT_EQ(a.log_lock_memory, b.log_lock_memory);

  EXPECT_TRUE(a.command_source == b.command_source);
  EXPECT_EQ(a.cartesian_limits.v_max_m_s, b.cartesian_limits.v_max_m_s);
  EXPECT_EQ(a.cartesian_limits.a_max_m_s2, b.cartesian_limits.a_max_m_s2);
  EXPECT_EQ(a.cartesian_limits.w_max_rad_s, b.cartesian_limits.w_max_rad_s);
  EXPECT_EQ(a.cartesian_limits.alpha_max_rad_s2, b.cartesian_limits.alpha_max_rad_s2);
  EXPECT_EQ(a.stop_time_s, b.stop_time_s);
  EXPECT_TRUE(a.tcp_offset.T_tip_tcp.R == b.tcp_offset.T_tip_tcp.R);
  EXPECT_TRUE(a.tcp_offset.T_tip_tcp.p == b.tcp_offset.T_tip_tcp.p);

  EXPECT_EQ(a.path_validation.reach_pos_m, b.path_validation.reach_pos_m);
  EXPECT_EQ(a.path_validation.reach_rot_rad, b.path_validation.reach_rot_rad);
  EXPECT_EQ(a.path_validation.joint_margin_rad, b.path_validation.joint_margin_rad);
  EXPECT_EQ(a.path_validation.torque_ratio, b.path_validation.torque_ratio);
  EXPECT_EQ(a.path_validation.max_stretch, b.path_validation.max_stretch);
  EXPECT_EQ(a.path_validation.dt_s, b.path_validation.dt_s);
  EXPECT_EQ(a.path_validation.path_sample_dt_s, b.path_validation.path_sample_dt_s);

  EXPECT_TRUE(a.limits.rated_torque_nm == b.limits.rated_torque_nm);
  EXPECT_TRUE(a.limits.peak_torque_nm == b.limits.peak_torque_nm);

  EXPECT_EQ(a.deflection_comp.enabled, b.deflection_comp.enabled);
  EXPECT_TRUE(a.deflection_comp.stiffness_k1_nm_rad == b.deflection_comp.stiffness_k1_nm_rad);
  EXPECT_TRUE(a.deflection_comp.stiffness_k2_nm_rad == b.deflection_comp.stiffness_k2_nm_rad);
  EXPECT_TRUE(a.deflection_comp.stiffness_k3_nm_rad == b.deflection_comp.stiffness_k3_nm_rad);
  EXPECT_TRUE(a.deflection_comp.torque_t1_nm == b.deflection_comp.torque_t1_nm);
  EXPECT_TRUE(a.deflection_comp.torque_t2_nm == b.deflection_comp.torque_t2_nm);

  EXPECT_TRUE(a.manual.joint_a_max_rad_s2 == b.manual.joint_a_max_rad_s2);
  EXPECT_EQ(a.manual.tcp_v_max_m_s, b.manual.tcp_v_max_m_s);
  EXPECT_EQ(a.manual.tcp_w_max_rad_s, b.manual.tcp_w_max_rad_s);
  EXPECT_EQ(a.manual.tcp_a_max_m_s2, b.manual.tcp_a_max_m_s2);
  EXPECT_EQ(a.manual.tcp_alpha_max_rad_s2, b.manual.tcp_alpha_max_rad_s2);
  EXPECT_EQ(a.manual.deadman_s, b.manual.deadman_s);
  EXPECT_EQ(a.manual.jog_margin_rad, b.manual.jog_margin_rad);
  EXPECT_EQ(a.manual.joint_speed_fraction, b.manual.joint_speed_fraction);

  EXPECT_EQ(a.payload.kg, b.payload.kg);
  EXPECT_TRUE(a.payload.offset_m == b.payload.offset_m);
  EXPECT_EQ(a.payload.radius_m, b.payload.radius_m);
}

}  // namespace cobot::pipeline::hw_test
