#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/control/computed_torque.hpp"
#include "cobot/control/friction_compensation.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/planning/clik.hpp"

// Configuration (feature `pipeline-config`). pipeline is the only module that parses YAML
// (yaml-cpp). Unknown keys, missing keys, wrong types and out-of-range values are errors;
// the message lists every offending key with its full path (e.g. "cobot.clik.kp: expected 6
// values, got 5"). Angles given in YAML with the `_deg` suffix are converted to radians.
namespace cobot::pipeline {

using common::Result;
using common::Vec3;
using common::VecN;

struct TrajectoryConfig {
  Vec3 goal_position_m = Vec3::Zero();  // in b
  Vec3 goal_rpy_rad = Vec3::Zero();     // roll, pitch, yaw: R = Rz(yaw) Ry(pitch) Rx(roll)
  double duration_s = 10.0;             // > 0
  double hold_s = 2.0;                  // >= 0
};

struct SafetyConfig {
  double soft_margin_rad = 0.0524;  // distance to a joint limit that triggers hold
  double hal_timeout_s = 0.005;     // no fresh read for this long -> hold
};

template <int N>
struct Config {
  std::filesystem::path urdf_path;  // resolved relative to the YAML file's directory
  std::string base_link = "base_link";
  std::string tip_link = "link_7";
  Vec3 gravity_b = Vec3(0.0, 0.0, -9.80665);
  double control_rate_hz = 1000.0;  // > 0
  bool deterministic = true;
  VecN<N> home_q_rad = VecN<N>::Zero();
  TrajectoryConfig trajectory;
  planning::ClikConfig clik;
  control::ComputedTorqueGains<N> computed_torque;
  control::FrictionCompensationConfig<N> friction_compensation;  // optional section: disabled
  SafetyConfig safety;
  std::filesystem::path log_output_dir = "results";
};

template <int N>
Result<Config<N>> load_config(const std::filesystem::path& yaml);

// config/imperfections/*.yaml; `seed` overrides the file's seed when provided by the caller.
template <int N>
Result<imperfect::ImperfectionConfig<N>> load_imperfections(const std::filesystem::path& yaml);

}  // namespace cobot::pipeline
