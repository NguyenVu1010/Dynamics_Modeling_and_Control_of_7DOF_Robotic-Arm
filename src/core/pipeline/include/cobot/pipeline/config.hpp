#pragma once

#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>

#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/control/computed_torque.hpp"
#include "cobot/control/friction_compensation.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/path_validation.hpp"
#include "cobot/planning/clik.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"

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

// Where the loop's trajectories come from (docs/design/2026-10-29-waypoint-gui-design.md 6.3),
// YAML `command_source: config | external`:
//   kConfig    start() plans `trajectory` from the config (the MVP code path);
//   kExternal  start() holds the measured pose in standby, trajectories arrive through load().
enum class CommandSource { kConfig, kExternal };

// Tool centre point in the tip frame (link_7). YAML `tcp_offset: {position_m: [3], rpy_deg: [3]}`,
// R = Rz(yaw) Ry(pitch) Rx(roll); identity (no tool) by default.
struct TcpOffset {
  common::Pose T_tip_tcp;
};

// Torque levels per joint (feature joint-limits, design 2026-11-02-realism-2 G1 and section 5),
// YAML `limits: {rated_torque_nm: [N], peak_torque_nm: [N]}`. The section is optional: without
// it load_config fills both from the URDF <limit effort> (the momentary torque M, which the plant
// enforces). The controller clamps and validates paths at the peak torque (controller_model); the
// rated torque is the continuous level of the Summary metric rms_tau_ratio. Until load_config
// fills them (or when it cannot read the URDF) both are +inf, so controller_model keeps the URDF
// tau_max.
template <int N>
struct JointLimitsConfig {
  VecN<N> rated_torque_nm = VecN<N>::Constant(std::numeric_limits<double>::infinity());  // > 0
  VecN<N> peak_torque_nm =  // > 0 and <= the URDF effort M of the joint
      VecN<N>::Constant(std::numeric_limits<double>::infinity());
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
  // Optional sections (feature `loop-external`); a missing one keeps these defaults.
  CommandSource command_source = CommandSource::kConfig;
  planning::CartesianLimits cartesian_limits;  // auto segment durations of waypoint trajectories
  double stop_time_s = 0.5;                    // > 0, duration of the request_stop() ramp
  TcpOffset tcp_offset;
  // YAML {reach_pos_m, reach_rot_deg, joint_margin_deg, torque_ratio, max_stretch}; dt_s and
  // path_sample_dt_s keep their defaults. In every new section each sub-key is optional too: a
  // partial section (`cartesian_limits: {v_max_m_s: 0.3}`) keeps the defaults of the others.
  ValidationLimits path_validation;
  // Read after the URDF named above: load_config rejects peak > URDF effort, naming
  // "limits.peak_torque_nm[j]" (0-based j). Missing section: URDF effort for both.
  JointLimitsConfig<N> limits;
};

template <int N>
Result<Config<N>> load_config(const std::filesystem::path& yaml);

// The controller's model: `urdf_model` with tau_max_i = min(config.limits.peak_torque_nm_i,
// urdf tau_max_i), which is the peak torque P for a config from load_config (P <= M is checked
// there). Everything else is copied. Every place that builds the controller's model (the bench,
// the ros2_control plugin, path validation) uses it; the plant keeps the URDF model (M).
template <int N>
model::Model<N> controller_model(const model::Model<N>& urdf_model, const Config<N>& config);

// config/imperfections/*.yaml; `seed` overrides the file's seed when provided by the caller.
template <int N>
Result<imperfect::ImperfectionConfig<N>> load_imperfections(const std::filesystem::path& yaml);

}  // namespace cobot::pipeline
