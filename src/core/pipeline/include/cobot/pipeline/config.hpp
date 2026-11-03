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

// Static deflection compensation (feature deflection-comp, realism-2 design 4.7, ADR-019), YAML
// `deflection_comp: {enabled, stiffness_k1_nm_rad, stiffness_k2_nm_rad, stiffness_k3_nm_rad,
// torque_t1_nm, torque_t2_nm}` (every key required in the section, N values each). The section is
// optional: missing means disabled. Nominal Harmonic Drive values on the controller side, kept
// separate from the plant's imperfections.transmission so a mismatch can be studied. When enabled
// every value is > 0 and t1 < t2 per joint.
//
// Applied by ControlLoop::step only when enabled and the state has q_from_motor: the loop works
// on q_hat_j = q_j - harmonic_spring_deflection(g_j(q), k1_j, k2_j, k3_j, t1_j, t2_j), g the
// gravity torque of the controller's model at the measured q; qd is unchanged. Not compensated:
// transmission error, lost motion and dynamic (inertial, friction) deflection.
template <int N>
struct DeflectionCompConfig {
  bool enabled = false;
  VecN<N> stiffness_k1_nm_rad = VecN<N>::Zero();  // N m/rad
  VecN<N> stiffness_k2_nm_rad = VecN<N>::Zero();  // N m/rad
  VecN<N> stiffness_k3_nm_rad = VecN<N>::Zero();  // N m/rad
  VecN<N> torque_t1_nm = VecN<N>::Zero();         // N m
  VecN<N> torque_t2_nm = VecN<N>::Zero();         // N m
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
  DeflectionCompConfig<N> deflection_comp;  // optional section: disabled
};

template <int N>
Result<Config<N>> load_config(const std::filesystem::path& yaml);

// load_config with the scalar `cobot.<section>.<key>` replaced by `level` before validation; `key`
// is the sweep param name `config.<section>.<key>` (e.g. config.safety.hal_timeout_ms, in the
// file's unit). A name that is not a scalar of the file is an error naming it.
template <int N>
Result<Config<N>> load_config_with(const std::filesystem::path& yaml,
                                   const std::string& key,
                                   double level);

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
