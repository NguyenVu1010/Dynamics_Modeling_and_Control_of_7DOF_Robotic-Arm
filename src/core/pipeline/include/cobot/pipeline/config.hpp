#pragma once

#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>

#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/control/algorithm.hpp"
#include "cobot/control/computed_torque.hpp"
#include "cobot/control/friction_compensation.hpp"
#include "cobot/control/pd_gravity.hpp"
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

// What the hosts record (YAML cobot.log.mode: off | recorder | recorder_live; feature
// pipeline-recorder, ADR-020). kRecorderLive behaves as kRecorder until the studio bridge exists.
enum class LogMode { kOff, kRecorder, kRecorderLive };

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

inline constexpr double kDeg = 3.14159265358979323846 / 180.0;  // rad per degree

// Manual commands of the studio (ADR-022), YAML `manual: {joint_a_max_rad_s2 (N values or a
// scalar), tcp_v_max_m_s, tcp_w_max_rad_s, tcp_a_max_m_s2, tcp_alpha_max_rad_s2, deadman_ms,
// jog_margin_deg}`. Optional section, every sub-key optional: a missing one keeps these defaults.
// "At 100 %": every command passes a speed fraction in (0, 1] that scales velocities and
// accelerations.
template <int N>
struct ManualConfig {
  VecN<N> joint_a_max_rad_s2 =
      VecN<N>::Constant(5.0);         // > 0; joint velocity limit: joint_speed_fraction * qd_max
  double tcp_v_max_m_s = 0.25;        // > 0
  double tcp_w_max_rad_s = 1.0;       // > 0
  double tcp_a_max_m_s2 = 0.5;        // > 0
  double tcp_alpha_max_rad_s2 = 2.0;  // > 0
  double deadman_s = 0.1;             // YAML deadman_ms, > 0: jog without refresh ramps to rest
  // YAML jog_margin_deg, >= 0: jog/move stop this far INSIDE the safety band, i.e. at
  // soft_margin_rad + jog_margin_rad from a URDF limit (plan ambiguity 1).
  double jog_margin_rad = 2.0 * kDeg;
  // YAML joint_speed_fraction, in (0, 1] (feature jog-headroom): joint jog and joint move use at
  // most this fraction of the URDF qd_max at 100 % speed, leaving headroom below the overspeed
  // hold |qd| > qd_max that the tracked qd would otherwise trip by overshooting a target of qd_max.
  double joint_speed_fraction = 0.9;
};

// The simulated joint drive of the algorithm drive_position (feature drive-loop, ADR-023), YAML
// `drive: {rate_hz, omega_rad_s, zeta, velocity_filter_hz, torque_feedforward}`, every key required
// in the section; the section is required when cobot.control.algorithm is drive_position (parsed
// and checked, unused, otherwise). Gains computed by the drive (pipeline::SimDrive):
// kp_j = J_j omega_j^2 (N m/rad), kd_j = 2 zeta J_j omega_j (N m s/rad), J = diag(M(home_q)) of
// the controller model.
template <int N>
struct DriveConfig {
  double rate_hz = 4000.0;  // > 0, rate_hz / control_rate_hz an integer >= 1 (1e-9 relative)
  VecN<N> omega_rad_s = VecN<N>::Constant(60.0);  // > 0, N values or a scalar: loop bandwidth
  double zeta = 1.0;                              // > 0, damping ratio
  double velocity_filter_hz = 500.0;              // > 0, low-pass of the encoder difference
  control::DriveFeedForward torque_feedforward = control::DriveFeedForward::kInverseDynamics;
};

// rate_hz / control_rate_hz when it is an integer >= 1 (1e-9 relative), else 0: the drive
// sub-steps per control period (rule of cobot.drive.rate_hz).
int drive_substeps(double rate_hz, double control_rate_hz);
// Reason text when drive_substeps is 0, without the key prefix: "must be an integer multiple of
// control_rate_hz (<c>), got <r>". The loader and pipeline::SimDrive both report it.
std::string drive_rate_reason(double rate_hz, double control_rate_hz);

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
  // Tracking law (ADR-023), YAML cobot.control.algorithm (optional, computed_torque); a name
  // outside control::kAlgorithms is an error listing the registered names.
  control::Algorithm algorithm = control::Algorithm::kComputedTorque;
  // YAML cobot.pd_gravity: {kp, kd} (1/s^2, 1/s; N values or a scalar, > 0). Required when the
  // algorithm is pd_gravity; parsed and checked (unused) otherwise.
  control::PdGravityGains<N> pd_gravity;
  DriveConfig<N> drive;  // YAML cobot.drive, see DriveConfig
  control::FrictionCompensationConfig<N> friction_compensation;  // optional section: disabled
  SafetyConfig safety;
  std::filesystem::path log_output_dir = "results";
  LogMode log_mode = LogMode::kRecorder;  // optional key cobot.log.mode
  double log_capacity_s = 60.0;           // optional key cobot.log.capacity_s, s, > 0
  bool log_lock_memory = false;           // optional key cobot.log.lock_memory (mlock the region)
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
  ManualConfig<N> manual;                   // optional section (feature manual-commands)
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
