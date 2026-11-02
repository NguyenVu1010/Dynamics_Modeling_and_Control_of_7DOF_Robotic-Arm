#pragma once

#include <cstdint>

#include "cobot/common/types.hpp"

// Imperfection parameters (design 11.1). Parsed from config/imperfections/*.yaml by
// pipeline::load_imperfections; all-zero (default) means an ideal robot.
// Standard deviations are of zero-mean Gaussians; "draw once" values are sampled at
// construction from the seed and kept for the whole run (calibration-type errors).
namespace cobot::imperfect {

using common::VecN;

// Which encoder gives the measured position and velocity (feature transmission).
enum class PositionSource { kLink, kMotor };

template <int N>
struct SensorConfig {
  VecN<N> encoder_bits = VecN<N>::Zero();           // 0 = no quantisation; else step 2*pi / 2^bits
  VecN<N> q_noise_std_rad = VecN<N>::Zero();        // white noise per sample
  VecN<N> q_offset_std_rad = VecN<N>::Zero();       // calibration offset, drawn once
  bool qd_from_diff = false;                        // true: qd = low-passed difference of output q
  VecN<N> qd_filter_hz = VecN<N>::Constant(200.0);  // cut-off of that low-pass, Hz
  VecN<N> qd_noise_std_rad_s = VecN<N>::Zero();     // white noise per sample (qd_from_diff=false)
  int sensor_delay_cycles = 0;                      // measurement age in control cycles, >= 0
  // kMotor: q_motor / qd_motor are the raw position and velocity when the inner interface
  // reports motor_valid, else the link values. YAML `position_source: link | motor`.
  PositionSource position_source = PositionSource::kLink;
};

template <int N>
struct ActuatorConfig {
  VecN<N> torque_scale_std = VecN<N>::Zero();    // relative torque-constant error, drawn once
  VecN<N> torque_ripple_frac = VecN<N>::Zero();  // amplitude / |tau_cmd|
  VecN<N> ripple_periods_per_rev = VecN<N>::Constant(6.0);  // cycles per output revolution
  VecN<N> torque_cmd_resolution_nm = VecN<N>::Zero();       // 0 = continuous
  VecN<N> motor_time_constant_s = VecN<N>::Zero();          // first-order lag, 0 = none
  int command_delay_cycles = 0;                             // >= 0
  // Actuator limits (realism-2 design 6.1); every value 0 = effect off.
  VecN<N> torque_scale_asym_frac = VecN<N>::Zero();   // negative gain = positive * (1 - asym)
  VecN<N> deadband_nm = VecN<N>::Zero();              // |tau| below it -> 0
  VecN<N> torque_slew_nm_s = VecN<N>::Zero();         // |d tau_out / dt| limit
  VecN<N> knee_speed_rad_s = VecN<N>::Zero();         // torque-speed curve: full peak up to here
  VecN<N> zero_torque_speed_rad_s = VecN<N>::Zero();  // ... falling linearly to 0 here
  // Shared supply sag: V from the previous cycle's motoring power; v_nom_v 0 = off.
  struct Bus {
    double v_nom_v = 0.0;       // V
    double r_source_ohm = 0.0;  // ohm
    double v_min_v = 0.0;       // V, lower clamp of the bus voltage
  } bus;
};

template <int N>
struct FrictionConfig {
  VecN<N> coulomb_nm = VecN<N>::Zero();
  VecN<N> viscous_nm_s_rad = VecN<N>::Zero();
};

template <int N>
struct ModelMismatchConfig {
  VecN<N> link_mass_scale_std = VecN<N>::Zero();  // relative, drawn once per link
  double com_shift_std_m = 0.0;                   // per axis, drawn once per link
  double payload_kg = 0.0;                        // added to the last link, kg
  // Payload centre of mass in the tip (link_7) frame, measured from the tip origin, m.
  common::Vec3 payload_offset_m = common::Vec3::Zero();
  double payload_radius_m = 0.0;  // solid sphere, I = 2/5 m r^2 about its centre; >= 0
};

struct TimingConfig {
  double period_jitter_std_us = 0.0;  // plant step = nominal dt + N(0, std), bench only
  // Bench only: with this probability per plant advance, the plant runs 1 + overrun_cycles
  // periods with the last command (the control step came too late).
  double overrun_prob = 0.0;  // [0, 1]
  int overrun_cycles = 0;     // >= 0
};

// Servo bus timing (realism-2 design 5 and 6.3), YAML `comm`, optional group; all 0 = off.
struct CommConfig {
  // 0 = a servo packet every control cycle; else control_rate_hz / servo_rate_hz must be an
  // integer >= 1 and a packet comes every that many cycles (zero-order hold in between).
  double servo_rate_hz = 0.0;
  double packet_loss_prob = 0.0;    // per servo packet (both directions), [0, 1]
  double latency_burst_prob = 0.0;  // per control cycle while no burst is active, [0, 1]
  int burst_cycles = 0;             // reads that repeat the last output, [0, kMaxDelayCycles]
};

// Elastic Harmonic Drive transmission (realism-2 design 6.2), YAML `transmission`, optional
// group; bench only. Stiffness in three segments split at torques t1, t2 (catalog K1/K2/K3,
// T1/T2); k3 == 0 is a rigid joint and its other values are ignored.
template <int N>
struct TransmissionConfig {
  VecN<N> stiffness_k1_nm_rad = VecN<N>::Zero();  // N m / rad, >= 0
  VecN<N> stiffness_k2_nm_rad = VecN<N>::Zero();
  VecN<N> stiffness_k3_nm_rad = VecN<N>::Zero();  // 0: rigid joint
  VecN<N> torque_t1_nm = VecN<N>::Zero();         // N m, 0 < t1 < t2 when elastic
  VecN<N> torque_t2_nm = VecN<N>::Zero();
  VecN<N> damping_ratio = VecN<N>::Constant(0.01);  // of 2 sqrt(k1 * B_j), >= 0
  VecN<N> error_amp_rad = VecN<N>::Zero();          // transmission error amplitude
  VecN<N> lost_motion_rad = VecN<N>::Zero();        // total width of the dead zone, >= 0
  VecN<N> ratio = VecN<N>::Constant(100.0);         // gear ratio, > 0
};

template <int N>
struct ImperfectionConfig {
  SensorConfig<N> sensor;
  ActuatorConfig<N> actuator;
  FrictionConfig<N> friction;
  ModelMismatchConfig<N> model;
  TimingConfig timing;
  CommConfig comm;
  TransmissionConfig<N> transmission;
  std::uint64_t seed = 1;
};

}  // namespace cobot::imperfect
