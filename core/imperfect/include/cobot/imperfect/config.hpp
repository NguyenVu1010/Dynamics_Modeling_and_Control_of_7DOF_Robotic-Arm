#pragma once

#include <cstdint>

#include "cobot/common/types.hpp"

// Imperfection parameters (design 11.1). Parsed from config/imperfections/*.yaml by
// pipeline::load_imperfections; all-zero (default) means an ideal robot.
// Standard deviations are of zero-mean Gaussians; "draw once" values are sampled at
// construction from the seed and kept for the whole run (calibration-type errors).
namespace cobot::imperfect {

using common::VecN;

template <int N>
struct SensorConfig {
  VecN<N> encoder_bits = VecN<N>::Zero();       // 0 = no quantisation; else step 2*pi / 2^bits
  VecN<N> q_noise_std_rad = VecN<N>::Zero();    // white noise per sample
  VecN<N> q_offset_std_rad = VecN<N>::Zero();   // calibration offset, drawn once
  bool qd_from_diff = false;                    // true: qd = low-passed difference of output q
  VecN<N> qd_filter_hz = VecN<N>::Constant(200.0);  // cut-off of that low-pass, Hz
  VecN<N> qd_noise_std_rad_s = VecN<N>::Zero();     // white noise per sample (qd_from_diff=false)
  int sensor_delay_cycles = 0;                  // measurement age in control cycles, >= 0
};

template <int N>
struct ActuatorConfig {
  VecN<N> torque_scale_std = VecN<N>::Zero();        // relative torque-constant error, drawn once
  VecN<N> torque_ripple_frac = VecN<N>::Zero();      // amplitude / |tau_cmd|
  VecN<N> ripple_periods_per_rev = VecN<N>::Constant(6.0);  // cycles per output revolution
  VecN<N> torque_cmd_resolution_nm = VecN<N>::Zero();  // 0 = continuous
  VecN<N> motor_time_constant_s = VecN<N>::Zero();     // first-order lag, 0 = none
  int command_delay_cycles = 0;                        // >= 0
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
  double payload_kg = 0.0;                        // point mass at the tip origin
};

struct TimingConfig {
  double period_jitter_std_us = 0.0;  // plant step = nominal dt + N(0, std), bench only
};

template <int N>
struct ImperfectionConfig {
  SensorConfig<N> sensor;
  ActuatorConfig<N> actuator;
  FrictionConfig<N> friction;
  ModelMismatchConfig<N> model;
  TimingConfig timing;
  std::uint64_t seed = 1;
};

}  // namespace cobot::imperfect
