#include "cobot/pipeline/sim_drive.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "cobot/common/dof.hpp"

namespace cobot::pipeline {
namespace {

constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
constexpr double kRatioTol = 1e-9;    // relative, drive rate / control rate is an integer
constexpr double kMaxSubsteps = 1e6;  // keeps the count an int

int checked_substeps(double rate_hz, double control_period_s) {
  const double control_rate_hz = 1.0 / control_period_s;
  const int s = drive_substeps(rate_hz, control_rate_hz);
  if (s == 0) {
    std::array<char, 128> text{};
    std::snprintf(
        text.data(),
        text.size(),
        "cobot.drive.rate_hz: must be an integer multiple of control_rate_hz (%g), got %g",
        control_rate_hz,
        rate_hz);
    throw std::invalid_argument(text.data());
  }
  return s;
}

}  // namespace

int drive_substeps(double rate_hz, double control_rate_hz) {
  const double ratio = rate_hz / control_rate_hz;
  if (!(rate_hz > 0.0) || !(control_rate_hz > 0.0) || !std::isfinite(ratio)) {
    return 0;
  }
  const double s = std::round(ratio);
  if (s < 1.0 || std::abs(ratio - s) > kRatioTol * ratio || s > kMaxSubsteps) {
    return 0;
  }
  return static_cast<int>(s);
}

template <int N>
SimDrive<N>::SimDrive(const DriveConfig<N>& drive,
                      const common::VecN<N>& inertia_kg_m2,
                      const common::VecN<N>& peak_torque_nm,
                      const imperfect::ImperfectionConfig<N>& profile,
                      const common::VecN<N>& torque_scale,
                      double control_period_s,
                      double setpoint_period_s)
    : substeps_(checked_substeps(drive.rate_hz, control_period_s)),
      kp_(inertia_kg_m2.cwiseProduct(drive.omega_rad_s.cwiseProduct(drive.omega_rad_s))),
      kd_(2.0 * drive.zeta * inertia_kg_m2.cwiseProduct(drive.omega_rad_s)),
      peak_(peak_torque_nm),
      encoder_bits_(profile.sensor.encoder_bits),
      filter_hz_(drive.velocity_filter_hz),
      setpoint_period_s_(setpoint_period_s),
      stage_(profile.actuator,
             torque_scale,
             peak_torque_nm,
             control_period_s / static_cast<double>(substeps_)) {}

template <int N>
void SimDrive<N>::advance(dynamics::SimPlant<N>& plant,
                          const hal::JointCommand<N>& cmd,
                          double dt_s) {
  const double h = dt_s / static_cast<double>(substeps_);
  const double a = h / (h + 1.0 / (kTwoPi * filter_hz_));
  stage_.set_period(h);
  for (int k = 0; k < substeps_; ++k) {
    ++substeps_run_;
    if (cmd.mode != hal::DriveMode::kPosition) {
      latched_ = false;
      encoder_ready_ = false;
      plant.step(cmd.tau, h);
      continue;
    }
    if (!latched_ || cmd.seq != seq_) {
      latched_ = true;
      seq_ = cmd.seq;
      q_s_ = cmd.q;
      qd_s_ = cmd.qd;
      tau_s_ = cmd.tau;
      age_s_ = 0.0;
    }
    common::VecN<N> q_m = plant.q_motor();
    for (int j = 0; j < N; ++j) {
      if (encoder_bits_[j] > 0.0) {
        const double step = kTwoPi / std::exp2(encoder_bits_[j]);
        q_m[j] = std::round(q_m[j] / step) * step;
      }
    }
    if (encoder_ready_) {
      for (int j = 0; j < N; ++j) {
        v_[j] += a * ((q_m[j] - q_m_prev_[j]) / h - v_[j]);
      }
    } else {
      v_ = plant.qd_motor();
      encoder_ready_ = true;
    }
    q_m_prev_ = q_m;
    const double t = std::min(age_s_, setpoint_period_s_);
    const bool extrapolate = age_s_ < setpoint_period_s_;
    common::VecN<N> tau;
    for (int j = 0; j < N; ++j) {
      target_q_[j] = q_s_[j] + qd_s_[j] * t;
      const double qd_t = extrapolate ? qd_s_[j] : 0.0;
      const double raw = tau_s_[j] + kp_[j] * (target_q_[j] - q_m[j]) + kd_[j] * (qd_t - v_[j]);
      tau[j] = std::clamp(raw, -peak_[j], peak_[j]);
    }
    if (!tau.allFinite()) {
      tau.setZero();
    }
    const common::VecN<N> shaped = stage_.shape(tau, plant.q());
    const common::VecN<N> out = stage_.limit(shaped, plant.qd_motor());
    plant.step(out, h);
    age_s_ += h;
  }
}

#define COBOT_PIPELINE_SIM_DRIVE_INSTANTIATE(N) template class SimDrive<(N)>;
COBOT_INSTANTIATE_DOF(COBOT_PIPELINE_SIM_DRIVE_INSTANTIATE)
#undef COBOT_PIPELINE_SIM_DRIVE_INSTANTIATE

}  // namespace cobot::pipeline
