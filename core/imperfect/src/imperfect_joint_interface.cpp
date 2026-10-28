#include "cobot/imperfect/imperfect_joint_interface.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "cobot/common/dof.hpp"

namespace cobot::imperfect {
namespace {

constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
constexpr int kRingSize = kMaxDelayCycles + 1;

bool delay_in_range(int cycles) {
  return cycles >= 0 && cycles <= kMaxDelayCycles;
}

// Ring slot `age` entries behind `head` (age 0 = head itself).
int ring_index(int head, int age) {
  return (head - age + kRingSize) % kRingSize;
}

double sign(double x) {
  return static_cast<double>((x > 0.0) - (x < 0.0));
}

// Rounds x to the nearest multiple of step; step <= 0 leaves x unchanged.
double round_to(double x, double step) {
  return step > 0.0 ? std::round(x / step) * step : x;
}

}  // namespace

template <int N>
ImperfectJointInterface<N>::ImperfectJointInterface(JointInterface<N>& inner,
                                                    const ImperfectionConfig<N>& config,
                                                    double control_period_s,
                                                    bool apply_plant_effects)
    : inner_(inner),
      config_(config),
      period_s_(control_period_s),
      apply_plant_effects_(apply_plant_effects),
      rng_(config.seed) {
  if (!delay_in_range(config.sensor.sensor_delay_cycles)) {
    throw std::invalid_argument("imperfections.sensor.sensor_delay_cycles outside [0, 32]");
  }
  if (!delay_in_range(config.actuator.command_delay_cycles)) {
    throw std::invalid_argument("imperfections.actuator.command_delay_cycles outside [0, 32]");
  }
  if (!(control_period_s > 0.0)) {
    throw std::invalid_argument("ImperfectJointInterface: control period must be > 0 s");
  }
  // Fixed draw order (all offsets, then all scales) keeps a run reproducible from the seed.
  std::normal_distribution<double> unit(0.0, 1.0);
  for (int j = 0; j < N; ++j) {
    q_offset_[j] = config.sensor.q_offset_std_rad[j] * unit(rng_);
  }
  for (int j = 0; j < N; ++j) {
    torque_scale_[j] = 1.0 + config.actuator.torque_scale_std[j] * unit(rng_);
  }
}

template <int N>
Status ImperfectJointInterface<N>::read(JointState<N>& out) {
  JointState<N> fresh;
  Status status = inner_.read(fresh);
  if (!status.is_ok()) {
    return status;
  }
  q_last_true_ = fresh.q;

  const SensorConfig<N>& sensor = config_.sensor;
  const int k = reads_;
  state_head_ = (state_head_ + 1) % kRingSize;
  state_ring_[static_cast<std::size_t>(state_head_)] = fresh;
  const int age = k < sensor.sensor_delay_cycles ? k : sensor.sensor_delay_cycles;
  const JointState<N>& delayed =
      state_ring_[static_cast<std::size_t>(ring_index(state_head_, age))];
  if (reads_ <= kMaxDelayCycles) {
    ++reads_;  // saturates: only compared against 0 and delays <= kMaxDelayCycles
  }

  std::normal_distribution<double> unit(0.0, 1.0);
  common::VecN<N> q_out;
  common::VecN<N> qd_out;
  for (int j = 0; j < N; ++j) {
    // Each effect is applied only when configured, so an all-zero config is bit-identical
    // (adding +0.0 would turn an input of -0.0 into +0.0).
    double q = delayed.q[j];
    if (sensor.q_offset_std_rad[j] > 0.0) {
      q += q_offset_[j];
    }
    if (sensor.q_noise_std_rad[j] > 0.0) {
      q += sensor.q_noise_std_rad[j] * unit(rng_);
    }
    if (sensor.encoder_bits[j] > 0.0) {
      q = round_to(q, kTwoPi / std::exp2(sensor.encoder_bits[j]));
    }
    q_out[j] = q;

    if (sensor.qd_from_diff) {
      const double raw = k == 0 ? 0.0 : (q - q_prev_out_[j]) / period_s_;
      const double fc = sensor.qd_filter_hz[j];
      const double alpha = fc > 0.0 ? period_s_ / (period_s_ + 1.0 / (kTwoPi * fc)) : 1.0;
      qd_filtered_[j] += alpha * (raw - qd_filtered_[j]);
      qd_out[j] = qd_filtered_[j];
    } else {
      double qd = delayed.qd[j];
      if (sensor.qd_noise_std_rad_s[j] > 0.0) {
        qd += sensor.qd_noise_std_rad_s[j] * unit(rng_);
      }
      qd_out[j] = qd;
    }
  }
  q_prev_out_ = q_out;
  qd_last_out_ = qd_out;

  out.t = delayed.t;
  out.q = q_out;
  out.qd = qd_out;
  out.tau_measured = delayed.tau_measured;
  out.tau_valid = delayed.tau_valid;
  return status;
}

template <int N>
Status ImperfectJointInterface<N>::write(const JointCommand<N>& cmd) {
  const ActuatorConfig<N>& act = config_.actuator;
  JointCommand<N> shaped;
  for (int j = 0; j < N; ++j) {
    double tau = cmd.tau[j] * torque_scale_[j];
    if (act.torque_ripple_frac[j] != 0.0) {
      tau *= 1.0 +
             act.torque_ripple_frac[j] * std::sin(act.ripple_periods_per_rev[j] * q_last_true_[j]);
    }
    shaped.tau[j] = round_to(tau, act.torque_cmd_resolution_nm[j]);
  }

  const int k = writes_;
  cmd_head_ = (cmd_head_ + 1) % kRingSize;
  cmd_ring_[static_cast<std::size_t>(cmd_head_)] = shaped;
  if (writes_ <= kMaxDelayCycles) {
    ++writes_;  // saturates: only compared against delays <= kMaxDelayCycles
  }
  JointCommand<N> sent;  // zeros until command_delay_cycles writes have been made
  if (k >= act.command_delay_cycles) {
    sent = cmd_ring_[static_cast<std::size_t>(ring_index(cmd_head_, act.command_delay_cycles))];
  }

  if (apply_plant_effects_) {
    const FrictionConfig<N>& friction = config_.friction;
    for (int j = 0; j < N; ++j) {
      const double tau_m = act.motor_time_constant_s[j];
      if (tau_m > 0.0) {
        tau_lagged_[j] += (period_s_ / tau_m) * (sent.tau[j] - tau_lagged_[j]);
      } else {
        tau_lagged_[j] = sent.tau[j];
      }
      const double coulomb = friction.coulomb_nm[j];
      const double viscous = friction.viscous_nm_s_rad[j];
      if (coulomb != 0.0 || viscous != 0.0) {
        const double qd = qd_last_out_[j];
        sent.tau[j] = tau_lagged_[j] - (coulomb * sign(qd) + viscous * qd);
      } else {
        sent.tau[j] = tau_lagged_[j];  // no friction: keep the torque bit-identical
      }
    }
  }
  return inner_.write(sent);
}

#define COBOT_INSTANTIATE(N) template class ImperfectJointInterface<N>;
COBOT_INSTANTIATE_DOF(COBOT_INSTANTIATE)
#undef COBOT_INSTANTIATE

}  // namespace cobot::imperfect
