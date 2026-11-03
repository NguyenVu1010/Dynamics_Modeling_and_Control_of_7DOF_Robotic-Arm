#include "cobot/imperfect/imperfect_joint_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "cobot/common/dof.hpp"

namespace cobot::imperfect {
namespace {

constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
constexpr int kRingSize = kMaxDelayCycles + 1;
constexpr double kDriveEfficiency = 0.85;  // electrical -> mechanical power, bus sag model
constexpr double kServoRatioTol = 1e-9;    // relative, control rate / servo rate is an integer

bool probability(double p) {
  return p >= 0.0 && p <= 1.0;
}

bool delay_in_range(int cycles) {
  return cycles >= 0 && cycles <= kMaxDelayCycles;
}

// Cycles per servo packet, 1 for servo_rate_hz 0; 0 when the ratio is not an integer >= 1.
std::int64_t servo_cycles(const CommConfig& comm, double period_s) {
  if (comm.servo_rate_hz == 0.0) {
    return 1;
  }
  const double ratio = 1.0 / (period_s * comm.servo_rate_hz);
  const std::int64_t s = std::llround(ratio);
  if (!(comm.servo_rate_hz > 0.0) || !std::isfinite(ratio) || s < 1 ||
      std::abs(ratio - static_cast<double>(s)) > kServoRatioTol * ratio) {
    return 0;
  }
  return s;
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

// Torque ceiling of the torque-speed curve at velocity w for torque tau: `peak` up to `knee`,
// linear to 0 at `zero` when motoring (tau * w > 0); braking is capped at `peak` only.
double torque_ceiling(double tau, double w, double peak, double knee, double zero) {
  const double speed = std::abs(w);
  if (tau * w <= 0.0 || speed <= knee) {
    return peak;
  }
  return speed < zero ? peak * (zero - speed) / (zero - knee) : 0.0;
}

}  // namespace

Status check_comm(const CommConfig& comm, double control_period_s) {
  if (!delay_in_range(comm.burst_cycles)) {
    return Status::error("imperfections.comm.burst_cycles outside [0, 32]");
  }
  if (!probability(comm.packet_loss_prob) || !probability(comm.latency_burst_prob)) {
    return Status::error(
        "imperfections.comm.packet_loss_prob and latency_burst_prob must be in [0, 1]");
  }
  if (!(control_period_s > 0.0) || servo_cycles(comm, control_period_s) == 0) {
    const double ratio = 1.0 / (control_period_s * comm.servo_rate_hz);
    return Status::error(
        "imperfections.comm.servo_rate_hz: control rate / servo rate must be an integer >= 1, "
        "got " +
        std::to_string(ratio));
  }
  return Status::ok();
}

template <int N>
ImperfectJointInterface<N>::ImperfectJointInterface(JointInterface<N>& inner,
                                                    const ImperfectionConfig<N>& config,
                                                    double control_period_s,
                                                    bool apply_plant_effects,
                                                    const common::VecN<N>& peak_torque_nm)
    : inner_(inner),
      config_(config),
      period_s_(control_period_s),
      apply_plant_effects_(apply_plant_effects),
      rng_(config.seed),
      peak_torque_(peak_torque_nm),
      bus_voltage_v_(config.actuator.bus.v_nom_v > 0.0 ? config.actuator.bus.v_nom_v : 1.0),
      comm_rng_(config.seed ^ kCommSalt) {
  if (!delay_in_range(config.sensor.sensor_delay_cycles)) {
    throw std::invalid_argument("imperfections.sensor.sensor_delay_cycles outside [0, 32]");
  }
  if (!delay_in_range(config.actuator.command_delay_cycles)) {
    throw std::invalid_argument("imperfections.actuator.command_delay_cycles outside [0, 32]");
  }
  if (!(control_period_s > 0.0)) {
    throw std::invalid_argument("ImperfectJointInterface: control period must be > 0 s");
  }
  const Status comm = check_comm(config.comm, control_period_s);
  if (!comm) {
    throw std::invalid_argument(comm.reason());
  }
  servo_cycles_ = servo_cycles(config.comm, control_period_s);
  for (int j = 0; j < N; ++j) {
    const double knee = config.actuator.knee_speed_rad_s[j];
    const std::string key = "imperfections.actuator.knee_speed_rad_s[" + std::to_string(j) + "]";
    if (knee > 0.0 && !std::isfinite(peak_torque_nm[j])) {
      throw std::invalid_argument(key + " is set but the peak torque is not finite");
    }
    if (knee > 0.0 && !(config.actuator.zero_torque_speed_rad_s[j] > knee)) {
      throw std::invalid_argument(key + " must be < zero_torque_speed_rad_s");
    }
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
bool ImperfectJointInterface<N>::comm_fresh() {
  const CommConfig& comm = config_.comm;
  const std::int64_t k = cycle_++;
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  bool received = false;
  if (k % servo_cycles_ == 0) {
    received = !(comm.packet_loss_prob > 0.0 && unit(comm_rng_) < comm.packet_loss_prob);
    if (!received) {
      ++packets_lost_;
      last_lost_cycle_ = k;
    }
  }
  forward_ = received;
  bool delayed = false;
  if (burst_left_ > 0) {
    --burst_left_;
    delayed = true;
  } else if (comm.burst_cycles > 0 && comm.latency_burst_prob > 0.0 &&
             unit(comm_rng_) < comm.latency_burst_prob) {
    burst_left_ = comm.burst_cycles;  // the next burst_cycles reads repeat this one
  }
  return !have_out_ || (received && !delayed);
}

template <int N>
Status ImperfectJointInterface<N>::read(JointState<N>& out) {
  JointState<N> fresh;
  Status status = inner_.read(fresh);
  if (!status.is_ok()) {
    return status;
  }
  q_last_true_ = fresh.q;
  // The torque-speed ceiling and the bus power see the motor speed when it is measured.
  qd_last_true_ = fresh.motor_valid ? fresh.qd_motor : fresh.qd;

  const SensorConfig<N>& sensor = config_.sensor;
  const int k = reads_;
  state_head_ = (state_head_ + 1) % kRingSize;
  state_ring_[static_cast<std::size_t>(state_head_)] = fresh;
  const int age = k < sensor.sensor_delay_cycles ? k : sensor.sensor_delay_cycles;
  const JointState<N>& delayed =
      state_ring_[static_cast<std::size_t>(ring_index(state_head_, age))];
  if (reads_ <= kMaxDelayCycles) {
    ++reads_;  // saturates: only compared against delays <= kMaxDelayCycles
  }
  if (!comm_fresh()) {
    out = last_out_;  // held or lost: the previous output, stamp unchanged
    return status;
  }
  // Time since the previous processed sample (one period without comm effects).
  const double dt = static_cast<double>(cycle_ - 1 - out_cycle_) * period_s_;
  out_cycle_ = cycle_ - 1;

  std::normal_distribution<double> unit(0.0, 1.0);
  common::VecN<N> q_out;
  common::VecN<N> qd_out;
  const bool motor = sensor.position_source == PositionSource::kMotor && delayed.motor_valid;
  const common::VecN<N>& q_raw = motor ? delayed.q_motor : delayed.q;
  const common::VecN<N>& qd_raw = motor ? delayed.qd_motor : delayed.qd;
  for (int j = 0; j < N; ++j) {
    // Each effect is applied only when configured, so an all-zero config is bit-identical
    // (adding +0.0 would turn an input of -0.0 into +0.0).
    double q = q_raw[j];
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
      const double raw = have_out_ ? (q - q_prev_out_[j]) / dt : 0.0;
      const double fc = sensor.qd_filter_hz[j];
      const double alpha = fc > 0.0 ? dt / (dt + 1.0 / (kTwoPi * fc)) : 1.0;
      qd_filtered_[j] += alpha * (raw - qd_filtered_[j]);
      qd_out[j] = qd_filtered_[j];
    } else {
      double qd = qd_raw[j];
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
  out.q_motor = delayed.q_motor;
  out.qd_motor = delayed.qd_motor;
  out.motor_valid = delayed.motor_valid;
  out.q_from_motor = motor;
  last_out_ = out;
  have_out_ = true;
  return status;
}

template <int N>
Status ImperfectJointInterface<N>::write(const JointCommand<N>& cmd) {
  const ActuatorConfig<N>& act = config_.actuator;
  JointCommand<N> shaped;
  for (int j = 0; j < N; ++j) {
    double tau = cmd.tau[j] * torque_scale_[j];
    if (act.torque_scale_asym_frac[j] != 0.0 && tau < 0.0) {
      tau *= 1.0 - act.torque_scale_asym_frac[j];
    }
    if (act.torque_ripple_frac[j] != 0.0) {
      tau *= 1.0 +
             act.torque_ripple_frac[j] * std::sin(act.ripple_periods_per_rev[j] * q_last_true_[j]);
    }
    if (std::abs(tau) < act.deadband_nm[j]) {
      tau = 0.0;
    }
    shaped.tau[j] = round_to(tau, act.torque_cmd_resolution_nm[j]);
  }
  // Servo hold: only a received packet forwards the command; otherwise the last one is re-sent.
  if (forward_) {
    held_cmd_ = shaped;
  }

  const int k = writes_;
  cmd_head_ = (cmd_head_ + 1) % kRingSize;
  cmd_ring_[static_cast<std::size_t>(cmd_head_)] = held_cmd_;
  if (writes_ <= kMaxDelayCycles) {
    ++writes_;  // saturates: only compared against delays <= kMaxDelayCycles
  }
  JointCommand<N> sent;  // zeros until command_delay_cycles writes have been made
  if (k >= act.command_delay_cycles) {
    sent = cmd_ring_[static_cast<std::size_t>(ring_index(cmd_head_, act.command_delay_cycles))];
  }

  // Bus voltage from the previous cycle's output torques (no algebraic loop); off: V = V_nom = 1.
  const double v_nom = act.bus.v_nom_v > 0.0 ? act.bus.v_nom_v : 1.0;
  double v_bus = v_nom;
  if (act.bus.v_nom_v > 0.0) {
    double power_w = 0.0;
    for (int j = 0; j < N; ++j) {
      power_w += std::max(tau_prev_out_[j] * qd_last_true_[j], 0.0);
    }
    const double disc = v_nom * v_nom - 4.0 * act.bus.r_source_ohm * power_w / kDriveEfficiency;
    v_bus =
        disc < 0.0 ? act.bus.v_min_v : std::max(act.bus.v_min_v, (v_nom + std::sqrt(disc)) / 2.0);
  }
  bus_voltage_v_ = v_bus;
  for (int j = 0; j < N; ++j) {
    double tau = sent.tau[j];
    const double slew = act.torque_slew_nm_s[j];
    if (slew > 0.0) {
      const double step = slew * period_s_;
      tau = tau_prev_out_[j] + std::clamp(tau - tau_prev_out_[j], -step, step);
    }
    const double knee = act.knee_speed_rad_s[j];
    if (knee > 0.0) {
      const double ceiling = torque_ceiling(tau,
                                            qd_last_true_[j],
                                            peak_torque_[j],
                                            knee * v_bus / v_nom,
                                            act.zero_torque_speed_rad_s[j] * v_bus / v_nom);
      tau = std::copysign(std::min(std::abs(tau), ceiling), tau);
    }
    sent.tau[j] = tau;
    tau_prev_out_[j] = tau;
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
