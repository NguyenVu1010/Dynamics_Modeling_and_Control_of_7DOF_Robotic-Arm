#include "cobot/pipeline/sim_joint_interface.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "cobot/common/dof.hpp"

namespace cobot::pipeline {

template <int N>
SimJointInterface<N>::SimJointInterface(dynamics::SimPlant<N>& plant,
                                        double period_s,
                                        double jitter_std_s,
                                        std::uint64_t seed,
                                        double overrun_prob,
                                        int overrun_cycles)
    : plant_(plant),
      period_s_(period_s),
      jitter_std_s_(jitter_std_s),
      rng_(seed),
      overrun_prob_(overrun_prob),
      overrun_cycles_(overrun_cycles),
      overrun_rng_(seed ^ kOverrunSalt) {
  if (!(overrun_prob >= 0.0 && overrun_prob <= 1.0)) {
    throw std::invalid_argument("timing.overrun_prob must be in [0, 1], got " +
                                std::to_string(overrun_prob));
  }
  if (overrun_cycles < 0) {
    throw std::invalid_argument("timing.overrun_cycles must be >= 0, got " +
                                std::to_string(overrun_cycles));
  }
}

template <int N>
common::Status SimJointInterface<N>::read(hal::JointState<N>& out) {
  out.t = common::Timestamp{std::llround(plant_.time_s() * 1e9)};
  out.q = plant_.q();
  out.qd = plant_.qd();
  out.tau_measured = plant_.tau_applied();
  out.tau_valid = true;
  out.q_motor = plant_.q_motor();
  out.qd_motor = plant_.qd_motor();
  out.motor_valid = true;
  return common::Status::ok();
}

template <int N>
common::Status SimJointInterface<N>::write(const hal::JointCommand<N>& cmd) {
  last_cmd_ = cmd;
  return common::Status::ok();
}

template <int N>
void SimJointInterface<N>::advance() {
  double dt = period_s_;
  if (jitter_std_s_ > 0.0) {
    std::normal_distribution<double> jitter(0.0, jitter_std_s_);
    dt = std::clamp(period_s_ + jitter(rng_), 0.5 * period_s_, 1.5 * period_s_);
  }
  plant_.step(last_cmd_.tau, dt);
  if (overrun_prob_ > 0.0) {
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    if (unit(overrun_rng_) < overrun_prob_) {
      for (int k = 0; k < overrun_cycles_; ++k) {
        plant_.step(last_cmd_.tau, period_s_);
      }
    }
  }
}

template <int N>
SimTruthProbe<N>::SimTruthProbe(const dynamics::SimPlant<N>& plant) : plant_(plant) {}

template <int N>
const PlantTruth<N>& SimTruthProbe<N>::sample() {
  const double t = plant_.time_s();
  if (first_) {
    truth_.qdd.setZero();
  } else if (t != time_prev_s_) {
    truth_.qdd = (plant_.qd() - qd_prev_) / (t - time_prev_s_);
  }
  first_ = false;
  qd_prev_ = plant_.qd();
  time_prev_s_ = t;
  truth_.q = plant_.q();
  truth_.qd = plant_.qd();
  truth_.q_motor = plant_.q_motor();
  truth_.tau_motor = plant_.tau_applied();
  truth_.tau_friction = plant_.friction_torque();
  return truth_;
}

#define COBOT_PIPELINE_SIM_JOINT_INTERFACE_INSTANTIATE(N) \
  template class SimJointInterface<(N)>;                  \
  template class SimTruthProbe<(N)>;
COBOT_INSTANTIATE_DOF(COBOT_PIPELINE_SIM_JOINT_INTERFACE_INSTANTIATE)
#undef COBOT_PIPELINE_SIM_JOINT_INTERFACE_INSTANTIATE

}  // namespace cobot::pipeline
