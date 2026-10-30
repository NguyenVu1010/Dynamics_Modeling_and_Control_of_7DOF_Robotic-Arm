#include "cobot/pipeline/sim_joint_interface.hpp"

#include <algorithm>
#include <cmath>

#include "cobot/common/dof.hpp"

namespace cobot::pipeline {

template <int N>
SimJointInterface<N>::SimJointInterface(dynamics::SimPlant<N>& plant,
                                        double period_s,
                                        double jitter_std_s,
                                        std::uint64_t seed)
    : plant_(plant), period_s_(period_s), jitter_std_s_(jitter_std_s), rng_(seed) {}

template <int N>
common::Status SimJointInterface<N>::read(hal::JointState<N>& out) {
  out.t = common::Timestamp{std::llround(plant_.time_s() * 1e9)};
  out.q = plant_.q();
  out.qd = plant_.qd();
  out.tau_measured = plant_.tau_applied();
  out.tau_valid = true;
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
}

#define COBOT_PIPELINE_SIM_JOINT_INTERFACE_INSTANTIATE(N) template class SimJointInterface<(N)>;
COBOT_INSTANTIATE_DOF(COBOT_PIPELINE_SIM_JOINT_INTERFACE_INSTANTIATE)
#undef COBOT_PIPELINE_SIM_JOINT_INTERFACE_INSTANTIATE

}  // namespace cobot::pipeline
