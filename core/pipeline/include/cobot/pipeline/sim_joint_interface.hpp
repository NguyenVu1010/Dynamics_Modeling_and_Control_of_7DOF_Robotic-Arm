#pragma once

#include <random>

#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/hal/joint_interface.hpp"

// JointInterface over dynamics::SimPlant (feature `control-loop`). write() stores the
// command; advance() steps the plant by the nominal period plus optional Gaussian jitter and
// stamps the state with the plant time. Lives in pipeline so hal depends only on common.
namespace cobot::pipeline {

template <int N>
class SimJointInterface final : public hal::JointInterface<N> {
 public:
  SimJointInterface(dynamics::SimPlant<N>& plant, double period_s, double jitter_std_s = 0.0,
                    std::uint64_t seed = 1);

  common::Status read(hal::JointState<N>& out) override;
  common::Status write(const hal::JointCommand<N>& cmd) override;
  // Advance the plant one control period with the last written command.
  void advance();

 private:
  dynamics::SimPlant<N>& plant_;
  double period_s_;
  double jitter_std_s_;
  std::mt19937_64 rng_;
  hal::JointCommand<N> last_cmd_;
};

}  // namespace cobot::pipeline
