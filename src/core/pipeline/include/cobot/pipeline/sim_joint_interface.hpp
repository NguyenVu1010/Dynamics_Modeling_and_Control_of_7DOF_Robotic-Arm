#pragma once

#include <cstdint>
#include <random>

#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/hal/joint_interface.hpp"

// JointInterface over dynamics::SimPlant (feature `control-loop`). write() stores the
// command; advance() steps the plant by the nominal period plus optional Gaussian jitter and
// stamps the state with the plant time. Overrun (feature comm-timing, bench only): one draw per
// advance(); with probability overrun_prob the plant then runs overrun_cycles more nominal
// periods with the same command. Lives in pipeline so hal depends only on common.
namespace cobot::pipeline {

inline constexpr std::uint64_t kOverrunSalt = 0x0E7A'0002;  // seed ^ salt: overrun generator

template <int N>
class SimJointInterface final : public hal::JointInterface<N> {
 public:
  // Throws std::invalid_argument naming timing.overrun_prob when it is outside [0, 1] and
  // timing.overrun_cycles when it is < 0.
  SimJointInterface(dynamics::SimPlant<N>& plant,
                    double period_s,
                    double jitter_std_s = 0.0,
                    std::uint64_t seed = 1,
                    double overrun_prob = 0.0,
                    int overrun_cycles = 0);

  common::Status read(hal::JointState<N>& out) override;
  common::Status write(const hal::JointCommand<N>& cmd) override;
  // Advance the plant one control period (more on an overrun) with the last written command.
  void advance();

 private:
  dynamics::SimPlant<N>& plant_;
  double period_s_;
  double jitter_std_s_;
  std::mt19937_64 rng_;
  double overrun_prob_;
  int overrun_cycles_;
  std::mt19937_64 overrun_rng_;
  hal::JointCommand<N> last_cmd_;
};

}  // namespace cobot::pipeline
