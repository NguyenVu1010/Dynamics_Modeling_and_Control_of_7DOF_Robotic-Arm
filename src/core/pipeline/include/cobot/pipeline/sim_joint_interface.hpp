#pragma once

#include <cstdint>
#include <random>

#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/pipeline/sim_drive.hpp"

// JointInterface over dynamics::SimPlant (feature `control-loop`). write() stores the
// command; advance() steps the plant by the nominal period plus optional Gaussian jitter and
// stamps the state with the plant time. Overrun (feature comm-timing, bench only): one draw per
// advance(); with probability overrun_prob the plant then runs overrun_cycles more nominal
// periods with the same command. With a drive attached (attach_drive, feature drive-loop) every
// period, nominal and overrun, goes through SimDrive::advance; without one, write() of a position
// command is an error and the previous command is kept. Lives in pipeline so hal depends only on
// common.
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
  // Non-owning; nullptr detaches (the plant then gets the command torque directly).
  void attach_drive(SimDrive<N>* drive) { drive_ = drive; }

 private:
  void step_period(double dt);  // the plant (or the drive) over one period of dt s

  dynamics::SimPlant<N>& plant_;
  double period_s_;
  double jitter_std_s_;
  std::mt19937_64 rng_;
  double overrun_prob_;
  int overrun_cycles_;
  std::mt19937_64 overrun_rng_;
  hal::JointCommand<N> last_cmd_;
  SimDrive<N>* drive_ = nullptr;
};

// The plant truth of a bench run (feature log-channels): call sample() after the decorator's
// read() and before ControlLoop::step(). q, qd, q_motor are the plant's state, tau_motor its
// tau_applied() and tau_friction its friction_torque() (of the last plant step); qdd =
// (qd - qd_prev) / (time_s - time_prev) on the plant clock (an overrun lengthens the interval),
// 0 on the first call and kept when the plant clock has not moved. Allocates nothing.
template <int N>
class SimTruthProbe {
 public:
  explicit SimTruthProbe(const dynamics::SimPlant<N>& plant);

  const PlantTruth<N>& sample();

 private:
  const dynamics::SimPlant<N>& plant_;
  PlantTruth<N> truth_;
  common::VecN<N> qd_prev_ = common::VecN<N>::Zero();
  double time_prev_s_ = 0.0;
  bool first_ = true;
};

}  // namespace cobot::pipeline
