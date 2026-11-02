#pragma once

#include <array>
#include <limits>
#include <random>

#include "cobot/hal/joint_interface.hpp"
#include "cobot/imperfect/config.hpp"

// Decorator that makes any JointInterface imperfect (feature `imperfect-io`, design 11.1).
// read():  delay -> offset -> noise -> quantisation of q; qd either noisy or from the
//          low-passed difference of the output q.
// write(): scale error -> ripple (phase from the last true q read from the inner interface)
//          -> deadband -> resolution rounding -> command delay -> slew limit -> torque-speed
//          ceiling (peak torque, knee and zero-torque speeds scaled by the bus voltage, which
//          sags with the previous cycle's motoring power; realism-2 design 6.1). Friction and motor
//          lag are plant effects (dynamics::PlantEffects) in bench; on the Gazebo path they are
//          approximated here (friction from the last measured qd, lag as a first-order filter) when
//          `apply_plant_effects` is true.
// Deterministic for a given seed; no heap allocation after construction.
namespace cobot::imperfect {

using common::Status;
using hal::JointCommand;
using hal::JointInterface;
using hal::JointState;

inline constexpr int kMaxDelayCycles = 32;

template <int N>
class ImperfectJointInterface final : public JointInterface<N> {
 public:
  // Throws std::invalid_argument when a delay is outside [0, kMaxDelayCycles],
  // control_period_s <= 0, or a joint has a torque-speed knee with an infinite peak torque
  // or a zero-torque speed not above the knee. `peak_torque_nm` is the P of the curve
  // (hosts pass config.limits.peak_torque_nm); it has no effect without a knee.
  ImperfectJointInterface(JointInterface<N>& inner,
                          const ImperfectionConfig<N>& config,
                          double control_period_s,
                          bool apply_plant_effects = false,
                          const common::VecN<N>& peak_torque_nm =
                              common::VecN<N>::Constant(std::numeric_limits<double>::infinity()));

  Status read(JointState<N>& out) override;
  Status write(const JointCommand<N>& cmd) override;

  // Values drawn once at construction, for reports.
  const common::VecN<N>& q_offset_rad() const { return q_offset_; }
  const common::VecN<N>& torque_scale() const { return torque_scale_; }
  // Bus voltage used by the last write, V; v_nom_v before the first write. With the bus model
  // off (v_nom_v 0) it is the normalised nominal 1.0 (V = V_nom := 1, no scaling), never 0.
  double bus_voltage_v() const { return bus_voltage_v_; }

 private:
  JointInterface<N>& inner_;
  ImperfectionConfig<N> config_;
  double period_s_;
  bool apply_plant_effects_;
  std::mt19937_64 rng_;
  common::VecN<N> q_offset_;
  common::VecN<N> torque_scale_;
  std::array<JointState<N>, kMaxDelayCycles + 1> state_ring_{};
  std::array<JointCommand<N>, kMaxDelayCycles + 1> cmd_ring_{};
  int state_head_ = 0;
  int cmd_head_ = 0;
  int reads_ = 0;
  int writes_ = 0;
  common::VecN<N> q_prev_out_ = common::VecN<N>::Zero();
  common::VecN<N> qd_filtered_ = common::VecN<N>::Zero();
  common::VecN<N> tau_lagged_ = common::VecN<N>::Zero();
  common::VecN<N> q_last_true_ = common::VecN<N>::Zero();
  common::VecN<N> qd_last_true_ = common::VecN<N>::Zero();  // torque-speed curve and bus
  common::VecN<N> peak_torque_;
  common::VecN<N> tau_prev_out_ = common::VecN<N>::Zero();  // slew and bus, before plant
  double bus_voltage_v_ = 1.0;
  common::VecN<N> qd_last_out_ = common::VecN<N>::Zero();  // friction phase, plant effects
};

}  // namespace cobot::imperfect
