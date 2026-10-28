#pragma once

#include "cobot/control/computed_torque.hpp"

// Safe hold (feature `control`): tau = diag(M(q)) .* (kp .* (q_hold - q) - kd .* qd) + G(q),
// clamped to tau_max; non-finite -> zero. Used when the loop leaves tracking (limits, bad data,
// HAL timeout).
namespace cobot::control {

template <int N>
class HoldController {
 public:
  HoldController(const Model<N>& model, const ComputedTorqueGains<N>& gains);
  void engage(const VecN<N>& q_hold);
  JointCommand<N> compute(const JointState<N>& s) const;
  const VecN<N>& q_hold() const { return q_hold_; }

 private:
  Model<N> model_;
  ComputedTorqueGains<N> gains_;
  VecN<N> q_hold_ = VecN<N>::Zero();
};

}  // namespace cobot::control
