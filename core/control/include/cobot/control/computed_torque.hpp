#pragma once

#include "cobot/common/types.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/model/model.hpp"
#include "cobot/planning/clik.hpp"

// Joint-space computed torque (feature `control`, design 5.4):
//   tau = M(q) (qdd_ref + kd .* (qd_ref - qd) + kp .* (q_ref - q)) + C(q, qd) qd + G(q) + D qd
// clamped element-wise to [-tau_max, tau_max]. Gains are diagonal, in 1/s^2 and 1/s.
namespace cobot::control {

using common::VecN;
using hal::JointCommand;
using hal::JointState;
using model::Model;
using planning::JointReference;

template <int N>
struct ComputedTorqueGains {
  VecN<N> kp = VecN<N>::Constant(100.0);
  VecN<N> kd = VecN<N>::Constant(20.0);
};

template <int N>
class ComputedTorque {
 public:
  ComputedTorque(const Model<N>& model, const ComputedTorqueGains<N>& gains);
  JointCommand<N> compute(const JointState<N>& s, const JointReference<N>& ref) const;

 private:
  Model<N> model_;
  ComputedTorqueGains<N> gains_;
};

}  // namespace cobot::control
