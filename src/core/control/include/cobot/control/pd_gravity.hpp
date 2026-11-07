#pragma once

#include "cobot/common/types.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/model/model.hpp"
#include "cobot/planning/clik.hpp"

// Joint PD plus gravity compensation (feature controller-registry, ADR-023, owner question C6):
//   tau = J .* (kp .* (q_ref - q) + kd .* (qd_ref - qd)) + G(q),  J = diag(M(q_inertia))
// clamped element-wise to [-tau_max, tau_max] (detail::clamp_or_zero). J is a constant per-joint
// inertia of `model` (armature included), taken once at construction (ControlLoop passes
// Config::home_q_rad), so the gains are in 1/s^2 and 1/s like the computed-torque gains: the same
// omega means the same bandwidth for both laws, and auto-tune sweeps one parameter per law.
// No qdd_ref, no C(q, qd) qd, no URDF damping term. A non-finite q, qd, q_ref or qd_ref gives the
// all-zero command. compute() allocates nothing.
namespace cobot::control {

using common::VecN;
using hal::JointCommand;
using hal::JointState;
using model::Model;
using planning::JointReference;

template <int N>
struct PdGravityGains {
  VecN<N> kp = VecN<N>::Constant(100.0);  // 1/s^2, > 0
  VecN<N> kd = VecN<N>::Constant(20.0);   // 1/s, > 0
};

template <int N>
class PdGravity {
 public:
  PdGravity(const Model<N>& model, const PdGravityGains<N>& gains, const VecN<N>& q_inertia);
  JointCommand<N> compute(const JointState<N>& s, const JointReference<N>& ref) const;
  const VecN<N>& inertia() const { return inertia_; }  // J, kg m^2

 private:
  Model<N> model_;
  PdGravityGains<N> gains_;
  VecN<N> inertia_;
};

}  // namespace cobot::control
