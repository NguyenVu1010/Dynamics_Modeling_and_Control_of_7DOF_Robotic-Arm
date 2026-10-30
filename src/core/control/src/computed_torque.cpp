#include "cobot/control/computed_torque.hpp"

#include "cobot/common/dof.hpp"
#include "cobot/dynamics/dynamics.hpp"

#include "torque_output.hpp"

namespace cobot::control {

template <int N>
ComputedTorque<N>::ComputedTorque(const Model<N>& model, const ComputedTorqueGains<N>& gains)
    : model_(model), gains_(gains) {}

template <int N>
JointCommand<N> ComputedTorque<N>::compute(const JointState<N>& s,
                                           const JointReference<N>& ref) const {
  if (!s.q.allFinite() || !s.qd.allFinite() || !ref.q.allFinite() || !ref.qd.allFinite() ||
      !ref.qdd.allFinite()) {
    return JointCommand<N>{};
  }
  // v = qdd_ref + kd .* (qd_ref - qd) + kp .* (q_ref - q)
  const VecN<N> v =
      ref.qdd + gains_.kd.cwiseProduct(ref.qd - s.qd) + gains_.kp.cwiseProduct(ref.q - s.q);
  // tau = M(q) v + C(q, qd) qd + G(q) + D qd; one RNEA call gives M v + C qd + G.
  const VecN<N> raw = dynamics::rnea(model_, s.q, s.qd, v, model_.gravity_b) +
                      detail::damping_of(model_).cwiseProduct(s.qd);
  return detail::clamp_or_zero(model_, raw);
}

#define COBOT_CONTROL_CT_INSTANTIATE(N) template class ComputedTorque<(N)>;
COBOT_INSTANTIATE_DOF(COBOT_CONTROL_CT_INSTANTIATE)
#undef COBOT_CONTROL_CT_INSTANTIATE

}  // namespace cobot::control
