#include "cobot/control/hold.hpp"

#include "cobot/common/dof.hpp"
#include "cobot/dynamics/dynamics.hpp"

#include "torque_output.hpp"

namespace cobot::control {

template <int N>
HoldController<N>::HoldController(const Model<N>& model, const ComputedTorqueGains<N>& gains)
    : model_(model), gains_(gains) {}

template <int N>
void HoldController<N>::engage(const VecN<N>& q_hold) {
  q_hold_ = q_hold;
}

template <int N>
JointCommand<N> HoldController<N>::compute(const JointState<N>& s) const {
  if (!s.q.allFinite() || !s.qd.allFinite() || !q_hold_.allFinite()) {
    return JointCommand<N>{};
  }
  // tau = diag(M(q)) .* (kp .* (q_hold - q) - kd .* qd) + G(q): scaling by the joint's own
  // inertia keeps the gains in 1/s^2 and 1/s.
  const VecN<N> pd = gains_.kp.cwiseProduct(q_hold_ - s.q) - gains_.kd.cwiseProduct(s.qd);
  const VecN<N> diag_m = dynamics::mass_matrix(model_, s.q).diagonal();
  const VecN<N> raw = diag_m.cwiseProduct(pd) + dynamics::gravity_torque(model_, s.q);
  return detail::clamp_or_zero(model_, raw);
}

#define COBOT_CONTROL_HOLD_INSTANTIATE(N) template class HoldController<(N)>;
COBOT_INSTANTIATE_DOF(COBOT_CONTROL_HOLD_INSTANTIATE)
#undef COBOT_CONTROL_HOLD_INSTANTIATE

}  // namespace cobot::control
