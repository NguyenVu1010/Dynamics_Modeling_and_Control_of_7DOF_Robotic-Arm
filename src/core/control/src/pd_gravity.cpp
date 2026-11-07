#include "cobot/control/pd_gravity.hpp"

#include "cobot/common/dof.hpp"
#include "cobot/dynamics/dynamics.hpp"

#include "torque_output.hpp"

namespace cobot::control {

template <int N>
PdGravity<N>::PdGravity(const Model<N>& model,
                        const PdGravityGains<N>& gains,
                        const VecN<N>& q_inertia)
    : model_(model), gains_(gains), inertia_(dynamics::mass_matrix(model, q_inertia).diagonal()) {}

template <int N>
JointCommand<N> PdGravity<N>::compute(const JointState<N>& s, const JointReference<N>& ref) const {
  // qdd_ref is not an input of the law, so it is not checked.
  if (!s.q.allFinite() || !s.qd.allFinite() || !ref.q.allFinite() || !ref.qd.allFinite()) {
    return JointCommand<N>{};
  }
  // a = kp .* (q_ref - q) + kd .* (qd_ref - qd) in 1/s^2; tau = J .* a + G(q).
  const VecN<N> a = gains_.kp.cwiseProduct(ref.q - s.q) + gains_.kd.cwiseProduct(ref.qd - s.qd);
  const VecN<N> raw = inertia_.cwiseProduct(a) + dynamics::gravity_torque(model_, s.q);
  return detail::clamp_or_zero(model_, raw);
}

#define COBOT_CONTROL_PDG_INSTANTIATE(N) template class PdGravity<(N)>;
COBOT_INSTANTIATE_DOF(COBOT_CONTROL_PDG_INSTANTIATE)
#undef COBOT_CONTROL_PDG_INSTANTIATE

}  // namespace cobot::control
