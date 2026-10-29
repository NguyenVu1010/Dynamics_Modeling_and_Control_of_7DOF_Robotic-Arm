#include "cobot/control/friction_compensation.hpp"

#include <cmath>

#include "cobot/common/dof.hpp"

namespace cobot::control {

template <int N>
FrictionCompensation<N>::FrictionCompensation(const FrictionCompensationConfig<N>& config)
    : config_(config) {}

template <int N>
VecN<N> FrictionCompensation<N>::torque(const VecN<N>& qd_ref) const {
  VecN<N> tau = VecN<N>::Zero();
  if (!config_.enabled || !qd_ref.allFinite()) {
    return tau;
  }
  const double eps = config_.velocity_eps_rad_s;
  for (int i = 0; i < N; ++i) {
    tau(i) = config_.coulomb_nm(i) * std::tanh(qd_ref(i) / eps) +
             config_.viscous_nm_s_rad(i) * qd_ref(i);
  }
  return tau;
}

#define COBOT_CONTROL_FC_INSTANTIATE(N) template class FrictionCompensation<(N)>;
COBOT_INSTANTIATE_DOF(COBOT_CONTROL_FC_INSTANTIATE)
#undef COBOT_CONTROL_FC_INSTANTIATE

}  // namespace cobot::control
