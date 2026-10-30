#include "cobot/dynamics/dynamics.hpp"

#include <cstddef>

#include <Eigen/Cholesky>

#include "cobot/common/dof.hpp"

namespace cobot::dynamics {

template <int N>
VecN<N> gravity_torque(const Model<N>& model, const VecN<N>& q) {
  const VecN<N> zero = VecN<N>::Zero();
  return rnea(model, q, zero, zero, model.gravity_b);
}

template <int N>
VecN<N> coriolis_torque(const Model<N>& model, const VecN<N>& q, const VecN<N>& qd) {
  const VecN<N> zero = VecN<N>::Zero();
  return rnea(model, q, qd, zero, Vec3::Zero());
}

template <int N>
VecN<N> forward_dynamics(const Model<N>& model,
                         const VecN<N>& q,
                         const VecN<N>& qd,
                         const VecN<N>& tau) {
  // One RNEA call with qdd = 0 gives C qd + G.
  const VecN<N> zero = VecN<N>::Zero();
  const VecN<N> bias = rnea(model, q, qd, zero, model.gravity_b);
  VecN<N> rhs = tau - bias;
  for (std::size_t i = 0; i < static_cast<std::size_t>(N); ++i) {
    const auto k = static_cast<Eigen::Index>(i);
    rhs(k) -= model.joints[i].damping * qd(k);
  }
  const Eigen::LDLT<MatN<N>> ldlt(mass_matrix(model, q));
  return ldlt.solve(rhs);
}

#define COBOT_DYNAMICS_INSTANTIATE(N)                                                             \
  template VecN<(N)> gravity_torque<(N)>(const Model<(N)>&, const VecN<(N)>&);                    \
  template VecN<(N)> coriolis_torque<(N)>(const Model<(N)>&, const VecN<(N)>&, const VecN<(N)>&); \
  template VecN<(N)> forward_dynamics<(N)>(                                                       \
      const Model<(N)>&, const VecN<(N)>&, const VecN<(N)>&, const VecN<(N)>&);
COBOT_INSTANTIATE_DOF(COBOT_DYNAMICS_INSTANTIATE)
#undef COBOT_DYNAMICS_INSTANTIATE

}  // namespace cobot::dynamics
