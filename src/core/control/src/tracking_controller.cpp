#include "cobot/control/tracking_controller.hpp"

#include "cobot/common/dof.hpp"

namespace cobot::control {

template <int N>
TrackingController<N>::TrackingController(const Model<N>& model, const ControllerConfig<N>& config)
    : algorithm_(config.algorithm),
      computed_torque_(model, config.computed_torque),
      pd_gravity_(model, config.pd_gravity, config.q_inertia) {}

// The one place a law is dispatched (with the kAlgorithms entry, the registration).
template <int N>
JointCommand<N> TrackingController<N>::compute(const JointState<N>& s,
                                               const JointReference<N>& ref) const noexcept {
  switch (algorithm_) {
    case Algorithm::kComputedTorque:
      return computed_torque_.compute(s, ref);
    case Algorithm::kPdGravity:
      return pd_gravity_.compute(s, ref);
  }
  return JointCommand<N>{};
}

#define COBOT_CONTROL_TC_INSTANTIATE(N) template class TrackingController<(N)>;
COBOT_INSTANTIATE_DOF(COBOT_CONTROL_TC_INSTANTIATE)
#undef COBOT_CONTROL_TC_INSTANTIATE

}  // namespace cobot::control
