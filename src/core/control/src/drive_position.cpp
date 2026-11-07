#include "cobot/control/drive_position.hpp"

#include "cobot/common/dof.hpp"
#include "cobot/dynamics/dynamics.hpp"

#include "torque_output.hpp"

namespace cobot::control {

template <int N>
DrivePosition<N>::DrivePosition(const Model<N>& model, DriveFeedForward ff)
    : model_(model), ff_(ff) {}

template <int N>
JointCommand<N> DrivePosition<N>::compute(const JointState<N>& /*s*/,
                                          const JointReference<N>& ref) const noexcept {
  if (!ref.q.allFinite() || !ref.qd.allFinite() || !ref.qdd.allFinite()) {
    return JointCommand<N>{};
  }
  VecN<N> raw = VecN<N>::Zero();
  if (ff_ == DriveFeedForward::kGravity) {
    raw = dynamics::gravity_torque(model_, ref.q);
  } else if (ff_ == DriveFeedForward::kInverseDynamics) {
    raw = dynamics::rnea(model_, ref.q, ref.qd, ref.qdd, model_.gravity_b) +
          detail::damping_of(model_).cwiseProduct(ref.qd);
  }
  if (!raw.allFinite()) {
    return JointCommand<N>{};
  }
  JointCommand<N> out = detail::clamp_or_zero(model_, raw);
  out.mode = hal::DriveMode::kPosition;
  out.q = ref.q;
  out.qd = ref.qd;
  return out;
}

template <int N>
JointCommand<N> DrivePosition<N>::hold(const VecN<N>& q_hold) const noexcept {
  if (!q_hold.allFinite()) {
    return JointCommand<N>{};
  }
  JointCommand<N> out;
  if (ff_ != DriveFeedForward::kNone) {
    const VecN<N> g = dynamics::gravity_torque(model_, q_hold);
    if (!g.allFinite()) {
      return JointCommand<N>{};
    }
    out = detail::clamp_or_zero(model_, g);
  }
  out.mode = hal::DriveMode::kPosition;
  out.q = q_hold;
  return out;
}

#define COBOT_CONTROL_DP_INSTANTIATE(N) template class DrivePosition<(N)>;
COBOT_INSTANTIATE_DOF(COBOT_CONTROL_DP_INSTANTIATE)
#undef COBOT_CONTROL_DP_INSTANTIATE

}  // namespace cobot::control
