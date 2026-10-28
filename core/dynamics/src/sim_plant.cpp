#include "cobot/dynamics/sim_plant.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

#include "cobot/common/dof.hpp"
#include "cobot/dynamics/dynamics.hpp"

namespace cobot::dynamics {
namespace {

// Coulomb friction uses sign_smooth(qd) = tanh(qd / kSignSmoothVel) instead of sign(qd):
// a continuous right-hand side keeps RK4 from chattering around qd = 0. Below breakaway the
// joint creeps at kSignSmoothVel * atanh(|tau| / coulomb), i.e. well under 1 mrad/s.
constexpr double kSignSmoothVel = 1e-3;  // rad/s

// Joint acceleration with the unmodelled friction of `effects` acting against qd.
template <int N>
VecN<N> plant_qdd(const Model<N>& model,
                  const PlantEffects<N>& effects,
                  const VecN<N>& q,
                  const VecN<N>& qd,
                  const VecN<N>& tau_applied) {
  const VecN<N> sign_smooth = (qd.array() / kSignSmoothVel).tanh().matrix();
  const VecN<N> tau = tau_applied - effects.coulomb_nm.cwiseProduct(sign_smooth) -
                      effects.viscous_nm_s_rad.cwiseProduct(qd);
  return forward_dynamics(model, q, qd, tau);
}

// Throws naming the field and joint of the first negative (or NaN) entry.
template <int N>
void check_non_negative(const VecN<N>& v, const char* field) {
  for (int j = 0; j < N; ++j) {
    if (!(v(j) >= 0.0)) {
      throw std::invalid_argument(std::string("SimPlant: ") + field + "[" + std::to_string(j) +
                                  "] must be >= 0");
    }
  }
}

}  // namespace

template <int N>
SimPlant<N>::SimPlant(const Model<N>& model, const VecN<N>& q0, const PlantEffects<N>& effects)
    : model_(model), effects_(effects), q_(q0) {
  check_non_negative(effects.coulomb_nm, "coulomb_nm");
  check_non_negative(effects.viscous_nm_s_rad, "viscous_nm_s_rad");
  check_non_negative(effects.motor_time_constant_s, "motor_time_constant_s");
}

template <int N>
void SimPlant<N>::step(const VecN<N>& tau_cmd, double dt_s) {
  // First-order motor lag, exact discretisation for a command held over the step.
  for (int j = 0; j < N; ++j) {
    const double tau_m = effects_.motor_time_constant_s(j);
    if (tau_m > 0.0) {
      tau_applied_(j) += -std::expm1(-dt_s / tau_m) * (tau_cmd(j) - tau_applied_(j));
    } else {
      tau_applied_(j) = tau_cmd(j);
    }
  }

  // Classical RK4 on (q, qd) with tau_applied held constant.
  const double h = dt_s;
  const VecN<N> a1 = plant_qdd(model_, effects_, q_, qd_, tau_applied_);
  const VecN<N> q2 = q_ + 0.5 * h * qd_;
  const VecN<N> v2 = qd_ + 0.5 * h * a1;
  const VecN<N> a2 = plant_qdd(model_, effects_, q2, v2, tau_applied_);
  const VecN<N> q3 = q_ + 0.5 * h * v2;
  const VecN<N> v3 = qd_ + 0.5 * h * a2;
  const VecN<N> a3 = plant_qdd(model_, effects_, q3, v3, tau_applied_);
  const VecN<N> q4 = q_ + h * v3;
  const VecN<N> v4 = qd_ + h * a3;
  const VecN<N> a4 = plant_qdd(model_, effects_, q4, v4, tau_applied_);
  q_ += (h / 6.0) * (qd_ + 2.0 * v2 + 2.0 * v3 + v4);
  qd_ += (h / 6.0) * (a1 + 2.0 * a2 + 2.0 * a3 + a4);

  // Hard joint stops: clamp the position, drop the velocity pointing out of the range.
  for (std::size_t i = 0; i < static_cast<std::size_t>(N); ++i) {
    const auto j = static_cast<Eigen::Index>(i);
    const model::JointParams& joint = model_.joints[i];
    if (q_(j) >= joint.q_max) {
      q_(j) = joint.q_max;
      qd_(j) = qd_(j) > 0.0 ? 0.0 : qd_(j);
    } else if (q_(j) <= joint.q_min) {
      q_(j) = joint.q_min;
      qd_(j) = qd_(j) < 0.0 ? 0.0 : qd_(j);
    }
  }
  time_s_ += dt_s;
}

#define COBOT_SIM_PLANT_INSTANTIATE(N) template class SimPlant<(N)>;
COBOT_INSTANTIATE_DOF(COBOT_SIM_PLANT_INSTANTIATE)
#undef COBOT_SIM_PLANT_INSTANTIATE

}  // namespace cobot::dynamics
