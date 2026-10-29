#include "cobot/dynamics/sim_plant.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

#include <Eigen/Cholesky>

#include "cobot/common/dof.hpp"
#include "cobot/dynamics/dynamics.hpp"

namespace cobot::dynamics {
namespace {

// Friction model (docs/derivations/sim_plant_friction.md).
//
// Coulomb friction coulomb * sign(qd) is discontinuous, and any smoothed sign steep enough to
// look like one is far too stiff for an explicit step on a light joint (slope coulomb / v_s,
// h * slope / I ~ 1e3 on the wrist). So it is not in the RK4 right-hand side: RK4 integrates
// the rigid-body dynamics, and the friction of every joint with coulomb > 0 is applied after
// the step as a velocity-level stick-slip projection (apply_friction). The viscous term of
// such a joint (PlantEffects viscous + URDF damping) moves into the projection too, treated
// implicitly: inside RK4 it would act on the velocity the step reaches before Coulomb takes
// it back (up to h * coulomb / m above the real one) and overstate the friction. Joints
// without Coulomb friction keep their viscous term inside RK4.

// Joint acceleration with the viscous torque `viscous_rk4` .* qd (URDF damping is applied by
// forward_dynamics; an entry of `viscous_rk4` equal to -damping cancels it).
template <int N>
VecN<N> plant_qdd(const Model<N>& model,
                  const VecN<N>& viscous_rk4,
                  const VecN<N>& q,
                  const VecN<N>& qd,
                  const VecN<N>& tau_applied) {
  const VecN<N> tau = tau_applied - viscous_rk4.cwiseProduct(qd);
  return forward_dynamics(model, q, qd, tau);
}

// Stick-slip projection, unconditionally stable. `q_start` / `qd_start` is the state at the
// start of the step, `q` / `qd` the friction-free RK4 result, updated in place.
//
// Friction on joint i over the step is a joint impulse -lambda_i * e_i, which changes the
// velocities by -lambda_i * M^-1 e_i, so joint i itself by -lambda_i / m_i with the effective
// inertia m_i = 1 / (M^-1)_ii. With v the joint velocity before and v' after:
//   lambda_i = h * (coulomb_i * sign(v') + viscous_i * v'),   v' = v - lambda_i / m_i.
// If |v| <= dv_i = h * coulomb_i / m_i the joint sticks (v' = 0, static friction balances the
// rest), else v' = (v - sign(v) * dv_i) / (1 + h * viscous_i / m_i), same sign as v.
// Applying the impulse through the whole column of M^-1 keeps the energy balance of the arm
// exact (Delta KE = -lambda_i * (v + v') / 2). Joints are processed one after the other with
// M at the end of the RK4 step, each seeing the velocities left by the previous ones
// (sequential approximation of the coupled stick-slip problem).
// Positions follow the velocity change by the trapezoid rule; on a joint with Coulomb
// friction the position is the trapezoid of the start and final velocity, so a stuck joint
// does not move at all and a constant net acceleration stays exact.
template <int N>
void apply_friction(const Model<N>& model,
                    const VecN<N>& coulomb,
                    const VecN<N>& viscous,
                    double h,
                    const VecN<N>& q_start,
                    const VecN<N>& qd_start,
                    VecN<N>& q,
                    VecN<N>& qd) {
  const Eigen::LDLT<MatN<N>> ldlt(mass_matrix(model, q));
  const VecN<N> qd_rk4 = qd;
  for (int i = 0; i < N; ++i) {
    if (!(coulomb(i) > 0.0)) {
      continue;
    }
    const VecN<N> m_inv_col = ldlt.solve(VecN<N>::Unit(i));  // M^-1 e_i; (M^-1)_ii = 1 / m_i
    const double v = qd(i);
    const double dv = h * coulomb(i) * m_inv_col(i);
    double v_new = 0.0;
    if (std::abs(v) > dv) {
      v_new = (v - std::copysign(dv, v)) / (1.0 + (h * viscous(i) * m_inv_col(i)));
    }
    const double lambda = (v - v_new) / m_inv_col(i);
    qd -= lambda * m_inv_col;
    qd(i) = v_new;  // exact, free of rounding (an exact 0 when stuck)
  }
  q += (0.5 * h) * (qd - qd_rk4);
  for (int i = 0; i < N; ++i) {
    if (coulomb(i) > 0.0) {
      q(i) = q_start(i) + (0.5 * h * (qd_start(i) + qd(i)));
    }
  }
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
  // Split the viscous terms between RK4 and the friction projection (see the top of the file).
  for (std::size_t i = 0; i < static_cast<std::size_t>(N); ++i) {
    const auto j = static_cast<Eigen::Index>(i);
    if (effects.coulomb_nm(j) > 0.0) {
      has_coulomb_ = true;
      viscous_rk4_(j) = -model.joints[i].damping;
      viscous_projection_(j) = effects.viscous_nm_s_rad(j) + model.joints[i].damping;
    } else {
      viscous_rk4_(j) = effects.viscous_nm_s_rad(j);
      viscous_projection_(j) = 0.0;
    }
  }
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

  // Classical RK4 on (q, qd) with tau_applied held constant, without the friction of the
  // joints that have Coulomb friction.
  const double h = dt_s;
  const VecN<N> q_start = q_;
  const VecN<N> qd_start = qd_;
  const VecN<N> a1 = plant_qdd(model_, viscous_rk4_, q_, qd_, tau_applied_);
  const VecN<N> q2 = q_ + 0.5 * h * qd_;
  const VecN<N> v2 = qd_ + 0.5 * h * a1;
  const VecN<N> a2 = plant_qdd(model_, viscous_rk4_, q2, v2, tau_applied_);
  const VecN<N> q3 = q_ + 0.5 * h * v2;
  const VecN<N> v3 = qd_ + 0.5 * h * a2;
  const VecN<N> a3 = plant_qdd(model_, viscous_rk4_, q3, v3, tau_applied_);
  const VecN<N> q4 = q_ + h * v3;
  const VecN<N> v4 = qd_ + h * a3;
  const VecN<N> a4 = plant_qdd(model_, viscous_rk4_, q4, v4, tau_applied_);
  q_ += (h / 6.0) * (qd_ + 2.0 * v2 + 2.0 * v3 + v4);
  qd_ += (h / 6.0) * (a1 + 2.0 * a2 + 2.0 * a3 + a4);

  if (has_coulomb_) {
    apply_friction(model_, effects_.coulomb_nm, viscous_projection_, h, q_start, qd_start, q_, qd_);
  }

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
