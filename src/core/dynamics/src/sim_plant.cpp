#include "cobot/dynamics/sim_plant.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

#include <Eigen/Cholesky>
#include <Eigen/LU>

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

constexpr int kMaxSweeps = 30;           // projected Gauss-Seidel sweeps per step
constexpr double kSweepTolRadS = 1e-12;  // stop when no joint velocity moves more (rad/s)

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

// One joint alone: velocity v_free before its friction impulse, dv = h * coulomb / m,
// damp = h * viscous / m. Stick (exact 0) when |v_free| <= dv, else slide with the viscous
// term implicit; never reverses the sign.
inline double slide_or_stick(double v_free, double dv, double damp) {
  if (std::abs(v_free) <= dv) {
    return 0.0;
  }
  return (v_free - std::copysign(dv, v_free)) / (1.0 + damp);
}

// Stick-slip projection, unconditionally stable. `q` / `qd` is the friction-free RK4 result,
// updated in place; `lambda` holds the joint friction impulses (N m s), warm-started from the
// previous step and updated.
//
// Friction on joint i over the step is a joint impulse -lambda_i * e_i; all of them change the
// velocities to v' = v - W lambda with W = M(q)^-1 (applying the impulse through the whole
// column keeps Delta KE = -sum_i lambda_i * (v_i + v'_i) / 2, the friction work). Per joint,
// with m_i = 1 / W_ii:
//   v'_i != 0:  lambda_i = h * (coulomb_i * sign(v'_i) + viscous_i * v'_i)
//   v'_i == 0:  |lambda_i| <= h * coulomb_i            (static friction)
// This coupled problem is solved by projected Gauss-Seidel on lambda: each joint solves its
// own 1-D problem (slide_or_stick) given the impulses of the others, which changes the
// velocity of every joint through W. The sweeps only find which joints stick and the sign of
// the sliding ones; that set then fixes a linear system in lambda, solved exactly, so stuck
// joints end at exactly qd = 0 whatever the coupling. If the exact solution contradicts the set
// (sweep cap hit mid-transition), the Gauss-Seidel result is kept.
// Positions follow the velocity change by the trapezoid rule, q += h / 2 * (qd' - qd_rk4):
// RK4 accuracy is kept while sliding, and a stuck joint starting at rest does not move for a
// constant acceleration (q_rk4 = q + h^2 a / 2, qd_rk4 = h a).
template <int N>
void apply_friction(const Model<N>& model,
                    const VecN<N>& coulomb,
                    const VecN<N>& viscous,
                    double h,
                    VecN<N>& lambda,
                    VecN<N>& q,
                    VecN<N>& qd) {
  const MatN<N> w = Eigen::LDLT<MatN<N>>(mass_matrix(model, q)).solve(MatN<N>::Identity());
  const VecN<N> qd_rk4 = qd;
  for (int i = 0; i < N; ++i) {
    if (!(coulomb(i) > 0.0)) {
      lambda(i) = 0.0;
    }
  }
  VecN<N> v = qd_rk4 - (w * lambda);
  // Per joint after the last sweep: +1 / -1 sliding direction, 0 stuck, 2 no Coulomb friction.
  VecN<N> state = VecN<N>::Constant(2.0);
  for (int sweep = 0; sweep < kMaxSweeps; ++sweep) {
    double change = 0.0;
    for (int i = 0; i < N; ++i) {
      if (!(coulomb(i) > 0.0)) {
        continue;
      }
      const double w_ii = w(i, i);
      const double v_free = v(i) + (w_ii * lambda(i));
      const double v_new = slide_or_stick(v_free, h * coulomb(i) * w_ii, h * viscous(i) * w_ii);
      const double d_lambda = ((v_free - v_new) / w_ii) - lambda(i);
      v -= d_lambda * w.col(i);
      lambda(i) += d_lambda;
      state(i) = v_new == 0.0 ? 0.0 : std::copysign(1.0, v_new);
      change = std::max(change, std::abs(d_lambda) * w_ii);
    }
    if (change <= kSweepTolRadS) {
      break;
    }
  }

  // Exact solve for the stick set and sliding signs found:
  //   stuck i:    (W lambda)_i = qd_rk4_i                                   (v'_i = 0)
  //   sliding i:  lambda_i + h b_i (W lambda)_i = h (c_i s_i + b_i qd_rk4_i)
  //   no Coulomb: lambda_i = 0
  MatN<N> a = MatN<N>::Identity();
  VecN<N> r = VecN<N>::Zero();
  for (int i = 0; i < N; ++i) {
    if (state(i) == 0.0) {
      a.row(i) = w.row(i);
      r(i) = qd_rk4(i);
    } else if (state(i) != 2.0) {
      a.row(i) += h * viscous(i) * w.row(i);
      r(i) = h * ((coulomb(i) * state(i)) + (viscous(i) * qd_rk4(i)));
    }
  }
  const VecN<N> lambda_x = a.partialPivLu().solve(r);
  const VecN<N> v_x = qd_rk4 - (w * lambda_x);
  bool consistent = lambda_x.allFinite();
  for (int i = 0; consistent && i < N; ++i) {
    if (state(i) == 0.0) {
      consistent = std::abs(lambda_x(i)) <= h * coulomb(i) * (1.0 + 1e-9);
    } else if (state(i) != 2.0) {
      consistent = v_x(i) * state(i) > 0.0;
    }
  }
  if (consistent) {
    lambda = lambda_x;
    v = v_x;
  }
  for (int i = 0; i < N; ++i) {
    if (state(i) == 0.0) {
      v(i) = 0.0;  // exact stick (the residual is rounding, or below the sweep tolerance)
    }
  }
  qd = v;
  q += (0.5 * h) * (qd - qd_rk4);
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
    apply_friction(model_, effects_.coulomb_nm, viscous_projection_, h, friction_impulse_, q_, qd_);
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
