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
#include "cobot/dynamics/harmonic_spring.hpp"

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

// Hard joint stops: clamp the position, drop the velocity pointing out of the range.
template <int N>
void clamp_to_stops(const Model<N>& model, VecN<N>& q, VecN<N>& qd) {
  for (std::size_t i = 0; i < static_cast<std::size_t>(N); ++i) {
    const auto j = static_cast<Eigen::Index>(i);
    const model::JointParams& joint = model.joints[i];
    if (q(j) >= joint.q_max) {
      q(j) = joint.q_max;
      qd(j) = qd(j) > 0.0 ? 0.0 : qd(j);
    } else if (q(j) <= joint.q_min) {
      q(j) = joint.q_min;
      qd(j) = qd(j) < 0.0 ? 0.0 : qd(j);
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

// Transmission (docs/derivations/sim_plant_transmission.md). The sub-step count keeps
// omega * h <= kOmegaStep for the stiffest two-mass mode, omega^2 = k_max (1 / B + W_jj),
// W = M_link^-1 at the start of the step.
constexpr double kOmegaStep = 0.05;
constexpr int kMaxSubsteps = 64;

void check_elastic(bool ok, int j, const char* what) {
  if (!ok) {
    throw std::invalid_argument("SimPlant: elastic joint " + std::to_string(j) + ": " + what);
  }
}

}  // namespace

template <int N>
SimPlant<N>::SimPlant(const Model<N>& model, const VecN<N>& q0, const PlantEffects<N>& effects)
    : model_(model), effects_(effects), q_(q0), link_model_(model), theta_(q0) {
  check_non_negative(effects.coulomb_nm, "coulomb_nm");
  check_non_negative(effects.viscous_nm_s_rad, "viscous_nm_s_rad");
  check_non_negative(effects.motor_time_constant_s, "motor_time_constant_s");
  check_non_negative(effects.stiffness_k3_nm_rad, "stiffness_k3_nm_rad");
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
    if (!(effects.stiffness_k3_nm_rad(j) > 0.0)) {
      rigid_coulomb_(j) = effects.coulomb_nm(j);
      continue;
    }
    // Elastic: the armature becomes the motor inertia B, joint friction moves to the motor.
    const double k1 = effects.stiffness_k1_nm_rad(j);
    const double b = model.joints[i].armature;
    const int ji = static_cast<int>(i);
    check_elastic(k1 > 0.0 && effects.stiffness_k2_nm_rad(j) > 0.0, ji, "k1 and k2 must be > 0");
    check_elastic(
        effects.torque_t1_nm(j) > 0.0 && effects.torque_t2_nm(j) > effects.torque_t1_nm(j),
        ji,
        "needs 0 < t1 < t2");
    check_elastic(effects.damping_ratio(j) >= 0.0 && effects.lost_motion_rad(j) >= 0.0 &&
                      std::isfinite(effects.error_amp_rad(j)) && effects.ratio(j) > 0.0,
                  ji,
                  "needs damping_ratio >= 0, lost_motion_rad >= 0, ratio > 0");
    check_elastic(b > 0.0, ji, "the URDF armature (motor inertia) must be > 0");
    has_elastic_ = true;
    elastic_(j) = 1.0;
    motor_inertia_(j) = b;
    spring_damping_(j) = effects.damping_ratio(j) * 2.0 * std::sqrt(k1 * b);
    stiffness_max_(j) =
        std::max({k1, effects.stiffness_k2_nm_rad(j), effects.stiffness_k3_nm_rad(j)});
    const double viscous = effects.viscous_nm_s_rad(j) + model.joints[i].damping;
    (effects.coulomb_nm(j) > 0.0 ? motor_viscous_proj_ : motor_viscous_rk4_)(j) = viscous;
    link_model_.joints[i].armature = 0.0;
    link_model_.joints[i].damping = 0.0;
  }
}

template <int N>
VecN<N> SimPlant<N>::spring(const State& x) const {
  VecN<N> tau = VecN<N>::Zero();
  const PlantEffects<N>& fx = effects_;
  for (int j = 0; j < N; ++j) {
    if (elastic_(j) == 0.0) {
      continue;
    }
    const double e =
        fx.error_amp_rad(j) * std::sin((2.0 * fx.ratio(j) * x.theta(j)) + fx.motor_phase_rad(j));
    const double phi = x.theta(j) - x.q(j) - e;
    const double half = 0.5 * fx.lost_motion_rad(j);
    const double phi_live = phi - std::clamp(phi, -half, half);
    tau(j) = harmonic_spring_torque(phi_live,
                                    fx.stiffness_k1_nm_rad(j),
                                    fx.stiffness_k2_nm_rad(j),
                                    fx.stiffness_k3_nm_rad(j),
                                    fx.torque_t1_nm(j),
                                    fx.torque_t2_nm(j)) +
             (spring_damping_(j) * (x.thetad(j) - x.qd(j)));
  }
  return tau;
}

// Link: M_link(q) qdd + h = tau_s (elastic) or tau_applied - viscous qd (rigid), locked joints
// held at qdd = 0. Motor: B thetadd = tau_m - tau_s - viscous thetad, 0 while stuck.
template <int N>
typename SimPlant<N>::State SimPlant<N>::derivative(const State& x) const {
  const VecN<N> tau_s = spring(x);
  const VecN<N> zero = VecN<N>::Zero();
  VecN<N> rhs = tau_applied_ - viscous_rk4_.cwiseProduct(x.qd);
  VecN<N> thetadd = VecN<N>::Zero();
  for (int j = 0; j < N; ++j) {
    if (elastic_(j) != 0.0) {
      rhs(j) = tau_s(j);
      if (motor_stuck_(j) == 0.0) {
        thetadd(j) = (tau_applied_(j) - tau_s(j) - (motor_viscous_rk4_(j) * x.thetad(j))) /
                     motor_inertia_(j);
      }
    }
    rhs(j) -= link_model_.joints[static_cast<std::size_t>(j)].damping * x.qd(j);
  }
  rhs -= rnea(link_model_, x.q, x.qd, zero, link_model_.gravity_b);
  MatN<N> m = mass_matrix(link_model_, x.q);
  for (int j = 0; j < N; ++j) {
    if (link_locked_(j) != 0.0) {
      m.row(j).setZero();
      m.col(j).setZero();
      m(j, j) = 1.0;
      rhs(j) = 0.0;
    }
  }
  const VecN<N> qdd = Eigen::LLT<MatN<N>>(m).solve(rhs);
  const VecN<N> thetad = x.thetad.cwiseProduct(elastic_);
  return State{x.qd, qdd, thetad, thetadd};
}

// One RK4 sub-step of the elastic path, then the motor stick-slip of elastic joints, the coupled
// projection of rigid joints with Coulomb friction and the hard stops.
template <int N>
void SimPlant<N>::substep(double h) {
  const State x{q_, qd_, theta_, thetad_};
  motor_stuck_.setZero();
  link_locked_.setZero();
  // A motor at rest whose net drive is below breakaway stays stuck for the whole sub-step.
  const VecN<N> tau_s = spring(x);
  for (int j = 0; j < N; ++j) {
    const double c = effects_.coulomb_nm(j);
    if (elastic_(j) != 0.0 && c > 0.0 && thetad_(j) == 0.0 &&
        std::abs(tau_applied_(j) - tau_s(j)) <= c) {
      motor_stuck_(j) = 1.0;
    }
  }
  // A link resting on a stop and pushed into it stays there (no stage drifts past the stop).
  bool at_stop = false;
  for (std::size_t i = 0; i < static_cast<std::size_t>(N); ++i) {
    const auto j = static_cast<Eigen::Index>(i);
    at_stop = at_stop || (qd_(j) == 0.0 &&
                          (q_(j) >= model_.joints[i].q_max || q_(j) <= model_.joints[i].q_min));
  }
  if (at_stop) {
    const VecN<N> qdd = derivative(x).qd;
    for (std::size_t i = 0; i < static_cast<std::size_t>(N); ++i) {
      const auto j = static_cast<Eigen::Index>(i);
      const bool up = q_(j) >= model_.joints[i].q_max && qdd(j) > 0.0;
      const bool down = q_(j) <= model_.joints[i].q_min && qdd(j) < 0.0;
      link_locked_(j) = qd_(j) == 0.0 && (up || down) ? 1.0 : 0.0;
    }
  }
  const auto ahead = [&x](const State& k, double s) {
    return State{x.q + s * k.q, x.qd + s * k.qd, x.theta + s * k.theta, x.thetad + s * k.thetad};
  };
  const State k1 = derivative(x);
  const State k2 = derivative(ahead(k1, 0.5 * h));
  const State k3 = derivative(ahead(k2, 0.5 * h));
  const State k4 = derivative(ahead(k3, h));
  const double w = h / 6.0;
  q_ += w * (k1.q + 2.0 * k2.q + 2.0 * k3.q + k4.q);
  qd_ += w * (k1.qd + 2.0 * k2.qd + 2.0 * k3.qd + k4.qd);
  theta_ += w * (k1.theta + 2.0 * k2.theta + 2.0 * k3.theta + k4.theta);
  thetad_ += w * (k1.thetad + 2.0 * k2.thetad + 2.0 * k3.thetad + k4.thetad);

  // Motor-side friction of the elastic joints over this sub-step (URDF damping excluded).
  VecN<N> friction = VecN<N>::Zero();
  for (int j = 0; j < N; ++j) {
    if (elastic_(j) == 0.0) {
      continue;
    }
    const double c = effects_.coulomb_nm(j);
    const double damping = model_.joints[static_cast<std::size_t>(j)].damping;
    if (c > 0.0 && motor_stuck_(j) == 0.0) {
      const double b = motor_inertia_(j);
      const double v_rk4 = thetad_(j);
      thetad_(j) = slide_or_stick(v_rk4, h * c / b, h * motor_viscous_proj_(j) / b);
      theta_(j) += 0.5 * h * (thetad_(j) - v_rk4);
      friction(j) = (damping * thetad_(j)) - (b * (v_rk4 - thetad_(j)) / h);
    } else if (c > 0.0) {
      friction(j) = tau_s(j) - tau_applied_(j);  // static friction holding the motor
    } else {
      friction(j) = 0.0 - (effects_.viscous_nm_s_rad(j) * thetad_(j));
    }
  }
  // With N == 1 the only joint is elastic, so there is no rigid joint to project.
  if constexpr (N > 1) {
    if (rigid_coulomb_.maxCoeff() > 0.0) {
      apply_friction(
          link_model_, rigid_coulomb_, viscous_projection_, h, friction_impulse_, q_, qd_);
    }
  }
  clamp_to_stops(model_, q_, qd_);
  friction_substep_ += friction + rigid_friction(h);
}

// Projected joints: -lambda / h is the Coulomb + viscous + URDF damping torque; the damping part
// (at the end velocity, implicit) is added back. A stuck joint's static friction stays within
// +-coulomb (the projection accepts lambda up to rounding above h * coulomb). Other rigid joints:
// the viscous torque inside RK4 at the end state. Elastic joints: 0 (motor side, substep()).
template <int N>
VecN<N> SimPlant<N>::rigid_friction(double h) const {
  VecN<N> friction = VecN<N>::Zero();
  for (int j = 0; j < N; ++j) {
    const double c = rigid_coulomb_(j);
    if (elastic_(j) != 0.0) {
      continue;
    }
    if (c > 0.0) {
      const double damping = model_.joints[static_cast<std::size_t>(j)].damping;
      const double f = (damping * qd_(j)) - (friction_impulse_(j) / h);
      friction(j) = qd_(j) == 0.0 ? std::clamp(f, -c, c) : f;
    } else {
      friction(j) = 0.0 - (viscous_rk4_(j) * qd_(j));
    }
  }
  return friction;
}

template <int N>
void SimPlant<N>::step(const VecN<N>& tau_cmd, double dt_s) {
  // First-order motor lag, exact discretisation for a command held over the step, then the
  // drive's hard cap at the plant model's tau_max (URDF effort, the momentary torque M). The lag
  // state is the uncapped torque, so a torque within the cap is untouched bit for bit.
  for (int j = 0; j < N; ++j) {
    const double tau_m = effects_.motor_time_constant_s(j);
    if (tau_m > 0.0) {
      tau_lagged_(j) += -std::expm1(-dt_s / tau_m) * (tau_cmd(j) - tau_lagged_(j));
    } else {
      tau_lagged_(j) = tau_cmd(j);
    }
    const double cap = model_.joints[static_cast<std::size_t>(j)].tau_max;
    tau_applied_(j) = std::clamp(tau_lagged_(j), -cap, cap);
  }

  if (has_elastic_) {
    const Eigen::LLT<MatN<N>> llt(mass_matrix(link_model_, q_));
    double omega2 = 0.0;
    for (int j = 0; j < N; ++j) {
      if (elastic_(j) != 0.0) {
        const VecN<N> w_col = llt.solve(VecN<N>::Unit(j));  // column j of M_link^-1
        omega2 = std::max(omega2, stiffness_max_(j) * ((1.0 / motor_inertia_(j)) + w_col(j)));
      }
    }
    const double n_real = std::ceil(dt_s * std::sqrt(omega2) / kOmegaStep);
    const int n = std::isfinite(n_real) ? std::clamp(static_cast<int>(n_real), 1, kMaxSubsteps)
                                        : kMaxSubsteps;
    friction_substep_.setZero();
    for (int k = 0; k < n; ++k) {
      substep(dt_s / n);
    }
    tau_friction_ = friction_substep_ / static_cast<double>(n);
    tau_spring_ = spring(State{q_, qd_, theta_, thetad_});
    for (int j = 0; j < N; ++j) {
      if (elastic_(j) == 0.0) {
        theta_(j) = q_(j);
        thetad_(j) = qd_(j);
        tau_spring_(j) = tau_applied_(j);
      }
    }
    time_s_ += dt_s;
    return;
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
  clamp_to_stops(model_, q_, qd_);
  tau_friction_ = rigid_friction(h);
  // Rigid plant: the motor is the link.
  theta_ = q_;
  thetad_ = qd_;
  tau_spring_ = tau_applied_;
  time_s_ += dt_s;
}

#define COBOT_SIM_PLANT_INSTANTIATE(N) template class SimPlant<(N)>;
COBOT_INSTANTIATE_DOF(COBOT_SIM_PLANT_INSTANTIATE)
#undef COBOT_SIM_PLANT_INSTANTIATE

}  // namespace cobot::dynamics
