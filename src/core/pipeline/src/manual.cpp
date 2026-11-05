#include "cobot/pipeline/manual.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "cobot/common/dof.hpp"
#include "cobot/common/lie.hpp"
#include "cobot/control/friction_compensation.hpp"
#include "cobot/dynamics/dynamics.hpp"

// Design docs/design/2026-11-02-cobot-studio-design.md 5.4, ADR-022 "Lệnh tay".
namespace cobot::pipeline {
namespace {

using common::Vec3;

constexpr double kAtGoalRad = 1e-9;  // plan_joint_move: |q_goal - q_start| at or below: no move
constexpr std::size_t kMessageChars = 128;
constexpr double kLandedRad = 1e-12;  // joint jog: q_ref this close to the jog limit is on it

template <typename... Args>
std::string format(const char* fmt, Args... args) {
  std::array<char, kMessageChars> text{};
  std::snprintf(text.data(), text.size(), fmt, args...);
  return {text.data()};
}

bool speed_ok(double speed) {
  return speed > 0.0 && speed <= 1.0;
}

// v one step toward `target`, by at most a_dt; lands exactly on the target.
double ramp_toward(double v, double target, double a_dt) {
  if (std::abs(target - v) <= a_dt) {
    return target;
  }
  return target > v ? v + a_dt : v - a_dt;
}

// Largest velocity v for this step from which the stepped ramp (v, v - a dt, v - 2 a dt, ...,
// each held for one step with q += v dt) covers at most `dist`. That distance is
// g(v) = dt (m + 1) (v - m a dt / 2) with m = ceil(v / (a dt)) - 1, so g(v) - g(v - a dt) = v dt:
// a step at the cap leaves exactly the cap minus a dt for the next one, and v lands on rest at
// the end of `dist` with |dv| <= a dt on every step (the continuous sqrt(2 a dist) does not).
double stop_cap(double dist, double a, double dt) {
  if (!(dist > 0.0)) {
    return 0.0;
  }
  if (!std::isfinite(dist)) {
    return std::numeric_limits<double>::infinity();  // continuous joint: no limit to brake for
  }
  const double u = a * dt;
  // Smallest m >= 0 with g((m + 1) u) = dt u (m + 1) (m + 2) / 2 >= dist.
  double m = std::max(std::ceil((-3.0 + std::sqrt(1.0 + 8.0 * dist / (dt * u))) / 2.0), 0.0);
  if (m > 0.0 && dt * u * m * (m + 1.0) / 2.0 >= dist) {
    m -= 1.0;  // rounding of the estimate
  } else if (dt * u * (m + 1.0) * (m + 2.0) / 2.0 < dist) {
    m += 1.0;
  }
  return dist / (dt * (m + 1.0)) + u * m / 2.0;
}

// Deadman count (one step) and the velocity of this step.
double next_velocity(ManualJog& jog, double cap, double dt) {
  if (++jog.idle_steps > jog.deadman_steps && jog.braking == ManualStop::kNone) {
    jog.braking = ManualStop::kDeadman;
  }
  double target = jog.braking == ManualStop::kNone ? jog.direction * jog.v_max : 0.0;
  if (std::abs(target) > cap) {
    target = jog.direction * cap;
  }
  return ramp_toward(jog.v, target, jog.a_max * dt);
}

}  // namespace

const char* to_string(ManualStop stop) {
  constexpr std::array<const char*, 6> kNames = {
      "none", "finished", "stopped", "deadman", "soft_limit", "singularity"};
  return kNames[static_cast<std::size_t>(stop)];
}

template <int N>
void JointMove<N>::sample(double t_s, double& s, double& s_d, double& s_dd) const {
  const double t = std::clamp(t_s, 0.0, duration_s);
  const double a = s_a_max;
  const double v = std::min(s_v_max, std::sqrt(a));  // the peak (triangle: sqrt(s_a_max))
  const double t_a = v / a;
  if (t < t_a) {
    s = 0.5 * a * t * t;
    s_d = a * t;
    s_dd = a;
  } else if (t <= duration_s - t_a) {
    s = 0.5 * a * t_a * t_a + v * (t - t_a);
    s_d = v;
    s_dd = 0.0;
  } else {
    const double t_r = duration_s - t;
    s = 1.0 - 0.5 * a * t_r * t_r;
    s_d = a * t_r;
    s_dd = -a;
  }
}

template <int N>
common::Result<JointMove<N>> plan_joint_move(const model::Model<N>& model,
                                             const Config<N>& config,
                                             const VecN<N>& q_start,
                                             const VecN<N>& q_goal,
                                             double speed) {
  using R = common::Result<JointMove<N>>;
  if (!speed_ok(speed)) {
    return R::error(format("move_joint: speed %g outside (0, 1]", speed));
  }
  const double band = config.safety.soft_margin_rad + config.manual.jog_margin_rad;
  for (int j = 0; j < N; ++j) {
    if (!std::isfinite(q_goal(j))) {
      return R::error(format("move_joint: goal joint %d not finite", j + 1));
    }
  }
  for (int j = 0; j < N; ++j) {
    const model::JointParams& p = model.joints[static_cast<std::size_t>(j)];
    const double to_min = q_goal(j) - p.q_min;
    const double to_max = p.q_max - q_goal(j);
    const double near = to_min < to_max ? p.q_min : p.q_max;
    if (std::min(to_min, to_max) < band) {
      return R::error(
          format("move_joint: goal joint %d %.4g rad is within %.3g rad of its limit %.4g",
                 j + 1,
                 q_goal(j),
                 band,
                 near));
    }
  }
  JointMove<N> mv;
  mv.q_start = q_start;
  mv.q_goal = q_goal;
  mv.s_v_max = std::numeric_limits<double>::infinity();
  mv.s_a_max = std::numeric_limits<double>::infinity();
  const VecN<N> delta = q_goal - q_start;
  if (delta.cwiseAbs().maxCoeff() <= kAtGoalRad) {
    return R::error("move_joint: already at the goal");
  }
  for (int j = 0; j < N; ++j) {
    const double d = std::abs(delta(j));
    if (d > 0.0) {
      const double qd_manual_max =
          config.manual.joint_speed_fraction * model.joints[static_cast<std::size_t>(j)].qd_max;
      mv.s_v_max = std::min(mv.s_v_max, speed * qd_manual_max / d);
      mv.s_a_max = std::min(mv.s_a_max, speed * config.manual.joint_a_max_rad_s2(j) / d);
    }
  }
  mv.duration_s = mv.s_v_max * mv.s_v_max / mv.s_a_max < 1.0
                      ? 1.0 / mv.s_v_max + mv.s_v_max / mv.s_a_max
                      : 2.0 * std::sqrt(1.0 / mv.s_a_max);

  const control::FrictionCompensation<N> friction(config.friction_compensation);
  const double dt = config.path_validation.dt_s;
  const double ratio_max = config.path_validation.torque_ratio;
  const auto steps = static_cast<long>(std::ceil(mv.duration_s / dt));
  for (long k = 0; k <= steps; ++k) {
    const double t = std::min(static_cast<double>(k) * dt, mv.duration_s);
    double s = 0.0;
    double s_d = 0.0;
    double s_dd = 0.0;
    mv.sample(t, s, s_d, s_dd);
    const VecN<N> q = q_start + s * delta;
    const VecN<N> qd = s_d * delta;
    const VecN<N> qdd = s_dd * delta;
    const VecN<N> tau = dynamics::rnea<N>(model, q, qd, qdd, model.gravity_b) + friction.torque(qd);
    for (int j = 0; j < N; ++j) {
      const double ratio = std::abs(tau(j)) / model.joints[static_cast<std::size_t>(j)].tau_max;
      if (ratio > ratio_max) {
        return R::error(format(
            "move_joint: torque joint %d %.3g > %.3g at t = %.3g s", j + 1, ratio, ratio_max, t));
      }
    }
  }
  return R::ok(mv);
}

template <int N>
common::Result<PathValidation<N>> plan_tcp_move(const model::Model<N>& model,
                                                const Config<N>& config,
                                                const VecN<N>& q_start,
                                                const common::Pose& T_b_start_tip,
                                                const common::Pose& T_b_goal_tip,
                                                double speed) {
  if (!speed_ok(speed)) {
    return common::Result<PathValidation<N>>::error(
        format("move_tcp: speed %g outside (0, 1]", speed));
  }
  const ManualConfig<N>& m = config.manual;
  planning::CartesianLimits limits;
  limits.v_max_m_s = m.tcp_v_max_m_s * speed;
  limits.a_max_m_s2 = m.tcp_a_max_m_s2 * speed;
  limits.w_max_rad_s = m.tcp_w_max_rad_s * speed;
  limits.alpha_max_rad_s2 = m.tcp_alpha_max_rad_s2 * speed;
  ValidationLimits checks = config.path_validation;
  checks.joint_margin_rad = config.safety.soft_margin_rad + m.jog_margin_rad;
  planning::Waypoint w;
  w.T_b_goal = T_b_goal_tip;
  w.duration_s = 0.0;  // auto
  w.stop = true;
  return validate_path<N>(model,
                          config.clik,
                          config.friction_compensation,
                          q_start,
                          T_b_start_tip,
                          std::vector<planning::Waypoint>{w},
                          limits,
                          checks);
}

template <int N>
bool joint_jog_step(
    ManualJog& jog, double q_lo, double q_hi, double dt, planning::JointReference<N>& ref) {
  const int j = jog.joint;
  const double dist = jog.direction > 0 ? q_hi - ref.q(j) : ref.q(j) - q_lo;
  const double v = next_velocity(jog, stop_cap(dist, jog.a_max, dt), dt);
  ref.q(j) += v * dt;
  // The cap lands q_ref on the limit; the snap only takes rounding. v (<= a_max dt there) ramps
  // to 0 on the next step, whose cap is 0.
  const double left = jog.direction > 0 ? q_hi - ref.q(j) : ref.q(j) - q_lo;
  if (left <= kLandedRad) {
    ref.q(j) = jog.direction > 0 ? q_hi : q_lo;
    jog.braking = jog.braking == ManualStop::kNone ? ManualStop::kSoftLimit : jog.braking;
  }
  ref.qdd(j) = (v - jog.v) / dt;
  ref.qd(j) = v;
  jog.v = v;
  return v == 0.0 && jog.braking != ManualStop::kNone;
}

bool tcp_jog_step(ManualJog& jog, double dt, planning::CartesianSample& x_d) {
  const double v = next_velocity(jog, std::numeric_limits<double>::infinity(), dt);
  const double a = (v - jog.v) / dt;
  jog.v = v;
  const bool rotation = jog.axis >= JogAxis::kRx;
  Vec3 lin = Vec3::Zero();  // v_tcp, a_tcp
  Vec3 lin_d = Vec3::Zero();
  Vec3 w = Vec3::Zero();  // w, alpha
  Vec3 alpha = Vec3::Zero();
  if (rotation) {
    const Vec3 phi = (v * dt) * jog.e;
    const common::Mat3 r = common::so3_exp(phi) * jog.T_b_tcp.R;
    jog.T_b_tcp.R = r;
    w = v * jog.e;
    alpha = a * jog.e;
  } else {
    jog.T_b_tcp.p += (v * dt) * jog.e;
    lin = v * jog.e;
    lin_d = a * jog.e;
  }
  x_d.T_b_d = common::compose(jog.T_b_tcp, jog.T_tcp_tip);
  const Vec3 r = x_d.T_b_d.p - jog.T_b_tcp.p;
  const Vec3 w_r = w.cross(r);
  x_d.xd_d << lin + w_r, w;
  x_d.xdd_d << lin_d + alpha.cross(r) + w.cross(w_r), alpha;
  return v == 0.0 && jog.braking != ManualStop::kNone;
}

template <int N>
bool tcp_jog_near_limit(const planning::JointReference<N>& ref,
                        const VecN<N>& q_lo,
                        const VecN<N>& q_hi,
                        const VecN<N>& a_max,
                        double speed) {
  for (int j = 0; j < N; ++j) {
    const double v = ref.qd(j);
    if (v == 0.0) {
      continue;
    }
    const double dist = v > 0.0 ? q_hi(j) - ref.q(j) : ref.q(j) - q_lo(j);
    if (dist - v * v / (2.0 * speed * a_max(j)) <= 0.0) {
      return true;
    }
  }
  return false;
}

#define COBOT_PIPELINE_MANUAL_INSTANTIATE(N)                                                     \
  template struct JointMove<(N)>;                                                                \
  template common::Result<JointMove<(N)>> plan_joint_move<(N)>(                                  \
      const model::Model<(N)>&, const Config<(N)>&, const VecN<(N)>&, const VecN<(N)>&, double); \
  template common::Result<PathValidation<(N)>> plan_tcp_move<(N)>(const model::Model<(N)>&,      \
                                                                  const Config<(N)>&,            \
                                                                  const VecN<(N)>&,              \
                                                                  const common::Pose&,           \
                                                                  const common::Pose&,           \
                                                                  double);                       \
  template bool joint_jog_step<(N)>(                                                             \
      ManualJog&, double, double, double, planning::JointReference<(N)>&);                       \
  template bool tcp_jog_near_limit<(N)>(const planning::JointReference<(N)>&,                    \
                                        const VecN<(N)>&,                                        \
                                        const VecN<(N)>&,                                        \
                                        const VecN<(N)>&,                                        \
                                        double);
COBOT_INSTANTIATE_DOF(COBOT_PIPELINE_MANUAL_INSTANTIATE)
#undef COBOT_PIPELINE_MANUAL_INSTANTIATE

}  // namespace cobot::pipeline
