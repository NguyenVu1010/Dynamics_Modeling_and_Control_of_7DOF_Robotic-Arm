#include "cobot/imperfect/actuator_stage.hpp"

#include <algorithm>
#include <cmath>

#include "cobot/common/dof.hpp"

namespace cobot::imperfect {
namespace {

constexpr double kDriveEfficiency = 0.85;  // electrical -> mechanical power, bus sag model

// Rounds x to the nearest multiple of step; step <= 0 leaves x unchanged.
double round_to(double x, double step) {
  return step > 0.0 ? std::round(x / step) * step : x;
}

// Torque ceiling of the torque-speed curve at velocity w for torque tau: `peak` up to `knee`,
// linear to 0 at `zero` when motoring (tau * w > 0); braking is capped at `peak` only.
double torque_ceiling(double tau, double w, double peak, double knee, double zero) {
  const double speed = std::abs(w);
  if (tau * w <= 0.0 || speed <= knee) {
    return peak;
  }
  return speed < zero ? peak * (zero - speed) / (zero - knee) : 0.0;
}

}  // namespace

template <int N>
ActuatorStage<N>::ActuatorStage(const ActuatorConfig<N>& act,
                                const common::VecN<N>& torque_scale,
                                const common::VecN<N>& peak_torque_nm,
                                double period_s)
    : act_(act),
      torque_scale_(torque_scale),
      peak_torque_(peak_torque_nm),
      period_s_(period_s),
      bus_voltage_v_(act.bus.v_nom_v > 0.0 ? act.bus.v_nom_v : 1.0) {}

template <int N>
common::VecN<N> ActuatorStage<N>::shape(const common::VecN<N>& tau_in,
                                        const common::VecN<N>& q_true) const {
  common::VecN<N> out;
  for (int j = 0; j < N; ++j) {
    double tau = tau_in[j] * torque_scale_[j];
    if (act_.torque_scale_asym_frac[j] != 0.0 && tau < 0.0) {
      tau *= 1.0 - act_.torque_scale_asym_frac[j];
    }
    if (act_.torque_ripple_frac[j] != 0.0) {
      tau *=
          1.0 + act_.torque_ripple_frac[j] * std::sin(act_.ripple_periods_per_rev[j] * q_true[j]);
    }
    if (std::abs(tau) < act_.deadband_nm[j]) {
      tau = 0.0;
    }
    out[j] = round_to(tau, act_.torque_cmd_resolution_nm[j]);
  }
  return out;
}

template <int N>
common::VecN<N> ActuatorStage<N>::limit(const common::VecN<N>& tau_in,
                                        const common::VecN<N>& qd_true) {
  // Bus voltage from the previous output torques (no algebraic loop); off: V = V_nom = 1.
  const double v_nom = act_.bus.v_nom_v > 0.0 ? act_.bus.v_nom_v : 1.0;
  double v_bus = v_nom;
  if (act_.bus.v_nom_v > 0.0) {
    double power_w = 0.0;
    for (int j = 0; j < N; ++j) {
      power_w += std::max(tau_prev_out_[j] * qd_true[j], 0.0);
    }
    const double disc = v_nom * v_nom - 4.0 * act_.bus.r_source_ohm * power_w / kDriveEfficiency;
    v_bus =
        disc < 0.0 ? act_.bus.v_min_v : std::max(act_.bus.v_min_v, (v_nom + std::sqrt(disc)) / 2.0);
  }
  bus_voltage_v_ = v_bus;
  common::VecN<N> out;
  for (int j = 0; j < N; ++j) {
    double tau = tau_in[j];
    const double slew = act_.torque_slew_nm_s[j];
    if (slew > 0.0) {
      const double step = slew * period_s_;
      tau = tau_prev_out_[j] + std::clamp(tau - tau_prev_out_[j], -step, step);
    }
    const double knee = act_.knee_speed_rad_s[j];
    if (knee > 0.0) {
      const double ceiling = torque_ceiling(tau,
                                            qd_true[j],
                                            peak_torque_[j],
                                            knee * v_bus / v_nom,
                                            act_.zero_torque_speed_rad_s[j] * v_bus / v_nom);
      tau = std::copysign(std::min(std::abs(tau), ceiling), tau);
    }
    out[j] = tau;
    tau_prev_out_[j] = tau;
  }
  return out;
}

#define COBOT_INSTANTIATE(N) template class ActuatorStage<N>;
COBOT_INSTANTIATE_DOF(COBOT_INSTANTIATE)
#undef COBOT_INSTANTIATE

}  // namespace cobot::imperfect
