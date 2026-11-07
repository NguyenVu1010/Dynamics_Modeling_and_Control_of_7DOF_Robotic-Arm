#pragma once

#include "cobot/common/types.hpp"
#include "cobot/imperfect/config.hpp"

// Per-joint torque shaping of the actuator group (realism-2 design 6.1), shared by
// ImperfectJointInterface::write (torque commands) and the simulated drive (pipeline::SimDrive,
// feature drive-loop). The decorator's constructor validates the config; the torque scale is drawn
// there. No allocation, no exception.
namespace cobot::imperfect {

template <int N>
class ActuatorStage {
 public:
  ActuatorStage(const ActuatorConfig<N>& act,
                const common::VecN<N>& torque_scale,
                const common::VecN<N>& peak_torque_nm,
                double period_s);

  // scale (+ asym) -> ripple at q_true -> deadband -> resolution rounding. Stateless.
  common::VecN<N> shape(const common::VecN<N>& tau, const common::VecN<N>& q_true) const;
  // slew over period_s -> bus voltage from the previous output and qd_true -> torque-speed
  // ceiling at qd_true. Updates the previous output and the bus voltage.
  common::VecN<N> limit(const common::VecN<N>& tau, const common::VecN<N>& qd_true);
  // Bus voltage of the last limit(), V; v_nom_v before it, the normalised 1.0 with the bus off.
  double bus_voltage_v() const { return bus_voltage_v_; }
  // Period of the next limit() calls (s): the drive sets its actual sub-step on a jittered period.
  void set_period(double period_s) { period_s_ = period_s; }

 private:
  ActuatorConfig<N> act_;
  common::VecN<N> torque_scale_;
  common::VecN<N> peak_torque_;
  double period_s_;
  common::VecN<N> tau_prev_out_ = common::VecN<N>::Zero();
  double bus_voltage_v_ = 1.0;
};

}  // namespace cobot::imperfect
