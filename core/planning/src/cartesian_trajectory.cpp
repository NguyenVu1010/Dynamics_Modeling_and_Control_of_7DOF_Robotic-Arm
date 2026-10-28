#include "cobot/planning/cartesian_trajectory.hpp"

#include "cobot/common/lie.hpp"

namespace cobot::planning {

using common::Vec3;

CartesianTrajectory::CartesianTrajectory(const Pose& T_b_start,
                                         const Pose& T_b_goal,
                                         double duration_s)
    : T_b_start_(T_b_start),
      T_b_goal_(T_b_goal),
      duration_s_(duration_s),
      dp_(T_b_goal.p - T_b_start.p),
      phi_(common::so3_log(T_b_start.R.transpose() * T_b_goal.R)),
      w_b_(T_b_start.R * phi_) {}

CartesianSample CartesianTrajectory::sample(double t_s) const {
  CartesianSample out;
  // Hold the endpoints exactly (zero twist and acceleration from the default members).
  if (t_s <= 0.0) {
    out.T_b_d = T_b_start_;
    return out;
  }
  if (t_s >= duration_s_) {
    out.T_b_d = T_b_goal_;
    return out;
  }

  // Quintic time scaling s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5. Both holds have returned, so
  // tau = t / T already lies in [0, 1] and needs no clamp.
  const double tau = t_s / duration_s_;
  const double tau2 = tau * tau;
  const double tau3 = tau2 * tau;
  const double s = tau3 * (10.0 + tau * (-15.0 + 6.0 * tau));
  const double ds_dtau = tau2 * (30.0 + tau * (-60.0 + 30.0 * tau));
  const double d2s_dtau2 = tau * (60.0 + tau * (-180.0 + 120.0 * tau));
  const double sd = ds_dtau / duration_s_;
  const double sdd = d2s_dtau2 / (duration_s_ * duration_s_);

  out.T_b_d.p = T_b_start_.p + s * dp_;
  const Vec3 phi_s = s * phi_;
  out.T_b_d.R = T_b_start_.R * common::so3_exp(phi_s);

  // Rotation about a constant axis: the angular velocity in b is R0 * (sdot * phi).
  out.xd_d.head<3>() = sd * dp_;
  out.xd_d.tail<3>() = sd * w_b_;
  out.xdd_d.head<3>() = sdd * dp_;
  out.xdd_d.tail<3>() = sdd * w_b_;
  return out;
}

}  // namespace cobot::planning
