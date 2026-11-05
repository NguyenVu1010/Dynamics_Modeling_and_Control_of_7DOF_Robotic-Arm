#pragma once

#include <limits>

#include "cobot/common/types.hpp"
#include "cobot/model/model.hpp"
#include "cobot/planning/cartesian_trajectory.hpp"

// Second-order closed-loop inverse kinematics run on the reference itself (feature
// `planning-clik`, design 5.4). It never reads the measured state after reset():
//   e    = pose_error(T_b_d, FK(q_ref)),  edot = xd_d - J(q_ref) qd_ref
//   qdd_ref = J#_lambda (xdd_d + kd .* edot + kp .* e - Jdot qd_ref)
//           + (I - J#_lambda J) (-k_null grad_h(q_ref) - k_damp qd_ref)
//   qd_ref += qdd_ref dt;  q_ref += qd_ref dt  (semi-implicit Euler), q_ref clamped to limits.
// J#_lambda is the damped pseudo-inverse; lambda grows from 0 to lambda_max as the
// condition number of J goes above cond_threshold. h(q) penalises distance to joint limits.
namespace cobot::planning {

using common::Vec6;
using common::VecN;
using model::Model;

struct ClikConfig {
  Vec6 kp = Vec6::Constant(100.0);  // 1/s^2
  Vec6 kd = Vec6::Constant(20.0);   // 1/s
  double k_null = 1.0;              // 1/s^2
  double k_damp = 2.0;              // 1/s
  double lambda_max = 0.05;         // damping of the pseudo-inverse
  double cond_threshold = 100.0;    // condition number where damping starts
};

template <int N>
struct JointReference {
  VecN<N> q = VecN<N>::Zero();     // rad
  VecN<N> qd = VecN<N>::Zero();    // rad/s
  VecN<N> qdd = VecN<N>::Zero();   // rad/s^2
  Vec6 task_error = Vec6::Zero();  // e above, [m; rad]
};

// sigma_max / sigma_min over the min(6, N) singular values of J (JacobiSVD); infinite when J loses
// rank. Allocation-free for fixed N.
template <int N>
double jacobian_condition_number(const common::Mat6N<N>& J);

template <int N>
class ClikReference {
 public:
  ClikReference(const Model<N>& model, const ClikConfig& config);

  // Start from the measured configuration with zero velocity.
  void reset(const VecN<N>& q_measured);
  const JointReference<N>& step(const CartesianSample& x_d, double dt_s);
  const JointReference<N>& reference() const { return ref_; }
  // Condition number of J(q_ref) computed in the last step(); NaN before the first step, kept by
  // reset().
  double condition_number() const { return condition_number_; }

 private:
  Model<N> model_;
  ClikConfig config_;
  JointReference<N> ref_;
  double condition_number_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace cobot::planning
