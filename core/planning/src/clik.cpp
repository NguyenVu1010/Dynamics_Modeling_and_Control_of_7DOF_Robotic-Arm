#include "cobot/planning/clik.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include <Eigen/Cholesky>
#include <Eigen/SVD>

#include "cobot/common/dof.hpp"
#include "cobot/common/lie.hpp"
#include "cobot/kinematics/kinematics.hpp"

// Derivation: docs/derivations/control.md (CLIK law, damped pseudo-inverse, joint-limit cost).
namespace cobot::planning {

using common::Mat6;
using common::Mat6N;
using common::MatN;

namespace {

template <int N>
using MatN6 = Eigen::Matrix<double, N, 6>;

// sigma_max / sigma_min over the min(6, N) singular values of J; infinite when J loses rank.
template <int N>
double condition_number(const Mat6N<N>& J) {
  if constexpr (N == 1) {  // one singular value, |J|
    return J.norm() > 0.0 ? 1.0 : std::numeric_limits<double>::infinity();
  } else {
    const Eigen::JacobiSVD<Mat6N<N>> svd(J);
    const typename Eigen::JacobiSVD<Mat6N<N>>::SingularValuesType& sv = svd.singularValues();
    const double sv_min = sv(sv.size() - 1);
    if (!(sv_min > 0.0)) {
      return std::numeric_limits<double>::infinity();
    }
    return sv(0) / sv_min;
  }
}

// lambda = 0 up to cond_threshold, then a linear ramp that reaches lambda_max at twice the
// threshold. A NaN condition number (J not finite) takes the fully damped branch.
double damping(double cond, const ClikConfig& config) {
  if (cond <= config.cond_threshold) {
    return 0.0;
  }
  const double ramp = (cond - config.cond_threshold) / config.cond_threshold;
  return config.lambda_max * (ramp < 1.0 ? ramp : 1.0);
}

// J# = J^T (J J^T + lambda^2 I)^-1 (6x6 LDLT) for N >= 6. For N < 6 the same matrix is
// computed as (J^T J + lambda^2 I)^-1 J^T (N x N LDLT), which stays regular at lambda = 0
// when J has full column rank (the planar test arm).
template <int N>
MatN6<N> damped_pinv(const Mat6N<N>& J, double lambda) {
  const double l2 = lambda * lambda;
  MatN6<N> J_pinv;
  if constexpr (N >= 6) {
    Mat6 A = J * J.transpose();
    A.diagonal().array() += l2;
    const Eigen::LDLT<Mat6> ldlt(A);
    const Mat6N<N> X = ldlt.solve(J);  // (J J^T + l2 I)^-1 J, A symmetric
    J_pinv = X.transpose();
  } else {
    MatN<N> A = J.transpose() * J;
    A.diagonal().array() += l2;
    const Eigen::LDLT<MatN<N>> ldlt(A);
    const MatN6<N> J_t = J.transpose();
    J_pinv = ldlt.solve(J_t);
  }
  return J_pinv;
}

// grad h, h(q) = sum_i ((q_i - mid_i) / range_i)^2 / (2N). Joints without finite limits
// (continuous) contribute nothing.
template <int N>
VecN<N> joint_limit_gradient(const Model<N>& model, const VecN<N>& q) {
  VecN<N> grad = VecN<N>::Zero();
  for (std::size_t i = 0; i < static_cast<std::size_t>(N); ++i) {
    const model::JointParams& joint = model.joints[i];
    const double range = joint.q_max - joint.q_min;
    if (!std::isfinite(range) || !(range > 0.0)) {
      continue;
    }
    const double mid = 0.5 * (joint.q_min + joint.q_max);
    const auto r = static_cast<Eigen::Index>(i);
    grad(r) = (q(r) - mid) / (range * range * static_cast<double>(N));
  }
  return grad;
}

}  // namespace

template <int N>
ClikReference<N>::ClikReference(const Model<N>& model, const ClikConfig& config) {
  // Copied in the body: the header fixes the const& signature, so there is nothing to move.
  model_ = model;
  config_ = config;
}

template <int N>
void ClikReference<N>::reset(const VecN<N>& q_measured) {
  for (std::size_t i = 0; i < static_cast<std::size_t>(N); ++i) {
    const auto r = static_cast<Eigen::Index>(i);
    ref_.q(r) = std::clamp(q_measured(r), model_.joints[i].q_min, model_.joints[i].q_max);
  }
  ref_.qd.setZero();
  ref_.qdd.setZero();
  ref_.task_error.setZero();
}

template <int N>
const JointReference<N>& ClikReference<N>::step(const CartesianSample& x_d, double dt_s) {
  kinematics::FkState<N> fk;
  kinematics::forward_kinematics(model_, ref_.q, fk);
  Mat6N<N> J;
  kinematics::geometric_jacobian(fk, J);
  const Vec6 jdot_qd = kinematics::jdot_qdot(model_, fk, ref_.qd);

  const Vec6 e = common::pose_error(x_d.T_b_d, fk.T_b_e);
  const Vec6 edot = x_d.xd_d - J * ref_.qd;
  const Vec6 task_acc =
      x_d.xdd_d + config_.kd.cwiseProduct(edot) + config_.kp.cwiseProduct(e) - jdot_qd;

  const double lambda = damping(condition_number<N>(J), config_);
  const MatN6<N> J_pinv = damped_pinv<N>(J, lambda);
  const MatN<N> null_proj = MatN<N>::Identity() - J_pinv * J;
  const VecN<N> null_acc =
      -config_.k_null * joint_limit_gradient<N>(model_, ref_.q) - config_.k_damp * ref_.qd;

  ref_.task_error = e;
  ref_.qdd = J_pinv * task_acc + null_proj * null_acc;

  // Semi-implicit Euler, then keep q_ref inside the limits without velocity into a limit.
  ref_.qd += ref_.qdd * dt_s;
  ref_.q += ref_.qd * dt_s;
  for (std::size_t i = 0; i < static_cast<std::size_t>(N); ++i) {
    const auto r = static_cast<Eigen::Index>(i);
    const model::JointParams& joint = model_.joints[i];
    if (ref_.q(r) >= joint.q_max) {
      ref_.q(r) = joint.q_max;
      ref_.qd(r) = std::min(ref_.qd(r), 0.0);
    } else if (ref_.q(r) <= joint.q_min) {
      ref_.q(r) = joint.q_min;
      ref_.qd(r) = std::max(ref_.qd(r), 0.0);
    }
  }
  return ref_;
}

#define COBOT_CLIK_INSTANTIATE(N) template class ClikReference<(N)>;
COBOT_INSTANTIATE_DOF(COBOT_CLIK_INSTANTIATE)
#undef COBOT_CLIK_INSTANTIATE

}  // namespace cobot::planning
