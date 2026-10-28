#include "cobot/common/lie.hpp"

#include <algorithm>
#include <cmath>

namespace cobot::common {
namespace {

// Below this rotation angle (rad) the Rodrigues coefficients use their Taylor series.
constexpr double kSmallAngleRad = 1e-8;
// Below this cosine of the rotation angle the log uses the symmetric part of R (angle > ~2.69 rad),
// where the antisymmetric part is too small to give an accurate axis.
constexpr double kNearPiCos = -0.9;
// Antisymmetric component magnitude treated as rounding noise when picking the axis sign at pi.
constexpr double kPiSignNoise = 1e-14;

// vee(R - R^T) / 2 = sin(angle) * axis
Vec3 antisymmetric_vee(const Mat3& R) {
  return Vec3(0.5 * (R(2, 1) - R(1, 2)), 0.5 * (R(0, 2) - R(2, 0)), 0.5 * (R(1, 0) - R(0, 1)));
}

}  // namespace

Mat3 so3_exp(const Vec3& phi) {
  const double theta2 = phi.squaredNorm();
  const double theta = std::sqrt(theta2);
  double a = 0.0;  // sin(theta) / theta
  double b = 0.0;  // (1 - cos(theta)) / theta^2
  if (theta < kSmallAngleRad) {
    a = 1.0 - theta2 / 6.0;
    b = 0.5 - theta2 / 24.0;
  } else {
    const double half_sin = std::sin(0.5 * theta);
    a = std::sin(theta) / theta;
    b = 2.0 * half_sin * half_sin / theta2;
  }
  const Mat3 K = skew(phi);
  const Mat3 K2 = phi * phi.transpose() - theta2 * Mat3::Identity();
  Mat3 R = Mat3::Identity();
  R += a * K;
  R += b * K2;
  return R;
}

Vec3 so3_log(const Mat3& R) {
  const Vec3 v = antisymmetric_vee(R);
  const double s = v.norm();
  const double c = std::clamp(0.5 * (R.trace() - 1.0), -1.0, 1.0);
  const double theta = std::atan2(s, c);
  if (c >= kNearPiCos) {
    if (theta < kSmallAngleRad) {
      // theta / sin(theta) ~ 1 + theta^2 / 6
      return v * (1.0 + s * s / 6.0);
    }
    return v * (theta / s);
  }
  // Near pi: (R + R^T) / 2 = c I + (1 - c) a a^T. Use the column with the largest diagonal.
  const Mat3 S = 0.5 * (R + R.transpose());
  Eigen::Index k = 0;
  S.diagonal().maxCoeff(&k);
  Vec3 axis = S.col(k);
  axis(k) -= c;
  axis /= std::sqrt(std::max((1.0 - c) * axis(k), 0.0));
  axis.normalize();
  // axis(k) >= 0 here; sin(theta) * axis(k) = v(k) fixes the sign unless theta is pi.
  if (v(k) < -kPiSignNoise) {
    axis = -axis;
  }
  return axis * theta;
}

Vec6 pose_error(const Pose& T_b_desired, const Pose& T_b_actual) {
  const Mat3 R_err = T_b_desired.R * T_b_actual.R.transpose();
  Vec6 e;
  e.head<3>() = T_b_desired.p - T_b_actual.p;
  e.tail<3>() = so3_log(R_err);
  return e;
}

}  // namespace cobot::common
