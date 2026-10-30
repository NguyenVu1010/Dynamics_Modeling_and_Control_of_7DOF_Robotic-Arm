#pragma once

#include <Eigen/Core>

// Frame conventions: see docs/conventions.md.
//   T_a_b maps points from frame b to frame a:  p_a = T_a_b * p_b.
//   Frames: b base_link, i link i, e end effector, d desired end effector.
//   Twists and Jacobian rows are ordered [v; w] and expressed in b.
namespace cobot::common {

template <int N>
using VecN = Eigen::Matrix<double, N, 1>;
template <int N>
using MatN = Eigen::Matrix<double, N, N>;
template <int N>
using Mat6N = Eigen::Matrix<double, 6, N>;

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

// Rigid transform T_a_b stored as rotation + translation.
struct Pose {
  Mat3 R = Mat3::Identity();  // R_a_b
  Vec3 p = Vec3::Zero();      // p_a_b: origin of b expressed in a
};

// T_a_c = T_a_b * T_b_c
inline Pose compose(const Pose& T_a_b, const Pose& T_b_c) {
  return Pose{T_a_b.R * T_b_c.R, T_a_b.R * T_b_c.p + T_a_b.p};
}

// T_b_a from T_a_b
inline Pose inverse(const Pose& T_a_b) {
  const Mat3 R_b_a = T_a_b.R.transpose();
  return Pose{R_b_a, -(R_b_a * T_a_b.p)};
}

// p_a = T_a_b * p_b
inline Vec3 transform(const Pose& T_a_b, const Vec3& p_b) {
  return T_a_b.R * p_b + T_a_b.p;
}

// skew(a) * b == a.cross(b)
inline Mat3 skew(const Vec3& v) {
  Mat3 S;
  S << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return S;
}

}  // namespace cobot::common
