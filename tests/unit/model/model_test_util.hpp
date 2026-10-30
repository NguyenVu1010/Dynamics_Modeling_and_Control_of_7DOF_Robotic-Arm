#pragma once

#include <string>

#include <Eigen/Geometry>

#include "cobot/common/types.hpp"

// Helpers shared by the model tests: fixture paths and URDF <origin> math written
// independently of the parser (R = Rz(yaw) Ry(pitch) Rx(roll)).
namespace cobot::model::test {

using common::Mat3;
using common::Pose;
using common::Vec3;

inline std::string repo_path(const std::string& rel) {
  return std::string(COBOT_REPO_DIR) + "/" + rel;
}
inline std::string fixture(const std::string& name) {
  return repo_path("tests/data/urdf/" + name);
}
inline std::string real_urdf() {
  return repo_path("src/sim/urdf/cobot30_5.urdf");
}

inline Mat3 rpy(double r, double p, double y) {
  const Mat3 R = (Eigen::AngleAxisd(y, Vec3::UnitZ()) * Eigen::AngleAxisd(p, Vec3::UnitY()) *
                  Eigen::AngleAxisd(r, Vec3::UnitX()))
                     .toRotationMatrix();
  return R;
}

inline Pose origin(const Vec3& xyz, const Vec3& rpy_rad) {
  return Pose{rpy(rpy_rad.x(), rpy_rad.y(), rpy_rad.z()), xyz};
}

// URDF <inertia> attributes as a symmetric tensor.
inline Mat3 inertia(double ixx, double ixy, double ixz, double iyy, double iyz, double izz) {
  Mat3 I;
  I << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz;
  return I;
}

inline double pose_diff(const Pose& a, const Pose& b) {
  return (a.R - b.R).norm() + (a.p - b.p).norm();
}

}  // namespace cobot::model::test
