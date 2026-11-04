#pragma once

// Shared by the studio_scene tests: the arm's URDF, the package map of the repository and pose
// comparisons.
#include <algorithm>
#include <filesystem>
#include <map>
#include <string>

#include <Eigen/Geometry>

#include "cobot/common/lie.hpp"
#include "cobot/common/types.hpp"

namespace cobot::studio::test {

namespace fs = std::filesystem;

inline const fs::path kRepo = COBOT_REPO_DIR;
inline const fs::path kUrdf = kRepo / "src/sim/urdf/cobot30_5.urdf";

// package://cobot_sim/<rest> -> <repo>/src/sim/<rest>
inline std::map<std::string, fs::path> repo_packages() {
  return {{"cobot_sim", kRepo / "src/sim"}};
}

// URDF <origin rpy>: R = Rz(yaw) Ry(pitch) Rx(roll), fixed axes.
inline common::Pose urdf_origin(const common::Vec3& xyz, const common::Vec3& rpy) {
  common::Pose T;
  T.R = (Eigen::AngleAxisd(rpy.z(), common::Vec3::UnitZ()) *
         Eigen::AngleAxisd(rpy.y(), common::Vec3::UnitY()) *
         Eigen::AngleAxisd(rpy.x(), common::Vec3::UnitX()))
            .toRotationMatrix();
  T.p = xyz;
  return T;
}

// Position distance in m.
inline double pos_err(const common::Pose& a, const common::Pose& b) {
  return (a.p - b.p).norm();
}

// Rotation angle of R_a R_b^T in rad.
inline double rot_err(const common::Pose& a, const common::Pose& b) {
  const common::Mat3 dR = a.R * b.R.transpose();
  return common::so3_log(dR).norm();
}

// Largest element difference of the rotation matrices and the positions.
inline double max_abs_diff(const common::Pose& a, const common::Pose& b) {
  return std::max((a.R - b.R).cwiseAbs().maxCoeff(), (a.p - b.p).cwiseAbs().maxCoeff());
}

}  // namespace cobot::studio::test
