#pragma once

#include "cobot/common/types.hpp"

// SO(3) helpers used by kinematics and planning. Implemented in feature `common-lie`
// (core/common/src/lie.cpp). All angles in radians.
namespace cobot::common {

// Rodrigues: rotation matrix of the rotation vector phi (axis * angle).
// Exact for |phi| -> 0 (uses the Taylor series below 1e-8 rad).
Mat3 so3_exp(const Vec3& phi);

// Inverse of so3_exp for a proper rotation matrix; returns phi with |phi| in [0, pi].
// At |phi| = pi the axis sign is chosen with a non-negative largest component.
Vec3 so3_log(const Mat3& R);

// Task-space error between a desired and an actual pose, both in b:
//   [ p_d - p ; so3_log(R_d * R^T) ]   (position m, orientation rad)
Vec6 pose_error(const Pose& T_b_desired, const Pose& T_b_actual);

}  // namespace cobot::common
