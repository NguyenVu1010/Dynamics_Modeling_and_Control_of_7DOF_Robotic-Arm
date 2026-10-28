#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"

// Robot model read from URDF. Implemented in feature `model-urdf` (tinyxml2).
// Only revolute joints are kept; fixed joints are merged into their parent link:
// masses add, inertias move with the parallel-axis theorem, and the fixed joint's
// <origin> is composed into the origin of the next revolute joint.
namespace cobot::model {

using common::Mat3;
using common::Pose;
using common::Result;
using common::Vec3;

// Inertial parameters of the link that a joint moves (after merging fixed children).
struct LinkInertia {
  double mass_kg = 0.0;
  Vec3 com_i = Vec3::Zero();      // centre of mass in the link frame i, m
  Mat3 I_com_i = Mat3::Zero();    // inertia about the centre of mass, axes of frame i, kg m^2
};

struct JointParams {
  std::string name;
  Pose T_parent_joint;            // joint frame in the parent link frame (URDF <origin>), q = 0
  Vec3 axis_joint = Vec3::UnitZ();  // unit rotation axis in the joint frame
  double q_min = 0.0;             // rad
  double q_max = 0.0;             // rad
  double qd_max = 0.0;            // rad/s
  double tau_max = 0.0;           // N m
  double damping = 0.0;           // N m s / rad (URDF <dynamics damping>)
};

// Variable-length chain as parsed, before the DOF is checked.
struct UrdfChain {
  std::vector<JointParams> joints;  // base -> tip order
  std::vector<LinkInertia> links;   // links[k] is moved by joints[k]
  Pose T_last_tip;                  // tip link frame in the frame of the last revolute joint
};

// Parse the serial chain from `base_link` to `tip_link`. Errors name the element and
// attribute (e.g. "joint_3: <limit effort> missing"). Non-standard attributes are ignored.
Result<UrdfChain> parse_urdf_chain(const std::filesystem::path& urdf, const std::string& base_link,
                                   const std::string& tip_link);

template <int N>
struct Model {
  std::array<JointParams, N> joints;
  std::array<LinkInertia, N> links;
  Pose T_last_tip;
  Vec3 gravity_b = Vec3(0.0, 0.0, -9.80665);  // m/s^2 in b
};

// Error "expected N revolute joints, got M" when the chain length differs from N.
template <int N>
Result<Model<N>> make_model(const UrdfChain& chain, const Vec3& gravity_b);

inline constexpr int kArmDof = 7;
using Arm7 = Model<kArmDof>;

}  // namespace cobot::model
