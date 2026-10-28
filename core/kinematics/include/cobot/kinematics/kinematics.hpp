#pragma once

#include <array>

#include "cobot/common/lie.hpp"
#include "cobot/common/types.hpp"
#include "cobot/model/model.hpp"

// Forward kinematics and Jacobians. Implemented in feature `kinematics` (src/, explicit
// instantiation via COBOT_INSTANTIATE_DOF, see cobot/common/dof.hpp). No heap allocation.
namespace cobot::kinematics {

using common::Mat6N;
using common::Pose;
using common::Vec3;
using common::Vec6;
using common::VecN;
using model::Model;

template <int N>
struct FkState {
  std::array<Pose, N> T_b_i;       // link frame i (= joint frame after rotation q_i) in b
  std::array<Vec3, N> axis_b;      // unit joint axis i in b
  std::array<Vec3, N> p_b_joint;   // joint i origin in b
  Pose T_b_e;                      // end effector (tip) in b
};

template <int N>
void forward_kinematics(const Model<N>& model, const VecN<N>& q, FkState<N>& out);

// Geometric Jacobian of the tip, rows [v; w], expressed in b:  [v; w] = J_e_b * qd.
template <int N>
void geometric_jacobian(const FkState<N>& fk, Mat6N<N>& J_e_b);

// d/dt(J_e_b) * qd, from acceleration propagation with qdd = 0 (no finite differences).
template <int N>
Vec6 jdot_qdot(const Model<N>& model, const FkState<N>& fk, const VecN<N>& qd);

}  // namespace cobot::kinematics
