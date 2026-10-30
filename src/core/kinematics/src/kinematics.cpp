#include "cobot/kinematics/kinematics.hpp"

#include <cstddef>

#include <Eigen/Geometry>

#include "cobot/common/dof.hpp"

// Derivation: docs/derivations/jacobian.md.
namespace cobot::kinematics {

using common::Mat3;

template <int N>
void forward_kinematics(const Model<N>& model, const VecN<N>& q, FkState<N>& out) {
  Pose T_b_prev;  // T_b_{-1} = identity
  for (std::size_t i = 0; i < static_cast<std::size_t>(N); ++i) {
    const model::JointParams& joint = model.joints[i];
    // Joint frame at q = 0, then the rotation about the joint axis through its origin.
    const Pose T_b_joint = common::compose(T_b_prev, joint.T_parent_joint);
    const Mat3 R_joint_i =
        Eigen::AngleAxisd(q(static_cast<Eigen::Index>(i)), joint.axis_joint).toRotationMatrix();
    out.T_b_i[i].R = T_b_joint.R * R_joint_i;
    out.T_b_i[i].p = T_b_joint.p;
    out.p_b_joint[i] = T_b_joint.p;
    out.axis_b[i] = T_b_joint.R * joint.axis_joint;
    T_b_prev = out.T_b_i[i];
  }
  out.T_b_e = common::compose(T_b_prev, model.T_last_tip);
}

template <int N>
void geometric_jacobian(const FkState<N>& fk, Mat6N<N>& J_e_b) {
  const Vec3& p_b_e = fk.T_b_e.p;
  for (std::size_t i = 0; i < static_cast<std::size_t>(N); ++i) {
    const auto c = static_cast<Eigen::Index>(i);
    const Vec3& z = fk.axis_b[i];
    J_e_b.template block<3, 1>(0, c) = z.cross(p_b_e - fk.p_b_joint[i]);
    J_e_b.template block<3, 1>(3, c) = z;
  }
}

// With qdd = 0, column i of J changes only through its axis and lever arm:
//   d/dt z_i = w_i x z_i,  d/dt (p_e - p_i) = v_e - v_i,
// where w_i is the angular velocity of link i and v_i the velocity of joint origin i.
template <int N>
Vec6 jdot_qdot(const Model<N>& /*model*/, const FkState<N>& fk, const VecN<N>& qd) {
  constexpr auto kN = static_cast<std::size_t>(N);
  const Vec3& p_b_e = fk.T_b_e.p;

  // Pass 1: joint origin velocities v_i and the tip velocity v_e. Origin i is fixed in link
  // i - 1, so v_i = v_{i-1} + w_{i-1} x (p_i - p_{i-1}); origin 0 is fixed in b.
  std::array<Vec3, N> v_joint;
  std::array<Vec3, N> w_link;
  Vec3 w = Vec3::Zero();
  Vec3 v = Vec3::Zero();
  for (std::size_t i = 0; i < kN; ++i) {
    if (i > 0) {
      v += w.cross(fk.p_b_joint[i] - fk.p_b_joint[i - 1]);
    }
    v_joint[i] = v;
    w += fk.axis_b[i] * qd(static_cast<Eigen::Index>(i));
    w_link[i] = w;
  }
  const Vec3 v_e = v + w.cross(p_b_e - fk.p_b_joint[kN - 1]);

  // Pass 2: sum the column derivatives weighted by qd_i.
  Vec3 a_lin = Vec3::Zero();
  Vec3 a_ang = Vec3::Zero();
  for (std::size_t i = 0; i < kN; ++i) {
    const double qd_i = qd(static_cast<Eigen::Index>(i));
    const Vec3& z = fk.axis_b[i];
    const Vec3 z_dot = w_link[i].cross(z);
    a_lin += (z_dot.cross(p_b_e - fk.p_b_joint[i]) + z.cross(v_e - v_joint[i])) * qd_i;
    a_ang += z_dot * qd_i;
  }
  Vec6 out;
  out.head<3>() = a_lin;
  out.tail<3>() = a_ang;
  return out;
}

#define COBOT_KINEMATICS_INSTANTIATE(N)                                                      \
  template void forward_kinematics<(N)>(const Model<(N)>&, const VecN<(N)>&, FkState<(N)>&); \
  template void geometric_jacobian<(N)>(const FkState<(N)>&, Mat6N<(N)>&);                   \
  template Vec6 jdot_qdot<(N)>(const Model<(N)>&, const FkState<(N)>&, const VecN<(N)>&);
COBOT_INSTANTIATE_DOF(COBOT_KINEMATICS_INSTANTIATE)
#undef COBOT_KINEMATICS_INSTANTIATE

}  // namespace cobot::kinematics
