#include <array>
#include <cstddef>

#include <Eigen/Geometry>

#include "cobot/common/dof.hpp"
#include "cobot/dynamics/dynamics.hpp"
#include "cobot/kinematics/kinematics.hpp"

// Recursive Newton-Euler with 3D vectors, every quantity expressed in b.
// Derivation: docs/derivations/rnea.md.
namespace cobot::dynamics {

using common::Mat3;

template <int N>
VecN<N> rnea(const Model<N>& model,
             const VecN<N>& q,
             const VecN<N>& qd,
             const VecN<N>& qdd,
             const Vec3& gravity_b) {
  constexpr auto kN = static_cast<std::size_t>(N);
  kinematics::FkState<N> fk;
  kinematics::forward_kinematics(model, q, fk);

  // Forward pass: w, w_dot of link i, acceleration a of joint origin i (gravity enters as
  // the base acceleration -g), then the inertial force F and moment about the CoM Nc.
  std::array<Vec3, N> force;   // F_i = m_i a_ci
  std::array<Vec3, N> moment;  // N_i = I_i w_dot_i + w_i x (I_i w_i), about the CoM
  std::array<Vec3, N> r_com;   // c_i - p_i
  Vec3 w = Vec3::Zero();
  Vec3 w_dot = Vec3::Zero();
  Vec3 a = -gravity_b;
  for (std::size_t i = 0; i < kN; ++i) {
    const auto k = static_cast<Eigen::Index>(i);
    if (i > 0) {
      const Vec3 d = fk.p_b_joint[i] - fk.p_b_joint[i - 1];
      a += w_dot.cross(d) + w.cross(w.cross(d));
    }
    const Vec3& z = fk.axis_b[i];
    w_dot += z * qdd(k) + w.cross(z) * qd(k);
    w += z * qd(k);

    const model::LinkInertia& link = model.links[i];
    const Mat3& R = fk.T_b_i[i].R;
    r_com[i] = R * link.com_i;
    const Mat3 I_b = R * link.I_com_i * R.transpose();
    const Vec3 a_com = a + w_dot.cross(r_com[i]) + w.cross(w.cross(r_com[i]));
    force[i] = link.mass_kg * a_com;
    moment[i] = I_b * w_dot + w.cross(I_b * w);
  }

  // Backward pass: force f and moment n about joint origin i exerted by link i - 1 on i.
  VecN<N> tau;
  Vec3 f = Vec3::Zero();
  Vec3 n = Vec3::Zero();
  for (std::size_t i = kN; i-- > 0;) {
    if (i + 1 < kN) {
      n += (fk.p_b_joint[i + 1] - fk.p_b_joint[i]).cross(f);
    }
    f += force[i];
    n += moment[i] + r_com[i].cross(force[i]);
    tau(static_cast<Eigen::Index>(i)) = fk.axis_b[i].dot(n);
  }
  return tau;
}

#define COBOT_RNEA_INSTANTIATE(N) \
  template VecN<(N)> rnea<(N)>(   \
      const Model<(N)>&, const VecN<(N)>&, const VecN<(N)>&, const VecN<(N)>&, const Vec3&);
COBOT_INSTANTIATE_DOF(COBOT_RNEA_INSTANTIATE)
#undef COBOT_RNEA_INSTANTIATE

}  // namespace cobot::dynamics
