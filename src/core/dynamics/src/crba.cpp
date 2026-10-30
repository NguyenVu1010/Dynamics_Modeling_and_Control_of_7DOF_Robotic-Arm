// clang-format off: the interface header goes first so this file proves it compiles on its own.
#include "cobot/dynamics/dynamics.hpp"
// clang-format on

#include <cstddef>

#include <Eigen/Geometry>

#include "cobot/common/dof.hpp"
#include "cobot/kinematics/kinematics.hpp"

// Composite rigid body algorithm with 3D vectors, every quantity expressed in b.
// Derivation: docs/derivations/crba.md.
namespace cobot::dynamics {

using common::Mat3;
using common::skew;

template <int N>
MatN<N> mass_matrix(const Model<N>& model, const VecN<N>& q) {
  constexpr auto kN = static_cast<std::size_t>(N);
  kinematics::FkState<N> fk;
  kinematics::forward_kinematics(model, q, fk);

  // Composite body C_i = links i..N-1: mass m, first moment h = sum m_k c_k and rotational
  // inertia I_o about the origin of b, accumulated from the tip.
  MatN<N> M;
  double m = 0.0;
  Vec3 h = Vec3::Zero();
  Mat3 I_o = Mat3::Zero();
  for (std::size_t i = kN; i-- > 0;) {
    const model::LinkInertia& link = model.links[i];
    const common::Pose& T_b_i = fk.T_b_i[i];
    const Vec3 c = common::transform(T_b_i, link.com_i);
    const Mat3 skew_c = skew(c);
    m += link.mass_kg;
    h += link.mass_kg * c;
    I_o += T_b_i.R * link.I_com_i * T_b_i.R.transpose() - link.mass_kg * skew_c * skew_c;

    // Unit qdd_i, zero velocity and gravity: C_i turns about z_i through p_i. Its inertial
    // force is F = z_i x (h - m p_i) and its moment about p_i is I_p z_i, with
    // I_p = I_o + S(h) S(p) + S(p) S(h) - m S(p)^2 (parallel axis, origin -> p_i).
    const Vec3& p = fk.p_b_joint[i];
    const Vec3& z = fk.axis_b[i];
    const Mat3 skew_p = skew(p);
    const Mat3 skew_h = skew(h);
    const Mat3 I_p = I_o + skew_h * skew_p + skew_p * skew_h - m * skew_p * skew_p;
    const Vec3 F = z.cross(h - m * p);
    const Vec3 n_i = I_p * z;
    const auto ci = static_cast<Eigen::Index>(i);
    for (std::size_t j = 0; j <= i; ++j) {
      // Moment about p_j: n_j = n_i + (p_i - p_j) x F;  M_ji = z_j . n_j.
      const Vec3 n_j = n_i + (p - fk.p_b_joint[j]).cross(F);
      const auto cj = static_cast<Eigen::Index>(j);
      M(cj, ci) = fk.axis_b[j].dot(n_j);
      M(ci, cj) = M(cj, ci);
    }
    // Reflected rotor inertia J_in * N^2 turns with the joint only: diagonal term.
    M(ci, ci) += model.joints[i].armature;
  }
  return M;
}

#define COBOT_CRBA_INSTANTIATE(N) \
  template MatN<(N)> mass_matrix<(N)>(const Model<(N)>&, const VecN<(N)>&);
COBOT_INSTANTIATE_DOF(COBOT_CRBA_INSTANTIATE)
#undef COBOT_CRBA_INSTANTIATE

}  // namespace cobot::dynamics
