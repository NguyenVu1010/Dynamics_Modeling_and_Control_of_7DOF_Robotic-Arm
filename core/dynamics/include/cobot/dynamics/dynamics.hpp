#pragma once

#include "cobot/common/types.hpp"
#include "cobot/model/model.hpp"

// Rigid-body dynamics  tau = M(q) qdd + C(q, qd) qd + G(q)  (joint damping excluded).
// Implemented in feature `dynamics` with 3D-vector recursive Newton-Euler (RNEA) and the
// composite rigid body algorithm (CRBA); see docs/derivations/. No heap allocation.
namespace cobot::dynamics {

using common::MatN;
using common::Vec3;
using common::VecN;
using model::Model;

// Inverse dynamics. gravity_b is the gravity vector in b (m/s^2); pass zero to drop G.
template <int N>
VecN<N> rnea(const Model<N>& model, const VecN<N>& q, const VecN<N>& qd, const VecN<N>& qdd,
             const Vec3& gravity_b);

// Joint-space inertia matrix M(q) (CRBA), symmetric positive definite.
template <int N>
MatN<N> mass_matrix(const Model<N>& model, const VecN<N>& q);

// G(q) = rnea(q, 0, 0, model.gravity_b)
template <int N>
VecN<N> gravity_torque(const Model<N>& model, const VecN<N>& q);

// C(q, qd) * qd = rnea(q, qd, 0, 0)
template <int N>
VecN<N> coriolis_torque(const Model<N>& model, const VecN<N>& q, const VecN<N>& qd);

// qdd = M^-1 (tau - C qd - G - damping .* qd)   (LDLT)
template <int N>
VecN<N> forward_dynamics(const Model<N>& model, const VecN<N>& q, const VecN<N>& qd,
                         const VecN<N>& tau);

}  // namespace cobot::dynamics
