#pragma once

#include "cobot/common/types.hpp"

// Friction feed-forward (feature `friction-comp`, docs/derivations/control.md "Bù ma sát"):
//   tau_ff = coulomb .* tanh(qd_ref / eps) + viscous .* qd_ref
// on the REFERENCE velocity, so sensor noise is not fed through and there is no chattering at
// qd ~ 0. Parameters are the nominal identified friction (cobot.friction_compensation).
namespace cobot::control {

using common::VecN;

template <int N>
struct FrictionCompensationConfig {
  bool enabled = false;
  VecN<N> coulomb_nm = VecN<N>::Zero();        // >= 0, nominal (identified) Coulomb friction
  VecN<N> viscous_nm_s_rad = VecN<N>::Zero();  // >= 0
  double velocity_eps_rad_s = 0.01;            // > 0, tanh smoothing width
};

template <int N>
class FrictionCompensation {
 public:
  explicit FrictionCompensation(const FrictionCompensationConfig<N>& config);
  // tau_ff = coulomb ⊙ tanh(qd_ref / eps) + viscous ⊙ qd_ref; all zero when !enabled.
  // Non-finite qd_ref -> all zero. No allocation.
  VecN<N> torque(const VecN<N>& qd_ref) const;

 private:
  FrictionCompensationConfig<N> config_;
};

}  // namespace cobot::control
