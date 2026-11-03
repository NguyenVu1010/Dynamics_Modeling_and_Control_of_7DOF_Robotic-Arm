#pragma once

#include <cmath>

// Three-segment stiffness of a Harmonic Drive gear (realism-2 design 6.2, catalog K1/K2/K3 with
// breakpoints T1/T2). Derivation: docs/derivations/sim_plant_transmission.md.
namespace cobot::dynamics {

// Spring torque (N m) at deflection phi (rad). Odd and piecewise linear in phi:
//   |phi| <= phi1 = t1 / k1                        -> k1 |phi|
//   phi1 < |phi| <= phi2 = phi1 + (t2 - t1) / k2   -> t1 + k2 (|phi| - phi1)
//   |phi| > phi2                                   -> t2 + k3 (|phi| - phi2)
// with the sign of phi. Requires k1, k2 > 0 and 0 < t1 < t2.
inline double harmonic_spring_torque(
    double phi, double k1, double k2, double k3, double t1, double t2) {
  const double a = std::abs(phi);
  const double phi1 = t1 / k1;
  const double phi2 = phi1 + ((t2 - t1) / k2);
  double tau = 0.0;
  if (a <= phi1) {
    tau = k1 * a;
  } else if (a <= phi2) {
    tau = t1 + (k2 * (a - phi1));
  } else {
    tau = t2 + (k3 * (a - phi2));
  }
  return std::copysign(tau, phi);
}

}  // namespace cobot::dynamics
