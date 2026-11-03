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

// Deflection phi (rad) such that harmonic_spring_torque(phi, ...) == tau: the exact inverse,
// odd and piecewise linear in tau:
//   |tau| <= t1        -> |tau| / k1
//   t1 < |tau| <= t2   -> phi1 + (|tau| - t1) / k2
//   |tau| > t2         -> phi2 + (|tau| - t2) / k3
// with the sign of tau (phi1, phi2 as above). Requires k1, k2, k3 > 0 and 0 < t1 < t2.
inline double harmonic_spring_deflection(
    double tau, double k1, double k2, double k3, double t1, double t2) {
  const double a = std::abs(tau);
  const double phi1 = t1 / k1;
  double phi = 0.0;
  if (a <= t1) {
    phi = a / k1;
  } else if (a <= t2) {
    phi = phi1 + ((a - t1) / k2);
  } else {
    const double phi2 = phi1 + ((t2 - t1) / k2);
    phi = phi2 + ((a - t2) / k3);
  }
  return std::copysign(phi, tau);
}

}  // namespace cobot::dynamics
