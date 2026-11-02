// harmonic_spring_torque: the three-segment Harmonic Drive stiffness curve (realism-2 design 6.2,
// brief "transmission" contract 4). Odd and piecewise linear in phi:
//   |phi| <= phi1 = t1 / k1        -> k1 |phi|
//   phi1 < |phi| <= phi2           -> t1 + k2 (|phi| - phi1),  phi2 = phi1 + (t2 - t1) / k2
//   |phi| > phi2                   -> t2 + k3 (|phi| - phi2)
// with the sign of phi.
#include <array>

#include <gtest/gtest.h>

#include "cobot/dynamics/harmonic_spring.hpp"

#include "transmission_test_util.hpp"

namespace cobot::dynamics::test {
namespace {

constexpr double kTol = 1e-12;  // N m

double torque(double phi, const Gear& g) {
  return harmonic_spring_torque(phi, g.k1, g.k2, g.k3, g.t1, g.t2);
}

// Round numbers: k1 = 1000, k2 = 500, k3 = 250, t1 = 2, t2 = 4 ->
// phi1 = 2 / 1000 = 0.002, phi2 = 0.002 + (4 - 2) / 500 = 0.006.
constexpr Gear kRound{1000.0, 500.0, 250.0, 2.0, 4.0};

TEST(HarmonicSpring, HandValuesOnEverySegment) {
  EXPECT_EQ(torque(0.0, kRound), 0.0);
  EXPECT_NEAR(torque(0.001, kRound), 1.0, kTol);  // 0.5 phi1: 1000 * 0.001
  EXPECT_NEAR(torque(0.002, kRound), 2.0, kTol);  // phi1: t1
  EXPECT_NEAR(torque(0.004, kRound), 3.0, kTol);  // 2 + 500 * (0.004 - 0.002)
  EXPECT_NEAR(torque(0.006, kRound), 4.0, kTol);  // phi2: t2
  EXPECT_NEAR(torque(0.010, kRound), 5.0, kTol);  // 4 + 250 * (0.010 - 0.006)
  EXPECT_NEAR(torque(-0.001, kRound), -1.0, kTol);
  EXPECT_NEAR(torque(-0.002, kRound), -2.0, kTol);
  EXPECT_NEAR(torque(-0.004, kRound), -3.0, kTol);
  EXPECT_NEAR(torque(-0.010, kRound), -5.0, kTol);
}

TEST(HarmonicSpring, OddAndContinuousForEveryGear) {
  for (const Gear& g : std::array<Gear, 4>{kRound, kShg25, kShg20, kShg14}) {
    const double phi1 = g.t1 / g.k1;
    const double phi2 = phi1 + ((g.t2 - g.t1) / g.k2);
    for (int i = 0; i <= 400; ++i) {
      const double phi = 3.0 * phi2 * i / 400.0;
      EXPECT_EQ(torque(-phi, g), -torque(phi, g)) << "phi " << phi;
      EXPECT_GE(torque(phi, g), 0.0) << "phi " << phi;
    }
    // No jump at either breakpoint (1e-12 rad either side moves the torque by at most k3 * 1e-12).
    for (const double b : {phi1, phi2}) {
      EXPECT_NEAR(torque(b - 1e-12, g), torque(b + 1e-12, g), 2e-12 * g.k3 + kTol);
    }
  }
}

}  // namespace
}  // namespace cobot::dynamics::test
