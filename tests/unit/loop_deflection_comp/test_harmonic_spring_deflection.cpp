// harmonic_spring_deflection: exact inverse of harmonic_spring_torque (brief "deflection-comp"
// contract 2). Odd and piecewise linear in tau:
//   |tau| <= t1        -> |tau| / k1
//   t1 < |tau| <= t2   -> phi1 + (|tau| - t1) / k2,   phi1 = t1 / k1
//   |tau| > t2         -> phi2 + (|tau| - t2) / k3,   phi2 = phi1 + (t2 - t1) / k2
// with the sign of tau.
#include <algorithm>
#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "cobot/dynamics/harmonic_spring.hpp"

#include "deflection_test_util.hpp"

namespace cobot::pipeline::test {
namespace {

using dynamics::harmonic_spring_deflection;
using dynamics::harmonic_spring_torque;

constexpr double kRel = 1e-15;  // relative round-trip tolerance (about 4.5 ulp)

double torque(double phi, const Gear& g) {
  return harmonic_spring_torque(phi, g.k1, g.k2, g.k3, g.t1, g.t2);
}
double deflection(double tau, const Gear& g) {
  return harmonic_spring_deflection(tau, g.k1, g.k2, g.k3, g.t1, g.t2);
}

// Round numbers: k1 = 1000, k2 = 500, k3 = 250, t1 = 2, t2 = 4 ->
// phi1 = 2 / 1000 = 0.002, phi2 = 0.002 + (4 - 2) / 500 = 0.006.
constexpr Gear kRound{1000.0, 500.0, 250.0, 2.0, 4.0};
constexpr std::array<Gear, 4> kGears = {kRound, kShg25, kShg20, kShg14};

TEST(HarmonicSpringDeflection, HandValuesOnEverySegmentAndAtTheBreakpoints) {
  constexpr double kTol = 1e-15;  // rad; every value below is <= 0.01 rad
  EXPECT_EQ(deflection(0.0, kRound), 0.0);
  EXPECT_NEAR(deflection(1.0, kRound), 0.001, kTol);  // 1 / 1000
  EXPECT_NEAR(deflection(2.0, kRound), 0.002, kTol);  // t1 -> phi1
  EXPECT_NEAR(deflection(3.0, kRound), 0.004, kTol);  // 0.002 + (3 - 2) / 500
  EXPECT_NEAR(deflection(4.0, kRound), 0.006, kTol);  // t2 -> phi2
  EXPECT_NEAR(deflection(5.0, kRound), 0.010, kTol);  // 0.006 + (5 - 4) / 250
  EXPECT_NEAR(deflection(-1.0, kRound), -0.001, kTol);
  EXPECT_NEAR(deflection(-2.0, kRound), -0.002, kTol);
  EXPECT_NEAR(deflection(-3.0, kRound), -0.004, kTol);
  EXPECT_NEAR(deflection(-4.0, kRound), -0.006, kTol);
  EXPECT_NEAR(deflection(-5.0, kRound), -0.010, kTol);
}

TEST(HarmonicSpringDeflection, HandValuesOfTheCatalogGears) {
  constexpr double kTol = 1e-15;  // rad
  // SHG-25: phi1 = 14 / 3.1e4 = 4.5161290322580643e-4,
  //         phi2 = phi1 + 34 / 5e4 = phi1 + 6.8e-4 = 1.1316129032258064e-3,
  //         58 N m: phi2 + 10 / 5.7e4 = phi2 + 1.7543859649e-4 = 1.3070514997170345e-3.
  EXPECT_NEAR(deflection(14.0, kShg25), 4.5161290322580643e-4, kTol);
  EXPECT_NEAR(deflection(48.0, kShg25), 1.1316129032258064e-3, kTol);
  EXPECT_NEAR(deflection(58.0, kShg25), 1.3070514997170345e-3, kTol);
  EXPECT_NEAR(deflection(-58.0, kShg25), -1.3070514997170345e-3, kTol);
  // SHG-20: phi1 = 7 / 1.3e4 = 5.384615384615384e-4,
  //         16 N m: phi1 + 9 / 2.5e4 = phi1 + 3.6e-4 = 8.984615384615385e-4,
  //         phi2 = phi1 + 18 / 2.5e4 = phi1 + 7.2e-4 = 1.2584615384615386e-3.
  EXPECT_NEAR(deflection(7.0, kShg20), 5.384615384615384e-4, kTol);
  EXPECT_NEAR(deflection(16.0, kShg20), 8.984615384615385e-4, kTol);
  EXPECT_NEAR(deflection(-16.0, kShg20), -8.984615384615385e-4, kTol);
  EXPECT_NEAR(deflection(25.0, kShg20), 1.2584615384615386e-3, kTol);
  // SHG-14: phi1 = 2 / 4700 = 4.25531914893617e-4,
  //         phi2 = phi1 + 4.9 / 6100 = phi1 + 8.0327868852e-4 = 1.2288106034182072e-3,
  //         70 N m: phi2 + 63.1 / 7100 = phi2 + 8.8873239437e-3 = 1.0116134547080179e-2
  //         (the hard-stop equilibrium of the transmission tests, 0.0101161).
  EXPECT_NEAR(deflection(2.0, kShg14), 4.25531914893617e-4, kTol);
  EXPECT_NEAR(deflection(6.9, kShg14), 1.2288106034182072e-3, kTol);
  EXPECT_NEAR(deflection(70.0, kShg14), 1.0116134547080179e-2, kTol);
  EXPECT_NEAR(deflection(-70.0, kShg14), -1.0116134547080179e-2, kTol);
}

// deflection(torque(phi)) == phi on all three segments, both signs, densely sampled up to
// 3 phi2 and close to both breakpoints. A straightforward evaluation of the formulas above in
// double gives at most ~1.5 ulp (3.2e-16) here.
TEST(HarmonicSpringDeflection, RoundTripFromTheDeflection) {
  for (const Gear& g : kGears) {
    const double phi1 = g.t1 / g.k1;
    const double phi2 = phi1 + ((g.t2 - g.t1) / g.k2);
    std::array<int, 3> segments_seen{};
    for (int i = 1; i <= 4000; ++i) {
      const double phi = 3.0 * phi2 * i / 4000.0;
      ++segments_seen[phi <= phi1 ? 0U : (phi <= phi2 ? 1U : 2U)];
      for (const double s : {1.0, -1.0}) {
        const double p = s * phi;
        EXPECT_NEAR(deflection(torque(p, g), g), p, kRel * phi) << "phi " << p;
      }
    }
    for (const int seen : segments_seen) {
      EXPECT_GT(seen, 100);
    }
    for (const double b : {phi1, phi2}) {
      for (const double f : {1.0 - 1e-6, 1.0 - 1e-12, 1.0, 1.0 + 1e-12, 1.0 + 1e-6}) {
        for (const double s : {1.0, -1.0}) {
          const double p = s * b * f;
          EXPECT_NEAR(deflection(torque(p, g), g), p, kRel * std::abs(p)) << "phi " << p;
        }
      }
    }
    for (const double p : {1e-12, 1e-6, 1.0, -1.0}) {
      EXPECT_NEAR(deflection(torque(p, g), g), p, kRel * std::abs(p)) << "phi " << p;
    }
  }
}

// torque(deflection(tau)) == tau as well (the two functions are inverse bijections); measured
// at most ~1.2 ulp for a straightforward evaluation.
TEST(HarmonicSpringDeflection, RoundTripFromTheTorque) {
  for (const Gear& g : kGears) {
    for (int i = 1; i <= 4000; ++i) {
      const double tau = 3.0 * g.t2 * i / 4000.0;
      for (const double s : {1.0, -1.0}) {
        EXPECT_NEAR(torque(deflection(s * tau, g), g), s * tau, kRel * tau) << "tau " << s * tau;
      }
    }
  }
}

TEST(HarmonicSpringDeflection, OddMonotoneAndContinuous) {
  for (const Gear& g : kGears) {
    double prev = 0.0;
    for (int i = 0; i <= 600; ++i) {
      const double tau = 3.0 * g.t2 * i / 600.0;
      const double phi = deflection(tau, g);
      EXPECT_EQ(deflection(-tau, g), -phi) << "tau " << tau;
      EXPECT_GE(phi, prev) << "tau " << tau;
      prev = phi;
    }
    // No jump at either breakpoint: 1e-9 N m either side moves phi by at most
    // (1 / k_left + 1 / k_right) * 1e-9 <= 2e-9 / min(k1, k2, k3).
    const double k_min = std::min({g.k1, g.k2, g.k3});
    for (const double b : {g.t1, g.t2}) {
      EXPECT_NEAR(deflection(b - 1e-9, g), deflection(b + 1e-9, g), 2e-9 / k_min + 1e-18);
    }
  }
}

}  // namespace
}  // namespace cobot::pipeline::test
