// Elastic joint in SimPlant (realism-2 design 6.2, 8; brief "transmission", model pinned there):
// link side M_link(q) qdd + h = tau_s, motor side B theta_dd = tau_m - tau_s - tau_f, with
//   phi = theta - q - e,  e = error_amp sin(2 ratio theta + motor_phase),
//   phi' = phi - clamp(phi, -lost / 2, lost / 2),
//   tau_s = harmonic_spring_torque(phi', ...) + d (theta_d - q_d),  d = damping_ratio 2 sqrt(k1 B).
// theta is q_motor() (link-side units). spring_torque() is tau_s at the state after the step, so
// the tests compare it with that state exactly.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "cobot/dynamics/dynamics.hpp"

#include "transmission_test_util.hpp"

namespace cobot::dynamics::test {
namespace {

using V7 = VecN<7>;

// Wrist-like joint: link 0.2 kg m^2 about z, motor B = 0.22674 kg m^2, SHG-14, no gravity torque.
// omega = sqrt(k1 (1/B + 1/J_l)) = sqrt(4700 * (4.4103378 + 5)) = sqrt(44228.588) = 210.306 rad/s,
// i.e. 33.471 Hz.
constexpr double kJLink = 0.2;
constexpr double kPulseNm = 5.0;

// Rest, then one step of kPulseNm on the motor: the relative velocity 5e-3 / 0.22674 = 0.0221 rad/s
// gives an oscillation of phi with amplitude ~0.0221 / 210 = 1.05e-4 rad, i.e. |tau_s| <= 0.49 N m,
// well inside K1 (t1 = 2 N m). The common motion of link and motor does not change phi.
SimPlant<1> pulsed_plant(double damping_ratio) {
  PlantEffects<1> fx;
  set_elastic(fx, 0, kShg14, damping_ratio);
  SimPlant<1> plant(one_joint(kJLink, kArmatureWrist), V1::Zero(), fx);
  plant.step(V1::Constant(kPulseNm), kDt);
  return plant;
}

double phi_of(const SimPlant<1>& p) {
  return p.q_motor()(0) - p.q()(0);
}

TEST(SimPlantTransmission, FreeOscillationAtTheTwoMassFrequency) {
  SimPlant<1> plant = pulsed_plant(0.0);
  const double omega = std::sqrt(kShg14.k1 * ((1.0 / kArmatureWrist) + (1.0 / kJLink)));
  const double f_expected = omega / (2.0 * kPi);  // 33.471 Hz
  std::vector<double> crossings;
  double prev = phi_of(plant);
  for (int k = 1; k <= 1000; ++k) {  // 1 s
    plant.step(V1::Zero(), kDt);
    const double now = phi_of(plant);
    if ((prev < 0.0) != (now < 0.0)) {
      crossings.push_back((k - 1 + (prev / (prev - now))) * kDt);  // linear interpolation
    }
    prev = now;
  }
  // 1 s at 33.5 Hz: 33.5 periods, ~67 zero crossings.
  ASSERT_GE(crossings.size(), 60U);
  const auto half_periods = static_cast<double>(crossings.size() - 1);
  const double f = half_periods / (2.0 * (crossings.back() - crossings.front()));
  EXPECT_NEAR(f, f_expected, 0.01 * f_expected);
}

TEST(SimPlantTransmission, UndampedTwoMassConservesEnergy) {
  SimPlant<1> plant = pulsed_plant(0.0);
  const auto energy = [&plant]() {
    const double qd = plant.qd()(0);
    const double wm = plant.qd_motor()(0);
    return (0.5 * kJLink * qd * qd) + (0.5 * kArmatureWrist * wm * wm) +
           spring_energy(phi_of(plant), kShg14);
  };
  const double e0 = energy();  // ~5.5e-5 J (0.5 * 5e-3^2 / 0.22674)
  ASSERT_GT(e0, 1e-5);
  double worst = 0.0;
  for (int k = 0; k < 2000; ++k) {  // 2 s free
    plant.step(V1::Zero(), kDt);
    worst = std::max(worst, std::abs(energy() - e0) / e0);
  }
  EXPECT_LT(worst, 1e-4);
}

// Motor PD on q_motor (collocated), computed by the test once per period.
double motor_pd(const SimPlant<1>& p, double ref, double ref_d, double kp, double kd) {
  return (kp * (ref - p.q_motor()(0))) + (kd * (ref_d - p.qd_motor()(0)));
}

TEST(SimPlantTransmission, LostMotionIsADeadZoneOfItsWidthInBothDirections) {
  // Link held by 1e6 kg m^2; the motor follows a +-4e-4 rad triangle of 4 s period twice through
  // a PD (kp 200 N m/rad, kd 13.5 N m s/rad, about critical for B). Lost motion 2.9e-4: half width
  // 1.45e-4, so each turn crosses the whole band and compresses the spring on both sides.
  constexpr double kLost = 2.9e-4;
  constexpr double kHalf = 0.5 * kLost;
  constexpr double kAmp = 4e-4;
  constexpr double kPeriodS = 4.0;
  PlantEffects<1> fx;
  set_elastic(fx, 0, kShg14, 0.0, 0.0, kLost);
  SimPlant<1> plant(one_joint(1e6, kArmatureWrist), V1::Zero(), fx);
  // [direction rising/falling][region below/inside/above the band]
  std::array<std::array<int, 3>, 2> seen{};
  for (int k = 0; k < 8000; ++k) {
    const double t = k * kDt;
    const double s = std::fmod(t, kPeriodS) / kPeriodS;  // 0..1
    const bool rising = s < 0.25 || s >= 0.75;
    const double ref = kAmp * (s < 0.25 ? 4.0 * s : (s < 0.75 ? 2.0 - (4.0 * s) : (4.0 * s) - 4.0));
    const double ref_d = (rising ? 4.0 : -4.0) * kAmp / kPeriodS;
    plant.step(V1::Constant(motor_pd(plant, ref, ref_d, 200.0, 13.5)), kDt);
    const double phi = phi_of(plant);
    const double tau_s = plant.spring_torque()(0);
    const int dir = rising ? 0 : 1;
    if (std::abs(phi) < kHalf - 1e-7) {
      ASSERT_NEAR(tau_s, 0.0, 1e-9) << "inside the band, step " << k << ", phi " << phi;
      ++seen[dir][1];
    } else if (std::abs(phi) > kHalf + 1e-7) {
      const double phi_prime = phi > 0.0 ? phi - kHalf : phi + kHalf;
      ASSERT_NEAR(tau_s, kShg14.k1 * phi_prime, 1e-6) << "outside, step " << k << ", phi " << phi;
      if (std::abs(phi_prime) > 2e-6) {
        ++seen[dir][phi > 0.0 ? 2 : 0];
      }
    }
  }
  for (int dir = 0; dir < 2; ++dir) {
    for (int region = 0; region < 3; ++region) {
      EXPECT_GT(seen[dir][region], 0) << "direction " << dir << " region " << region;
    }
  }
}

TEST(SimPlantTransmission, TransmissionErrorHasPeriodPiOverRatioInTheMotorAngle) {
  // error_amp 1e-4, ratio 100, phase 0.7; the motor is ramped 0 -> 0.15 rad in 3 s (0.05 rad/s, so
  // e varies at 2 * 100 * 0.05 = 10 rad/s, far below the 210 rad/s spring mode) and the free link
  // follows. With no lost motion and no damping, tau_s = k1 phi in K1, so
  // e = (theta - q) - tau_s / k1 is read back exactly from the plant.
  constexpr double kAmp = 1e-4;
  constexpr double kRatio = 100.0;
  constexpr double kPhase = 0.7;
  PlantEffects<1> fx;
  set_elastic(fx, 0, kShg14, 0.0, kAmp, 0.0, kRatio, kPhase);
  SimPlant<1> plant(one_joint(kJLink, kArmatureWrist), V1::Zero(), fx);
  std::vector<double> rising_crossings;  // theta where e goes from < 0 to >= 0
  double e_prev = 0.0;
  double theta_prev = 0.0;
  constexpr double kSpeed = 0.05;  // rad/s
  for (int k = 0; k < 3000; ++k) {
    plant.step(V1::Constant(motor_pd(plant, kSpeed * k * kDt, kSpeed, 200.0, 13.5)), kDt);
    const double theta = plant.q_motor()(0);
    const double tau_s = plant.spring_torque()(0);
    ASSERT_LT(std::abs(tau_s), kShg14.t1) << "left K1 at step " << k;
    const double e = phi_of(plant) - (tau_s / kShg14.k1);
    ASSERT_NEAR(e, kAmp * std::sin((2.0 * kRatio * theta) + kPhase), 1e-10) << "step " << k;
    if (k > 0 && e_prev < 0.0 && e >= 0.0) {
      rising_crossings.push_back(theta_prev + ((theta - theta_prev) * (-e_prev / (e - e_prev))));
    }
    e_prev = e;
    theta_prev = theta;
  }
  // Rising crossings at 200 theta + 0.7 = 2 pi n: theta = 0.0279, 0.0593, 0.0907, 0.1221 rad
  // (period pi / 100 = 0.0314159 rad) within the 0.15 rad ramp.
  ASSERT_GE(rising_crossings.size(), 4U);
  const double period = (rising_crossings.back() - rising_crossings.front()) /
                        static_cast<double>(rising_crossings.size() - 1);
  EXPECT_NEAR(period, kPi / kRatio, 0.01 * kPi / kRatio);
}

TEST(SimPlantTransmission, HardStopWithTheMotorPushingStaysBounded) {
  // Link at q_max = 0 (range [-1, 0]), motor commanded +M = 70 N m (tau_max) for 1 s, SHG-14,
  // 1 % damping. The link is clamped as today; the motor winds the spring up to the static
  // equilibrium harmonic(phi_eq) = 70: phi_eq = phi2 + (70 - 6.9) / 7100 = 1.2288e-3 + 8.8873e-3
  // = 0.0101161 rad. Energy: the spring stores at most the work M (theta - theta0); undamped the
  // turning point solves U(x) = 70 x, x = phi2 + y with 3550 y^2 - 63.1 y - 0.0820167 = 0,
  // y = 0.0189911, so the spring peaks at 6.9 + 7100 y = 141.74 N m. Damping only lowers that
  // peak; its own term d |theta_d| <= 0.653 * 3.53 = 2.3 N m (d = 0.02 sqrt(4700 * 0.22674),
  // |theta_d| <= sqrt(2 * 70 * 0.0202 / 0.22674)). Bound: 2.1 M = 147 N m.
  constexpr double kM = 70.0;
  model::Model<1> mdl = one_joint(kJLink, kArmatureWrist);
  mdl.joints[0].q_min = -1.0;
  mdl.joints[0].q_max = 0.0;
  mdl.joints[0].tau_max = kM;
  PlantEffects<1> fx;
  set_elastic(fx, 0, kShg14, 0.01);
  SimPlant<1> plant(mdl, V1::Zero(), fx);
  double tau_sum = 0.0;
  double phi_sum = 0.0;
  for (int k = 0; k < 1000; ++k) {
    plant.step(V1::Constant(kM), kDt);
    const double theta = plant.q_motor()(0);
    const double wm = plant.qd_motor()(0);
    const double tau_s = plant.spring_torque()(0);
    ASSERT_TRUE(std::isfinite(theta) && std::isfinite(wm) && std::isfinite(tau_s)) << k;
    ASSERT_LE(plant.q()(0), 0.0) << "step " << k;
    ASSERT_GT(plant.q()(0), -1e-6) << "step " << k;
    ASSERT_LE(std::abs(tau_s), 2.1 * kM) << "step " << k;
    const double stored = (0.5 * kArmatureWrist * wm * wm) + spring_energy(phi_of(plant), kShg14);
    ASSERT_LE(stored, (kM * theta * 1.001) + 1e-9) << "step " << k;  // work in, theta0 = 0
    if (k >= 500) {
      tau_sum += tau_s;
      phi_sum += phi_of(plant);
    }
  }
  // Last 0.5 s (14 periods at sqrt(7100 / 0.22674) = 177 rad/s, all in K3): mean = equilibrium.
  EXPECT_NEAR(tau_sum / 500.0, kM, 0.05 * kM);
  EXPECT_NEAR(phi_sum / 500.0, 0.0101161, 0.05 * 0.0101161);
}

TEST(SimPlantTransmission, CoulombFrictionOfAnElasticJointActsOnTheMotor) {
  // 1 N m Coulomb on the motor side: 0.5 N m sticks exactly (spring untouched, link at rest);
  // 1.5 N m slides and drags the link through the spring.
  PlantEffects<1> fx;
  set_elastic(fx, 0, kShg14, 0.01);
  fx.coulomb_nm(0) = 1.0;
  SimPlant<1> plant(one_joint(kJLink, kArmatureWrist), V1::Zero(), fx);
  for (int k = 0; k < 500; ++k) {
    plant.step(V1::Constant(0.5), kDt);
    ASSERT_EQ(plant.qd_motor()(0), 0.0) << "step " << k;
    ASSERT_EQ(plant.q_motor()(0), 0.0) << "step " << k;
    ASSERT_EQ(plant.q()(0), 0.0) << "step " << k;
  }
  for (int k = 0; k < 200; ++k) {
    plant.step(V1::Constant(1.5), kDt);
  }
  EXPECT_GT(plant.qd_motor()(0), 0.0);
  EXPECT_GT(plant.q()(0), 0.0);
}

// 7-DOF arm at home, every joint elastic with the starting values and every transmission effect
// on, plus friction and motor lag; command the gravity torque of home.
struct ArmRig {
  model::Arm7 arm;
  PlantEffects<7> fx;
  V7 q0;
  V7 tau;
};

ArmRig arm_rig() {
  ArmRig r;
  const std::string urdf = std::string(COBOT_REPO_DIR) + "/src/sim/urdf/cobot30_5.urdf";
  const auto chain = model::parse_urdf_chain(urdf, "base_link", "link_7");
  EXPECT_TRUE(chain.is_ok()) << chain.reason();
  const auto arm = model::make_model<7>(chain.value(), Vec3(0.0, 0.0, -9.80665));
  EXPECT_TRUE(arm.is_ok()) << arm.reason();
  r.arm = arm.value();
  const std::array<Gear, 7> gears = {kShg25, kShg25, kShg20, kShg20, kShg14, kShg14, kShg14};
  for (int j = 0; j < 7; ++j) {
    set_elastic(r.fx, j, gears[static_cast<std::size_t>(j)], 0.01, 4.4e-4, 2.9e-4, 100.0, 0.5 * j);
  }
  r.fx.coulomb_nm << 1.7, 1.7, 1.0, 1.0, 0.2, 0.2, 0.2;
  r.fx.viscous_nm_s_rad << 0.5, 0.5, 0.5, 0.5, 0.1, 0.1, 0.1;
  r.fx.motor_time_constant_s.setConstant(0.001);
  r.q0 << 0.0, 30.0, 0.0, -90.0, 0.0, 60.0, 0.0;  // config/default.yaml home_q_deg
  r.q0 *= kPi / 180.0;
  r.tau = gravity_torque(r.arm, r.q0);
  return r;
}

TEST(SimPlantTransmission, ArmStepDoesNotAllocate) {
  const ArmRig r = arm_rig();
  SimPlant<7> plant(r.arm, r.q0, r.fx);
  plant.step(r.tau, kDt);
  Eigen::internal::set_is_malloc_allowed(false);
  for (int k = 0; k < 200; ++k) {
    plant.step(r.tau, kDt);
  }
  Eigen::internal::set_is_malloc_allowed(true);
  EXPECT_TRUE(plant.q().allFinite() && plant.q_motor().allFinite());
  EXPECT_TRUE(plant.spring_torque().allFinite());
}

TEST(SimPlantTransmission, ArmIsDeterministic) {
  const ArmRig r = arm_rig();
  SimPlant<7> a(r.arm, r.q0, r.fx);
  SimPlant<7> b(r.arm, r.q0, r.fx);
  for (int k = 0; k < 300; ++k) {
    a.step(r.tau, kDt);
    b.step(r.tau, kDt);
  }
  EXPECT_EQ(a.q(), b.q());
  EXPECT_EQ(a.qd(), b.qd());
  EXPECT_EQ(a.q_motor(), b.q_motor());
  EXPECT_EQ(a.qd_motor(), b.qd_motor());
  EXPECT_EQ(a.spring_torque(), b.spring_torque());
  EXPECT_NE(a.q_motor(), a.q());  // the transmission is really on
}

}  // namespace
}  // namespace cobot::dynamics::test
