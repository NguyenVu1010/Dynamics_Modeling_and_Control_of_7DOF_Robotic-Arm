// Feature payload-inertia (design 2026-11-02-realism-2 6.4): the payload of
// ModelMismatchConfig is a solid sphere of mass payload_kg and radius payload_radius_m whose
// centre sits at payload_offset_m in the tip frame (link_7; T_last_tip is the tip in the frame of
// the last joint). perturb_model merges it into the last link with the parallel-axis theorem:
//   m' = m + m_p,  c' = (m c + m_p p) / m',
//   I' = I + P(m, c - c') + P(m_p, p - c') + (2/5) m_p r^2 1,  P(m, d) = m (|d|^2 1 - d d^T),
// where p = T_last_tip * payload_offset_m. Offset 0 and radius 0 give today's point mass at the
// tip origin bit for bit; payload_kg = 0 ignores offset and radius.
#include <cmath>
#include <cstdint>
#include <string>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/dynamics/dynamics.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/imperfect/perturb.hpp"
#include "cobot/kinematics/kinematics.hpp"
#include "cobot/model/model.hpp"

namespace cobot::imperfect {
namespace {

using common::Mat3;
using common::Vec3;
using M1 = model::Model<1>;
using M2 = model::Model<2>;
constexpr double kTol = 1e-12;

Mat3 point(double m, const Vec3& d) {
  return m * (d.squaredNorm() * Mat3::Identity() - d * d.transpose());
}

double max_abs(const Mat3& a) {
  return a.cwiseAbs().maxCoeff();
}

// The test link of the brief: 1 kg, centre of mass at the link origin, diagonal inertia, tip at
// the link origin (T_last_tip identity).
M1 unit_link() {
  M1 m;
  m.links[0].mass_kg = 1.0;
  m.links[0].I_com_i = Vec3(0.01, 0.02, 0.03).asDiagonal();
  return m;
}

// Two links with full inertia tensors and a tip away from the last joint (same as the sim-plant
// perturb tests), so every component of the merge is visible.
M2 two_link() {
  M2 m;
  m.joints[1].T_parent_joint.p = Vec3(0.3, 0.0, 0.1);
  m.links[0].mass_kg = 3.0;
  m.links[0].com_i = Vec3(0.1, -0.02, 0.05);
  m.links[0].I_com_i << 0.05, 0.001, -0.002, 0.001, 0.04, 0.003, -0.002, 0.003, 0.03;
  m.links[1].mass_kg = 1.5;
  m.links[1].com_i = Vec3(0.02, 0.01, -0.1);
  m.links[1].I_com_i << 0.02, -0.001, 0.0005, -0.001, 0.025, 0.002, 0.0005, 0.002, 0.01;
  m.T_last_tip.p = Vec3(0.1, -0.05, 0.2);
  return m;
}

template <int N>
void expect_bit_identical(const model::Model<N>& a, const model::Model<N>& b) {
  for (int k = 0; k < N; ++k) {
    EXPECT_EQ(a.links[k].mass_kg, b.links[k].mass_kg) << "link " << k;
    EXPECT_TRUE(a.links[k].com_i == b.links[k].com_i) << "link " << k;
    EXPECT_TRUE(a.links[k].I_com_i == b.links[k].I_com_i) << "link " << k;
  }
  EXPECT_TRUE(a.T_last_tip.p == b.T_last_tip.p);
  EXPECT_TRUE(a.T_last_tip.R == b.T_last_tip.R);
}

// Today's point-mass merge (before this feature), expression for expression, applied to the last
// link of `out`: the bit-identity reference for offset 0 and radius 0.
template <int N>
void merge_point_mass_at_tip(model::Model<N>& out, const Vec3& tip, double payload_kg) {
  model::LinkInertia& last = out.links[N - 1];
  const double m_new = last.mass_kg + payload_kg;
  const Vec3 com_new = (last.mass_kg * last.com_i + payload_kg * tip) / m_new;
  last.I_com_i += point(last.mass_kg, last.com_i - com_new) + point(payload_kg, tip - com_new);
  last.mass_kg = m_new;
  last.com_i = com_new;
}

// ---------------------------------------------------------------- hand-computed merges

// 2 kg at (0, 0, 0.1) on the 1 kg link with its CoM at 0:
//   m' = 1 + 2 = 3,  c' = (1 * 0 + 2 * 0.1) / 3 = 0.2 / 3 along z
//   d0 = 0 - c' = (0, 0, -0.2/3)  ->  P(1, d0) = diag(0.04/9, 0.04/9, 0)
//   dp = 0.1 - 0.2/3 = 0.1/3       ->  P(2, dp) = diag(2 * 0.01/9, 2 * 0.01/9, 0)
//   sum = diag(0.06/9, 0.06/9, 0) = diag(0.02/3, 0.02/3, 0)
//   (check: reduced mass 1 * 2 / 3 times 0.1^2 = 0.02/3)
TEST(PayloadInertia, OffsetMergedByParallelAxis) {
  const M1 nominal = unit_link();
  ModelMismatchConfig<1> mm;
  mm.payload_kg = 2.0;
  mm.payload_offset_m = Vec3(0.0, 0.0, 0.1);
  const M1 out = perturb_model(nominal, mm, 7);
  const model::LinkInertia& l = out.links[0];
  EXPECT_NEAR(l.mass_kg, 3.0, kTol);
  EXPECT_LT((l.com_i - Vec3(0.0, 0.0, 0.2 / 3.0)).cwiseAbs().maxCoeff(), kTol);
  const Mat3 expected = Vec3(0.01 + 0.02 / 3.0, 0.02 + 0.02 / 3.0, 0.03).asDiagonal();
  EXPECT_LT(max_abs(l.I_com_i - expected), kTol) << l.I_com_i;
}

// Same with radius 0.05: the sphere adds (2/5) m r^2 = 0.4 * 2 * 0.0025 = 0.002 to every
// diagonal entry about its own centre; isotropic, so the shift to c' leaves it as is. Mass and
// centre of mass do not depend on the radius.
TEST(PayloadInertia, RadiusAddsSolidSphereInertia) {
  const M1 nominal = unit_link();
  ModelMismatchConfig<1> mm;
  mm.payload_kg = 2.0;
  mm.payload_offset_m = Vec3(0.0, 0.0, 0.1);
  mm.payload_radius_m = 0.05;
  const M1 out = perturb_model(nominal, mm, 7);
  const model::LinkInertia& l = out.links[0];
  EXPECT_NEAR(l.mass_kg, 3.0, kTol);
  EXPECT_LT((l.com_i - Vec3(0.0, 0.0, 0.2 / 3.0)).cwiseAbs().maxCoeff(), kTol);
  constexpr double kSphere = 2.0 / 5.0 * 2.0 * 0.05 * 0.05;  // 0.002 kg m^2
  const Mat3 expected =
      Vec3(0.01 + 0.02 / 3.0 + kSphere, 0.02 + 0.02 / 3.0 + kSphere, 0.03 + kSphere).asDiagonal();
  EXPECT_LT(max_abs(l.I_com_i - expected), kTol) << l.I_com_i;
  EXPECT_NEAR(l.I_com_i(0, 0), 0.01 + 0.02 / 3.0 + 0.002, kTol);
}

// The offset is measured from the tip origin (offset 0 = today's point at the tip): two_link
// has its tip at (0.1, -0.05, 0.2) in the last link frame. 1 kg at offset (0.05, 0, -0.1):
//   p  = (0.15, -0.05, 0.1)
//   c' = (1.5 (0.02, 0.01, -0.1) + 1 (0.15, -0.05, 0.1)) / 2.5
//      = ((0.03 + 0.15), (0.015 - 0.05), (-0.15 + 0.1)) / 2.5 = (0.072, -0.014, -0.02)
//   d0 = c - c' = (-0.052, 0.024, -0.08),  dp = p - c' = (0.078, -0.036, 0.12)
//   check 1.5 d0 + 1 dp = (-0.078 + 0.078, 0.036 - 0.036, -0.12 + 0.12) = 0
// Radius 0.04: sphere (2/5) 1 0.04^2 = 6.4e-4 on the diagonal.
TEST(PayloadInertia, OffsetIsFromTheTipOriginWithFullInertia) {
  const M2 nominal = two_link();
  ModelMismatchConfig<2> mm;
  mm.payload_kg = 1.0;
  mm.payload_offset_m = Vec3(0.05, 0.0, -0.1);
  mm.payload_radius_m = 0.04;
  const M2 out = perturb_model(nominal, mm, 3);
  EXPECT_EQ(out.links[0].mass_kg, nominal.links[0].mass_kg);
  EXPECT_TRUE(out.links[0].com_i == nominal.links[0].com_i);
  EXPECT_TRUE(out.links[0].I_com_i == nominal.links[0].I_com_i);
  const model::LinkInertia& l = out.links[1];
  EXPECT_NEAR(l.mass_kg, 2.5, kTol);
  EXPECT_LT((l.com_i - Vec3(0.072, -0.014, -0.02)).cwiseAbs().maxCoeff(), kTol) << l.com_i;
  const Vec3 d0(-0.052, 0.024, -0.08);
  const Vec3 dp(0.078, -0.036, 0.12);
  const Mat3 expected =
      nominal.links[1].I_com_i + point(1.5, d0) + point(1.0, dp) + 6.4e-4 * Mat3::Identity();
  EXPECT_LT(max_abs(l.I_com_i - expected), kTol) << l.I_com_i;
}

// The offset is in the tip (link_7) frame: with the tip turned +90 deg about z relative to the
// last joint frame, offset (0, 0.1, 0) points along -x of the last link frame.
//   R_z(90) (0, 0.1, 0) = (-0.1, 0, 0)  ->  p = (-0.1, 0, 0) on the unit link (tip at 0)
//   c' = 2 (-0.1, 0, 0) / 3 = (-0.2/3, 0, 0)
//   inertia: reduced mass 2/3 times 0.1^2 = 0.02/3 on yy and zz
TEST(PayloadInertia, OffsetIsExpressedInTheTipFrame) {
  M1 nominal = unit_link();
  nominal.T_last_tip.R << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;  // R_z(+90 deg)
  ModelMismatchConfig<1> mm;
  mm.payload_kg = 2.0;
  mm.payload_offset_m = Vec3(0.0, 0.1, 0.0);
  const M1 out = perturb_model(nominal, mm, 7);
  const model::LinkInertia& l = out.links[0];
  EXPECT_NEAR(l.mass_kg, 3.0, kTol);
  EXPECT_LT((l.com_i - Vec3(-0.2 / 3.0, 0.0, 0.0)).cwiseAbs().maxCoeff(), kTol) << l.com_i;
  const Mat3 expected = Vec3(0.01, 0.02 + 0.02 / 3.0, 0.03 + 0.02 / 3.0).asDiagonal();
  EXPECT_LT(max_abs(l.I_com_i - expected), kTol) << l.I_com_i;
}

// ---------------------------------------------------------------- bit identity

// Offset 0 and radius 0: today's point mass at the tip origin, bit for bit, for any payload_kg,
// also after the mass scale and centre-of-mass shift draws.
TEST(PayloadInertia, ZeroOffsetAndRadiusBitIdenticalToPointMass) {
  const M2 nominal = two_link();
  for (const bool perturbed : {false, true}) {
    for (const double kg : {0.25, 1.0, 2.0, 7.5}) {
      ModelMismatchConfig<2> mm;
      if (perturbed) {
        mm.link_mass_scale_std.setConstant(0.1);
        mm.com_shift_std_m = 0.01;
      }
      constexpr std::uint64_t kSeed = 11;
      M2 expected = perturb_model(nominal, mm, kSeed);  // no payload
      merge_point_mass_at_tip(expected, nominal.T_last_tip.p, kg);
      mm.payload_kg = kg;
      mm.payload_offset_m = Vec3::Zero();
      mm.payload_radius_m = 0.0;
      SCOPED_TRACE(::testing::Message() << "payload " << kg << " kg, perturbed " << perturbed);
      expect_bit_identical(perturb_model(nominal, mm, kSeed), expected);
    }
  }
}

// payload_kg = 0: offset and radius are ignored, the model is the one without payload.
TEST(PayloadInertia, ZeroPayloadIgnoresOffsetAndRadius) {
  const M2 nominal = two_link();
  for (const bool perturbed : {false, true}) {
    ModelMismatchConfig<2> mm;
    if (perturbed) {
      mm.link_mass_scale_std.setConstant(0.1);
      mm.com_shift_std_m = 0.01;
    }
    constexpr std::uint64_t kSeed = 4;
    const M2 expected = perturb_model(nominal, mm, kSeed);
    mm.payload_offset_m = Vec3(0.1, -0.2, 0.3);
    mm.payload_radius_m = 0.05;
    SCOPED_TRACE(::testing::Message() << "perturbed " << perturbed);
    expect_bit_identical(perturb_model(nominal, mm, kSeed), expected);
    if (!perturbed) {
      expect_bit_identical(perturb_model(nominal, mm, kSeed), nominal);
    }
  }
}

// ---------------------------------------------------------------- arm gravity torque

// Gravity torque is linear in the mass distribution and G = dV/dq with V = m g0 h, so moving a
// point mass m by dp (in b) changes tau_j by  m g0 e_z . (z_j x dp)  (revolute j, axis z_j).
// Joint 2 of the arm is horizontal at every pose (joint 1 vertical); with dp = dx u,
// u = e_z x z_2 (horizontal, normal to z_2):  e_z . (z_2 x u) = u . (e_z x z_2) = 1, so tau_2
// grows by m g0 dx (times s = |e_z x z_2|, 1 up to the URDF angle rounding). Here m = 2 kg, dx =
// 0.1 m: 2 * 9.80665 * 0.1 = 1.96133 N m.
TEST(PayloadInertia, ArmGravityTorqueGrowsByMgDx) {
  const auto chain = model::parse_urdf_chain(
      std::string(COBOT_REPO_DIR) + "/src/sim/urdf/cobot30_5.urdf", "base_link", "link_7");
  ASSERT_TRUE(chain.is_ok()) << chain.reason();
  constexpr double kG0 = 9.80665;
  const auto arm = model::make_model<model::kArmDof>(chain.value(), Vec3(0.0, 0.0, -kG0));
  ASSERT_TRUE(arm.is_ok()) << arm.reason();
  using V7 = common::VecN<model::kArmDof>;

  // Shoulder at 90 deg so the upper arm is horizontal (the identity holds at any pose).
  V7 q;
  q << 0.3, 1.5707963267948966, 0.0, -0.4, 0.2, 0.5, -0.1;
  kinematics::FkState<model::kArmDof> fk;
  kinematics::forward_kinematics(arm.value(), q, fk);
  const Vec3 z2 = fk.axis_b[1];
  // The URDF rounds pi/2 to 1.5708, which tilts z_2 by 3.7e-6 rad: s = |e_z x z_2| = 1 - 7e-12.
  ASSERT_LT(std::abs(z2.z()), 1e-5) << "joint 2 axis not horizontal: " << z2.transpose();
  const double s = Vec3::UnitZ().cross(z2).norm();
  const Vec3 u = Vec3::UnitZ().cross(z2) / s;
  // Upper arm (joint 2 -> joint 3 origin) horizontal at this pose.
  ASSERT_LT(std::abs(fk.p_b_joint[2].z() - fk.p_b_joint[1].z()), 1e-3)
      << fk.p_b_joint[1].transpose() << " / " << fk.p_b_joint[2].transpose();

  constexpr double kKg = 2.0;
  constexpr double kDx = 0.1;  // m
  ModelMismatchConfig<model::kArmDof> mm;
  mm.payload_kg = kKg;
  mm.payload_offset_m = Vec3(0.01, -0.02, 0.03);  // any start point
  const V7 tau_a = dynamics::gravity_torque(perturb_model(arm.value(), mm, 1), q);
  const Vec3 dp_b = kDx * u;
  mm.payload_offset_m += fk.T_b_e.R.transpose() * dp_b;  // the offset lives in the tip frame
  const V7 tau_b = dynamics::gravity_torque(perturb_model(arm.value(), mm, 1), q);

  // e_z . (z_2 x u) = u . (e_z x z_2) = s:  m g0 dx s = 1.96133 N m (s - 1 = -7e-12)
  EXPECT_NEAR(tau_b[1] - tau_a[1], kKg * kG0 * kDx * s, 1e-9);
  EXPECT_NEAR(tau_b[1] - tau_a[1], 1.96133, 1e-9);
  for (int j = 0; j < model::kArmDof; ++j) {
    const double expected = kKg * kG0 * Vec3::UnitZ().dot(fk.axis_b[j].cross(dp_b));
    EXPECT_NEAR(tau_b[j] - tau_a[j], expected, 1e-9) << "joint " << j + 1;
  }
}

}  // namespace
}  // namespace cobot::imperfect
