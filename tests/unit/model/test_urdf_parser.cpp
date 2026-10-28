#include <cmath>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "cobot/model/model.hpp"

#include "model_test_util.hpp"

namespace cobot::model {
namespace {

using test::fixture;
using test::inertia;
using test::origin;
using test::pose_diff;
using test::real_urdf;
using test::rpy;

constexpr double kTol = 1e-12;
constexpr double kHalfPi = 1.5707963267948966;

UrdfChain parse_ok(const std::string& path, const std::string& base, const std::string& tip) {
  Result<UrdfChain> r = parse_urdf_chain(path, base, tip);
  EXPECT_TRUE(r.is_ok()) << path << ": " << r.reason();
  return r.is_ok() ? std::move(r).value() : UrdfChain{};
}

void expect_error_mentions(const std::string& path,
                           const std::string& base,
                           const std::string& tip,
                           const std::string& needle) {
  const Result<UrdfChain> r = parse_urdf_chain(path, base, tip);
  EXPECT_FALSE(r.is_ok()) << path;
  EXPECT_NE(r.reason().find(needle), std::string::npos)
      << path << ": reason '" << r.reason() << "' should mention '" << needle << "'";
}

// joint_6_2_fix <origin> of the real robot, from the raw XML numbers.
Pose real_t_6_62() {
  return origin(Vec3(0.0860000000000007, 0, 0), Vec3(1.5707963267949, 3.1416, 1.5707963267949));
}

// ---- the real robot -------------------------------------------------------

TEST(UrdfRealRobot, SevenJointsInOrder) {
  const UrdfChain c = parse_ok(real_urdf(), "base_link", "link_7");
  ASSERT_EQ(c.joints.size(), 7U);
  ASSERT_EQ(c.links.size(), 7U);
  for (int k = 0; k < 7; ++k) {
    EXPECT_EQ(c.joints[k].name, "joint_" + std::to_string(k + 1));
    EXPECT_LT((c.joints[k].axis_joint - Vec3::UnitZ()).norm(), kTol);
    EXPECT_DOUBLE_EQ(c.joints[k].damping, 0.003);
  }
}

TEST(UrdfRealRobot, LimitsFromXml) {
  const UrdfChain c = parse_ok(real_urdf(), "base_link", "link_7");
  ASSERT_EQ(c.joints.size(), 7U);
  EXPECT_DOUBLE_EQ(c.joints[3].q_min, -3.1416);
  EXPECT_DOUBLE_EQ(c.joints[3].q_max, -0.4);
  EXPECT_DOUBLE_EQ(c.joints[1].tau_max, 87.0);
  EXPECT_DOUBLE_EQ(c.joints[5].qd_max, 3.0);
  EXPECT_DOUBLE_EQ(c.joints[0].qd_max, 2.5);
}

TEST(UrdfRealRobot, MassBudgetAndLink62Merge) {
  const UrdfChain c = parse_ok(real_urdf(), "base_link", "link_7");
  ASSERT_EQ(c.links.size(), 7U);
  double total = 0.0;
  for (const LinkInertia& l : c.links) {
    total += l.mass_kg;
  }
  EXPECT_NEAR(total, 17.053, 1e-6);
  EXPECT_NEAR(c.links[5].mass_kg, 1.840, 1e-9);
  EXPECT_NEAR(c.links[0].mass_kg, 3.18, kTol);
  EXPECT_NEAR(c.links[6].mass_kg, 0.227, kTol);

  // link_1 is not merged with anything: raw values.
  const Vec3 c1(0.00568622330320681, -8.20843742846758E-06, -0.0114699821762951);
  EXPECT_LT((c.links[0].com_i - c1).norm(), kTol);
  const Mat3 I_link1 = inertia(0.006332753276957904,
                               -2.127510845206133e-06,
                               -0.00023970888735097348,
                               0.006642894673699633,
                               8.81076661193496e-07,
                               0.005998003761121196);
  EXPECT_LT((c.links[0].I_com_i - I_link1).norm(), kTol);

  // links[5] centre of mass: link_6 and link_6_2 (through joint_6_2_fix), in link_6 frame.
  const Vec3 c6(0.00290799885931436, 7.79152273618633E-05, -0.00368819421517057);
  const Vec3 c62(0.00290799920267108, 7.79156612489354E-05, -0.00368819425960698);
  const Vec3 com = (0.92 * c6 + 0.92 * common::transform(real_t_6_62(), c62)) / 1.84;
  EXPECT_LT((c.links[5].com_i - com).norm(), kTol);
}

TEST(UrdfRealRobot, Joint7OriginComposesFixedJoint) {
  const UrdfChain c = parse_ok(real_urdf(), "base_link", "link_7");
  ASSERT_EQ(c.joints.size(), 7U);
  const Pose T_62_j7 = origin(Vec3(0.0725, 0, 0), Vec3(0, 1.5708, 0));
  EXPECT_LT(pose_diff(c.joints[6].T_parent_joint, common::compose(real_t_6_62(), T_62_j7)), kTol);
  // Joints without a fixed joint before them keep their own <origin>.
  EXPECT_LT(pose_diff(c.joints[0].T_parent_joint, origin(Vec3(0, 0, 0.159), Vec3::Zero())), kTol);
  EXPECT_LT(
      pose_diff(c.joints[1].T_parent_joint, origin(Vec3(0.134, 0, 0), Vec3(0, -1.5708, 3.1416))),
      kTol);
  EXPECT_LT(pose_diff(c.T_last_tip, Pose{}), kTol);
}

// ---- synthetic fixtures ---------------------------------------------------

TEST(UrdfPlanar, EveryField) {
  const UrdfChain c = parse_ok(fixture("planar_2link.urdf"), "base_link", "link_2");
  ASSERT_EQ(c.joints.size(), 2U);
  ASSERT_EQ(c.links.size(), 2U);
  const JointParams& j1 = c.joints[0];
  EXPECT_EQ(j1.name, "joint_1");
  EXPECT_LT(pose_diff(j1.T_parent_joint, origin(Vec3(0, 0, 0.1), Vec3::Zero())), kTol);
  EXPECT_LT((j1.axis_joint - Vec3::UnitZ()).norm(), kTol);
  EXPECT_DOUBLE_EQ(j1.q_min, -1.5);
  EXPECT_DOUBLE_EQ(j1.q_max, 1.5);
  EXPECT_DOUBLE_EQ(j1.qd_max, 2.0);
  EXPECT_DOUBLE_EQ(j1.tau_max, 10.0);
  EXPECT_DOUBLE_EQ(j1.damping, 0.1);

  const JointParams& j2 = c.joints[1];
  EXPECT_EQ(j2.name, "joint_2");
  EXPECT_LT(pose_diff(j2.T_parent_joint, origin(Vec3(0.5, 0, 0), Vec3(0, 0, 0.5))), kTol);
  EXPECT_LT((j2.axis_joint - Vec3::UnitZ()).norm(), kTol);  // "0 0 3" normalised
  EXPECT_DOUBLE_EQ(j2.q_min, -2.0);
  EXPECT_DOUBLE_EQ(j2.q_max, 2.5);
  EXPECT_DOUBLE_EQ(j2.qd_max, 3.0);
  EXPECT_DOUBLE_EQ(j2.tau_max, 5.0);
  EXPECT_DOUBLE_EQ(j2.damping, 0.0);  // no <dynamics>

  EXPECT_DOUBLE_EQ(c.links[0].mass_kg, 2.0);
  EXPECT_LT((c.links[0].com_i - Vec3(0.25, 0, 0)).norm(), kTol);
  EXPECT_LT((c.links[0].I_com_i - inertia(0.001, 0, 0, 0.0427, 0, 0.0427)).norm(), kTol);
  EXPECT_DOUBLE_EQ(c.links[1].mass_kg, 1.0);
  EXPECT_LT((c.links[1].com_i - Vec3(0.2, 0.01, 0)).norm(), kTol);
  EXPECT_LT((c.links[1].I_com_i - inertia(0.0005, 0.0001, 0, 0.0135, 0, 0.0137)).norm(), kTol);
  EXPECT_LT(pose_diff(c.T_last_tip, Pose{}), kTol);
}

TEST(UrdfPlanar, SubChain) {
  const UrdfChain c = parse_ok(fixture("planar_2link.urdf"), "link_1", "link_2");
  ASSERT_EQ(c.joints.size(), 1U);
  EXPECT_EQ(c.joints[0].name, "joint_2");
  EXPECT_DOUBLE_EQ(c.links[0].mass_kg, 1.0);
}

TEST(UrdfFixedMerge, MassComSteinerAndOrigins) {
  const UrdfChain c = parse_ok(fixture("fixed_merge.urdf"), "base_link", "ee");
  ASSERT_EQ(c.joints.size(), 2U);
  ASSERT_EQ(c.links.size(), 2U);
  // RPY convention R = Rz(y) Ry(p) Rx(r).
  EXPECT_LT((c.joints[0].T_parent_joint.R - rpy(0.1, 0.2, 0.3)).norm(), kTol);
  EXPECT_LT((c.joints[0].T_parent_joint.p - Vec3(0, 0, 0.05)).norm(), kTol);

  // link_1 (2 kg at x=0.1) + tool (0.5 kg, rotated 90 deg about z, CoM at x=0.2, rotated
  // inertia diag(0.002, 0.001, 0.003)): CoM x = 0.12; Steiner adds
  // 2*0.02^2 + 0.5*0.08^2 = 0.004 to Iyy and Izz.
  EXPECT_NEAR(c.links[0].mass_kg, 2.5, kTol);
  EXPECT_LT((c.links[0].com_i - Vec3(0.12, 0, 0)).norm(), kTol);
  EXPECT_LT((c.links[0].I_com_i - inertia(0.012, 0, 0, 0.025, 0, 0.037)).norm(), kTol);

  // joint_2 origin = tool_fix origin composed with its own.
  const Pose T_1_j2 = common::compose(origin(Vec3(0.3, 0, 0), Vec3(0, 0, kHalfPi)),
                                      origin(Vec3(0.05, 0, 0), Vec3::Zero()));
  EXPECT_LT(pose_diff(c.joints[1].T_parent_joint, T_1_j2), kTol);
  EXPECT_LT((c.joints[1].axis_joint - Vec3::UnitY()).norm(), kTol);

  // link_2 (1 kg at 0) + ee (0.2 kg at z=0.1, after the last revolute joint):
  // CoM z = 0.2 * 0.1 / 1.2; Steiner about the combined CoM adds
  // m1 m2 / (m1 + m2) * d^2 = (0.2 / 1.2) * 0.01 to Ixx and Iyy.
  constexpr double kComZ = 0.2 * 0.1 / 1.2;
  constexpr double kIxx = 0.001 + (0.2 / 1.2) * 0.01;
  EXPECT_NEAR(c.links[1].mass_kg, 1.2, kTol);
  EXPECT_LT((c.links[1].com_i - Vec3(0, 0, kComZ)).norm(), kTol);
  EXPECT_LT((c.links[1].I_com_i - inertia(kIxx, 0, 0, kIxx, 0, 0.001)).norm(), kTol);
  EXPECT_LT(pose_diff(c.T_last_tip, origin(Vec3(0, 0, 0.1), Vec3::Zero())), kTol);
}

TEST(UrdfInertialOrigin, RotatedInertiaFrame) {
  const UrdfChain c = parse_ok(fixture("rotated_inertial.urdf"), "base_link", "link_1");
  ASSERT_EQ(c.links.size(), 1U);
  const Mat3 R = rpy(0.3, -0.2, 0.5);
  const Mat3 I = inertia(0.01, 0.001, -0.002, 0.02, 0.003, 0.03);
  const Mat3 expected = R * I * R.transpose();
  EXPECT_DOUBLE_EQ(c.links[0].mass_kg, 1.5);
  EXPECT_LT((c.links[0].com_i - Vec3(0.1, 0.2, 0.3)).norm(), kTol);
  EXPECT_LT((c.links[0].I_com_i - expected).norm(), kTol);
  EXPECT_LT((c.links[0].I_com_i - c.links[0].I_com_i.transpose()).norm(), kTol);
}

TEST(UrdfContinuous, InfiniteLimits) {
  const UrdfChain c = parse_ok(fixture("continuous.urdf"), "base_link", "wheel");
  ASSERT_EQ(c.joints.size(), 1U);
  const JointParams& j = c.joints[0];
  EXPECT_EQ(j.name, "wheel_joint");
  EXPECT_TRUE(std::isinf(j.q_min) && j.q_min < 0.0);
  EXPECT_TRUE(std::isinf(j.q_max) && j.q_max > 0.0);
  EXPECT_DOUBLE_EQ(j.tau_max, 4.0);
  EXPECT_DOUBLE_EQ(j.qd_max, 6.0);
  EXPECT_LT((j.axis_joint - Vec3::UnitX()).norm(), kTol);
}

// ---- errors ---------------------------------------------------------------

TEST(UrdfErrors, MissingLimitEffortNamesJoint) {
  expect_error_mentions(fixture("missing_effort.urdf"), "base_link", "link_1", "joint_3");
  expect_error_mentions(fixture("missing_effort.urdf"), "base_link", "link_1", "effort");
}

TEST(UrdfErrors, PrismaticNamesJoint) {
  expect_error_mentions(fixture("prismatic.urdf"), "base_link", "link_1", "slider");
}

TEST(UrdfErrors, ZeroAxisNamesJoint) {
  expect_error_mentions(fixture("zero_axis.urdf"), "base_link", "link_1", "bad_axis");
}

TEST(UrdfErrors, UnknownLinks) {
  expect_error_mentions(
      fixture("planar_2link.urdf"), "base_link", "no_such_tip", "link 'no_such_tip' not found");
  expect_error_mentions(
      fixture("planar_2link.urdf"), "no_such_base", "link_2", "link 'no_such_base' not found");
}

TEST(UrdfErrors, TipNotDescendantOfBase) {
  expect_error_mentions(
      fixture("planar_2link.urdf"), "link_2", "link_1", "no chain from 'link_2' to 'link_1'");
}

TEST(UrdfErrors, MissingFileAndBadXml) {
  EXPECT_FALSE(parse_urdf_chain(fixture("does_not_exist.urdf"), "base_link", "link_1").is_ok());
  EXPECT_FALSE(parse_urdf_chain(fixture("not_xml.urdf"), "base_link", "link_1").is_ok());
}

}  // namespace
}  // namespace cobot::model
