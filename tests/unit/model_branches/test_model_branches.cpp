#include <string>

#include <gtest/gtest.h>

#include "cobot/model/model.hpp"

#include "model_test_util.hpp"

namespace cobot::model {
namespace {

using test::fixture;
using test::inertia;
using test::origin;
using test::pose_diff;

constexpr double kTol = 1e-12;

// base_fixed_side_branch.urdf: plate (5 kg) is fixed to base_link before joint_1 and is dropped;
// the moving finger branch on link_1 (finger 0.7 kg + finger_pad 0.3 kg) is ignored.
TEST(UrdfBranches, FixedLinkBeforeFirstJointIsDroppedAndComposed) {
  const Result<UrdfChain> r =
      parse_urdf_chain(fixture("base_fixed_side_branch.urdf"), "base_link", "link_2");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const UrdfChain& c = r.value();
  ASSERT_EQ(c.joints.size(), 2U);
  ASSERT_EQ(c.links.size(), 2U);
  EXPECT_EQ(c.joints[0].name, "joint_1");
  EXPECT_EQ(c.joints[1].name, "joint_2");
  // joint_1 origin in base_link = base_fix origin composed with joint_1's own origin.
  const Pose T_b_j1 = common::compose(origin(Vec3(0, 0, 0.2), Vec3(0, 0, 0.5)),
                                      origin(Vec3(0.1, 0, 0.05), Vec3::Zero()));
  EXPECT_LT(pose_diff(c.joints[0].T_parent_joint, T_b_j1), kTol);
  // Only link_1's own inertial: plate is not merged anywhere.
  EXPECT_NEAR(c.links[0].mass_kg, 2.0, kTol);
}

TEST(UrdfBranches, MovingSideBranchIsIgnored) {
  const Result<UrdfChain> r =
      parse_urdf_chain(fixture("base_fixed_side_branch.urdf"), "base_link", "link_2");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const UrdfChain& c = r.value();
  ASSERT_EQ(c.links.size(), 2U);
  // link_1 keeps exactly its own inertial; finger and finger_pad add nothing.
  EXPECT_NEAR(c.links[0].mass_kg, 2.0, kTol);
  EXPECT_LT((c.links[0].com_i - Vec3(0.1, 0, 0)).norm(), kTol);
  EXPECT_LT((c.links[0].I_com_i - inertia(0.01, 0, 0, 0.02, 0, 0.03)).norm(), kTol);
  EXPECT_NEAR(c.links[1].mass_kg, 1.0, kTol);
  EXPECT_LT((c.links[1].com_i - Vec3(0.2, 0, 0)).norm(), kTol);
  // joint_2 is placed directly in link_1; the finger joint does not enter the chain.
  EXPECT_LT(pose_diff(c.joints[1].T_parent_joint, origin(Vec3(0.4, 0, 0), Vec3::Zero())), kTol);
  EXPECT_LT((c.joints[1].axis_joint - Vec3::UnitY()).norm(), kTol);
  EXPECT_LT(pose_diff(c.T_last_tip, Pose{}), kTol);
}

TEST(UrdfBranches, ParseErrorsNameTheFile) {
  const std::string bad_axis = fixture("zero_axis.urdf");
  const Result<UrdfChain> r = parse_urdf_chain(bad_axis, "base_link", "link_1");
  ASSERT_FALSE(r.is_ok());
  EXPECT_NE(r.reason().find(bad_axis), std::string::npos) << r.reason();
  EXPECT_NE(r.reason().find("bad_axis"), std::string::npos) << r.reason();

  const std::string missing = fixture("does_not_exist.urdf");
  const Result<UrdfChain> m = parse_urdf_chain(missing, "base_link", "link_1");
  ASSERT_FALSE(m.is_ok());
  EXPECT_NE(m.reason().find(missing), std::string::npos) << m.reason();

  const std::string not_xml = fixture("not_xml.urdf");
  const Result<UrdfChain> x = parse_urdf_chain(not_xml, "base_link", "link_1");
  ASSERT_FALSE(x.is_ok());
  EXPECT_NE(x.reason().find(not_xml), std::string::npos) << x.reason();
}

}  // namespace
}  // namespace cobot::model
