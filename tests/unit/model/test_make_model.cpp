#include <string>

#include <gtest/gtest.h>

#include "cobot/model/model.hpp"

#include "model_test_util.hpp"

namespace cobot::model {
namespace {

using test::fixture;
using test::pose_diff;
using test::real_urdf;

const Vec3 kGravity(0.0, 0.0, -9.80665);

// make_model is instantiated only for the DOFs in cobot/common/dof.hpp (1, 2, 7), so the
// size mismatch is checked with those.

TEST(MakeModel, RealChainArm7) {
  const Result<UrdfChain> chain = parse_urdf_chain(real_urdf(), "base_link", "link_7");
  ASSERT_TRUE(chain.is_ok()) << chain.reason();
  const Result<Arm7> m = make_model<kArmDof>(chain.value(), kGravity);
  ASSERT_TRUE(m.is_ok()) << m.reason();
  EXPECT_EQ(m.value().joints[0].name, "joint_1");
  EXPECT_EQ(m.value().joints[6].name, "joint_7");
  EXPECT_DOUBLE_EQ(m.value().links[5].mass_kg, chain.value().links[5].mass_kg);
  EXPECT_LT((m.value().gravity_b - kGravity).norm(), 1e-15);
}

TEST(MakeModel, WrongDofIsAnError) {
  const Result<UrdfChain> arm = parse_urdf_chain(real_urdf(), "base_link", "link_7");
  ASSERT_TRUE(arm.is_ok()) << arm.reason();
  const Result<Model<2>> m2 = make_model<2>(arm.value(), kGravity);
  EXPECT_FALSE(m2.is_ok());
  EXPECT_NE(m2.reason().find("expected 2 revolute joints, got 7"), std::string::npos)
      << m2.reason();

  const Result<UrdfChain> planar =
      parse_urdf_chain(fixture("planar_2link.urdf"), "base_link", "link_2");
  ASSERT_TRUE(planar.is_ok()) << planar.reason();
  const Result<Arm7> m7 = make_model<7>(planar.value(), kGravity);
  EXPECT_FALSE(m7.is_ok());
  EXPECT_NE(m7.reason().find("expected 7 revolute joints, got 2"), std::string::npos)
      << m7.reason();

  const Result<Model<1>> m1 = make_model<1>(UrdfChain{}, kGravity);
  EXPECT_FALSE(m1.is_ok());
  EXPECT_NE(m1.reason().find("expected 1 revolute joints, got 0"), std::string::npos)
      << m1.reason();
}

TEST(MakeModel, PlanarCopiesAllFieldsAndGravity) {
  const Result<UrdfChain> r = parse_urdf_chain(fixture("planar_2link.urdf"), "base_link", "link_2");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  UrdfChain chain = r.value();
  chain.T_last_tip.p = Vec3(0.01, 0.02, 0.03);  // non-identity, to see that it is copied
  const Vec3 g(0.1, -0.2, -9.7);
  const Result<Model<2>> m = make_model<2>(chain, g);
  ASSERT_TRUE(m.is_ok()) << m.reason();
  const Model<2>& model = m.value();
  for (int k = 0; k < 2; ++k) {
    const JointParams& a = model.joints[k];
    const JointParams& b = chain.joints[k];
    EXPECT_EQ(a.name, b.name);
    EXPECT_EQ(pose_diff(a.T_parent_joint, b.T_parent_joint), 0.0);
    EXPECT_EQ(a.axis_joint, b.axis_joint);
    EXPECT_EQ(a.q_min, b.q_min);
    EXPECT_EQ(a.q_max, b.q_max);
    EXPECT_EQ(a.qd_max, b.qd_max);
    EXPECT_EQ(a.tau_max, b.tau_max);
    EXPECT_EQ(a.damping, b.damping);
    EXPECT_EQ(model.links[k].mass_kg, chain.links[k].mass_kg);
    EXPECT_EQ(model.links[k].com_i, chain.links[k].com_i);
    EXPECT_EQ(model.links[k].I_com_i, chain.links[k].I_com_i);
  }
  EXPECT_EQ(pose_diff(model.T_last_tip, chain.T_last_tip), 0.0);
  EXPECT_EQ(model.gravity_b, g);
}

}  // namespace
}  // namespace cobot::model
