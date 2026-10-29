// Feature model-armature, parser side: every revolute joint may carry
//   <cobot_armature value="..."/>   (kg m^2, reflected rotor inertia J_in * N^2)
// as a child element. Missing element -> 0; a negative, non-number, non-finite or absent value
// is a parse error that names the joint, the element and the file.
#include <array>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "cobot/model/model.hpp"

#include "model_test_util.hpp"

namespace cobot::model {
namespace {

using test::fixture;
using test::real_urdf;

constexpr double kTol = 1e-12;

UrdfChain parse_ok(const std::string& path, const std::string& base, const std::string& tip) {
  Result<UrdfChain> r = parse_urdf_chain(path, base, tip);
  EXPECT_TRUE(r.is_ok()) << path << ": " << r.reason();
  return r.is_ok() ? std::move(r).value() : UrdfChain{};
}

// The reason must name the joint, the <cobot_armature> element and the file.
void expect_armature_error(const std::string& file_name, const std::string& joint) {
  const std::string path = fixture(file_name);
  const Result<UrdfChain> r = parse_urdf_chain(path, "base_link", "link_2");
  ASSERT_FALSE(r.is_ok()) << path << " should not parse";
  for (const std::string& needle : {joint, std::string("cobot_armature"), file_name}) {
    EXPECT_NE(r.reason().find(needle), std::string::npos)
        << "reason '" << r.reason() << "' should mention '" << needle << "'";
  }
}

TEST(UrdfArmature, DefaultIsZero) {
  const JointParams p;
  EXPECT_EQ(p.armature, 0.0);
}

TEST(UrdfArmature, ValueReadAndMissingIsZero) {
  const UrdfChain c = parse_ok(fixture("armature_2link.urdf"), "base_link", "link_2");
  ASSERT_EQ(c.joints.size(), 2U);
  EXPECT_DOUBLE_EQ(c.joints[0].armature, 0.25);
  EXPECT_EQ(c.joints[1].armature, 0.0);
  // The new element does not disturb the standard ones.
  EXPECT_DOUBLE_EQ(c.joints[0].damping, 0.1);
  EXPECT_DOUBLE_EQ(c.joints[0].tau_max, 10.0);
  EXPECT_DOUBLE_EQ(c.joints[1].q_max, 2.5);
}

TEST(UrdfArmature, ZeroAllowedAndExponentForm) {
  const UrdfChain c = parse_ok(fixture("armature_zero.urdf"), "base_link", "link_2");
  ASSERT_EQ(c.joints.size(), 2U);
  EXPECT_EQ(c.joints[0].armature, 0.0);
  EXPECT_DOUBLE_EQ(c.joints[1].armature, 0.15);  // "1.5e-1"
}

TEST(UrdfArmature, NoArmatureAnywhereGivesZero) {
  const UrdfChain c = parse_ok(fixture("planar_2link.urdf"), "base_link", "link_2");
  ASSERT_EQ(c.joints.size(), 2U);
  for (const JointParams& j : c.joints) {
    EXPECT_EQ(j.armature, 0.0) << j.name;
  }
}

TEST(UrdfArmature, NegativeIsError) {
  expect_armature_error("armature_negative.urdf", "joint_2");
}

TEST(UrdfArmature, TextIsError) {
  expect_armature_error("armature_text.urdf", "joint_1");
}

TEST(UrdfArmature, MissingValueIsError) {
  expect_armature_error("armature_no_value.urdf", "joint_2");
}

TEST(UrdfArmature, NonFiniteIsError) {
  expect_armature_error("armature_inf.urdf", "joint_2");
}

TEST(UrdfArmature, MakeModelKeepsArmature) {
  const UrdfChain c = parse_ok(fixture("armature_2link.urdf"), "base_link", "link_2");
  const Result<Model<2>> m = make_model<2>(c, Vec3(0.0, 0.0, -9.80665));
  ASSERT_TRUE(m.is_ok()) << m.reason();
  EXPECT_DOUBLE_EQ(m.value().joints[0].armature, 0.25);
  EXPECT_EQ(m.value().joints[1].armature, 0.0);
}

// sim/urdf/mass_budget.yaml armature section, armature = J_in * N^2 with N = 100 and
// J_in in g mm^2 (1 g mm^2 = 1e-3 kg * 1e-6 m^2 = 1e-9 kg m^2):
//   joints 1, 2 (eRob110): 308632 g mm^2 = 3.08632e-4 kg m^2 -> * 1e4 = 3.08632 kg m^2
//   joints 3, 4 (eRob90):  100000 g mm^2 = 1.0e-4 kg m^2    -> * 1e4 = 1.0 kg m^2
//   joints 5-7 (eRob70):    22674 g mm^2 = 2.2674e-5 kg m^2 -> * 1e4 = 0.22674 kg m^2
TEST(UrdfArmature, RealRobotValues) {
  const UrdfChain c = parse_ok(real_urdf(), "base_link", "link_7");
  ASSERT_EQ(c.joints.size(), 7U);
  const std::array<double, 7> expected = {3.08632, 3.08632, 1.0, 1.0, 0.22674, 0.22674, 0.22674};
  for (int k = 0; k < 7; ++k) {
    EXPECT_NEAR(c.joints[k].armature, expected[k], kTol) << c.joints[k].name;
  }
  const Result<Arm7> arm = make_model<kArmDof>(c, Vec3(0.0, 0.0, -9.80665));
  ASSERT_TRUE(arm.is_ok()) << arm.reason();
  for (int k = 0; k < 7; ++k) {
    EXPECT_NEAR(arm.value().joints[k].armature, expected[k], kTol) << "joint " << k + 1;
  }
}

}  // namespace
}  // namespace cobot::model
