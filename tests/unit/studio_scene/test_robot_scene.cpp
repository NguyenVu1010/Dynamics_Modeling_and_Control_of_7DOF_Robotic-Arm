// RobotScene (feature studio-3d, cobot_studio design 6.6 and 7): the visual links of
// src/sim/urdf/cobot30_5.urdf in URDF order, their meshes resolved through the package map, and
// their world poses from forward_kinematics on the kinematic model of the same URDF.
//
// URDF facts used below: world -fixed-> base_link -joint_1..joint_6-> link_1..link_6,
// link_6 -fixed joint_6_2_fix-> link_6_2 -joint_7-> link_7 (the tip, frame e). Every <visual>
// has an identity <origin> and the colour rgba 0.792156862745098 0.819607843137255
// 0.933333333333333 1.
#include <array>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/kinematics/kinematics.hpp"
#include "cobot/model/model.hpp"
#include "cobot/studio/robot_scene.hpp"

#include "studio_scene_test_util.hpp"

namespace cobot::studio {
namespace {

using test::kRepo;
using test::kUrdf;
using test::max_abs_diff;
using test::pos_err;
using test::repo_packages;
using test::rot_err;
using test::urdf_origin;
namespace fs = std::filesystem;
constexpr int kN = model::kArmDof;

// URDF order of the links that have a <visual>; world has none.
const std::array<std::string, 9> kLinkNames = {
    "base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6", "link_6_2", "link_7"};
// Revolute joint (0-based, = FkState::T_b_i index) whose frame carries each link; -1: base.
// link_6_2 hangs off link_6 by a fixed joint, so it rides on joint_6 (index 5).
constexpr std::array<int, 9> kJointOf = {-1, 0, 1, 2, 3, 4, 5, 5, 6};
constexpr std::size_t kLink6 = 6;
constexpr std::size_t kLink62 = 7;
constexpr std::size_t kLink7 = 8;

// joint_6_2_fix <origin xyz="0.0860000000000007 0 0" rpy="1.5707963267949 3.1416 1.5707963267949">
common::Pose joint_6_2_fix_origin() {
  return urdf_origin(common::Vec3(0.0860000000000007, 0.0, 0.0),
                     common::Vec3(1.5707963267949, 3.1416, 1.5707963267949));
}

model::Arm7 load_arm() {
  const auto chain = model::parse_urdf_chain(kUrdf, "base_link", "link_7");
  EXPECT_TRUE(chain.is_ok()) << chain.reason();
  const auto arm = model::make_model<kN>(chain.value(), common::Vec3(0.0, 0.0, -9.80665));
  EXPECT_TRUE(arm.is_ok()) << arm.reason();
  return arm.value();
}

common::Result<RobotScene> load_scene(const fs::path& urdf = kUrdf) {
  return RobotScene::load(urdf, "base_link", "link_7", repo_packages());
}

// A copy of the arm's URDF whose link_3 <visual> mesh (the first occurrence of its URI; the
// <collision> one follows) is replaced by `uri`.
fs::path urdf_with_link_3_mesh(const std::string& uri, const std::string& file_name) {
  std::ifstream in(kUrdf, std::ios::binary);
  std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const std::string old_uri = "package://cobot_sim/meshes/link_3.STL";
  const std::size_t at = text.find(old_uri);
  EXPECT_NE(at, std::string::npos);
  text.replace(at, old_uri.size(), uri);
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_scene";
  fs::create_directories(dir);
  const fs::path out = dir / file_name;
  std::ofstream os(out, std::ios::binary | std::ios::trunc);
  os << text;
  return out;
}

TEST(RobotScene, NineVisualLinksInUrdfOrderWithResolvedMeshes) {
  const auto scene = load_scene();
  ASSERT_TRUE(scene.is_ok()) << scene.reason();
  const std::vector<VisualLink>& links = scene.value().links();
  ASSERT_EQ(links.size(), kLinkNames.size());
  for (std::size_t i = 0; i < links.size(); ++i) {
    SCOPED_TRACE(kLinkNames[i]);
    EXPECT_EQ(links[i].link, kLinkNames[i]);
    const fs::path mesh = kRepo / "src/sim/meshes" / (kLinkNames[i] + ".STL");
    ASSERT_TRUE(fs::exists(links[i].mesh)) << links[i].mesh;
    EXPECT_TRUE(fs::equivalent(links[i].mesh, mesh)) << links[i].mesh << " vs " << mesh;
    EXPECT_EQ(links[i].joint, kJointOf[i]);
    EXPECT_LE(max_abs_diff(links[i].T_frame_visual, common::Pose{}), 1e-15) << "identity origin";
    EXPECT_NEAR(links[i].rgba[0], 0.792156862745098, 1e-6);
    EXPECT_NEAR(links[i].rgba[1], 0.819607843137255, 1e-6);
    EXPECT_NEAR(links[i].rgba[2], 0.933333333333333, 1e-6);
    EXPECT_NEAR(links[i].rgba[3], 1.0, 1e-6);
    if (i != kLink62) {
      // Every link but link_6_2 is the child of its revolute joint: no fixed offset.
      EXPECT_LE(max_abs_diff(links[i].T_joint_link, common::Pose{}), 1e-15);
    }
  }
  // link_6_2: the fixed joint_6_2_fix folded into the offset from joint_6's frame.
  EXPECT_LE(max_abs_diff(links[kLink62].T_joint_link, joint_6_2_fix_origin()), 1e-12);
}

TEST(RobotScene, BaseLinkIsTheIdentityForAnyJointAngles) {
  const auto scene = load_scene();
  ASSERT_TRUE(scene.is_ok()) << scene.reason();
  std::vector<common::Pose> poses(scene.value().links().size());
  const common::VecN<kN> q =
      (common::VecN<kN>() << 0.3, -0.7, 1.1, -1.5, 0.4, 2.0, -0.9).finished();
  scene.value().visual_poses(q, poses);
  EXPECT_LE(max_abs_diff(poses[0], common::Pose{}), 1e-15);
}

// 200 joint vectors uniform inside the URDF limits (std::mt19937_64(7)): tip_pose and the link_7
// visual are FK T_b_e; link_1..link_6 are FK T_b_i (identity visual origins); link_6_2 is
// link_6 composed with the joint_6_2_fix origin.
TEST(RobotScene, PosesMatchForwardKinematicsInsideTheLimits) {
  const auto scene = load_scene();
  ASSERT_TRUE(scene.is_ok()) << scene.reason();
  const model::Arm7 arm = load_arm();
  std::mt19937_64 rng(7);
  std::vector<common::Pose> poses(scene.value().links().size());
  kinematics::FkState<kN> fk;
  const common::Pose fix = joint_6_2_fix_origin();
  for (int sample = 0; sample < 200; ++sample) {
    common::VecN<kN> q;
    for (int j = 0; j < kN; ++j) {
      const auto& joint = arm.joints[static_cast<std::size_t>(j)];
      std::uniform_real_distribution<double> uniform(joint.q_min, joint.q_max);
      q[j] = uniform(rng);
    }
    SCOPED_TRACE(testing::Message() << "sample " << sample << " q " << q.transpose());
    kinematics::forward_kinematics(arm, q, fk);
    scene.value().visual_poses(q, poses);
    const common::Pose tip = scene.value().tip_pose(q);
    EXPECT_LE(pos_err(tip, fk.T_b_e), 1e-9);
    EXPECT_LE(rot_err(tip, fk.T_b_e), 1e-9);
    EXPECT_LE(pos_err(poses[kLink7], fk.T_b_e), 1e-9);
    EXPECT_LE(rot_err(poses[kLink7], fk.T_b_e), 1e-9);
    for (std::size_t i = 1; i <= kLink6; ++i) {
      EXPECT_LE(pos_err(poses[i], fk.T_b_i[i - 1]), 1e-9) << kLinkNames[i];
      EXPECT_LE(rot_err(poses[i], fk.T_b_i[i - 1]), 1e-9) << kLinkNames[i];
    }
    const common::Pose link_6_2 = common::compose(poses[kLink6], fix);
    EXPECT_LE(max_abs_diff(poses[kLink62], link_6_2), 1e-12);
  }
}

TEST(RobotScene, UnknownPackageIsAnErrorNamingTheUriAndTheLink) {
  const fs::path urdf = urdf_with_link_3_mesh("package://other/x.STL", "unknown_package.urdf");
  const auto scene = load_scene(urdf);
  ASSERT_FALSE(scene.is_ok());
  EXPECT_NE(scene.reason().find("package://other/x.STL"), std::string::npos) << scene.reason();
  EXPECT_NE(scene.reason().find("link_3"), std::string::npos) << scene.reason();
}

TEST(RobotScene, MissingMeshFileIsAnErrorNamingThePathAndTheLink) {
  const fs::path urdf =
      urdf_with_link_3_mesh("package://cobot_sim/meshes/missing_link_3.STL", "missing_mesh.urdf");
  const auto scene = load_scene(urdf);
  ASSERT_FALSE(scene.is_ok());
  // Resolved through the package map: <repo>/src/sim/meshes/missing_link_3.STL.
  EXPECT_NE(scene.reason().find("meshes/missing_link_3.STL"), std::string::npos) << scene.reason();
  EXPECT_NE(scene.reason().find("link_3"), std::string::npos) << scene.reason();
}

TEST(RobotScene, MissingUrdfIsAnErrorNamingIt) {
  const fs::path urdf = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_scene" / "no_such_robot.urdf";
  const auto scene = load_scene(urdf);
  ASSERT_FALSE(scene.is_ok());
  EXPECT_NE(scene.reason().find("no_such_robot.urdf"), std::string::npos) << scene.reason();
}

}  // namespace
}  // namespace cobot::studio
