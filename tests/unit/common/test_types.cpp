#include "cobot/common/types.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "cobot/common/timestamp.hpp"

namespace cobot::common {
namespace {

Pose make_pose() {
  Pose T;
  T.R = Eigen::AngleAxisd(0.3, Vec3(1.0, 2.0, 3.0).normalized()).toRotationMatrix();
  T.p = Vec3(0.1, -0.2, 0.5);
  return T;
}

TEST(Pose, ComposeWithInverseIsIdentity) {
  const Pose T = make_pose();
  const Pose I = compose(T, inverse(T));
  EXPECT_LT((I.R - Mat3::Identity()).norm(), 1e-12);
  EXPECT_LT(I.p.norm(), 1e-12);
}

TEST(Pose, TransformMatchesRotationPlusTranslation) {
  const Pose T = make_pose();
  const Vec3 p_b(1.0, 0.0, 0.0);
  const Vec3 expected = T.R * p_b + T.p;
  EXPECT_LT((transform(T, p_b) - expected).norm(), 1e-15);
}

TEST(Pose, ComposeIsAssociativeWithTransform) {
  const Pose T_a_b = make_pose();
  Pose T_b_c;
  T_b_c.R = Eigen::AngleAxisd(-1.1, Vec3::UnitY()).toRotationMatrix();
  T_b_c.p = Vec3(0.0, 0.3, -0.2);
  const Vec3 p_c(0.4, -0.5, 0.6);
  const Vec3 lhs = transform(compose(T_a_b, T_b_c), p_c);
  const Vec3 rhs = transform(T_a_b, transform(T_b_c, p_c));
  EXPECT_LT((lhs - rhs).norm(), 1e-14);
}

TEST(Skew, EqualsCrossProduct) {
  const Vec3 a(1.0, 2.0, 3.0);
  const Vec3 b(-4.0, 5.0, 0.5);
  EXPECT_LT((skew(a) * b - a.cross(b)).norm(), 1e-15);
  EXPECT_LT((skew(a) + skew(a).transpose()).norm(), 1e-15);
}

TEST(Timestamp, SecondsBetween) {
  EXPECT_DOUBLE_EQ(seconds_between(Timestamp{0}, Timestamp{1'500'000'000}), 1.5);
  EXPECT_DOUBLE_EQ(seconds_between(Timestamp{2'000}, Timestamp{1'000}), -1e-6);
  EXPECT_TRUE(Timestamp{1} < Timestamp{2});
}

}  // namespace
}  // namespace cobot::common
