#include <cmath>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "cobot/common/lie.hpp"
#include "cobot/common/types.hpp"

namespace cobot::common {
namespace {

constexpr double kPi = 3.14159265358979323846;

Vec3 random_axis(std::mt19937_64& rng) {
  std::normal_distribution<double> n(0.0, 1.0);
  Vec3 a(n(rng), n(rng), n(rng));
  while (a.norm() < 1e-6) {
    a = Vec3(n(rng), n(rng), n(rng));
  }
  return a.normalized();
}

// 10 000 rotation vectors with angle uniform in [0, pi - 1e-6] plus 100 with angle < 1e-10.
std::vector<Vec3> rotation_samples() {
  std::mt19937_64 rng(42);
  std::uniform_real_distribution<double> angle(0.0, kPi - 1e-6);
  std::uniform_real_distribution<double> tiny(0.0, 1e-10);
  std::vector<Vec3> out;
  out.reserve(10100);
  for (int i = 0; i < 10000; ++i) {
    out.emplace_back(random_axis(rng) * angle(rng));
  }
  for (int i = 0; i < 100; ++i) {
    out.emplace_back(random_axis(rng) * tiny(rng));
  }
  return out;
}

Mat3 rz(double a) {
  return Eigen::AngleAxisd(a, Vec3::UnitZ()).toRotationMatrix();
}

TEST(So3, ExpLogRoundTrip) {
  for (const Vec3& phi : rotation_samples()) {
    const Vec3 back = so3_log(so3_exp(phi));
    ASSERT_LT((back - phi).norm(), 1e-9) << "phi = " << phi.transpose();
  }
}

TEST(So3, ExpIsRotation) {
  for (const Vec3& phi : rotation_samples()) {
    const Mat3 R = so3_exp(phi);
    ASSERT_LT((R.transpose() * R - Mat3::Identity()).cwiseAbs().maxCoeff(), 1e-12)
        << "phi = " << phi.transpose();
    ASSERT_NEAR(R.determinant(), 1.0, 1e-12) << "phi = " << phi.transpose();
  }
}

TEST(So3, ExpMatchesEigenAngleAxis) {
  for (const Vec3& phi : rotation_samples()) {
    const double a = phi.norm();
    const Mat3 ref =
        a > 0.0 ? Mat3(Eigen::AngleAxisd(a, phi / a).toRotationMatrix()) : Mat3(Mat3::Identity());
    ASSERT_LT((so3_exp(phi) - ref).cwiseAbs().maxCoeff(), 1e-12) << "phi = " << phi.transpose();
  }
}

TEST(So3, ExpLeavesAxisInvariant) {
  std::mt19937_64 rng(42);
  for (int i = 0; i < 100; ++i) {
    const Vec3 phi = random_axis(rng) * 1.7;
    EXPECT_LT((so3_exp(phi) * phi - phi).norm(), 1e-12);
  }
}

TEST(So3, LogOfTransposeIsNegated) {
  std::mt19937_64 rng(7);
  std::uniform_real_distribution<double> angle(0.0, kPi - 1e-3);
  for (int i = 0; i < 1000; ++i) {
    const Mat3 R = so3_exp(random_axis(rng) * angle(rng));
    EXPECT_LT((so3_log(R.transpose()) + so3_log(R)).norm(), 1e-10);
  }
}

TEST(So3, LogNearPi) {
  std::mt19937_64 rng(42);
  const double angles[] = {kPi - 1e-6, kPi - 1e-9, kPi};
  for (const double a : angles) {
    for (int i = 0; i < 200; ++i) {
      const Vec3 axis = random_axis(rng);
      const Mat3 R = Eigen::AngleAxisd(a, axis).toRotationMatrix();
      const Vec3 phi = so3_log(R);
      EXPECT_LT((so3_exp(phi) - R).cwiseAbs().maxCoeff(), 1e-8)
          << "angle = " << a << " axis = " << axis.transpose();
      EXPECT_LE(phi.norm(), kPi + 1e-12) << "angle = " << a << " axis = " << axis.transpose();
      // The recovered axis is +-axis (sign is free only at exactly pi).
      EXPECT_GT(std::abs(phi.normalized().dot(axis)), 1.0 - 1e-8);
    }
  }
}

TEST(So3, LogAtPiPicksNonNegativeLargestComponent) {
  std::mt19937_64 rng(3);
  for (int i = 0; i < 200; ++i) {
    const Vec3 axis = random_axis(rng);
    const Mat3 R = Eigen::AngleAxisd(kPi, axis).toRotationMatrix();
    const Vec3 phi = so3_log(R);
    Eigen::Index k = 0;
    phi.cwiseAbs().maxCoeff(&k);
    EXPECT_GE(phi(k), 0.0) << "axis = " << axis.transpose();
    EXPECT_NEAR(phi.norm(), kPi, 1e-8);
  }
  // Axis-aligned half turns: R = diag(1,-1,-1) etc.
  for (int k = 0; k < 3; ++k) {
    const Vec3 axis = Vec3::Unit(k);
    const Mat3 R = 2.0 * axis * axis.transpose() - Mat3::Identity();
    const Vec3 phi = so3_log(R);
    EXPECT_LT((phi - kPi * axis).norm(), 1e-12) << "k = " << k;
  }
}

TEST(So3, SmallAngleSeries) {
  std::mt19937_64 rng(42);
  for (int i = 0; i < 100; ++i) {
    const Vec3 phi = random_axis(rng) * 1e-12;
    const Mat3 R = so3_exp(phi);
    const Mat3 expected = Mat3::Identity() + skew(phi);
    EXPECT_LT((R - expected).cwiseAbs().maxCoeff(), 1e-20) << "phi = " << phi.transpose();
  }
  const Vec3 zero = so3_log(Mat3::Identity());
  EXPECT_EQ(zero.x(), 0.0);
  EXPECT_EQ(zero.y(), 0.0);
  EXPECT_EQ(zero.z(), 0.0);
  EXPECT_EQ(so3_exp(Vec3::Zero()), Mat3::Identity());
}

TEST(So3, KnownValues) {
  const Mat3 R = so3_exp(Vec3(0.0, 0.0, kPi / 2.0));
  EXPECT_LT((R * Vec3::UnitX() - Vec3::UnitY()).cwiseAbs().maxCoeff(), 1e-15);
  const Vec3 phi = so3_log(rz(0.3));
  EXPECT_LT((phi - Vec3(0.0, 0.0, 0.3)).cwiseAbs().maxCoeff(), 1e-15);
}

TEST(So3, ResultsAreFinite) {
  for (const Vec3& phi : rotation_samples()) {
    ASSERT_TRUE(so3_exp(phi).allFinite());
    ASSERT_TRUE(so3_log(so3_exp(phi)).allFinite());
  }
  ASSERT_TRUE(so3_log(Eigen::AngleAxisd(kPi, Vec3::UnitX()).toRotationMatrix()).allFinite());
}

Pose sample_pose() {
  Pose T;
  T.R = Eigen::AngleAxisd(0.7, Vec3(0.2, -1.0, 0.5).normalized()).toRotationMatrix();
  T.p = Vec3(0.4, -0.1, 0.6);
  return T;
}

TEST(PoseError, ZeroForEqualPoses) {
  const Pose T = sample_pose();
  const Vec6 e = pose_error(T, T);
  EXPECT_LT(e.cwiseAbs().maxCoeff(), 1e-12);
  const Vec6 e0 = pose_error(Pose{}, Pose{});
  EXPECT_EQ(e0, Vec6::Zero());
}

TEST(PoseError, PositionPart) {
  const Pose T_d = sample_pose();
  Pose T = T_d;
  T.p = Vec3(-0.3, 0.25, 1.1);
  const Vec6 e = pose_error(T_d, T);
  EXPECT_LT((e.head<3>() - (T_d.p - T.p)).cwiseAbs().maxCoeff(), 1e-15);
  EXPECT_LT(e.tail<3>().cwiseAbs().maxCoeff(), 1e-12);
}

TEST(PoseError, OrientationPart) {
  std::mt19937_64 rng(42);
  std::uniform_real_distribution<double> angle(0.0, 0.999);
  const Pose T_d = sample_pose();
  for (int i = 0; i < 1000; ++i) {
    const Vec3 delta = random_axis(rng) * angle(rng);
    Pose T = T_d;
    T.R = so3_exp(-delta) * T_d.R;
    const Vec6 e = pose_error(T_d, T);
    ASSERT_LT((e.tail<3>() - delta).norm(), 1e-12) << "delta = " << delta.transpose();
    ASSERT_LT(e.head<3>().cwiseAbs().maxCoeff(), 1e-15);
  }
}

TEST(PoseError, MatchesDefinition) {
  const Pose T_d = sample_pose();
  Pose T;
  T.R = Eigen::AngleAxisd(-0.4, Vec3(1.0, 1.0, 0.0).normalized()).toRotationMatrix();
  T.p = Vec3(0.1, 0.2, 0.3);
  const Vec6 e = pose_error(T_d, T);
  Vec6 expected;
  expected << T_d.p - T.p, so3_log(T_d.R * T.R.transpose());
  EXPECT_LT((e - expected).cwiseAbs().maxCoeff(), 1e-15);
}

TEST(Lie, DoesNotAllocate) {
  const Vec3 phi(0.1, -0.2, 0.3);
  const Mat3 R_pi = Eigen::AngleAxisd(kPi, Vec3(1.0, 2.0, 2.0).normalized()).toRotationMatrix();
  const Pose T_d = sample_pose();
  Pose T = T_d;
  T.p += Vec3(0.01, 0.0, 0.0);
  Mat3 R;
  Vec3 back;
  Vec3 back_pi;
  Vec3 back_small;
  Vec6 e;
  Eigen::internal::set_is_malloc_allowed(false);
  R = so3_exp(phi);
  back = so3_log(R);
  back_pi = so3_log(R_pi);
  back_small = so3_log(so3_exp(Vec3(1e-11, 0.0, 0.0)));
  e = pose_error(T_d, T);
  Eigen::internal::set_is_malloc_allowed(true);
  EXPECT_LT((back - phi).norm(), 1e-9);
  EXPECT_NEAR(back_pi.norm(), kPi, 1e-8);
  EXPECT_LT((back_small - Vec3(1e-11, 0.0, 0.0)).norm(), 1e-9);
  EXPECT_TRUE(e.allFinite());
}

}  // namespace
}  // namespace cobot::common
