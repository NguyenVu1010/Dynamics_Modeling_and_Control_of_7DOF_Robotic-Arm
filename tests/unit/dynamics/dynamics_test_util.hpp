#pragma once

#include <algorithm>
#include <cmath>
#include <random>
#include <string>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/model/model.hpp"

// Shared helpers for the dynamics tests: the real arm model from the committed URDF, random
// states inside the joint limits, and element-wise closeness checks.
namespace cobot::dynamics::test {

using common::VecN;
using model::Arm7;
using model::kArmDof;
using model::Model;

inline const common::Vec3 kGravityB(0.0, 0.0, -9.80665);  // m/s^2, docs/conventions.md

inline std::string repo_path(const std::string& rel) {
  return std::string(COBOT_REPO_DIR) + "/" + rel;
}

// Arm7 from sim/urdf/cobot30_5.urdf, chain base_link -> link_7, gravity (0, 0, -9.80665).
inline ::testing::AssertionResult load_arm(Arm7& out) {
  const common::Result<model::UrdfChain> chain =
      model::parse_urdf_chain(repo_path("src/sim/urdf/cobot30_5.urdf"), "base_link", "link_7");
  if (!chain.is_ok()) {
    return ::testing::AssertionFailure() << "parse_urdf_chain: " << chain.reason();
  }
  const common::Result<Arm7> m = model::make_model<kArmDof>(chain.value(), kGravityB);
  if (!m.is_ok()) {
    return ::testing::AssertionFailure() << "make_model: " << m.reason();
  }
  out = m.value();
  return ::testing::AssertionSuccess();
}

template <int N>
VecN<N> random_q(const Model<N>& model, std::mt19937& rng) {
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  VecN<N> q;
  for (int j = 0; j < N; ++j) {
    const model::JointParams& p = model.joints[j];
    q[j] = p.q_min + unit(rng) * (p.q_max - p.q_min);
  }
  return q;
}

template <int N>
VecN<N> random_sym(double bound, std::mt19937& rng) {
  std::uniform_real_distribution<double> u(-bound, bound);
  VecN<N> v;
  for (int j = 0; j < N; ++j) {
    v[j] = u(rng);
  }
  return v;
}

// |a_k - b_k| <= abs_tol + rel_tol * |b_k| for every element (b is the reference).
template <typename A, typename B>
::testing::AssertionResult all_close(const Eigen::MatrixBase<A>& a,
                                     const Eigen::MatrixBase<B>& b,
                                     double rel_tol,
                                     double abs_tol) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) {
    return ::testing::AssertionFailure() << "size mismatch";
  }
  for (Eigen::Index r = 0; r < a.rows(); ++r) {
    for (Eigen::Index c = 0; c < a.cols(); ++c) {
      const double err = std::abs(a(r, c) - b(r, c));
      if (!(err <= abs_tol + rel_tol * std::abs(b(r, c)))) {
        return ::testing::AssertionFailure()
               << "element (" << r << ", " << c << "): got " << a(r, c) << ", expected " << b(r, c)
               << ", error " << err;
      }
    }
  }
  return ::testing::AssertionSuccess();
}

// ||a - b|| <= rel_tol * max(||b||, 1): relative check with a floor for near-zero vectors.
template <typename A, typename B>
::testing::AssertionResult norm_close(const Eigen::MatrixBase<A>& a,
                                      const Eigen::MatrixBase<B>& b,
                                      double rel_tol) {
  const double err = (a - b).norm();
  const double scale = std::max(b.norm(), 1.0);
  if (err <= rel_tol * scale) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "||a - b|| = " << err << " > " << rel_tol << " * " << scale
         << "\n a = " << a.transpose() << "\n b = " << b.transpose();
}

}  // namespace cobot::dynamics::test
