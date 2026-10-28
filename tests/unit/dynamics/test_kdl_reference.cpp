// Independent reference (ADR-004 check 6): tests/data/reference_dynamics_kdl.csv, written once
// by tools/gen_reference_dynamics.cpp with KDL (ChainIdSolver_RNE, ChainDynParam) on the chain
// base_link -> link_7 of sim/urdf/cobot30_5.urdf, gravity (0, 0, -9.80665). KDL merges the
// fixed joint joint_6_2_fix on its own, so a wrong merge in Model<7> shows up here.
// 100 rows, seed 3; columns q1..7, qd1..7, qdd1..7, tau1..7, G1..7, M11..M77 (row-major).
// tau and G exclude joint damping.
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/dynamics/dynamics.hpp"

#include "dynamics_test_util.hpp"

namespace cobot::dynamics {
namespace {

using test::all_close;
using test::Arm7;
using test::kArmDof;

constexpr int kN = kArmDof;
using Vec7 = VecN<kN>;
using Mat7 = MatN<kN>;

constexpr int kRows = 100;
constexpr int kCols = 5 * kN + kN * kN;  // 35 + 49 = 84
constexpr double kRel = 1e-6;
constexpr double kAbs = 1e-9;

struct RefRow {
  Vec7 q;
  Vec7 qd;
  Vec7 qdd;
  Vec7 tau;
  Vec7 G;
  Mat7 M;
};

std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> out;
  std::string cell;
  std::stringstream ss(line);
  while (std::getline(ss, cell, ',')) {
    out.push_back(cell);
  }
  return out;
}

std::string expected_header() {
  std::string h;
  for (const char* g : {"q", "qd", "qdd", "tau", "G"}) {
    for (int j = 1; j <= kN; ++j) {
      h += (h.empty() ? "" : ",") + std::string(g) + std::to_string(j);
    }
  }
  for (int r = 1; r <= kN; ++r) {
    for (int c = 1; c <= kN; ++c) {
      h += ",M" + std::to_string(r) + std::to_string(c);
    }
  }
  return h;
}

::testing::AssertionResult load_reference(std::vector<RefRow>& rows) {
  const std::string path = test::repo_path("tests/data/reference_dynamics_kdl.csv");
  std::ifstream in(path);
  if (!in) {
    return ::testing::AssertionFailure() << "cannot open " << path;
  }
  std::string line;
  std::getline(in, line);
  if (line != expected_header()) {
    return ::testing::AssertionFailure() << "unexpected header: " << line;
  }
  while (std::getline(in, line)) {
    if (line.empty()) {
      continue;
    }
    const std::vector<std::string> cells = split(line);
    if (static_cast<int>(cells.size()) != kCols) {
      return ::testing::AssertionFailure()
             << "row " << rows.size() << " has " << cells.size() << " cells";
    }
    std::vector<double> v(cells.size());
    for (std::size_t k = 0; k < cells.size(); ++k) {
      char* end = nullptr;
      v[k] = std::strtod(cells[k].c_str(), &end);
      if (end == cells[k].c_str() || *end != '\0') {
        return ::testing::AssertionFailure() << "bad number '" << cells[k] << "'";
      }
    }
    RefRow r;
    for (int j = 0; j < kN; ++j) {
      r.q[j] = v[j];
      r.qd[j] = v[kN + j];
      r.qdd[j] = v[2 * kN + j];
      r.tau[j] = v[3 * kN + j];
      r.G[j] = v[4 * kN + j];
    }
    for (int a = 0; a < kN; ++a) {
      for (int b = 0; b < kN; ++b) {
        r.M(a, b) = v[5 * kN + a * kN + b];
      }
    }
    rows.push_back(r);
  }
  return ::testing::AssertionSuccess();
}

class KdlReference : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(test::load_arm(arm_));
    ASSERT_TRUE(load_reference(rows_));
    ASSERT_EQ(static_cast<int>(rows_.size()), kRows);
  }
  Arm7 arm_;
  std::vector<RefRow> rows_;
};

TEST_F(KdlReference, FixtureStatesInsideSampledRanges) {
  for (const RefRow& r : rows_) {
    for (int j = 0; j < kN; ++j) {
      EXPECT_GE(r.q[j], arm_.joints[j].q_min);
      EXPECT_LE(r.q[j], arm_.joints[j].q_max);
      EXPECT_LE(std::abs(r.qd[j]), 1.0);
      EXPECT_LE(std::abs(r.qdd[j]), 2.0);
    }
  }
}

TEST_F(KdlReference, InverseDynamicsMatchesKdl) {
  for (std::size_t k = 0; k < rows_.size(); ++k) {
    const RefRow& r = rows_[k];
    const Vec7 tau = rnea(arm_, r.q, r.qd, r.qdd, arm_.gravity_b);
    EXPECT_TRUE(all_close(tau, r.tau, kRel, kAbs)) << "row " << k;
  }
}

TEST_F(KdlReference, GravityTorqueMatchesKdl) {
  for (std::size_t k = 0; k < rows_.size(); ++k) {
    const RefRow& r = rows_[k];
    EXPECT_TRUE(all_close(gravity_torque(arm_, r.q), r.G, kRel, kAbs)) << "row " << k;
  }
}

TEST_F(KdlReference, MassMatrixMatchesKdl) {
  for (std::size_t k = 0; k < rows_.size(); ++k) {
    const RefRow& r = rows_[k];
    EXPECT_TRUE(all_close(mass_matrix(arm_, r.q), r.M, kRel, kAbs)) << "row " << k;
  }
}

}  // namespace
}  // namespace cobot::dynamics
