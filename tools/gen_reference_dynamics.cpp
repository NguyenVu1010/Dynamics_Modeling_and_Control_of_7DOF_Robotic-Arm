// Writes the KDL reference fixture for the dynamics tests (tests/data/reference_dynamics_kdl.csv).
// Built only with -DCOBOT_WITH_KDL_REF=ON; run once, and the CSV is committed. The core never
// links KDL (ADR-004).
//
// Usage: cobot_gen_reference_dynamics <urdf> <out.csv>
//
// 100 rows, std::mt19937 seed 3. Per row, in this draw order: q_j uniform in
// [lower_j, upper_j] (URDF limits), qd_j uniform in [-1, 1], qdd_j uniform in [-2, 2].
// Columns: q1..7, qd1..7, qdd1..7, tau1..7, G1..7, M11..M77 (row-major), printed with %.17g.
// tau = ChainIdSolver_RNE (no external wrench), G = ChainDynParam::JntToGravity,
// M = ChainDynParam::JntToMass, chain base_link -> link_7, gravity (0, 0, -9.80665).
// Joint damping is not part of any column.
//
// The KDL part is guarded by __has_include so that clang-tidy in the lint job, which has no
// ROS packages and no compile command for this file, still parses it.
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#if __has_include(<kdl/chain.hpp>) && __has_include(<kdl_parser/kdl_parser.hpp>)
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

namespace {

constexpr int kDof = 7;
constexpr int kRows = 100;
constexpr unsigned kSeed = 3;
constexpr double kGravityZ = -9.80665;  // m/s^2 in base_link
constexpr double kQdMax = 1.0;          // rad/s
constexpr double kQddMax = 2.0;         // rad/s^2

void print_row(std::FILE* f, const std::vector<double>& row) {
  for (std::size_t k = 0; k < row.size(); ++k) {
    std::fprintf(f, k == 0 ? "%.17g" : ",%.17g", row[k]);
  }
  std::fprintf(f, "\n");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: %s <urdf> <out.csv>\n", argv[0]);
    return 2;
  }
  urdf::Model urdf_model;
  if (!urdf_model.initFile(argv[1])) {
    std::fprintf(stderr, "cannot parse %s\n", argv[1]);
    return 1;
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
    std::fprintf(stderr, "kdl_parser failed\n");
    return 1;
  }
  KDL::Chain chain;
  if (!tree.getChain("base_link", "link_7", chain) ||
      static_cast<int>(chain.getNrOfJoints()) != kDof) {
    std::fprintf(stderr, "chain base_link -> link_7 must have %d joints\n", kDof);
    return 1;
  }

  std::vector<double> lower;
  std::vector<double> upper;
  for (unsigned s = 0; s < chain.getNrOfSegments(); ++s) {
    const KDL::Joint& joint = chain.getSegment(s).getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;
    }
    const urdf::JointConstSharedPtr uj = urdf_model.getJoint(joint.getName());
    if (!uj || !uj->limits) {
      std::fprintf(stderr, "joint %s has no limits\n", joint.getName().c_str());
      return 1;
    }
    lower.push_back(uj->limits->lower);
    upper.push_back(uj->limits->upper);
  }

  const KDL::Vector gravity(0.0, 0.0, kGravityZ);
  KDL::ChainIdSolver_RNE id_solver(chain, gravity);
  KDL::ChainDynParam dyn_param(chain, gravity);

  std::FILE* out = std::fopen(argv[2], "w");
  if (out == nullptr) {
    std::fprintf(stderr, "cannot write %s\n", argv[2]);
    return 1;
  }
  std::string header;
  const char* groups[] = {"q", "qd", "qdd", "tau", "G"};
  for (const char* g : groups) {
    for (int j = 1; j <= kDof; ++j) {
      header += (header.empty() ? "" : ",") + std::string(g) + std::to_string(j);
    }
  }
  for (int r = 1; r <= kDof; ++r) {
    for (int c = 1; c <= kDof; ++c) {
      header += ",M" + std::to_string(r) + std::to_string(c);
    }
  }
  std::fprintf(out, "%s\n", header.c_str());

  std::mt19937 rng(kSeed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> vel(-kQdMax, kQdMax);
  std::uniform_real_distribution<double> acc(-kQddMax, kQddMax);
  KDL::JntArray q(kDof);
  KDL::JntArray qd(kDof);
  KDL::JntArray qdd(kDof);
  KDL::JntArray tau(kDof);
  KDL::JntArray grav(kDof);
  KDL::JntSpaceInertiaMatrix mass(kDof);
  const KDL::Wrenches f_ext(chain.getNrOfSegments(), KDL::Wrench::Zero());
  for (int row = 0; row < kRows; ++row) {
    for (int j = 0; j < kDof; ++j) {
      q(j) = lower[j] + unit(rng) * (upper[j] - lower[j]);
    }
    for (int j = 0; j < kDof; ++j) {
      qd(j) = vel(rng);
    }
    for (int j = 0; j < kDof; ++j) {
      qdd(j) = acc(rng);
    }
    if (id_solver.CartToJnt(q, qd, qdd, f_ext, tau) < 0 || dyn_param.JntToGravity(q, grav) < 0 ||
        dyn_param.JntToMass(q, mass) < 0) {
      std::fprintf(stderr, "KDL solver failed at row %d\n", row);
      std::fclose(out);
      return 1;
    }
    std::vector<double> values;
    for (const KDL::JntArray* a : {&q, &qd, &qdd, &tau, &grav}) {
      for (int j = 0; j < kDof; ++j) {
        values.push_back((*a)(j));
      }
    }
    for (int r = 0; r < kDof; ++r) {
      for (int c = 0; c < kDof; ++c) {
        values.push_back(mass(r, c));
      }
    }
    print_row(out, values);
  }
  std::fclose(out);
  std::printf("wrote %d rows to %s\n", kRows, argv[2]);
  return 0;
}

#else

int main() {
  std::fprintf(stderr, "built without KDL: configure with -DCOBOT_WITH_KDL_REF=ON\n");
  return 2;
}

#endif
