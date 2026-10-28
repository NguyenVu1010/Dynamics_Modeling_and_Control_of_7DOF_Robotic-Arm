#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <new>

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <gtest/gtest.h>

#include "cobot/common/lie.hpp"
#include "cobot/kinematics/kinematics.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/planning/cartesian_trajectory.hpp"
#include "cobot/planning/clik.hpp"

// Allocation counter for NoHeapAllocationInStep. EIGEN_RUNTIME_NO_MALLOC only checks Eigen code
// compiled in this file, so operator new and (outside ASan, which owns malloc) malloc are
// counted as well; that also covers allocations made inside the cobot libraries.
namespace {
bool g_count_allocs = false;
std::size_t g_allocs = 0;
}  // namespace

void* operator new(std::size_t n) {
  if (g_count_allocs) {
    ++g_allocs;
  }
  void* p = std::malloc(n == 0 ? 1 : n);  // NOLINT(cppcoreguidelines-no-malloc)
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}
void operator delete(void* p) noexcept {
  std::free(p);  // NOLINT(cppcoreguidelines-no-malloc)
}
void operator delete(void* p, std::size_t /*n*/) noexcept {
  std::free(p);  // NOLINT(cppcoreguidelines-no-malloc)
}

#if !defined(__SANITIZE_ADDRESS__) && defined(__GLIBC__)
// NOLINTNEXTLINE(bugprone-reserved-identifier,readability-identifier-naming)
extern "C" void* __libc_malloc(std::size_t n);
extern "C" void* malloc(std::size_t n) noexcept {  // NOLINT(bugprone-reserved-identifier)
  if (g_count_allocs) {
    ++g_allocs;
  }
  return __libc_malloc(n);
}
#endif

namespace cobot::planning {
namespace {

namespace fs = std::filesystem;
using common::Mat6N;
using common::Pose;
using common::Vec3;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 1e-3;  // s, 1 kHz

Model<7> load_arm(pipeline::Config<7>* cfg_out = nullptr) {
  const auto cfg = pipeline::load_config<7>(fs::path(COBOT_REPO_DIR) / "config/default.yaml");
  EXPECT_TRUE(cfg.is_ok()) << cfg.reason();
  const auto chain =
      model::parse_urdf_chain(cfg.value().urdf_path, cfg.value().base_link, cfg.value().tip_link);
  EXPECT_TRUE(chain.is_ok()) << chain.reason();
  const auto m = model::make_model<7>(chain.value(), cfg.value().gravity_b);
  EXPECT_TRUE(m.is_ok()) << m.reason();
  if (cfg_out != nullptr) {
    *cfg_out = cfg.value();
  }
  return m.value();
}

VecN<7> home_q() {
  // config/default.yaml home_q_deg (0, 30, 0, -90, 0, 60, 0)
  return (VecN<7>() << 0.0, 30.0, 0.0, -90.0, 0.0, 60.0, 0.0).finished() * (kPi / 180.0);
}

template <int N>
Pose fk(const Model<N>& m, const VecN<N>& q) {
  kinematics::FkState<N> s;
  kinematics::forward_kinematics(m, q, s);
  return s.T_b_e;
}

template <int N>
double cond_j(const Model<N>& m, const VecN<N>& q) {
  kinematics::FkState<N> s;
  kinematics::forward_kinematics(m, q, s);
  Mat6N<N> J;
  kinematics::geometric_jacobian(s, J);
  const Eigen::JacobiSVD<Mat6N<N>> svd(J);
  const Eigen::VectorXd sv = svd.singularValues();
  return sv(0) / sv(sv.size() - 1);
}

CartesianSample static_sample(const Pose& T_b_d) {
  CartesianSample x;
  x.T_b_d = T_b_d;
  return x;
}

template <int N>
void expect_inside_limits(const Model<N>& m, const JointReference<N>& r) {
  for (int i = 0; i < N; ++i) {
    ASSERT_GE(r.q(i), m.joints[i].q_min) << "joint " << i;
    ASSERT_LE(r.q(i), m.joints[i].q_max) << "joint " << i;
  }
}

// Compares the bit patterns (not the values), so -0.0 vs 0.0 or two NaNs are told apart.
template <int R>
bool bit_identical(const VecN<R>& a, const VecN<R>& b) {
  for (int i = 0; i < R; ++i) {
    std::uint64_t x = 0;
    std::uint64_t y = 0;
    std::memcpy(&x, &a(i), sizeof(x));
    std::memcpy(&y, &b(i), sizeof(y));
    if (x != y) {
      return false;
    }
  }
  return true;
}

// Planar arm: z axes, link lengths l1 = 1.0, l2 = 0.7 along x (as in the kinematics tests).
Model<2> planar(double q2_min = -kPi, double q2_max = kPi) {
  Model<2> m;
  for (int i = 0; i < 2; ++i) {
    m.joints[i].name = i == 0 ? "joint_1" : "joint_2";
    m.joints[i].q_min = -kPi;
    m.joints[i].q_max = kPi;
    m.joints[i].qd_max = 10.0;
    m.joints[i].tau_max = 10.0;
    m.links[i].mass_kg = 1.0;
  }
  m.joints[1].T_parent_joint.p = Vec3(1.0, 0.0, 0.0);
  m.joints[1].q_min = q2_min;
  m.joints[1].q_max = q2_max;
  m.T_last_tip.p = Vec3(0.7, 0.0, 0.0);
  return m;
}

// Target = pose of the planar arm at q* = (0.5, 0.8), so the whole 6D task is reachable:
//   p = (cos 0.5 + 0.7 cos 1.3, sin 0.5 + 0.7 sin 1.3, 0)
//     = (0.8775826 + 0.7 * 0.2674988, 0.4794255 + 0.7 * 0.9635582, 0) = (1.0648317, 1.1539163, 0)
//   R = Rz(0.5 + 0.8) = Rz(1.3)
Pose planar_target() {
  Pose T;
  T.p = Vec3(std::cos(0.5) + 0.7 * std::cos(1.3), std::sin(0.5) + 0.7 * std::sin(1.3), 0.0);
  T.R = Eigen::AngleAxisd(1.3, Vec3::UnitZ()).toRotationMatrix();
  return T;
}

TEST(Clik, PlanarTwoLinkConvergesToFixedTarget) {
  // Start q = (0.2, 0.4): |p - p*| is about 0.6 m. kp = 100, kd = 20 is critically damped
  // (wn = 10 rad/s), e(3 s) ~ (1 + 30) e^-30 * 0.6 m = 1.7e-12 m in the linear regime.
  const Model<2> m = planar();
  ClikReference<2> clik(m, ClikConfig{});
  clik.reset(VecN<2>(0.2, 0.4));
  const CartesianSample x = static_sample(planar_target());
  for (int k = 0; k < 3000; ++k) {  // 3 s at 1 kHz
    clik.step(x, kDt);
  }
  const VecN<2> q = clik.reference().q;
  // Analytic FK of the planar arm, independent of the kinematics module.
  const Vec3 p(std::cos(q(0)) + 0.7 * std::cos(q(0) + q(1)),
               std::sin(q(0)) + 0.7 * std::sin(q(0) + q(1)),
               0.0);
  EXPECT_LT((p - x.T_b_d.p).norm(), 1e-6);
  EXPECT_LT(clik.reference().task_error.head<3>().norm(), 1e-6);
}

TEST(Clik, StepReturnsTheStoredReference) {
  ClikReference<2> clik(planar(), ClikConfig{});
  clik.reset(VecN<2>(0.2, 0.4));
  const JointReference<2>& r = clik.step(static_sample(planar_target()), kDt);
  EXPECT_EQ(&r, &clik.reference());
  EXPECT_GT(r.task_error.head<3>().norm(), 0.1);  // error at q = (0.2, 0.4), about 0.6 m
}

TEST(Clik, JointLimitSaturationStopsVelocityIntoTheLimit) {
  // q2 limited to [-0.5, 0.5]; the target needs q2 = 0.8, so q2 runs into its upper limit.
  const Model<2> m = planar(-0.5, 0.5);
  ClikReference<2> clik(m, ClikConfig{});
  clik.reset(VecN<2>(0.2, 0.4));
  const CartesianSample x = static_sample(planar_target());
  int at_limit = 0;
  for (int k = 0; k < 3000; ++k) {
    const JointReference<2>& r = clik.step(x, kDt);
    expect_inside_limits(m, r);
    ASSERT_TRUE(r.qd.allFinite() && r.qdd.allFinite());
    if (r.q(1) == 0.5) {
      ++at_limit;
      ASSERT_LE(r.qd(1), 0.0) << "step " << k;
    }
  }
  EXPECT_GT(at_limit, 0);
}

TEST(Clik, ResetClampsOutOfLimitConfiguration) {
  const Model<2> m = planar(-0.5, 0.5);
  ClikReference<2> clik(m, ClikConfig{});
  clik.reset(VecN<2>(0.2, 0.4));
  for (int k = 0; k < 10; ++k) {
    clik.step(static_sample(planar_target()), kDt);
  }
  clik.reset(VecN<2>(4.0, -0.9));  // above pi on joint 1, below -0.5 on joint 2
  const JointReference<2>& r = clik.reference();
  EXPECT_EQ(r.q(0), kPi);
  EXPECT_EQ(r.q(1), -0.5);
  EXPECT_TRUE(r.qd.isZero(0.0));
  EXPECT_TRUE(r.qdd.isZero(0.0));
  EXPECT_TRUE(r.task_error.isZero(0.0));
}

TEST(Clik, FollowsMvpTrajectoryFromHome) {
  // A reference run of the design law (same gains) gives max |e_p| 5e-6 m, max |e_w| 6e-4 deg,
  // max |qd|/qd_max 0.07 during tracking and |e| 1e-11 at 12 s.
  pipeline::Config<7> cfg;
  const Model<7> m = load_arm(&cfg);
  const VecN<7> q0 = cfg.home_q_rad;
  Pose goal;
  goal.p = cfg.trajectory.goal_position_m;
  const Vec3& rpy = cfg.trajectory.goal_rpy_rad;  // R = Rz(yaw) Ry(pitch) Rx(roll)
  goal.R = (Eigen::AngleAxisd(rpy(2), Vec3::UnitZ()) * Eigen::AngleAxisd(rpy(1), Vec3::UnitY()) *
            Eigen::AngleAxisd(rpy(0), Vec3::UnitX()))
               .toRotationMatrix();
  ASSERT_DOUBLE_EQ(cfg.trajectory.duration_s, 10.0);
  const CartesianTrajectory traj(fk(m, q0), goal, cfg.trajectory.duration_s);
  ClikReference<7> clik(m, cfg.clik);
  clik.reset(q0);
  const double max_orient = 0.5 * kPi / 180.0;  // 0.5 deg = 8.727e-3 rad
  for (int k = 0; k <= 12000; ++k) {            // t = 0 .. 12 s at 1 kHz
    const JointReference<7>& r = clik.step(traj.sample(k * kDt), kDt);
    ASSERT_LT(r.task_error.head<3>().norm(), 1e-3) << "t = " << k * kDt;
    ASSERT_LT(r.task_error.tail<3>().norm(), max_orient) << "t = " << k * kDt;
    expect_inside_limits(m, r);
    for (int i = 0; i < 7; ++i) {
      ASSERT_LE(std::abs(r.qd(i)), m.joints[i].qd_max) << "joint " << i << " t = " << k * kDt;
    }
  }
  const JointReference<7>& r = clik.reference();  // t = 12 s
  EXPECT_LT(r.task_error.head<3>().norm(), 1e-5);
  EXPECT_LT(r.task_error.tail<3>().norm(), 1e-4);
  const Vec6 e = common::pose_error(goal, fk(m, r.q));
  EXPECT_LT(e.head<3>().norm(), 1e-5);
  EXPECT_LT(e.tail<3>().norm(), 1e-4);
}

TEST(Clik, StaticTargetWithoutNullSpaceGainHoldsResetConfiguration) {
  const Model<7> m = load_arm();
  ClikConfig cfg;
  cfg.k_null = 0.0;  // nothing but the task drives q_ref
  ClikReference<7> clik(m, cfg);
  const VecN<7> q0 = home_q();
  clik.reset(q0);
  const CartesianSample x = static_sample(fk(m, q0));
  for (int k = 0; k < 2000; ++k) {  // 2 s
    const JointReference<7>& r = clik.step(x, kDt);
    ASSERT_LT((r.q - q0).cwiseAbs().maxCoeff(), 1e-9) << "step " << k;
    ASSERT_LT(r.task_error.norm(), 1e-9) << "step " << k;
  }
}

TEST(Clik, StaticTargetNullSpaceMotionKeepsTaskError) {
  // Home is not mid-range, so with default gains q_ref moves (about 3e-4 rad in 2 s in a
  // reference run), but only in the null space: task error stays about 1e-12.
  const Model<7> m = load_arm();
  ClikReference<7> clik(m, ClikConfig{});
  const VecN<7> q0 = home_q();
  clik.reset(q0);
  const CartesianSample x = static_sample(fk(m, q0));
  for (int k = 0; k < 2000; ++k) {
    const JointReference<7>& r = clik.step(x, kDt);
    ASSERT_LT(r.task_error.head<3>().norm(), 1e-9) << "step " << k;
    ASSERT_LT(r.task_error.tail<3>().norm(), 1e-9) << "step " << k;
  }
  const Vec6 e = common::pose_error(x.T_b_d, fk(m, clik.reference().q));
  EXPECT_LT(e.norm(), 1e-9);
}

TEST(Clik, NullSpaceMovesJointAwayFromLimit) {
  const Model<7> m = load_arm();
  VecN<7> q0 = home_q();
  q0(0) = 2.8;  // joint_1 limit 2.9671 rad: 0.167 rad (9.6 deg) margin
  const double mid = 0.5 * (m.joints[0].q_min + m.joints[0].q_max);  // 0
  ClikReference<7> clik(m, ClikConfig{});
  clik.reset(q0);
  const CartesianSample x = static_sample(fk(m, q0));
  double prev = std::abs(q0(0) - mid);
  for (int k = 0; k < 5000; ++k) {  // 5 s
    const JointReference<7>& r = clik.step(x, kDt);
    ASSERT_LT(r.task_error.norm(), 1e-6) << "step " << k;
    const double d = std::abs(r.q(0) - mid);
    ASSERT_LE(d, prev + 1e-12) << "step " << k;
    prev = d;
  }
  // A reference run of the design law gives q1 2.8 -> 2.7932 rad after 5 s (6.8e-3 rad).
  EXPECT_LT(prev, std::abs(q0(0) - mid) - 1e-3);
}

TEST(Clik, NearSingularityStaysFiniteAndBounded) {
  const Model<7> m = load_arm();
  VecN<7> q0 = home_q();
  q0(3) = m.joints[3].q_min + 1e-4;  // elbow folded (joint_4 -> -pi): cond(J) about 5e4
  ASSERT_GT(cond_j(m, q0), 1e4);
  const Pose T_b_s = fk(m, q0);
  const Vec6 offsets[] = {(Vec6() << 0.01, 0.01, 0.01, 0.0, 0.0, 0.0).finished(),
                          (Vec6() << 0.02, 0.0, 0.0, 0.0, 0.0, 0.0).finished(),
                          (Vec6() << 0.05, -0.05, 0.05, 0.1, 0.0, 0.0).finished()};
  for (const Vec6& off : offsets) {
    Pose T_b_d = T_b_s;
    T_b_d.p += off.head<3>();
    T_b_d.R = common::so3_exp(off.tail<3>()) * T_b_s.R;
    ClikReference<7> clik(m, ClikConfig{});
    clik.reset(q0);
    for (int k = 0; k < 2000; ++k) {
      const JointReference<7>& r = clik.step(static_sample(T_b_d), kDt);
      ASSERT_TRUE(r.q.allFinite() && r.qd.allFinite() && r.qdd.allFinite());
      ASSERT_TRUE(r.task_error.allFinite());
      ASSERT_LE(r.qdd.cwiseAbs().maxCoeff(), 1e3) << "step " << k;
    }
  }
}

TEST(Clik, Deterministic) {
  const Model<7> m = load_arm();
  const VecN<7> q0 = home_q();
  Pose goal = fk(m, q0);
  goal.p += Vec3(-0.2, 0.1, 0.1);
  const CartesianTrajectory traj(fk(m, q0), goal, 1.0);
  ClikReference<7> a(m, ClikConfig{});
  ClikReference<7> b(m, ClikConfig{});
  a.reset(q0);
  b.reset(q0);
  for (int k = 0; k < 1500; ++k) {
    const JointReference<7>& ra = a.step(traj.sample(k * kDt), kDt);
    const JointReference<7>& rb = b.step(traj.sample(k * kDt), kDt);
    ASSERT_TRUE(bit_identical(ra.q, rb.q)) << k;
    ASSERT_TRUE(bit_identical(ra.qd, rb.qd)) << k;
    ASSERT_TRUE(bit_identical(ra.qdd, rb.qdd)) << k;
    ASSERT_TRUE(bit_identical(ra.task_error, rb.task_error)) << k;
  }
}

TEST(Clik, NoHeapAllocationInStep) {
  const Model<7> m = load_arm();
  const VecN<7> q0 = home_q();
  Pose goal = fk(m, q0);
  goal.p += Vec3(-0.2, 0.1, 0.1);
  const CartesianTrajectory traj(fk(m, q0), goal, 1.0);
  const CartesianSample x = traj.sample(0.5);
  ClikReference<7> clik(m, ClikConfig{});
  clik.reset(q0);
  VecN<7> q_sing = q0;
  q_sing(3) = m.joints[3].q_min + 1e-4;
  ClikReference<7> sing(m, ClikConfig{});
  sing.reset(q_sing);  // damped branch of the pseudo-inverse
  CartesianSample xs = static_sample(fk(m, q_sing));
  xs.T_b_d.p += Vec3(0.01, 0.01, 0.01);
  ClikReference<2> planar_clik(planar(), ClikConfig{});
  planar_clik.reset(VecN<2>(0.2, 0.4));
  const CartesianSample xp = static_sample(planar_target());
  g_allocs = 0;
  g_count_allocs = true;
  Eigen::internal::set_is_malloc_allowed(false);
  for (int k = 0; k < 100; ++k) {
    clik.step(x, kDt);
    sing.step(xs, kDt);
    planar_clik.step(xp, kDt);
  }
  Eigen::internal::set_is_malloc_allowed(true);
  g_count_allocs = false;
  EXPECT_EQ(g_allocs, 0U);
  EXPECT_TRUE(clik.reference().q.allFinite());
}

}  // namespace
}  // namespace cobot::planning
