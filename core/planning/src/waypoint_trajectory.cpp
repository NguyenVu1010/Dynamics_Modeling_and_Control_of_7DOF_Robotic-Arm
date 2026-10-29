#include "cobot/planning/waypoint_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include <Eigen/Geometry>

#include "cobot/common/lie.hpp"

namespace cobot::planning {

using common::Mat3;
using common::Pose;
using common::Vec3;

namespace {

// Shortest auto segment, s (design section 6.1).
constexpr double kMinSegmentDurationS = 0.1;
// Rest-to-rest quintic: peak rate = 15/8 * D / T, peak second derivative = 10/sqrt(3) * D / T^2.
constexpr double kQuinticPeakRate = 15.0 / 8.0;
constexpr double kQuinticPeakAccel = 5.773502691896258;
// Allowed rotation matrix error |R^T R - I|_max.
constexpr double kOrthonormalTol = 1e-6;
// Segment translation (m) or rotation (rad) at or below this counts as zero: the tip rests there.
constexpr double kZeroLength = 1e-12;
// Auto durations: grid intervals per segment, target fraction of the binding limit on the grid
// (leaves room for peaks between grid points), extra stretch per step, step limit.
constexpr int kGridIntervals = 256;
constexpr double kTargetRatio = 0.999;
constexpr double kStretchStep = 1e-3;
constexpr int kMaxStretchIterations = 200;
// Below this angle (rad) the Jacobian coefficients use their Taylor series.
constexpr double kSeriesAngleRad = 0.1;

bool finite(const Pose& T) {
  return T.p.allFinite() && T.R.allFinite();
}

bool orthonormal(const Mat3& R) {
  const Mat3 E = R.transpose() * R - Mat3::Identity();
  return E.cwiseAbs().maxCoeff() <= kOrthonormalTol;
}

// Left Jacobian of SO(3): J_l(phi) = I + a [phi]x + b [phi]x^2 with a = (1 - cos t) / t^2,
// b = (t - sin t) / t^3 (t = |phi|); da/dt = ca (phi . phid), db/dt = cb (phi . phid).
struct JlCoeffs {
  double a = 0.0;
  double b = 0.0;
  double ca = 0.0;
  double cb = 0.0;
};

JlCoeffs jl_coeffs(double theta) {
  JlCoeffs c;
  const double t2 = theta * theta;
  if (theta < kSeriesAngleRad) {
    const double t4 = t2 * t2;
    c.a = 0.5 - t2 / 24.0 + t4 / 720.0;
    c.b = 1.0 / 6.0 - t2 / 120.0 + t4 / 5040.0;
    c.ca = -1.0 / 12.0 + t2 / 180.0 - t4 / 6720.0;
    c.cb = -1.0 / 60.0 + t2 / 1260.0 - t4 / 60480.0;
    return c;
  }
  const double sin_t = std::sin(theta);
  const double cos_t = std::cos(theta);
  c.a = (1.0 - cos_t) / t2;
  c.b = (theta - sin_t) / (t2 * theta);
  c.ca = (theta * sin_t - 2.0 * (1.0 - cos_t)) / (t2 * t2);
  c.cb = (3.0 * sin_t - 2.0 * theta - theta * cos_t) / (t2 * t2 * theta);
  return c;
}

// J_l(phi)^-1 y = y - phi x y / 2 + c phi x (phi x y), c = (1 - (t/2) cot(t/2)) / t^2.
// Finite for |phi| <= pi.
Vec3 jl_inverse_times(const Vec3& phi, const Vec3& y) {
  const double theta = phi.norm();
  const double t2 = theta * theta;
  double c = 1.0 / 12.0 + t2 / 720.0 + t2 * t2 / 30240.0;
  if (theta >= kSeriesAngleRad) {
    const double half = 0.5 * theta;
    c = (1.0 - half * std::cos(half) / std::sin(half)) / t2;
  }
  const Vec3 pxy = phi.cross(y);
  return y - 0.5 * pxy + c * phi.cross(pxy);
}

std::string at(std::size_t i) {
  return "waypoints[" + std::to_string(i) + "]";
}

std::string check_limits(const CartesianLimits& lim) {
  const std::pair<const char*, double> items[] = {{"v_max_m_s", lim.v_max_m_s},
                                                  {"a_max_m_s2", lim.a_max_m_s2},
                                                  {"w_max_rad_s", lim.w_max_rad_s},
                                                  {"alpha_max_rad_s2", lim.alpha_max_rad_s2}};
  for (const auto& [name, value] : items) {
    if (!std::isfinite(value) || value <= 0.0) {
      return std::string("limits.") + name + ": must be finite and > 0";
    }
  }
  return {};
}

}  // namespace

common::Result<WaypointTrajectory> WaypointTrajectory::create(
    const Pose& T_b_start, const std::vector<Waypoint>& waypoints, const CartesianLimits& limits) {
  using Built = common::Result<WaypointTrajectory>;
  if (waypoints.empty()) {
    return Built::error("waypoints: empty list");
  }
  if (waypoints.size() > kMaxWaypoints) {
    return Built::error("waypoints: " + std::to_string(waypoints.size()) + " points, at most " +
                        std::to_string(kMaxWaypoints));
  }
  if (!finite(T_b_start) || !orthonormal(T_b_start.R)) {
    return Built::error("T_b_start: not finite or rotation not orthonormal");
  }
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const Waypoint& w = waypoints[i];
    if (!w.T_b_goal.p.allFinite()) {
      return Built::error(at(i) + ".T_b_goal.p: not finite");
    }
    if (!w.T_b_goal.R.allFinite()) {
      return Built::error(at(i) + ".T_b_goal.R: not finite");
    }
    if (!orthonormal(w.T_b_goal.R)) {
      return Built::error(at(i) + ".T_b_goal.R: not orthonormal (|R^T R - I|_max > 1e-6)");
    }
    if (!std::isfinite(w.duration_s)) {
      return Built::error(at(i) + ".duration_s: not finite");
    }
  }
  const std::string bad_limit = check_limits(limits);
  if (!bad_limit.empty()) {
    return Built::error(bad_limit);
  }

  WaypointTrajectory traj;
  traj.start_ = T_b_start;
  traj.count_ = waypoints.size();
  for (std::size_t i = 0; i < traj.count_; ++i) {
    Waypoint& w = traj.waypoints_[i];
    w = waypoints[i];
    w.stop = w.stop || i + 1 == traj.count_;
    traj.user_duration_[i] = w.duration_s > 0.0;
    const Pose& from = traj.node_pose(i);
    Segment& seg = traj.segments_[i];
    seg.dp = w.T_b_goal.p - from.p;
    seg.phi = common::so3_log(from.R.transpose() * w.T_b_goal.R);
    if (seg.dp.norm() <= kZeroLength) {
      seg.dp.setZero();
    }
    if (seg.phi.norm() <= kZeroLength) {
      seg.phi.setZero();
    }
    if (!traj.user_duration_[i]) {
      const double d = seg.dp.norm();
      const double th = seg.phi.norm();
      w.duration_s = std::max({kMinSegmentDurationS,
                               kQuinticPeakRate * d / limits.v_max_m_s,
                               std::sqrt(kQuinticPeakAccel * d / limits.a_max_m_s2),
                               kQuinticPeakRate * th / limits.w_max_rad_s,
                               std::sqrt(kQuinticPeakAccel * th / limits.alpha_max_rad_s2)});
    }
  }

  // Node rates depend on the durations, so stretch the auto segments until their sampled peaks
  // fit. A longer segment only lowers the node rates, so the loop settles.
  for (int iter = 0; iter < kMaxStretchIterations; ++iter) {
    traj.set_node_rates();
    bool stretched = false;
    for (std::size_t i = 0; i < traj.count_; ++i) {
      if (traj.user_duration_[i]) {
        continue;
      }
      const double r = traj.peak_ratio(i, limits);
      if (r > kTargetRatio) {
        traj.waypoints_[i].duration_s *= r / kTargetRatio + kStretchStep;
        stretched = true;
      }
    }
    if (!stretched) {
      break;
    }
  }
  traj.set_node_rates();

  double t = 0.0;
  for (std::size_t i = 0; i < traj.count_; ++i) {
    t += traj.waypoints_[i].duration_s;
    traj.node_time_s_[i] = t;
  }
  return Built::ok(std::move(traj));
}

const Pose& WaypointTrajectory::node_pose(std::size_t i) const {
  return i == 0 ? start_ : waypoints_[i - 1].T_b_goal;
}

void WaypointTrajectory::set_node_rates() {
  segments_[0].v0.setZero();
  segments_[0].u0.setZero();
  for (std::size_t i = 0; i < count_; ++i) {
    Segment& seg = segments_[i];
    Vec3 v = Vec3::Zero();  // linear velocity at node i, in b
    Vec3 w = Vec3::Zero();  // angular velocity at node i, in b
    if (!waypoints_[i].stop) {
      // Catmull-Rom style: the mean rate over the two neighbouring segments; zero next to a
      // segment that does not move in that part (so it stays at rest).
      const Segment& next = segments_[i + 1];
      const double T_sum = waypoints_[i].duration_s + waypoints_[i + 1].duration_s;
      if (!seg.dp.isZero(0.0) && !next.dp.isZero(0.0)) {
        v = (seg.dp + next.dp) / T_sum;
      }
      if (!seg.phi.isZero(0.0) && !next.phi.isZero(0.0)) {
        const Vec3 dw = node_pose(i).R * seg.phi + waypoints_[i].T_b_goal.R * next.phi;
        w = dw / T_sum;
      }
    }
    seg.v1 = v;
    seg.u1 = jl_inverse_times(seg.phi, node_pose(i).R.transpose() * w);
    if (i + 1 < count_) {
      segments_[i + 1].v0 = v;
      segments_[i + 1].u0 = waypoints_[i].T_b_goal.R.transpose() * w;
    }
  }
}

double WaypointTrajectory::peak_ratio(std::size_t i, const CartesianLimits& limits) const {
  double r = 0.0;
  for (int k = 0; k <= kGridIntervals; ++k) {
    const CartesianSample s = sample_segment(i, static_cast<double>(k) / kGridIntervals);
    r = std::max({r,
                  s.xd_d.head<3>().norm() / limits.v_max_m_s,
                  std::sqrt(s.xdd_d.head<3>().norm() / limits.a_max_m_s2),
                  s.xd_d.tail<3>().norm() / limits.w_max_rad_s,
                  std::sqrt(s.xdd_d.tail<3>().norm() / limits.alpha_max_rad_s2)});
  }
  return r;
}

CartesianSample WaypointTrajectory::sample_segment(std::size_t i, double tau) const {
  const Segment& seg = segments_[i];
  const Pose& from = node_pose(i);
  const double T = waypoints_[i].duration_s;
  const double tau2 = tau * tau;
  const double tau3 = tau2 * tau;
  // Quintic Hermite basis with zero second derivative at both ends; suffix _1, _2 = d/dtau,
  // d2/dtau2. s: 0 -> 1 at rest; h10: unit rate at the start; h11: unit rate at the end.
  const double s = tau3 * (10.0 + tau * (-15.0 + 6.0 * tau));
  const double s_1 = tau2 * (30.0 + tau * (-60.0 + 30.0 * tau));
  const double s_2 = tau * (60.0 + tau * (-180.0 + 120.0 * tau));
  const double h10 = tau + tau3 * (-6.0 + tau * (8.0 - 3.0 * tau));
  const double h10_1 = 1.0 + tau2 * (-18.0 + tau * (32.0 - 15.0 * tau));
  const double h10_2 = tau * (-36.0 + tau * (96.0 - 60.0 * tau));
  const double h11 = tau3 * (-4.0 + tau * (7.0 - 3.0 * tau));
  const double h11_1 = tau2 * (-12.0 + tau * (28.0 - 15.0 * tau));
  const double h11_2 = tau * (-24.0 + tau * (84.0 - 60.0 * tau));
  const double sd = s_1 / T;
  const double sdd = s_2 / (T * T);

  CartesianSample out;
  out.T_b_d.p = from.p + s * seg.dp + T * (h10 * seg.v0 + h11 * seg.v1);
  out.xd_d.head<3>() = sd * seg.dp + h10_1 * seg.v0 + h11_1 * seg.v1;
  out.xdd_d.head<3>() = sdd * seg.dp + (h10_2 * seg.v0 + h11_2 * seg.v1) / T;

  // Rotation vector phi(t) in the segment start frame: R = R0 exp(phi).
  const Vec3 ph = s * seg.phi + T * (h10 * seg.u0 + h11 * seg.u1);
  const Vec3 phd = sd * seg.phi + h10_1 * seg.u0 + h11_1 * seg.u1;
  const Vec3 phdd = sdd * seg.phi + (h10_2 * seg.u0 + h11_2 * seg.u1) / T;
  out.T_b_d.R = from.R * common::so3_exp(ph);

  // omega = J_l(phi) phid and its time derivative, rotated into b.
  const JlCoeffs c = jl_coeffs(ph.norm());
  const double pdot = ph.dot(phd);
  const Vec3 x1 = ph.cross(phd);
  const Vec3 x2 = ph.cross(x1);
  const Vec3 y1 = ph.cross(phdd);
  const Vec3 w = phd + c.a * x1 + c.b * x2;
  const Vec3 alpha = phdd + (c.ca * pdot) * x1 + c.a * y1 + (c.cb * pdot) * x2 +
                     c.b * (phd.cross(x1) + ph.cross(y1));
  out.xd_d.tail<3>() = from.R * w;
  out.xdd_d.tail<3>() = from.R * alpha;
  return out;
}

CartesianSample WaypointTrajectory::sample(double t_s) const {
  if (t_s <= 0.0) {
    CartesianSample out;
    out.T_b_d = start_;
    return out;
  }
  const std::size_t i = segment_at(t_s);
  if (i >= count_) {
    CartesianSample out;
    out.T_b_d = waypoints_[count_ - 1].T_b_goal;
    return out;
  }
  const double t0 = i == 0 ? 0.0 : node_time_s_[i - 1];
  return sample_segment(i, (t_s - t0) / waypoints_[i].duration_s);
}

double WaypointTrajectory::duration_s() const {
  return node_time_s_[count_ - 1];
}

std::size_t WaypointTrajectory::size() const {
  return count_;
}

double WaypointTrajectory::node_time_s(std::size_t i) const {
  return node_time_s_[i];
}

double WaypointTrajectory::segment_duration_s(std::size_t i) const {
  return waypoints_[i].duration_s;
}

bool WaypointTrajectory::user_duration(std::size_t i) const {
  return user_duration_[i];
}

bool WaypointTrajectory::stops_at(std::size_t i) const {
  return waypoints_[i].stop;
}

std::size_t WaypointTrajectory::segment_at(double t_s) const {
  const double* first = node_time_s_.data();
  return static_cast<std::size_t>(std::upper_bound(first, first + count_, t_s) - first);
}

const Pose& WaypointTrajectory::start_pose() const {
  return start_;
}

const Waypoint& WaypointTrajectory::waypoint(std::size_t i) const {
  return waypoints_[i];
}

}  // namespace cobot::planning
