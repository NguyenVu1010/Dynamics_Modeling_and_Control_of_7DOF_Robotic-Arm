#include "cobot/pipeline/path_validation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "cobot/common/dof.hpp"
#include "cobot/dynamics/dynamics.hpp"
#include "cobot/planning/cartesian_trajectory.hpp"

// Design docs/design/2026-10-29-waypoint-gui-design.md section 6.2.
namespace cobot::pipeline {
namespace {

using common::Pose;
using common::VecN;
using planning::WaypointTrajectory;

constexpr std::size_t kIssueKinds = 4;  // PathIssue values
constexpr double kStretchStep = 1.25;   // factor applied to an auto segment per rerun
// Relative slack (of a sample step) before a path sample time counts as the end time.
constexpr double kSampleTimeEps = 1e-9;

// Worst occurrence of one issue kind on one waypoint. `severity` orders occurrences of the same
// kind: value / limit, except joint_limit where it is the depth inside the margin (rad).
struct Worst {
  bool found = false;
  double severity = 0.0;
  WaypointIssue issue;
};

using WaypointWorst = std::array<Worst, kIssueKinds>;

struct Occurrence {
  PathIssue kind = PathIssue::kUnreachable;
  int joint = -1;
  double value = 0.0;
  double limit = 0.0;
  double severity = 0.0;
};

void record(WaypointWorst& worst, std::size_t waypoint, const Occurrence& o) {
  Worst& w = worst[static_cast<std::size_t>(o.kind)];
  if (w.found && o.severity <= w.severity) {
    return;
  }
  w.found = true;
  w.severity = o.severity;
  w.issue.waypoint = waypoint;
  w.issue.issue = o.kind;
  w.issue.joint = o.joint;
  w.issue.value = o.value;
  w.issue.limit = o.limit;
}

common::Status check_limits(const ValidationLimits& l) {
  const std::array<std::pair<const char*, double>, 7> fields{{
      {"reach_pos_m", l.reach_pos_m},
      {"reach_rot_rad", l.reach_rot_rad},
      {"joint_margin_rad", l.joint_margin_rad},
      {"torque_ratio", l.torque_ratio},
      {"max_stretch", l.max_stretch},
      {"dt_s", l.dt_s},
      {"path_sample_dt_s", l.path_sample_dt_s},
  }};
  for (const auto& [name, value] : fields) {
    if (!std::isfinite(value) || value <= 0.0) {
      return common::Status::error(std::string("validate_path: limits.") + name +
                                   " must be finite and > 0");
    }
  }
  return common::Status::ok();
}

// Joint checks of one CLIK step against the reference (q_ref, qd_ref) and feed-forward torque.
template <int N>
void check_joints(const model::Model<N>& model,
                  const planning::JointReference<N>& ref,
                  const VecN<N>& tau,
                  const ValidationLimits& limits,
                  std::size_t waypoint,
                  WaypointWorst& worst) {
  for (int j = 0; j < N; ++j) {
    const model::JointParams& joint = model.joints[static_cast<std::size_t>(j)];
    if (std::isfinite(joint.q_min) && std::isfinite(joint.q_max)) {  // continuous joints skipped
      const double dist = std::min(ref.q(j) - joint.q_min, joint.q_max - ref.q(j));
      if (dist < limits.joint_margin_rad) {
        record(worst,
               waypoint,
               {PathIssue::kJointLimit,
                j,
                dist,
                limits.joint_margin_rad,
                limits.joint_margin_rad - dist});
      }
    }
    const double qd = std::abs(ref.qd(j));
    if (qd > joint.qd_max) {
      record(worst, waypoint, {PathIssue::kJointVelocity, j, qd, joint.qd_max, qd / joint.qd_max});
    }
    const double ratio = std::abs(tau(j)) / joint.tau_max;
    if (ratio > limits.torque_ratio) {
      record(worst,
             waypoint,
             {PathIssue::kTorque, j, ratio, limits.torque_ratio, ratio / limits.torque_ratio});
    }
  }
}

// One CLIK run along `traj` at limits.dt_s; the worst issue of each kind per waypoint.
template <int N>
std::vector<WaypointWorst> run_clik(const model::Model<N>& model,
                                    const planning::ClikConfig& clik_config,
                                    const control::FrictionCompensation<N>& friction,
                                    const VecN<N>& q_start,
                                    const WaypointTrajectory& traj,
                                    const ValidationLimits& limits) {
  const std::size_t n = traj.size();
  std::vector<WaypointWorst> worst(n);
  const double dt = limits.dt_s;
  // The task error of step k is the error at its own time t_k = k dt, so node i is checked on
  // the step nearest to its time.
  std::vector<long long> node_step(n);
  for (std::size_t i = 0; i < n; ++i) {
    node_step[i] = std::llround(traj.node_time_s(i) / dt);
  }
  const long long last_step =
      std::max(static_cast<long long>(std::ceil(traj.duration_s() / dt)), node_step[n - 1]);

  planning::ClikReference<N> clik(model, clik_config);
  clik.reset(q_start);
  std::size_t next_node = 0;
  for (long long k = 0; k <= last_step; ++k) {
    const double t = static_cast<double>(k) * dt;
    const std::size_t seg = std::min(traj.segment_at(t), n - 1);
    const planning::JointReference<N>& ref = clik.step(traj.sample(t), dt);

    for (; next_node < n && node_step[next_node] <= k; ++next_node) {
      const double pos_err = ref.task_error.template head<3>().norm();
      const double rot_err = ref.task_error.template tail<3>().norm();
      const double pos_ratio = pos_err / limits.reach_pos_m;
      const double rot_ratio = rot_err / limits.reach_rot_rad;
      if (pos_ratio > 1.0 || rot_ratio > 1.0) {
        const bool by_pos = pos_ratio >= rot_ratio;
        record(worst[next_node],
               next_node,
               {PathIssue::kUnreachable,
                -1,
                by_pos ? pos_err : rot_err,
                by_pos ? limits.reach_pos_m : limits.reach_rot_rad,
                std::max(pos_ratio, rot_ratio)});
      }
    }

    const VecN<N> tau =
        dynamics::rnea<N>(model, ref.q, ref.qd, ref.qdd, model.gravity_b) + friction.torque(ref.qd);
    check_joints<N>(model, ref, tau, limits, seg, worst[seg]);
  }
  return worst;
}

// Some issue, and only joint_velocity / torque: a longer segment may fix it.
bool stretchable(const WaypointWorst& w) {
  const bool fixable = w[static_cast<std::size_t>(PathIssue::kJointVelocity)].found ||
                       w[static_cast<std::size_t>(PathIssue::kTorque)].found;
  return fixable && !w[static_cast<std::size_t>(PathIssue::kUnreachable)].found &&
         !w[static_cast<std::size_t>(PathIssue::kJointLimit)].found;
}

// Desired tip pose every sample_dt over [0, T), then at T.
std::vector<Pose> sample_path(const WaypointTrajectory& traj, double sample_dt) {
  std::vector<Pose> path;
  const double T = traj.duration_s();
  for (long long j = 0;; ++j) {
    const double t = static_cast<double>(j) * sample_dt;
    if (t >= T - kSampleTimeEps * sample_dt) {
      break;
    }
    path.push_back(traj.sample(t).T_b_d);
  }
  path.push_back(traj.sample(T).T_b_d);
  return path;
}

}  // namespace

const char* to_string(PathIssue issue) {
  switch (issue) {
    case PathIssue::kUnreachable:
      return "unreachable";
    case PathIssue::kJointLimit:
      return "joint_limit";
    case PathIssue::kJointVelocity:
      return "joint_velocity";
    case PathIssue::kTorque:
      return "torque";
  }
  return "unknown";
}

template <int N>
common::Result<PathValidation<N>> validate_path(
    const model::Model<N>& model,
    const planning::ClikConfig& clik,
    const control::FrictionCompensationConfig<N>& friction,
    const VecN<N>& q_start,
    const Pose& T_b_start,
    const std::vector<planning::Waypoint>& waypoints,
    const planning::CartesianLimits& cartesian,
    const ValidationLimits& limits) {
  using Out = common::Result<PathValidation<N>>;
  common::Result<WaypointTrajectory> built =
      WaypointTrajectory::create(T_b_start, waypoints, cartesian);
  if (!built.is_ok()) {
    return Out::error(built.reason());
  }
  if (!q_start.allFinite()) {
    return Out::error("validate_path: q_start not finite");
  }
  const common::Status limits_ok = check_limits(limits);
  if (!limits_ok.is_ok()) {
    return Out::error(limits_ok.reason());
  }

  const std::size_t n = waypoints.size();
  std::optional<WaypointTrajectory> traj(std::move(built).value());
  std::vector<double> base_s(n);
  std::vector<bool> is_auto(n);
  for (std::size_t i = 0; i < n; ++i) {
    base_s[i] = traj->segment_duration_s(i);
    is_auto[i] = !traj->user_duration(i);
  }
  std::vector<double> factor(n, 1.0);  // 1.25^k, exact in binary
  const control::FrictionCompensation<N> friction_ff(friction);

  std::vector<WaypointWorst> worst;
  for (;;) {
    worst = run_clik<N>(model, clik, friction_ff, q_start, *traj, limits);
    bool stretched = false;
    for (std::size_t i = 0; i < n; ++i) {
      if (is_auto[i] && stretchable(worst[i]) && factor[i] * kStretchStep <= limits.max_stretch) {
        factor[i] *= kStretchStep;
        stretched = true;
      }
    }
    if (!stretched) {
      break;
    }
    // Every duration pinned, so the rerun keeps the unstretched segments exactly as they were.
    std::vector<planning::Waypoint> pinned = waypoints;
    for (std::size_t i = 0; i < n; ++i) {
      pinned[i].duration_s = base_s[i] * factor[i];
    }
    common::Result<WaypointTrajectory> rebuilt =
        WaypointTrajectory::create(T_b_start, pinned, cartesian);
    if (!rebuilt.is_ok()) {
      return Out::error(rebuilt.reason());
    }
    traj.emplace(std::move(rebuilt).value());
  }

  PathValidation<N> out;
  for (const WaypointWorst& w : worst) {
    for (const Worst& x : w) {
      if (x.found) {
        out.issues.push_back(x.issue);
      }
    }
  }
  out.ok = out.issues.empty();
  out.path = sample_path(*traj, limits.path_sample_dt_s);
  out.segment_durations_s.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    out.segment_durations_s[i] = traj->segment_duration_s(i);
  }
  if (out.ok) {
    out.trajectory = std::move(traj);
  }
  return Out::ok(std::move(out));
}

#define INSTANTIATE(N)                                         \
  template common::Result<PathValidation<N>> validate_path<N>( \
      const model::Model<N>&,                                  \
      const planning::ClikConfig&,                             \
      const control::FrictionCompensationConfig<N>&,           \
      const VecN<N>&,                                          \
      const Pose&,                                             \
      const std::vector<planning::Waypoint>&,                  \
      const planning::CartesianLimits&,                        \
      const ValidationLimits&);
COBOT_INSTANTIATE_DOF(INSTANTIATE)
#undef INSTANTIATE

}  // namespace cobot::pipeline
