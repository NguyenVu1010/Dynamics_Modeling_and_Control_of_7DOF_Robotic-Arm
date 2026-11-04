#include "cobot/studio/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "cobot/common/lie.hpp"

namespace cobot::studio {
namespace {

using Record = pipeline::LogRecord<bench::kN>;

double time_s(const Record& r) {
  return static_cast<double>(r.t.ns) * 1e-9;
}

double rot_angle(const common::Mat3& r_d, const common::Mat3& r) {
  return common::so3_log(r_d * r.transpose()).norm();
}

// First row whose time is nearest `t` (numpy argmin, as cobot_eval).
std::size_t nearest(const std::vector<double>& t_s, double t) {
  std::size_t best = 0;
  double gap = std::numeric_limits<double>::infinity();
  for (std::size_t k = 0; k < t_s.size(); ++k) {
    const double d = std::abs(t_s[k] - t);
    if (d < gap) {
      gap = d;
      best = k;
    }
  }
  return best;
}

bool within(double pos, double rot, double pos_tol, double rot_tol) {
  return pos <= pos_tol && rot <= rot_tol;
}

std::optional<double> settle_time(const ErrorTrace& e,
                                  const bench::RunMeta& meta,
                                  const bench::RunSummary& summary,
                                  const Tolerance& tol) {
  if (summary.held || e.eval_row < 0 || e.t_s.empty()) {
    return std::nullopt;
  }
  auto k = static_cast<std::size_t>(e.eval_row);
  const double t0 = e.t_s.front() + meta.goal_time_s;
  const auto inside = [&](std::size_t i) {
    return within(e.goal_pos_m[i], e.goal_rot_rad[i], tol.goal_pos_m, tol.goal_rot_rad);
  };
  if (e.t_s[k] < t0 || !inside(k)) {
    return std::nullopt;
  }
  while (k > 0 && e.t_s[k - 1] >= t0 && inside(k - 1)) {
    --k;
  }
  return e.t_s[k] - t0;
}

StopMetrics stop_metrics(const StopTrace& s, const Tolerance& tol) {
  StopMetrics m;
  m.waypoint = s.waypoint;
  m.t_arrive_s = s.t_arrive_s;
  m.pos_err_m = std::numeric_limits<double>::infinity();
  m.rot_err_rad = std::numeric_limits<double>::infinity();
  for (std::size_t k = 0; k < s.t_s.size(); ++k) {
    m.pos_err_m = std::min(m.pos_err_m, s.pos_m[k]);
    m.rot_err_rad = std::min(m.rot_err_rad, s.rot_rad[k]);
    if (!m.time_to_enter_s && within(s.pos_m[k], s.rot_rad[k], tol.goal_pos_m, tol.goal_rot_rad)) {
      m.time_to_enter_s = s.t_s[k] - s.t_arrive_s;
    }
  }
  return m;
}

template <typename T>
void worst(T& acc, T v) {
  acc = std::max(acc, v);
}

}  // namespace

ErrorTrace make_error_trace(const bench::RunData& run) {
  ErrorTrace e;
  const std::vector<Record>& rows = run.records;
  if (rows.empty()) {
    return e;
  }
  const std::int64_t t_eval_ns = std::llround(run.meta.window_end_s * 1e9);
  std::size_t eval = 0;
  for (std::size_t k = 0; k < rows.size(); ++k) {
    eval = rows[k].t.ns <= t_eval_ns ? k : eval;
  }
  e.eval_row = static_cast<std::ptrdiff_t>(eval);
  const common::Pose& goal = rows[eval].T_b_d;
  const std::size_t n = rows.size();
  for (std::vector<double>* v :
       {&e.t_s, &e.path_pos_m, &e.path_rot_rad, &e.goal_pos_m, &e.goal_rot_rad}) {
    v->reserve(n);
  }
  e.on_path.reserve(n);
  for (const Record& r : rows) {
    e.t_s.push_back(time_s(r));
    e.path_pos_m.push_back((r.T_b_d.p - r.T_b_e.p).norm());
    e.path_rot_rad.push_back(rot_angle(r.T_b_d.R, r.T_b_e.R));
    e.goal_pos_m.push_back((goal.p - r.T_b_e.p).norm());
    e.goal_rot_rad.push_back(rot_angle(goal.R, r.T_b_e.R));
    e.on_path.push_back(
        r.mode == pipeline::LoopMode::kTracking || r.mode == pipeline::LoopMode::kFinished ? 1 : 0);
  }
  const bench::RunMeta& m = run.meta;
  const std::size_t waypoints = std::min(
      {m.waypoint_node_times_s.size(), m.waypoint_stop.size(), m.waypoint_positions_m.size()});
  for (std::size_t w = 0; w + 1 < waypoints; ++w) {
    if (!m.waypoint_stop[w]) {
      continue;
    }
    StopTrace s;
    s.waypoint = w;
    s.t_arrive_s = e.t_s.front() + m.waypoint_node_times_s[w];
    const std::size_t k0 = nearest(e.t_s, s.t_arrive_s);
    const std::size_t k1 = nearest(e.t_s, s.t_arrive_s + kStopSettleS);
    const common::Mat3& r_w = rows[k0].T_b_d.R;
    for (std::size_t k = k0; k <= k1; ++k) {
      s.t_s.push_back(e.t_s[k]);
      s.pos_m.push_back((rows[k].T_b_e.p - m.waypoint_positions_m[w]).norm());
      s.rot_rad.push_back(rot_angle(r_w, rows[k].T_b_e.R));
    }
    e.stops.push_back(std::move(s));
  }
  return e;
}

TrackingMetrics compute_metrics(const ErrorTrace& errors,
                                const bench::RunMeta& meta,
                                const bench::RunSummary& summary,
                                const Tolerance& tolerance) {
  TrackingMetrics m;
  m.summary = summary;
  double sum_sq = 0.0;
  std::size_t count = 0;
  for (std::size_t k = 0; k < errors.t_s.size(); ++k) {
    if (errors.on_path[k] == 0) {
      continue;
    }
    worst(m.path_pos_err_max_m, errors.path_pos_m[k]);
    worst(m.path_rot_err_max_rad, errors.path_rot_rad[k]);
    sum_sq += errors.path_pos_m[k] * errors.path_pos_m[k];
    ++count;
  }
  m.path_pos_err_rms_m = count > 0 ? std::sqrt(sum_sq / static_cast<double>(count)) : 0.0;
  m.settle_time_s = settle_time(errors, meta, summary, tolerance);
  for (const StopTrace& s : errors.stops) {
    m.stops.push_back(stop_metrics(s, tolerance));
  }
  m.goal_ok = within(summary.final_pos_err_m,
                     summary.final_rot_err_rad,
                     tolerance.goal_pos_m,
                     tolerance.goal_rot_rad);
  m.path_ok = within(
      m.path_pos_err_max_m, m.path_rot_err_max_rad, tolerance.path_pos_m, tolerance.path_rot_rad);
  return m;
}

TrackingMetrics compute_metrics(const bench::RunData& run, const Tolerance& tolerance) {
  return compute_metrics(make_error_trace(run), run.meta, run.summary, tolerance);
}

LevelRow worst_case(const std::vector<const RunEntry*>& same_level, const Tolerance& tolerance) {
  LevelRow row;
  bool first = true;
  bool settled = true;
  for (const RunEntry* e : same_level) {
    if (e == nullptr || e->errors == nullptr) {
      continue;
    }
    const TrackingMetrics m = compute_metrics(*e->errors, e->meta, e->summary, tolerance);
    ++row.seeds;
    row.held += e->summary.held ? 1U : 0U;
    if (first) {
      row.param = e->sweep ? e->sweep->param : std::string();
      row.level = e->sweep ? e->sweep->level : 0.0;
      row.worst = m;
      settled = m.settle_time_s.has_value();
      first = false;
      continue;
    }
    TrackingMetrics& w = row.worst;
    bench::RunSummary& s = w.summary;
    worst(s.final_pos_err_m, m.summary.final_pos_err_m);
    worst(s.final_rot_err_rad, m.summary.final_rot_err_rad);
    worst(s.rms_joint_err_rad, m.summary.rms_joint_err_rad);
    worst(s.peak_tau_ratio, m.summary.peak_tau_ratio);
    worst(s.sat_time_s, m.summary.sat_time_s);
    worst(s.rms_tau_ratio, m.summary.rms_tau_ratio);
    if (m.summary.held && (!s.held || m.summary.hold_time_s < s.hold_time_s)) {
      s.hold_time_s = m.summary.hold_time_s;  // the earliest hold
      s.hold_reason = m.summary.hold_reason;
    }
    s.held = s.held || m.summary.held;
    worst(w.path_pos_err_max_m, m.path_pos_err_max_m);
    worst(w.path_rot_err_max_rad, m.path_rot_err_max_rad);
    worst(w.path_pos_err_rms_m, m.path_pos_err_rms_m);
    settled = settled && m.settle_time_s.has_value();
    w.settle_time_s = settled ? std::optional<double>(std::max(*w.settle_time_s, *m.settle_time_s))
                              : std::nullopt;
    for (std::size_t i = 0; i < w.stops.size() && i < m.stops.size(); ++i) {
      StopMetrics& a = w.stops[i];
      const StopMetrics& b = m.stops[i];
      worst(a.pos_err_m, b.pos_err_m);
      worst(a.rot_err_rad, b.rot_err_rad);
      a.time_to_enter_s =
          a.time_to_enter_s && b.time_to_enter_s
              ? std::optional<double>(std::max(*a.time_to_enter_s, *b.time_to_enter_s))
              : std::nullopt;
    }
    w.goal_ok = w.goal_ok && m.goal_ok;
    w.path_ok = w.path_ok && m.path_ok;
  }
  return row;
}

}  // namespace cobot::studio
