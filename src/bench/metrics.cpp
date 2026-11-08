#include "metrics.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>

#include "cobot/common/lie.hpp"
#include "cobot/pipeline/recorder.hpp"

namespace cobot::bench {
namespace {

constexpr double kSatFraction = 0.999;  // cobot_eval.metrics.SAT_FRACTION

// Design G3 (spec 11.3): informative `passed` column only, the scenario files hold the
// thresholds that decide a test.
constexpr double kG3PosErrM = 0.002;
constexpr double kG3RotErrRad = 3.14159265358979323846 / 180.0;
constexpr double kG3RmsJointErrRad = 0.01;

// Post-goal swing windows: 0.5 s after the goal and before the end
// (tests/unit/loop_drive_profiles).
constexpr std::int64_t kSwingWindowNs = 500000000;

// RunSummary of the logged rows, fed one record at a time.
class Metrics {
 public:
  Metrics(const Vec& tau_max, const Vec& rated, std::int64_t t_eval_ns, TorqueSource torque)
      : t_eval_ns_(t_eval_ns), motor_torque_(torque == TorqueSource::kMotor) {
    tau_max_ = tau_max;
    rated_ = rated;
  }

  void add(const pipeline::LogRecord<kN>& r) {
    if (rows_ > 0) {
      last_dt_ns_ = r.t.ns - prev_t_ns_;
      sat_ns_ += prev_sat_ ? last_dt_ns_ : 0;
    }
    if (rows_ == 0) {
      first_t_ns_ = r.t.ns;
    }
    if (!s_.held && r.mode == pipeline::LoopMode::kHolding) {
      s_.hold_time_s = static_cast<double>(r.t.ns - first_t_ns_) /
                       static_cast<double>(common::kNanosecondsPerSecond);
      s_.hold_reason = r.hold_reason;
    }
    if (rows_ == 0 || r.t.ns <= t_eval_ns_) {
      final_e_ = r.T_b_e;
      final_d_ = r.T_b_d;
    }
    const Vec& tau = motor_torque_ ? r.tau_motor : r.tau;
    // PropagateNaN and the explicit isnan keep a NaN torque visible in the summary
    // (std::max and the default maxCoeff may drop it).
    const double ratio =
        (tau.cwiseAbs().array() / tau_max_.array()).maxCoeff<Eigen::PropagateNaN>();
    if (std::isnan(ratio) || ratio > s_.peak_tau_ratio) {
      s_.peak_tau_ratio = std::isnan(s_.peak_tau_ratio) ? s_.peak_tau_ratio : ratio;
    }
    prev_sat_ = ratio >= kSatFraction;
    prev_t_ns_ = r.t.ns;
    ++rows_;
    s_.held = s_.held || r.mode == pipeline::LoopMode::kHolding;
    if (r.mode == pipeline::LoopMode::kTracking || r.mode == pipeline::LoopMode::kFinished) {
      const Vec e = r.q_ref - r.q;
      sum_sq_ += e.cwiseProduct(e);
      tau_sum_sq_ += tau.cwiseProduct(tau);
      ++error_rows_;
    }
  }

  RunSummary finish() const {
    RunSummary s = s_;
    s.final_pos_err_m = (final_d_.p - final_e_.p).norm();
    s.final_rot_err_rad = common::so3_log(final_d_.R * final_e_.R.transpose()).norm();
    if (error_rows_ > 0) {
      const Vec mean_sq = sum_sq_ / static_cast<double>(error_rows_);
      s.rms_joint_err_rad = mean_sq.cwiseSqrt().maxCoeff<Eigen::PropagateNaN>();
      const Vec tau_mean_sq = tau_sum_sq_ / static_cast<double>(error_rows_);
      s.rms_tau_ratio =
          (tau_mean_sq.cwiseSqrt().array() / rated_.array()).maxCoeff<Eigen::PropagateNaN>();
    }
    const std::int64_t sat_ns = sat_ns_ + (prev_sat_ && rows_ > 1 ? last_dt_ns_ : 0);
    s.sat_time_s = static_cast<double>(sat_ns) / static_cast<double>(common::kNanosecondsPerSecond);
    if (rows_ > 0) {
      s.run_time_s = static_cast<double>(prev_t_ns_ - first_t_ns_ + last_dt_ns_) /
                     static_cast<double>(common::kNanosecondsPerSecond);
    }
    return s;
  }

 private:
  Vec tau_max_ = Vec::Zero();
  Vec rated_ = Vec::Zero();
  std::int64_t t_eval_ns_;
  bool motor_torque_;
  RunSummary s_;
  common::Pose final_e_;
  common::Pose final_d_;
  Vec sum_sq_ = Vec::Zero();
  Vec tau_sum_sq_ = Vec::Zero();
  std::size_t rows_ = 0;
  std::size_t error_rows_ = 0;
  std::int64_t prev_t_ns_ = 0;
  std::int64_t first_t_ns_ = 0;
  std::int64_t last_dt_ns_ = 0;
  std::int64_t sat_ns_ = 0;
  bool prev_sat_ = false;
};

void write_list(std::ostream& os, const char* key, const Vec& v) {
  os << key << ": [";
  for (int i = 0; i < kN; ++i) {
    os << (i > 0 ? ", " : "") << v(i);
  }
  os << "]\n";
}

// Meta keys of a waypoint run: node times, stop flags and tip positions of the validated list.
void write_waypoint_meta(std::ostream& os, const RunMeta& m) {
  os << "waypoints_file: '" << m.waypoints_yaml->string() << "'\nwaypoint_node_times_s: [";
  for (std::size_t i = 0; i < m.waypoint_node_times_s.size(); ++i) {
    os << (i > 0 ? ", " : "") << m.waypoint_node_times_s[i];
  }
  os << "]\nwaypoint_stop: [";
  for (std::size_t i = 0; i < m.waypoint_stop.size(); ++i) {
    os << (i > 0 ? ", " : "") << (m.waypoint_stop[i] ? "true" : "false");
  }
  os << "]\nwaypoint_positions_m: [";
  for (std::size_t i = 0; i < m.waypoint_positions_m.size(); ++i) {
    const common::Vec3& p = m.waypoint_positions_m[i];
    os << (i > 0 ? ", " : "") << "[" << p.x() << ", " << p.y() << ", " << p.z() << "]";
  }
  os << "]\n";
}

// Max over joints of the range of q - q_ref over the rows with begin <= t < end; NaN without rows.
double swing_in(const std::vector<pipeline::LogRecord<kN>>& rows,
                std::int64_t begin_ns,
                std::int64_t end_ns) {
  Vec lo = Vec::Constant(std::numeric_limits<double>::infinity());
  Vec hi = -lo;
  bool any = false;
  for (const pipeline::LogRecord<kN>& r : rows) {
    if (r.t.ns >= begin_ns && r.t.ns < end_ns) {
      const Vec e = r.q - r.q_ref;
      lo = lo.cwiseMin(e);
      hi = hi.cwiseMax(e);
      any = true;
    }
  }
  const Vec range = hi - lo;
  return any ? range.maxCoeff<Eigen::PropagateNaN>() : std::numeric_limits<double>::quiet_NaN();
}

// swing_first_rad / swing_last_rad of a run that did not hold: the hold is [goal, end), end the
// last row plus one cycle; windows of 0.5 s, or the two halves of a hold under 1 s.
void add_swing(const std::vector<pipeline::LogRecord<kN>>& rows,
               std::int64_t t_goal_ns,
               RunSummary& s) {
  if (s.held || rows.empty() || rows.back().t.ns < t_goal_ns) {
    return;
  }
  const std::int64_t last_dt_ns =
      rows.size() > 1 ? rows.back().t.ns - rows[rows.size() - 2].t.ns : 0;
  const std::int64_t end_ns = rows.back().t.ns + last_dt_ns;
  const std::int64_t hold_ns = end_ns - t_goal_ns;
  const std::int64_t window_ns = hold_ns < 2 * kSwingWindowNs ? hold_ns / 2 : kSwingWindowNs;
  s.swing_first_rad = swing_in(rows, t_goal_ns, t_goal_ns + window_ns);
  s.swing_last_rad = swing_in(rows, end_ns - window_ns, end_ns);
}

// settle_time_s of a run that did not hold (metrics.hpp): walk back from the evaluation row while
// the TCP is inside the band around that row's T_b_d.
double settle_time(const std::vector<pipeline::LogRecord<kN>>& rows,
                   std::int64_t t_eval_ns,
                   const SettleThresholds& band) {
  std::size_t eval = 0;
  while (eval + 1 < rows.size() && rows[eval + 1].t.ns <= t_eval_ns) {
    ++eval;
  }
  const common::Pose& goal = rows[eval].T_b_d;
  const auto inside = [&](const pipeline::LogRecord<kN>& r) {
    return (goal.p - r.T_b_e.p).norm() <= band.pos_m &&
           common::so3_log(goal.R * r.T_b_e.R.transpose()).norm() <= band.rot_rad;
  };
  if (!inside(rows[eval])) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::size_t entry = eval;
  while (entry > 0 && inside(rows[entry - 1])) {
    --entry;
  }
  return static_cast<double>(rows[entry].t.ns - rows.front().t.ns) /
         static_cast<double>(common::kNanosecondsPerSecond);
}

std::string number(double v) {
  std::array<char, 32> text{};
  std::snprintf(text.data(), text.size(), "%.17g", v);
  return text.data();
}

bool passes(const RunSummary& s) {
  return s.final_pos_err_m <= kG3PosErrM && s.final_rot_err_rad <= kG3RotErrRad &&
         s.rms_joint_err_rad <= kG3RmsJointErrRad;
}

}  // namespace

RunSummary summarize(const std::vector<pipeline::LogRecord<kN>>& rows,
                     const Vec& tau_max,
                     const Vec& rated,
                     std::int64_t t_eval_ns,
                     TorqueSource torque) {
  Metrics metrics(tau_max, rated, t_eval_ns, torque);
  for (const pipeline::LogRecord<kN>& r : rows) {
    metrics.add(r);
  }
  return metrics.finish();
}

RunSummary summarize(const std::vector<pipeline::LogRecord<kN>>& rows,
                     const Vec& tau_max,
                     const Vec& rated,
                     std::int64_t t_eval_ns,
                     TorqueSource torque,
                     std::int64_t t_goal_ns) {
  RunSummary s = summarize(rows, tau_max, rated, t_eval_ns, torque);
  add_swing(rows, t_goal_ns, s);
  return s;
}

RunSummary summarize(const std::vector<pipeline::LogRecord<kN>>& rows,
                     const Vec& tau_max,
                     const Vec& rated,
                     std::int64_t t_eval_ns,
                     TorqueSource torque,
                     std::int64_t t_goal_ns,
                     const std::optional<SettleThresholds>& settle) {
  RunSummary s = summarize(rows, tau_max, rated, t_eval_ns, torque, t_goal_ns);
  if (settle && !s.held && !rows.empty()) {
    s.settle_time_s = settle_time(rows, t_eval_ns, *settle);
  }
  return s;
}

common::Status write_run_files(const RunData& run, const std::filesystem::path& out_csv) {
  std::ofstream csv(out_csv, std::ios::binary);
  std::ofstream meta(out_csv.string() + ".meta.yaml", std::ios::binary);
  if (!csv || !meta) {
    return common::Status::error("cannot open " + out_csv.string() +
                                 " (or its .meta.yaml) for writing");
  }
  const RunMeta& m = run.meta;
  meta << std::setprecision(17) << "config: '" << m.config_yaml.string() << "'\n"
       << "imperfections: '" << m.imperfections_label.string() << "'\n"
       << "seed: " << m.seed << "\n";
  write_list(meta, "q_offset_rad", m.q_offset_rad);
  write_list(meta, "torque_scale", m.torque_scale);
  write_list(meta, "plant_link_mass_kg", m.plant_link_mass_kg);
  if (m.waypoints_yaml) {
    write_waypoint_meta(meta, m);
  }
  if (!m.git.empty()) {
    meta << "git: '" << m.git << "'\n";
  }
  const pipeline::RecordSpan<kN> rows{run.records.data(), run.records.size(), nullptr, 0};
  pipeline::write_csv<kN>(csv, rows, true);
  csv.flush();
  meta.flush();
  if (!csv || !meta) {
    return common::Status::error("cannot write " + out_csv.string());
  }
  return common::Status::ok();
}

void write_summary_header(std::ostream& os) {
  os << "run_id,param,level,seed,final_pos_err_m,final_rot_err_rad,rms_joint_err_rad,"
        "peak_tau_ratio,sat_time_s,rms_tau_ratio,held,hold_time_s,hold_reason,passed\n";
}

void write_summary_row(std::ostream& os, const SummaryRow& row) {
  std::array<char, 24> id{};
  std::snprintf(id.data(), id.size(), "%04zu", row.run_id);
  const RunSummary& s = row.summary;
  os << id.data() << ',' << row.param << ',' << number(row.level) << ',' << row.seed << ','
     << number(s.final_pos_err_m) << ',' << number(s.final_rot_err_rad) << ','
     << number(s.rms_joint_err_rad) << ',' << number(s.peak_tau_ratio) << ','
     << number(s.sat_time_s) << ',' << number(s.rms_tau_ratio) << ',' << (s.held ? 1 : 0) << ','
     << (s.held ? number(s.hold_time_s) : "") << ',' << pipeline::to_string(s.hold_reason) << ','
     << (passes(s) ? 1 : 0) << '\n';
}

}  // namespace cobot::bench
