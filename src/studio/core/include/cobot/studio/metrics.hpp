#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "cobot/studio/run_store.hpp"
#include "cobot/studio/studio_config.hpp"

#include "engine.hpp"
#include "run_data.hpp"

// Tracking metrics of a studio run (feature studio-core, cobot_studio design 5.2, ADR-021). The
// RunSummary metrics are copied from the run, never computed again. Definitions:
// - Path error: position |p_d - p| and the rotation angle of R_d R^T (T_b_d, T_b_e of the row),
//   over the rows in tracking or finished (holding, fault, standby and idle rows excluded).
// - Settle time (final goal): t0 = first row + meta.goal_time_s; the error to the goal pose (T_b_d
//   of the last row with t <= window_end_s, the RunSummary evaluation row); settle = time from t0
//   to the last entry into {pos <= goal_pos_m and rot <= goal_rot_rad} after which the run stays
//   inside until that row (0 when inside from t0); nullopt when not inside at that row, when the
//   run ends before t0, or when the run held. Waypoint and config modes alike.
// - Stop waypoints (every stop but the last waypoint): t_arrive = first row + node time; window =
//   rows from the one nearest t_arrive to the one nearest t_arrive + kStopSettleS, both included
//   (cobot_eval STOP_SETTLE_S); error to the waypoint: position to meta.waypoint_positions_m,
//   rotation to T_b_d of the row nearest t_arrive (the reference rests on the waypoint there).
//   pos_err_m / rot_err_rad = the minimum error in the window; time_to_enter_s = time of the
//   first window row within the goal tolerance, relative to t_arrive; nullopt = "not entered".
// - goal_ok: the summary's final errors within the goal tolerance; path_ok: both path maxima
//   within the path tolerance.
namespace cobot::studio {

inline constexpr double kStopSettleS = 0.2;  // cobot_eval.metrics.STOP_SETTLE_S

struct StopMetrics {
  std::size_t waypoint = 0;
  double t_arrive_s = 0.0;
  double pos_err_m = 0.0;
  double rot_err_rad = 0.0;
  std::optional<double> time_to_enter_s;  // nullopt = "not entered"
};

struct TrackingMetrics {
  bench::RunSummary summary;
  double path_pos_err_max_m = 0.0;  // tracking/finished rows
  double path_rot_err_max_rad = 0.0;
  double path_pos_err_rms_m = 0.0;
  std::optional<double> settle_time_s;  // final goal; nullopt = not reached
  std::vector<StopMetrics> stops;       // intermediate stop waypoints
  bool goal_ok = false;
  bool path_ok = false;
};

// Per-row errors of one stop window (rows k0 .. k0 + size - 1).
struct StopTrace {
  std::size_t waypoint = 0;
  double t_arrive_s = 0.0;
  std::vector<double> t_s, pos_m, rot_rad;
};

// Per-row errors kept for every run (also summary-only sweep seeds), so a tolerance edit
// recomputes every metric without the records.
struct ErrorTrace {
  std::vector<double> t_s, path_pos_m, path_rot_rad, goal_pos_m, goal_rot_rad;
  std::vector<std::uint8_t> on_path;  // 1: tracking or finished row
  std::ptrdiff_t eval_row = -1;       // last row with t <= window_end_s, -1 when none
  std::vector<StopTrace> stops;
};

ErrorTrace make_error_trace(const bench::RunData& run);
TrackingMetrics compute_metrics(const ErrorTrace& errors,
                                const bench::RunMeta& meta,
                                const bench::RunSummary& summary,
                                const Tolerance& tolerance);
TrackingMetrics compute_metrics(const bench::RunData& run, const Tolerance& tolerance);

// The table row of one sweep level: worst case over its seeds (max of every error and time,
// nullopt settle when any seed has none, ok flags only when every seed is ok).
struct LevelRow {
  std::string param;
  double level = 0.0;
  std::size_t seeds = 0;
  std::size_t held = 0;
  TrackingMetrics worst;
};
LevelRow worst_case(const std::vector<const RunEntry*>& same_level, const Tolerance& tolerance);

}  // namespace cobot::studio
