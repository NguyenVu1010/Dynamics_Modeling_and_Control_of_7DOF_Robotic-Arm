#include "cobot/studio/playback.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace cobot::studio {

std::size_t row_at(const bench::RunData& run, double t_s) {
  const std::vector<pipeline::LogRecord<model::kArmDof>>& rows = run.records;
  if (rows.empty() || !(t_s > -1e9)) {
    return 0;
  }
  // Integer nanoseconds (docs/conventions.md: never compare floating-point times).
  constexpr double kMaxS = 9e9;  // keeps llround inside int64
  const std::int64_t t_ns = std::llround(std::min(t_s, kMaxS) * 1e9);
  const auto after = std::upper_bound(
      rows.begin(), rows.end(), t_ns, [](std::int64_t t, const auto& r) { return t < r.t.ns; });
  return after == rows.begin() ? 0 : static_cast<std::size_t>(after - rows.begin()) - 1;
}

const common::VecN<model::kArmDof>& display_q(const pipeline::LogRecord<model::kArmDof>& row) {
  return row.q_true.allFinite() ? row.q_true : row.q;
}

Traces make_traces(const RobotScene& scene, const bench::RunData& run) {
  Traces traces;
  traces.tip.reserve(run.records.size());
  traces.desired.reserve(run.records.size());
  for (const pipeline::LogRecord<model::kArmDof>& row : run.records) {
    traces.tip.push_back(scene.tip_pose(display_q(row)).p);
    traces.desired.push_back(row.T_b_d.p);
  }
  traces.goals = run.meta.waypoint_positions_m;
  if (traces.goals.empty() && !run.records.empty()) {
    // The reference rests on the final goal from first row + goal_time_s (metrics.hpp). A run that
    // ends before then has no final goal to draw.
    const std::int64_t t_goal_ns =
        run.records.front().t.ns + std::llround(run.meta.goal_time_s * 1e9);
    if (run.records.back().t.ns >= t_goal_ns) {
      const std::size_t k = row_at(run, static_cast<double>(t_goal_ns) * 1e-9);
      traces.goals.push_back(run.records[k].T_b_d.p);
    }
  }
  if (run.summary.held) {
    traces.hold_t_s = run.summary.hold_time_s;
  }
  return traces;
}

}  // namespace cobot::studio
