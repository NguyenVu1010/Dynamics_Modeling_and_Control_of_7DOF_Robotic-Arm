#include "cobot/studio/live_run.hpp"

#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>

#include "cobot/studio/metrics.hpp"
#include "cobot/studio/run_csv.hpp"

#include "metrics.hpp"

namespace cobot::studio {

common::Result<RunEntry> make_live_entry(const fs::path& segment_csv,
                                         const LiveRunInfo& info,
                                         const bridge::SessionInfo<bench::kN>& session,
                                         int number) {
  using Out = common::Result<RunEntry>;
  common::Result<std::vector<pipeline::LogRecord<bench::kN>>> rows = read_run_csv(segment_csv);
  if (!rows) {
    return Out::error(rows.reason());
  }
  if (rows.value().empty()) {
    return Out::error(segment_csv.string() + ": no rows");
  }
  auto data = std::make_shared<bench::RunData>();
  data->records = std::move(rows.value());
  bench::RunMeta& meta = data->meta;
  meta.config_yaml = session.config_path;
  meta.imperfections_label = session.imperfections_path;
  meta.seed = session.seed;
  for (int j = 0; j < bench::kN; ++j) {
    meta.tau_max(j) = session.tau_max[static_cast<std::size_t>(j)];
    meta.rated(j) = session.rated[static_cast<std::size_t>(j)];
  }
  meta.period_s = session.control_rate_hz > 0.0 ? 1.0 / session.control_rate_hz : 0.0;
  const std::int64_t t0 = data->records.front().t.ns;
  meta.goal_time_s = static_cast<double>(info.t_end_ns - t0) * 1e-9;
  meta.window_end_s = meta.goal_time_s + info.settle_window_s;
  double node = 0.0;
  for (const double d : info.segment_durations_s) {
    node += d;
    meta.waypoint_node_times_s.push_back(node);
  }
  meta.waypoint_stop = info.waypoint_stop;
  meta.waypoint_positions_m = info.waypoint_positions_m;
  // t_eval = first row + window_end_s, in integer ns: t_end + the settle window.
  const std::int64_t t_eval = info.t_end_ns + std::llround(info.settle_window_s * 1e9);
  data->summary = bench::summarize(data->records, meta.tau_max, meta.rated, t_eval);

  RunEntry e;
  e.label = std::string("live ") + pipeline::to_string(info.cmd) + " #" + std::to_string(number);
  if (info.truncated) {
    e.label += " (truncated)";
    e.note = "the segment starts after the motion did: older rows were overwritten in the log";
  }
  e.kind = RunKind::kGazebo;
  e.config = info.config;
  e.inputs = Materialized{session.config_path, session.imperfections_path};
  e.diff = describe_diff(info.config);
  e.meta = meta;
  e.summary = data->summary;
  e.errors = std::make_shared<const ErrorTrace>(make_error_trace(*data));
  e.data = std::move(data);
  return Out::ok(std::move(e));
}

}  // namespace cobot::studio
