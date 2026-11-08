#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/pipeline/log_record.hpp"

#include "engine.hpp"
#include "run_data.hpp"

// RunSummary from recorded rows and the bench's output files (feature studio-core): shared by
// cobot_bench run|montecarlo|sweep and the studio, so both give the same numbers and bytes.
namespace cobot::bench {

// Torque the peak, saturation and rms torque metrics use: the commanded `tau` (torque laws) or
// the plant's applied `tau_motor` (drive_position, where `tau` is only the feed-forward sent with
// the set point and the drive's PD output reaches the motor).
enum class TorqueSource { kCommand, kMotor };

// RunSummary of `rows` (definitions in engine.hpp); t_eval_ns is the evaluation time of the
// final errors (duration_s + hold_s in ns, absolute row time).
RunSummary summarize(const std::vector<pipeline::LogRecord<kN>>& rows,
                     const Vec& tau_max,
                     const Vec& rated,
                     std::int64_t t_eval_ns,
                     TorqueSource torque = TorqueSource::kCommand);

// The same, with the post-goal swing: t_goal_ns is the time the reference reaches the final goal
// (absolute row time).
RunSummary summarize(const std::vector<pipeline::LogRecord<kN>>& rows,
                     const Vec& tau_max,
                     const Vec& rated,
                     std::int64_t t_eval_ns,
                     TorqueSource torque,
                     std::int64_t t_goal_ns);

// The same, with RunSummary::settle_time_s when `settle` is given (NaN otherwise; no other field
// changes). Reading of the task's cycle_time_s (ADR-023, ambiguity 11): the time from the first row
// until the TCP enters and stays within `settle` around the final goal, the desired pose T_b_d of
// the evaluation row (the row of final_pos_err_m), on every row from that instant up to the
// evaluation row. The last entry counts; outside at the evaluation row (never settled) or a held
// run is NaN; inside from the first row is 0.
RunSummary summarize(const std::vector<pipeline::LogRecord<kN>>& rows,
                     const Vec& tau_max,
                     const Vec& rated,
                     std::int64_t t_eval_ns,
                     TorqueSource torque,
                     std::int64_t t_goal_ns,
                     const std::optional<SettleThresholds>& settle);

// The Run CSV of `run` and `<out_csv>.meta.yaml`, the bytes `cobot_bench run` writes.
common::Status write_run_files(const RunData& run, const std::filesystem::path& out_csv);

// One Summary CSV row (docs/conventions.md).
struct SummaryRow {
  std::size_t run_id = 0;
  std::string param;
  double level = 0.0;
  std::uint64_t seed = 0;
  RunSummary summary;
};

void write_summary_header(std::ostream& os);
void write_summary_row(std::ostream& os, const SummaryRow& row);

}  // namespace cobot::bench
