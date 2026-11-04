#pragma once

#include <cstdint>
#include <filesystem>
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

// RunSummary of `rows` (definitions in engine.hpp); t_eval_ns is the evaluation time of the
// final errors (duration_s + hold_s in ns, absolute row time).
RunSummary summarize(const std::vector<pipeline::LogRecord<kN>>& rows,
                     const Vec& tau_max,
                     const Vec& rated,
                     std::int64_t t_eval_ns);

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
