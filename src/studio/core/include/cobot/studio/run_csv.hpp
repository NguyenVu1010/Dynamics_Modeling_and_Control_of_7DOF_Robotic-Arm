#pragma once

#include <filesystem>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/pipeline/log_record.hpp"

#include "run_data.hpp"

// The Run CSV read back (feature studio-control; contract in docs/conventions.md "Run CSV"): the
// segments a host exports for a live run. Not on the hot path (allocates).
namespace cobot::studio {

namespace fs = std::filesystem;

// 114 columns for N = 7, or the legacy 57 (up to `hold`: truth columns NaN, qraw = q, qddref 0,
// cmd config). T_b_e / T_b_d from p and so3_exp(r); mode, hold and cmd strings mapped back to the
// enums. Errors name the file, the 1-based line and the column.
common::Result<std::vector<pipeline::LogRecord<bench::kN>>> read_run_csv(const fs::path& csv);

}  // namespace cobot::studio
