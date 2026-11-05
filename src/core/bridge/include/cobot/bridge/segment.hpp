#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>

#include "cobot/common/result.hpp"

// Segment export of the studio bridge (feature controller-bridge, plan ambiguity 6): a segment is
// a copy of rows already in the Run CSV, so the exactly-once export of the Run CSV is untouched.
namespace cobot::bridge {

struct SegmentCopy {
  std::size_t rows = 0;
  std::int64_t first_t_ns = 0, last_t_ns = 0;
  bool truncated = false;
};

// Copies the header line of run_csv and, from byte `offset` on (0 or the start of a line), every
// row whose first field t_ns lies in [t0_ns, t1_ns], in file order, to out_csv (replaced). Only
// the leading integer of each line is parsed. truncated = the first copied row is later than
// t0_ns + period_ns (older rows were overwritten before any export). No row -> error "segment: no
// row in [t0, t1]"; errors name the files.
common::Result<SegmentCopy> copy_csv_range(const std::filesystem::path& run_csv,
                                           std::uint64_t offset,
                                           std::int64_t t0_ns,
                                           std::int64_t t1_ns,
                                           std::int64_t period_ns,
                                           const std::filesystem::path& out_csv);

}  // namespace cobot::bridge
