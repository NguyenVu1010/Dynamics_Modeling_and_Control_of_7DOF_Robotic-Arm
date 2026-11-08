#pragma once

#include <atomic>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <string_view>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/control/algorithm.hpp"

#include "compare.hpp"  // bench
#include "verify.hpp"   // bench

// The studio's verify job (feature studio-verify, cobot_studio design 10 G14/G16, ADR-023): the
// same loading, checks, engine (bench::compare) and writer (bench::write_verify_outputs) as
// `cobot_bench verify`, so the studio and the CLI never disagree. No Qt.
namespace cobot::studio {

// phase: the bench phase of VerifyOptions::progress ("tune", "runs", "sensitivity", "ablation");
// pair: 0-based index of the running pair among the listed pairs (the ones that run), pairs: their
// number; done / total: as VerifyOptions::progress.
using VerifyProgressFn = std::function<void(std::string_view phase,
                                            std::size_t pair,
                                            std::size_t pairs,
                                            std::size_t done,
                                            std::size_t total)>;

struct VerifyRequest {
  std::vector<std::filesystem::path> hardware;  // config/hardware/<model>.yaml files
  std::vector<control::Algorithm> algorithms;
  std::filesystem::path task;     // config/tasks/<task>.yaml
  bench::VerifyOptions options;   // progress and cancel are the job's own
  std::filesystem::path out_dir;  // where the outputs go; empty: an error
};

struct VerifyOutcome {
  std::vector<bench::ComparisonRow> rows;    // bench::compare's rows
  std::vector<std::filesystem::path> files;  // every file written, each out_dir / <relative>
};

// Loads the task and the models and checks them as `cobot_bench verify` does, with the same
// messages, all before the first run; then bench::compare; then bench::write_verify_outputs into
// out_dir (1x1 call -> report.{json,md} (+ tune.csv) in out_dir itself). `cancel` set at any time
// -> the error "cancelled" and no file. progress may be empty; it is called between runs, never
// from two threads at once.
common::Result<VerifyOutcome> run_verify_job(const VerifyRequest& request,
                                             const std::atomic<bool>& cancel,
                                             const VerifyProgressFn& progress);

// A new, empty <work_dir>/verify_NNNN (the first free number from 0001, like run_NNNN); work_dir
// is created when missing.
common::Result<std::filesystem::path> new_verify_dir(const std::filesystem::path& work_dir);

}  // namespace cobot::studio
