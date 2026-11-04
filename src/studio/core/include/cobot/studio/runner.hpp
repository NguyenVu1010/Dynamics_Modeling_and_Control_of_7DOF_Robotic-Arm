#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/studio/run_store.hpp"
#include "cobot/studio/studio_config.hpp"

// Studio runs on the bench engine in process (feature studio-core, cobot_studio design 6.3,
// ADR-021). Each run materialises its inputs as <work_dir>/<run-id>/{config.yaml,
// imperfections.yaml, studio.yaml} (rerunnable with cobot_bench) and writes nothing else; the
// work directory is the caller's, never the repository.
namespace cobot::studio {

using ProgressFn = std::function<void(std::size_t done, std::size_t total)>;
using ResultFn = std::function<void(RunEntry&&)>;

// validate, materialise, run (bench::run_in_memory) and keep the records; the entry's id is 0
// until a RunStore adds it.
common::Result<RunEntry> run_once(const StudioConfig& config, const fs::path& work_dir);

struct SweepRequest {
  StudioConfig base;
  std::string param;  // a sweepable catalog key
  std::vector<double> levels;
  std::vector<std::uint64_t> seeds{1, 2, 3};
};

class SweepRunner {
 public:
  // Runs one at a time: max_parallel other than 1 throws std::invalid_argument.
  explicit SweepRunner(fs::path work_dir, int max_parallel = 1);

  // Blocking (worker thread). Validates the param and every level first (catalog limits, the
  // bench's check_comm, loader errors) and returns the error before any run. Order: level-major,
  // then seed. Records kept only for seeds.front(); `cancel` is checked between runs (returns ok
  // with fewer results). on_result and on_progress run on the calling thread.
  common::Status run(const SweepRequest& request,
                     const std::atomic<bool>& cancel,
                     const ResultFn& on_result,
                     const ProgressFn& on_progress);

 private:
  fs::path work_dir_;
};

}  // namespace cobot::studio
