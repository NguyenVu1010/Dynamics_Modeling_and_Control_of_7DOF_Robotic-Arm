#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/task.hpp"

#include "engine.hpp"
#include "tune.hpp"

// Verify (feature verify, ADR-023 "Kết luận ba mức từ Monte Carlo"): the tune step, then Monte
// Carlo runs of a hardware model, an algorithm and a task, each judged against the task's
// requirements, and the three-level verdict. The report files are report.hpp.
namespace cobot::bench {

struct Interval {
  double lo = 0.0;
  double hi = 1.0;
};

// Wilson score interval of successes / n (z = 1.96: 95 % two-sided), clamped to [0, 1];
// n == 0 -> {0, 1}.
Interval wilson(std::size_t successes, std::size_t n, double z = 1.959963984540054);

enum class Verdict { kPass, kFail, kMarginal };
const char* to_string(Verdict v);  // "pass" | "fail" | "marginal"

struct Counts {
  std::size_t runs = 0;
  std::size_t passed = 0;
  std::size_t held = 0;
};

// The worst value over the runs that did not hold (NaN when every run held);
// margin_limit = (1 - margin_frac) * threshold.
struct WorstMetric {
  std::string name;
  double value = 0.0;
  double threshold = 0.0;
  double margin_limit = 0.0;
};

// FAIL when wilson(passed).hi < target_pass_rate or wilson(held).lo > max_hold_rate. Else PASS
// when not quick (quick: the flag or runs < 100), wilson(passed).lo >= target_pass_rate,
// held / runs <= max_hold_rate and every worst value <= its margin_limit (NaN never is). Else
// MARGINAL. The one verdict function of the CLI and the studio.
Verdict verdict(const Counts& counts,
                const std::vector<WorstMetric>& worst,
                const pipeline::TaskRequirements& requirements,
                bool quick);

// Per-run pass (ambiguity 13): not held, and final_pos_err_m, final_rot_err_rad (in degrees),
// rms_joint_err_rad and settle_time_s each <= its requirement (cycle_time_s for the settle time).
// NaN fails.
bool run_passes(const RunSummary& summary, const pipeline::TaskRequirements& requirements);

enum class GainSource { kTuned, kDefaultNoAdmissible, kDefaultTuningOff };
// "tuned" | "default_no_admissible" | "default_tuning_off"
const char* to_string(GainSource source);

// A provenance entry of kind assumed or derived (the seam of the sensitivity analysis, PR 6b).
struct Assumption {
  std::string key;
  pipeline::ProvenanceKind kind = pipeline::ProvenanceKind::kAssumed;
  std::optional<std::array<double, 2>> range;
  std::optional<std::array<double, 2>> scale;
  std::string sensitivity = "not run";
};

struct SeedRun {
  std::uint64_t seed = 0;
  RunSummary summary;
  bool passed = false;  // run_passes
};

struct VerifyOptions {
  std::size_t runs = 100;
  std::uint64_t seed0 = 1;
  bool quick = false;
  bool tune = true;
  TuneOptions tuning;  // seeds empty: 1..10; levels empty: default_levels; jobs 0: `jobs`
  unsigned jobs = 0;   // 0: as TuneOptions
  // Called between runs only, never from two threads at once; may be empty.
  std::function<void(std::string_view phase, std::size_t done, std::size_t total)> progress;
  // Set: no further run starts and verify returns the error "cancelled".
  const std::atomic<bool>* cancel = nullptr;
};

struct VerifyResult {
  control::Algorithm algorithm = control::Algorithm::kComputedTorque;
  std::string model;
  std::string task;
  std::string gain_key;  // tuning_key(algorithm)
  double gain_used = 0.0;
  GainSource gain_source = GainSource::kTuned;
  std::optional<TuneResult> tuning;  // absent with tuning off
  std::vector<SeedRun> runs;         // seed order
  Counts counts;
  double pass_rate = 0.0;
  Interval wilson;  // of the pass count
  double hold_rate = 0.0;
  // final_pos_err_m, final_rot_err_deg, rms_joint_err_rad, cycle_time_s (the max settle time).
  std::vector<WorstMetric> worst;
  Verdict verdict = Verdict::kFail;
  bool quick = false;  // the flag or runs < 100
  std::vector<Assumption> assumptions;
  std::vector<std::string> causes;       // PR 6b
  std::vector<std::string> suggestions;  // PR 6b
};

// tune (unless options.tune is false), then options.runs runs with seeds seed0.. on run_queue,
// stored by seed index. An algorithm the model does not list is an error before any run; the first
// error of a run stops the remaining work and comes back.
common::Result<VerifyResult> verify(const pipeline::HardwareModel& model,
                                    control::Algorithm algorithm,
                                    const pipeline::Task& task,
                                    const VerifyOptions& options);

}  // namespace cobot::bench
