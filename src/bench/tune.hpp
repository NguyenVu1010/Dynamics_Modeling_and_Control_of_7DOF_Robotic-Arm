#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/overrides.hpp"
#include "cobot/pipeline/task.hpp"

#include "engine.hpp"

// Auto-tune (feature auto-tune, ADR-023 "Tự chỉnh hệ số trước khi kết luận"): for a hardware model,
// an algorithm and a task, the law's one gain is swept over a few seeds and the best admissible
// level is chosen. No verdict here (that is verify).
namespace cobot::bench {

// The one gain of each law: config.computed_torque.omega, config.pd_gravity.omega (both virtual:
// kp = omega^2, kd = 2 omega) and config.drive.omega_rad_s.
const char* tuning_key(control::Algorithm algorithm);

// 20..150 step 10 for the torque laws, 20..200 step 10 for drive_position (rad/s).
std::vector<double> default_levels(control::Algorithm algorithm);

// `overrides` without what already sets the gain of `algorithm` (its key and, for a virtual omega
// key, the kp and kd of its section), plus tuning_key(algorithm) = level. Other keys stay in order.
std::vector<pipeline::KeyOverride> with_gain(std::vector<pipeline::KeyOverride> overrides,
                                             control::Algorithm algorithm,
                                             double level);

// Range of q - q_ref at rest in a 0.5 s window: encoder quantisation and noise (2.9e-4 to 4.1e-4
// rad measured in the drive-loop PR).
inline constexpr double kSwingNoiseFloorRad = 5.0e-4;

// The post-goal swing decays: last <= first, or last <= kSwingNoiseFloorRad. NaN never decays.
bool swing_decays(double first, double last);

// One level of the table, over its seeds.
struct TuneRow {
  double level = 0.0;
  std::size_t seeds = 0;
  std::size_t held = 0;               // seeds that held
  double sat_frac_max = 0.0;          // max over seeds of sat_time_s / run_time_s
  bool decays_all = false;            // every seed decays (a held seed does not)
  double rms_joint_err_median = 0.0;  // over the seeds that did not hold; NaN when none
  double final_pos_err_max = 0.0;     // same seeds; NaN when none
};

struct TuneResult {
  control::Algorithm algorithm = control::Algorithm::kComputedTorque;
  std::string key;  // tuning_key(algorithm)
  std::vector<double> levels;
  std::vector<TuneRow> rows;  // one per level, in the order of `levels`
  std::optional<double> chosen;
  std::string reason;  // one line, for the report
};

// The row of `level` from the RunSummary of each seed.
TuneRow tune_row(double level, const std::vector<RunSummary>& seeds);

// Admissible: held == 0, sat_frac_max < 0.05, decays_all and a rms that is not NaN. The lowest
// rms_joint_err_median among those; an exact tie goes to the lower level. None: nullopt.
std::optional<std::size_t> select_gain(const std::vector<TuneRow>& rows);

// One line: the chosen level and why, or "no admissible gain" with the number of levels failing
// each criterion (a level can fail several).
std::string tune_reason(const std::vector<TuneRow>& rows, const std::optional<std::size_t>& chosen);

struct TuneOptions {
  std::vector<double> levels;        // empty: default_levels
  std::vector<std::uint64_t> seeds;  // empty: 1..10
  unsigned jobs = 0;                 // 0: min(hardware_concurrency, 8)
  // Set: no further run starts and tune returns the error "cancelled" (read between runs only).
  const std::atomic<bool>* cancel = nullptr;
  // May be empty: called after each run with the runs done and the total, one call at a time.
  std::function<void(std::size_t done, std::size_t total)> progress;
};

// Every (level, seed) run of `algorithm` on `model` and `task` (pipeline::run_overrides plus
// with_gain; the profile's seed set per run), on worker threads (run_queue). An algorithm the model
// does not list and a level the loader refuses are errors before any run; the first error of a
// run (a rejected waypoint list included) is returned and the remaining work stops.
common::Result<TuneResult> tune(const pipeline::HardwareModel& model,
                                control::Algorithm algorithm,
                                const pipeline::Task& task,
                                const TuneOptions& options);

// tune.csv (docs/conventions.md "Tune CSV"): one line per row, %.17g, flags 0/1.
common::Status write_tune_csv(const std::filesystem::path& path, const TuneResult& result);

}  // namespace cobot::bench
