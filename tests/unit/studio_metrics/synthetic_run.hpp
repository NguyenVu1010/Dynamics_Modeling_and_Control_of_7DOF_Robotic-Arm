#pragma once

// Synthetic bench::RunData for the studio metric tests (feature studio-core): rows every 1 ms from
// t = 0 to t = 3.0 s (3001 rows), desired pose T_b_d = identity at the origin, actual pose T_b_e =
// identity rotation at (err(k), 0, 0), so the position error of row k is exactly err(k) and every
// rotation error is 0. The summary comes from bench::summarize (t_eval = window_end_s), as a run
// of run_in_memory would carry it.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "cobot/common/timestamp.hpp"
#include "cobot/common/types.hpp"
#include "cobot/pipeline/log_record.hpp"

#include "metrics.hpp"
#include "run_data.hpp"

namespace cobot::studio::test {

constexpr int kJoints = 7;
constexpr std::int64_t kRowNs = 1000000;  // 1 ms
constexpr std::size_t kRows = 3001;       // t = 0 .. 3.0 s
constexpr double kGoalTimeS = 1.0;
constexpr double kWindowEndS = 3.0;
constexpr double kMm = 1e-3;  // m per mm

using ErrFn = std::function<double(std::size_t k)>;               // m, row k
using ModeFn = std::function<pipeline::LoopMode(std::size_t k)>;  // mode of row k

inline double t_of(std::size_t k) {  // s
  return static_cast<double>(k) * 1e-3;
}

// Tracking before goal_time, finished after.
inline pipeline::LoopMode default_mode(std::size_t k) {
  return t_of(k) < kGoalTimeS ? pipeline::LoopMode::kTracking : pipeline::LoopMode::kFinished;
}

inline bench::RunMeta synthetic_meta() {
  bench::RunMeta meta;
  meta.seed = 1;
  meta.q_offset_rad = common::VecN<kJoints>::Zero();
  meta.torque_scale = common::VecN<kJoints>::Ones();
  meta.plant_link_mass_kg = common::VecN<kJoints>::Ones();
  meta.tau_max = common::VecN<kJoints>::Ones();
  meta.rated = common::VecN<kJoints>::Ones();
  meta.period_s = 1e-3;
  meta.goal_time_s = kGoalTimeS;
  meta.window_end_s = kWindowEndS;
  return meta;
}

inline bench::RunData synthetic_run(const ErrFn& err,
                                    const ModeFn& mode = default_mode,
                                    const bench::RunMeta& meta = synthetic_meta()) {
  bench::RunData run;
  run.meta = meta;
  run.records.resize(kRows);
  for (std::size_t k = 0; k < kRows; ++k) {
    pipeline::LogRecord<kJoints>& r = run.records[k];
    r.t = common::Timestamp{static_cast<std::int64_t>(k) * kRowNs};
    r.mode = mode(k);
    r.hold_reason = r.mode == pipeline::LoopMode::kHolding ? pipeline::HoldReason::kStateTimeout
                                                           : pipeline::HoldReason::kNone;
    r.T_b_d = common::Pose{};
    r.T_b_e = common::Pose{};
    r.T_b_e.p = common::Vec3(err(k), 0.0, 0.0);
  }
  const std::int64_t t_eval_ns = std::llround(meta.window_end_s * 1e9);  // 3.0 s -> 3e9 ns
  run.summary = bench::summarize(run.records, meta.tau_max, meta.rated, t_eval_ns);
  return run;
}

}  // namespace cobot::studio::test
