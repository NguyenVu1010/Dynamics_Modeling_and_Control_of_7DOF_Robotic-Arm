#include "verify.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <thread>
#include <utility>
#include <variant>

#include "cobot/imperfect/config.hpp"
#include "cobot/pipeline/config.hpp"

#include "run_data.hpp"
#include "work_queue.hpp"

namespace cobot::bench {
namespace {

using control::Algorithm;
using pipeline::KeyOverride;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegPerRad = 180.0 / kPi;
constexpr std::size_t kFullRuns = 100;  // fewer runs is quick mode
constexpr unsigned kMaxDefaultJobs = 8;
const double kNan = std::numeric_limits<double>::quiet_NaN();

double ratio(std::size_t k, std::size_t n) {
  return n == 0 ? kNan : static_cast<double>(k) / static_cast<double>(n);
}

// The model's own gain: its override of tuning_key, else the loaded controller's value (joint 0).
double own_gain(const std::vector<KeyOverride>& overrides,
                const pipeline::Config<kN>& config,
                Algorithm algorithm) {
  for (const KeyOverride& o : overrides) {
    if (o.key == tuning_key(algorithm) && std::holds_alternative<double>(o.value)) {
      return std::get<double>(o.value);
    }
  }
  switch (algorithm) {
    case Algorithm::kComputedTorque:
      return std::sqrt(config.computed_torque.kp(0));
    case Algorithm::kPdGravity:
      return std::sqrt(config.pd_gravity.kp(0));
    case Algorithm::kDrivePosition:
      return config.drive.omega_rad_s(0);
  }
  return kNan;
}

// Max over the runs that did not hold of `metric`; NaN when none, or when one is NaN. A run that
// did not hold but never settled has a NaN settle time: the worst cycle time is then unknown (null
// in the report), and that run already fails run_passes, so PASS is blocked either way.
template <typename Metric>
double worst_of(const std::vector<SeedRun>& runs, Metric metric) {
  double worst = kNan;
  bool any = false;
  for (const SeedRun& r : runs) {
    if (r.summary.held) {
      continue;
    }
    const double v = metric(r.summary);
    if (!any || (!std::isnan(worst) && (std::isnan(v) || v > worst))) {
      worst = v;
    }
    any = true;
  }
  return worst;
}

}  // namespace

Interval wilson(std::size_t successes, std::size_t n, double z) {
  if (n == 0) {
    return Interval{0.0, 1.0};
  }
  const auto nd = static_cast<double>(n);
  const double p = static_cast<double>(successes) / nd;
  const double z2 = z * z;
  const double denom = 1.0 + z2 / nd;
  const double centre = (p + z2 / (2.0 * nd)) / denom;
  const double half = z * std::sqrt(p * (1.0 - p) / nd + z2 / (4.0 * nd * nd)) / denom;
  return Interval{std::max(0.0, centre - half), std::min(1.0, centre + half)};
}

const char* to_string(Verdict v) {
  switch (v) {
    case Verdict::kPass:
      return "pass";
    case Verdict::kMarginal:
      return "marginal";
    case Verdict::kFail:
      return "fail";
  }
  return "fail";
}

const char* to_string(GainSource source) {
  switch (source) {
    case GainSource::kTuned:
      return "tuned";
    case GainSource::kDefaultNoAdmissible:
      return "default_no_admissible";
    case GainSource::kDefaultTuningOff:
      return "default_tuning_off";
  }
  return "tuned";
}

Verdict verdict(const Counts& counts,
                const std::vector<WorstMetric>& worst,
                const pipeline::TaskRequirements& requirements,
                bool quick) {
  const Interval passed = wilson(counts.passed, counts.runs);
  const Interval held = wilson(counts.held, counts.runs);
  if (passed.hi < requirements.target_pass_rate || held.lo > requirements.max_hold_rate) {
    return Verdict::kFail;
  }
  const bool in_margin = std::all_of(worst.begin(), worst.end(), [](const WorstMetric& w) {
    return w.value <= w.margin_limit;  // NaN: false
  });
  const bool full = !quick && counts.runs >= kFullRuns;
  if (full && passed.lo >= requirements.target_pass_rate &&
      ratio(counts.held, counts.runs) <= requirements.max_hold_rate && in_margin) {
    return Verdict::kPass;
  }
  return Verdict::kMarginal;
}

bool run_passes(const RunSummary& s, const pipeline::TaskRequirements& r) {
  return !s.held && s.final_pos_err_m <= r.final_pos_err_m &&
         s.final_rot_err_rad * kDegPerRad <= r.final_rot_err_deg &&
         s.rms_joint_err_rad <= r.rms_joint_err_rad && s.settle_time_s <= r.cycle_time_s;
}

common::Result<VerifyResult> verify(const pipeline::HardwareModel& model,
                                    Algorithm algorithm,
                                    const pipeline::Task& task,
                                    const VerifyOptions& options) {
  using Out = common::Result<VerifyResult>;
  const common::Result<pipeline::Config<kN>> base = pipeline::config_for<kN>(model, algorithm);
  if (!base) {
    return Out::error(base.reason());
  }
  if (options.runs == 0) {
    return Out::error("verify: runs must be >= 1");
  }
  const auto cancelled = [&options]() {
    return options.cancel != nullptr && options.cancel->load();
  };
  const auto progress = [&options](std::string_view phase, std::size_t done, std::size_t total) {
    if (options.progress) {
      options.progress(phase, done, total);
    }
  };
  if (cancelled()) {
    return Out::error("cancelled");
  }
  VerifyResult result;
  result.algorithm = algorithm;
  result.model = model.name;
  result.task = task.name;
  result.gain_key = tuning_key(algorithm);
  result.quick = options.quick || options.runs < kFullRuns;
  const pipeline::RunOverrides ro = pipeline::run_overrides(model, algorithm, task);
  std::vector<KeyOverride> overrides = ro.config;
  const common::Result<pipeline::Config<kN>> own =
      pipeline::load_config_with<kN>(model.controller, ro.config);
  if (!own) {
    return Out::error(model.name + ": " + own.reason());
  }
  result.gain_used = own_gain(ro.config, own.value(), algorithm);
  result.gain_source = GainSource::kDefaultTuningOff;
  if (options.tune) {
    TuneOptions tuning = options.tuning;
    tuning.jobs = tuning.jobs == 0 ? options.jobs : tuning.jobs;
    tuning.cancel = options.cancel;
    tuning.progress = [&progress](std::size_t done, std::size_t total) {
      progress("tune", done, total);
    };
    const std::size_t levels =
        tuning.levels.empty() ? default_levels(algorithm).size() : tuning.levels.size();
    const std::size_t tune_runs = levels * (tuning.seeds.empty() ? 10 : tuning.seeds.size());
    progress("tune", 0, tune_runs);
    common::Result<TuneResult> tuned = tune(model, algorithm, task, tuning);
    if (!tuned) {
      return Out::error(tuned.reason());
    }
    result.tuning = std::move(tuned.value());
    result.gain_source = GainSource::kDefaultNoAdmissible;
    if (result.tuning->chosen) {
      result.gain_source = GainSource::kTuned;
      result.gain_used = *result.tuning->chosen;
      overrides = with_gain(ro.config, algorithm, result.gain_used);
    }
  }
  common::Result<imperfect::ImperfectionConfig<kN>> profile =
      pipeline::load_imperfections_with<kN>(model.imperfections, ro.imperfections);
  if (!profile) {
    return Out::error(model.name + ": " + profile.reason());
  }
  const pipeline::TaskRequirements& req = task.requirements;
  const SettleThresholds band{req.final_pos_err_m, req.final_rot_err_deg / kDegPerRad};
  result.runs.resize(options.runs);
  unsigned jobs = options.jobs;
  if (jobs == 0) {
    jobs = std::max(1U, std::min(std::thread::hardware_concurrency(), kMaxDefaultJobs));
  }
  std::mutex mutex;
  std::size_t done = 0;
  progress("runs", 0, options.runs);
  // Each run keeps its rows only until it is summarised (run_with); cancel is read between runs.
  const common::Status ran = run_queue(options.runs, jobs, [&](std::size_t i) {
    if (cancelled()) {
      return common::Status::error("cancelled");
    }
    imperfect::ImperfectionConfig<kN> p = profile.value();
    p.seed = options.seed0 + i;
    std::optional<WaypointInput> waypoints;
    if (ro.waypoints) {
      waypoints = WaypointInput{*ro.waypoints};
    }
    common::Result<RunSummary> run = run_with(
        model.controller, overrides, p, model.name, waypoints ? &*waypoints : nullptr, band);
    if (!run) {
      return common::Status::error(model.name + " seed " + std::to_string(p.seed) + ": " +
                                   run.reason());
    }
    SeedRun& slot = result.runs[i];
    slot.seed = p.seed;
    slot.summary = run.value();
    slot.passed = run_passes(slot.summary, req);
    const std::lock_guard<std::mutex> lock(mutex);
    progress("runs", ++done, options.runs);
    return common::Status::ok();
  });
  if (!ran) {
    return Out::error(ran.reason());
  }
  for (const SeedRun& r : result.runs) {
    result.counts.passed += r.passed ? 1 : 0;
    result.counts.held += r.summary.held ? 1 : 0;
  }
  result.counts.runs = result.runs.size();
  result.pass_rate = ratio(result.counts.passed, result.counts.runs);
  result.hold_rate = ratio(result.counts.held, result.counts.runs);
  result.wilson = wilson(result.counts.passed, result.counts.runs);
  const double m = 1.0 - req.margin_frac;
  const auto add = [&](const char* name, double threshold, double value) {
    result.worst.push_back(WorstMetric{name, value, threshold, m * threshold});
  };
  add("final_pos_err_m", req.final_pos_err_m, worst_of(result.runs, [](const RunSummary& s) {
        return s.final_pos_err_m;
      }));
  add("final_rot_err_deg", req.final_rot_err_deg, worst_of(result.runs, [](const RunSummary& s) {
        return s.final_rot_err_rad * kDegPerRad;
      }));
  add("rms_joint_err_rad", req.rms_joint_err_rad, worst_of(result.runs, [](const RunSummary& s) {
        return s.rms_joint_err_rad;
      }));
  add("cycle_time_s", req.cycle_time_s, worst_of(result.runs, [](const RunSummary& s) {
        return s.settle_time_s;
      }));
  result.verdict = verdict(result.counts, result.worst, req, result.quick);
  for (const pipeline::ProvenanceEntry& e : model.provenance) {
    if (e.kind != pipeline::ProvenanceKind::kPublished) {
      result.assumptions.push_back(Assumption{e.key, e.kind, e.range, e.scale, "not run"});
    }
  }
  return Out::ok(std::move(result));
}

}  // namespace cobot::bench
