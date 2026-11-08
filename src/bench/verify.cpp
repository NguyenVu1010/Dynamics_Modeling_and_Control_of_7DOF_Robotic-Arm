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
#include "cobot/pipeline/log_record.hpp"

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

// Counts of runs[begin, begin + n).
Counts tally(const std::vector<SeedRun>& runs, std::size_t begin, std::size_t n) {
  Counts c;
  for (std::size_t i = begin; i < begin + n && i < runs.size(); ++i) {
    ++c.runs;
    c.passed += runs[i].passed ? 1 : 0;
    c.held += runs[i].summary.held ? 1 : 0;
  }
  return c;
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
  // The ends are exact where the formula only rounds to them (0/100 gave 3.5e-18).
  const double lo = successes == 0 ? 0.0 : std::max(0.0, centre - half);
  const double hi = successes == n ? 1.0 : std::min(1.0, centre + half);
  return Interval{lo, hi};
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
  if (options.sensitivity_seeds == 0 || options.ablation_seeds == 0) {
    return Out::error("verify: sensitivity_seeds and ablation_seeds must be >= 1");
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
  unsigned jobs = options.jobs;
  if (jobs == 0) {
    jobs = std::max(1U, std::min(std::thread::hardware_concurrency(), kMaxDefaultJobs));
  }
  using Profile = imperfect::ImperfectionConfig<kN>;
  // One phase: item i runs profiles[i / seeds] with seed seed0 + i % seeds, stored at out[i]. Each
  // run keeps its rows only until it is summarised (run_with); cancel is read between runs.
  const auto run_phase = [&](const char* phase,
                             const std::vector<Profile>& profiles,
                             std::size_t seeds,
                             std::vector<SeedRun>& out) {
    const std::size_t total = profiles.size() * seeds;
    out.assign(total, SeedRun{});
    std::mutex mutex;
    std::size_t done = 0;
    progress(phase, 0, total);
    return run_queue(total, jobs, [&](std::size_t i) {
      if (cancelled()) {
        return common::Status::error("cancelled");
      }
      Profile p = profiles[i / seeds];
      p.seed = options.seed0 + i % seeds;
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
      SeedRun& slot = out[i];
      slot.seed = p.seed;
      slot.summary = run.value();
      slot.passed = run_passes(slot.summary, req);
      const std::lock_guard<std::mutex> lock(mutex);
      progress(phase, ++done, total);
      return common::Status::ok();
    });
  };
  const common::Status ran = run_phase("runs", {profile.value()}, options.runs, result.runs);
  if (!ran) {
    return Out::error(ran.reason());
  }
  result.counts = tally(result.runs, 0, result.runs.size());
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
  // A profile with `extra` on top of the run's own imperfection overrides.
  const auto load_with = [&](std::vector<KeyOverride> extra) {
    extra.insert(extra.end(), ro.imperfections.begin(), ro.imperfections.end());
    return pipeline::load_imperfections_with<kN>(model.imperfections, extra);
  };
  const auto first = [&result](std::size_t n) { return tally(result.runs, 0, n); };
  const std::vector<std::string>& only = options.sensitivity_keys;
  const bool sensitivity =
      options.sensitivity && !options.quick && (options.runs >= kFullRuns || !only.empty());
  const std::size_t s_seeds = std::min(options.sensitivity_seeds, options.runs);
  std::vector<Profile> variants;
  std::vector<std::size_t> varied;  // index into result.assumptions of each pair of variants
  for (const pipeline::ProvenanceEntry& e : model.provenance) {
    if (e.kind == pipeline::ProvenanceKind::kPublished) {
      continue;
    }
    Assumption a{e.key, e.kind, e.range, e.scale, "not run", {}, {}, {}};
    if (sensitivity && (only.empty() || std::find(only.begin(), only.end(), e.key) != only.end())) {
      a.sensitivity = to_string(Sensitivity::kNotVaried);
      for (const End end : {End::kLow, End::kHigh}) {
        if (!e.range && !e.scale) {
          break;
        }
        const common::Result<KeyOverride> o = variant_override(model.imperfections, e, end);
        const common::Result<Profile> p =
            o ? load_with({o.value()}) : common::Result<Profile>::error(o.reason());
        if (!p) {
          return Out::error(model.name + ": " + e.key + ": " + p.reason());
        }
        variants.push_back(p.value());
        varied.push_back(result.assumptions.size());
      }
    }
    result.assumptions.push_back(std::move(a));
  }
  if (!variants.empty()) {
    std::vector<SeedRun> runs;
    const common::Status st = run_phase("sensitivity", variants, s_seeds, runs);
    if (!st) {
      return Out::error(st.reason());
    }
    for (std::size_t v = 0; v < varied.size(); v += 2) {
      Assumption& a = result.assumptions[varied[v]];
      const std::array<double, 2> b = a.range ? *a.range : *a.scale;
      a.base = first(s_seeds);
      a.low = SensitivityEnd{b[0], tally(runs, v * s_seeds, s_seeds)};
      a.high = SensitivityEnd{b[1], tally(runs, (v + 1) * s_seeds, s_seeds)};
      const std::array<Counts, 2> ends{a.low->counts, a.high->counts};
      a.sensitivity =
          to_string(sensitivity_rule(result.counts, *a.base, ends, req.target_pass_rate));
    }
  }
  if (result.verdict != Verdict::kFail) {
    return Out::ok(std::move(result));
  }
  // The ablation (plan ambiguity 17): each active group at its off values.
  const std::size_t a_seeds = std::min(options.ablation_seeds, options.runs);
  std::vector<Profile> ablated;
  for (const char* group : kAblationGroups) {
    const common::Result<std::vector<KeyOverride>> off =
        group_off_overrides(model.imperfections, group);
    if (!off) {
      return Out::error(model.name + ": " + off.reason());
    }
    if (off.value().empty()) {
      continue;
    }
    const common::Result<Profile> p = load_with(off.value());
    if (!p) {
      return Out::error(model.name + ": " + group + " off: " + p.reason());
    }
    ablated.push_back(p.value());
    result.causes.push_back(Cause{group, 0, first(a_seeds).passed, false});
  }
  if (!ablated.empty()) {
    std::vector<SeedRun> runs;
    const common::Status st = run_phase("ablation", ablated, a_seeds, runs);
    if (!st) {
      return Out::error(st.reason());
    }
    for (std::size_t g = 0; g < result.causes.size(); ++g) {
      result.causes[g].passed_without = tally(runs, g * a_seeds, a_seeds).passed;
    }
  }
  SuggestionFacts facts;
  facts.algorithm = algorithm;
  facts.servo_rate_hz = profile.value().comm.servo_rate_hz;
  facts.no_admissible_gain = result.gain_source == GainSource::kDefaultNoAdmissible;
  for (const SeedRun& r : result.runs) {
    if (r.summary.held) {
      ++result.hold_reasons[pipeline::to_string(r.summary.hold_reason)];
    }
  }
  facts.hold_reasons = result.hold_reasons;
  if (mark_main_causes(result.causes)) {
    for (const Cause& c : result.causes) {
      if (c.main) {
        facts.main_causes.push_back(c.group);
      }
    }
  }
  result.suggestions = suggest(facts);
  return Out::ok(std::move(result));
}

}  // namespace cobot::bench
