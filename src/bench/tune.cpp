#include "tune.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <thread>
#include <utility>

#include "cobot/imperfect/config.hpp"
#include "cobot/pipeline/config.hpp"

#include "run_data.hpp"
#include "work_queue.hpp"

namespace cobot::bench {
namespace {

using control::Algorithm;
using pipeline::KeyOverride;

constexpr double kSatFracLimit = 0.05;  // saturation under 5 % of the run
constexpr unsigned kMaxDefaultJobs = 8;
constexpr std::uint64_t kDefaultSeeds = 10;

const double kNan = std::numeric_limits<double>::quiet_NaN();

std::string number(double v) {
  std::array<char, 32> text{};
  std::snprintf(text.data(), text.size(), "%.17g", v);
  return text.data();
}

std::string short_number(double v) {
  std::array<char, 32> text{};
  std::snprintf(text.data(), text.size(), "%g", v);
  return text.data();
}

double median(std::vector<double> v) {
  if (v.empty()) {
    return kNan;
  }
  std::sort(v.begin(), v.end());
  const std::size_t n = v.size();
  return n % 2 == 1 ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2.0;
}

bool saturated(const TuneRow& r) {
  return !(r.sat_frac_max < kSatFracLimit);  // NaN counts as saturated
}

bool admissible(const TuneRow& r) {
  return r.held == 0 && !saturated(r) && r.decays_all && !std::isnan(r.rms_joint_err_median);
}

}  // namespace

const char* tuning_key(Algorithm algorithm) {
  switch (algorithm) {
    case Algorithm::kComputedTorque:
      return pipeline::kOmegaKey;
    case Algorithm::kPdGravity:
      return pipeline::kPdOmegaKey;
    case Algorithm::kDrivePosition:
      return "config.drive.omega_rad_s";
  }
  return pipeline::kOmegaKey;
}

std::vector<double> default_levels(Algorithm algorithm) {
  const int last = algorithm == Algorithm::kDrivePosition ? 200 : 150;
  std::vector<double> levels;
  for (int level = 20; level <= last; level += 10) {
    levels.push_back(static_cast<double>(level));
  }
  return levels;
}

std::vector<KeyOverride> with_gain(std::vector<KeyOverride> overrides,
                                   Algorithm algorithm,
                                   double level) {
  const std::string key = tuning_key(algorithm);
  const bool omega = algorithm != Algorithm::kDrivePosition;  // virtual: replaces kp and kd
  const std::string section = key.substr(0, key.rfind('.') + 1);
  const auto sets_gain = [&](const KeyOverride& o) {
    return o.key == key || (omega && (o.key == section + "kp" || o.key == section + "kd"));
  };
  overrides.erase(std::remove_if(overrides.begin(), overrides.end(), sets_gain), overrides.end());
  overrides.push_back(KeyOverride{key, level});
  return overrides;
}

bool swing_decays(double first, double last) {
  if (std::isnan(first) || std::isnan(last)) {
    return false;
  }
  return last <= first || last <= kSwingNoiseFloorRad;
}

TuneRow tune_row(double level, const std::vector<RunSummary>& seeds) {
  TuneRow row{};
  row.level = level;
  row.seeds = seeds.size();
  row.decays_all = true;
  std::vector<double> rms;
  double final_max = kNan;
  for (const RunSummary& s : seeds) {
    const double frac = s.sat_time_s / s.run_time_s;
    if (std::isnan(frac) || frac > row.sat_frac_max) {
      row.sat_frac_max = std::isnan(row.sat_frac_max) ? row.sat_frac_max : frac;
    }
    row.decays_all = row.decays_all && !s.held && swing_decays(s.swing_first_rad, s.swing_last_rad);
    if (s.held) {
      ++row.held;
      continue;
    }
    rms.push_back(s.rms_joint_err_rad);
    if (std::isnan(final_max) || s.final_pos_err_m > final_max) {
      final_max = s.final_pos_err_m;
    }
  }
  row.rms_joint_err_median = median(rms);
  row.final_pos_err_max = final_max;
  return row;
}

std::optional<std::size_t> select_gain(const std::vector<TuneRow>& rows) {
  std::optional<std::size_t> best;
  for (std::size_t i = 0; i < rows.size(); ++i) {
    if (!admissible(rows[i])) {
      continue;
    }
    const TuneRow* b = best ? &rows[*best] : nullptr;
    if (b == nullptr || rows[i].rms_joint_err_median < b->rms_joint_err_median ||
        (rows[i].rms_joint_err_median == b->rms_joint_err_median && rows[i].level < b->level)) {
      best = i;
    }
  }
  return best;
}

std::string tune_reason(const std::vector<TuneRow>& rows,
                        const std::optional<std::size_t>& chosen) {
  std::size_t ok = 0;
  std::size_t held = 0;
  std::size_t sat = 0;
  std::size_t growing = 0;
  std::size_t no_rms = 0;
  for (const TuneRow& r : rows) {
    ok += admissible(r) ? 1 : 0;
    held += r.held > 0 ? 1 : 0;
    sat += saturated(r) ? 1 : 0;
    growing += r.decays_all ? 0 : 1;
    no_rms += std::isnan(r.rms_joint_err_median) ? 1 : 0;
  }
  const std::string of = " of " + std::to_string(rows.size()) + " levels";
  if (chosen && *chosen < rows.size()) {
    const TuneRow& r = rows[*chosen];
    std::array<char, 32> rms{};
    std::snprintf(rms.data(), rms.size(), "%.3g", r.rms_joint_err_median);
    return "level " + short_number(r.level) + ": lowest median rms joint error (" + rms.data() +
           " rad) of " + std::to_string(ok) + " admissible" + of;
  }
  std::string text = "no admissible gain" + of + ": held " + std::to_string(held) + ", saturated " +
                     std::to_string(sat) + ", not decaying " + std::to_string(growing);
  if (no_rms > 0) {
    text += ", no rms " + std::to_string(no_rms);
  }
  return text;
}

common::Result<TuneResult> tune(const pipeline::HardwareModel& model,
                                Algorithm algorithm,
                                const pipeline::Task& task,
                                const TuneOptions& options) {
  using Out = common::Result<TuneResult>;
  // run_overrides' precondition: the model lists the algorithm.
  const common::Result<pipeline::Config<kN>> base = pipeline::config_for<kN>(model, algorithm);
  if (!base) {
    return Out::error(base.reason());
  }
  TuneResult result;
  result.algorithm = algorithm;
  result.key = tuning_key(algorithm);
  result.levels = options.levels.empty() ? default_levels(algorithm) : options.levels;
  std::vector<std::uint64_t> seeds = options.seeds;
  if (seeds.empty()) {
    for (std::uint64_t s = 1; s <= kDefaultSeeds; ++s) {
      seeds.push_back(s);
    }
  }
  const pipeline::RunOverrides ro = pipeline::run_overrides(model, algorithm, task);
  common::Result<imperfect::ImperfectionConfig<kN>> profile =
      pipeline::load_imperfections_with<kN>(model.imperfections, ro.imperfections);
  if (!profile) {
    return Out::error(model.name + ": " + profile.reason());
  }
  // Every level is checked by the loader before the first run.
  std::vector<std::vector<KeyOverride>> lists;
  for (const double level : result.levels) {
    lists.push_back(with_gain(ro.config, algorithm, level));
    const common::Result<pipeline::Config<kN>> c =
        pipeline::load_config_with<kN>(model.controller, lists.back());
    if (!c) {
      return Out::error(model.name + " " + result.key + " " + short_number(level) + ": " +
                        c.reason());
    }
  }
  const std::size_t n_seeds = seeds.size();
  std::vector<RunSummary> summaries(lists.size() * n_seeds);
  unsigned jobs = options.jobs;
  if (jobs == 0) {
    jobs = std::max(1U, std::min(std::thread::hardware_concurrency(), kMaxDefaultJobs));
  }
  // Each run keeps its rows only until it is summarised (run_with).
  const common::Status ran = run_queue(summaries.size(), jobs, [&](std::size_t i) {
    const std::size_t l = i / n_seeds;
    imperfect::ImperfectionConfig<kN> p = profile.value();
    p.seed = seeds[i % n_seeds];
    std::optional<WaypointInput> waypoints;
    if (ro.waypoints) {
      waypoints = WaypointInput{*ro.waypoints};
    }
    common::Result<RunSummary> run =
        run_with(model.controller, lists[l], p, model.name, waypoints ? &*waypoints : nullptr);
    if (!run) {
      return common::Status::error(model.name + " " + result.key + " " +
                                   short_number(result.levels[l]) + " seed " +
                                   std::to_string(p.seed) + ": " + run.reason());
    }
    summaries[i] = run.value();
    return common::Status::ok();
  });
  if (!ran) {
    return Out::error(ran.reason());
  }
  for (std::size_t l = 0; l < lists.size(); ++l) {
    const auto first = summaries.begin() + static_cast<std::ptrdiff_t>(l * n_seeds);
    result.rows.push_back(
        tune_row(result.levels[l],
                 std::vector<RunSummary>(first, first + static_cast<std::ptrdiff_t>(n_seeds))));
  }
  const std::optional<std::size_t> chosen = select_gain(result.rows);
  if (chosen) {
    result.chosen = result.rows[*chosen].level;
  }
  result.reason = tune_reason(result.rows, chosen);
  return Out::ok(std::move(result));
}

common::Status write_tune_csv(const std::filesystem::path& path, const TuneResult& result) {
  std::ofstream csv(path, std::ios::binary | std::ios::trunc);
  if (!csv) {
    return common::Status::error("cannot open " + path.string() + " for writing");
  }
  csv << "level,seeds,held,sat_frac_max,decays_all,rms_joint_err_median,final_pos_err_max,"
         "chosen\n";
  for (const TuneRow& r : result.rows) {
    const bool chosen = result.chosen && *result.chosen == r.level;
    csv << number(r.level) << ',' << r.seeds << ',' << r.held << ',' << number(r.sat_frac_max)
        << ',' << (r.decays_all ? 1 : 0) << ',' << number(r.rms_joint_err_median) << ','
        << number(r.final_pos_err_max) << ',' << (chosen ? 1 : 0) << '\n';
  }
  csv.flush();
  if (!csv) {
    return common::Status::error("cannot write " + path.string());
  }
  return common::Status::ok();
}

}  // namespace cobot::bench
