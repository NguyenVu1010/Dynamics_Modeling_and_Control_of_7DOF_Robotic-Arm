#include <array>
#include <cstdio>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/sweep_config.hpp"

#include "engine.hpp"
#include "run.hpp"

namespace cobot::bench {
namespace {

constexpr int kN = model::kArmDof;
using common::Status;
using Profile = imperfect::ImperfectionConfig<kN>;

// Design G3 (spec 11.3): informative `passed` column only, the scenario files hold the
// thresholds that decide a test.
constexpr double kG3PosErrM = 0.002;
constexpr double kG3RotErrRad = 3.14159265358979323846 / 180.0;
constexpr double kG3RmsJointErrRad = 0.01;

struct Job {
  std::string param;
  double level = 0.0;
  Profile profile;
};

std::string number(double v) {
  std::array<char, 32> text{};
  std::snprintf(text.data(), text.size(), "%.17g", v);
  return text.data();
}

bool passes(const RunSummary& s) {
  return s.final_pos_err_m <= kG3PosErrM && s.final_rot_err_rad <= kG3RotErrRad &&
         s.rms_joint_err_rad <= kG3RmsJointErrRad;
}

// Runs every job in order, one Summary CSV row each.
Status run_jobs(const std::filesystem::path& config_yaml,
                const std::filesystem::path& label,
                const std::vector<Job>& jobs,
                const std::filesystem::path& out_dir,
                bool keep_runs,
                std::optional<double> duration_s) {
  std::error_code ec;
  std::filesystem::create_directories(out_dir, ec);
  const std::filesystem::path path = out_dir / "summary.csv";
  std::ofstream csv(path, std::ios::binary);
  if (!csv) {
    return Status::error("cannot open " + path.string() + " for writing");
  }
  csv << "run_id,param,level,seed,final_pos_err_m,final_rot_err_rad,rms_joint_err_rad,"
         "peak_tau_ratio,sat_time_s,held,passed\n";
  for (std::size_t k = 0; k < jobs.size(); ++k) {
    std::array<char, 24> id{};
    std::snprintf(id.data(), id.size(), "%04zu", k);
    const std::filesystem::path run_csv =
        keep_runs ? out_dir / ("run_" + std::string(id.data()) + ".csv") : "";
    const common::Result<RunSummary> r =
        run_profile(config_yaml, jobs[k].profile, label, run_csv, duration_s);
    if (!r) {
      return Status::error(r.reason());
    }
    const RunSummary& s = r.value();
    csv << id.data() << ',' << jobs[k].param << ',' << number(jobs[k].level) << ','
        << jobs[k].profile.seed << ',' << number(s.final_pos_err_m) << ','
        << number(s.final_rot_err_rad) << ',' << number(s.rms_joint_err_rad) << ','
        << number(s.peak_tau_ratio) << ',' << number(s.sat_time_s) << ',' << (s.held ? 1 : 0) << ','
        << (passes(s) ? 1 : 0) << '\n';
  }
  csv.flush();
  return csv ? Status::ok() : Status::error("cannot write " + path.string());
}

}  // namespace

Status montecarlo(const std::filesystem::path& config_yaml,
                  const std::filesystem::path& imperfections_yaml,
                  std::size_t runs,
                  std::uint64_t seed0,
                  const std::filesystem::path& out_dir,
                  bool keep_runs,
                  std::optional<double> duration_s) {
  if (runs == 0) {
    return Status::error("runs must be > 0");
  }
  const common::Result<Profile> profile = pipeline::load_imperfections<kN>(imperfections_yaml);
  if (!profile) {
    return Status::error(profile.reason());
  }
  std::vector<Job> jobs(runs, Job{"none", 0.0, profile.value()});
  for (std::size_t k = 0; k < runs; ++k) {
    jobs[k].profile.seed = seed0 + k;
  }
  return run_jobs(config_yaml, imperfections_yaml, jobs, out_dir, keep_runs, duration_s);
}

Status sweep(const std::filesystem::path& config_yaml,
             const std::filesystem::path& sweep_yaml,
             const std::filesystem::path& out_dir,
             std::optional<double> duration_s) {
  const common::Result<pipeline::SweepSpec> spec = pipeline::load_sweep(sweep_yaml);
  if (!spec) {
    return Status::error(spec.reason());
  }
  std::vector<Job> jobs;
  for (const pipeline::SweepParam& p : spec.value().params) {
    for (const double level : p.levels) {
      common::Result<Profile> profile =
          pipeline::load_imperfections_with<kN>(spec.value().base_profile, p.name, level);
      if (!profile) {
        return Status::error(sweep_yaml.string() + ": " + profile.reason());
      }
      for (const std::uint64_t seed : spec.value().seeds) {
        profile.value().seed = seed;
        jobs.push_back(Job{p.name, level, profile.value()});
      }
    }
  }
  return run_jobs(config_yaml, spec.value().base_profile, jobs, out_dir, false, duration_s);
}

}  // namespace cobot::bench
