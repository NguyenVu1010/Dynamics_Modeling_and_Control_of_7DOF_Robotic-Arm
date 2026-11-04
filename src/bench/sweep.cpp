#include <array>
#include <cstdio>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/sweep_config.hpp"

#include "engine.hpp"
#include "metrics.hpp"
#include "run.hpp"

namespace cobot::bench {
namespace {

using common::Status;
using Profile = imperfect::ImperfectionConfig<kN>;

struct Job {
  std::string param;
  double level = 0.0;
  Profile profile;
  std::optional<ConfigOverride> config_override;  // a `config.` param
};

// Sweep params of the controller config: `config.<section>.<key>`.
bool config_param(const std::string& name) {
  return name.rfind("config.", 0) == 0;
}

std::string number(double v) {
  std::array<char, 32> text{};
  std::snprintf(text.data(), text.size(), "%.17g", v);
  return text.data();
}

// Checks every job's comm group against the control rate of its config (servo rate ratio,
// probabilities, burst length), so a bad level fails before the first run writes anything.
Status check_jobs(const std::filesystem::path& config_yaml, const std::vector<Job>& jobs) {
  const common::Result<pipeline::Config<kN>> base = pipeline::load_config<kN>(config_yaml);
  if (!base) {
    return Status::error(base.reason());
  }
  for (const Job& job : jobs) {
    double rate_hz = base.value().control_rate_hz;
    if (job.config_override) {
      const common::Result<pipeline::Config<kN>> c = pipeline::load_config_with<kN>(
          config_yaml, job.config_override->first, job.config_override->second);
      if (!c) {
        return Status::error(c.reason());
      }
      rate_hz = c.value().control_rate_hz;
    }
    const Status comm = imperfect::check_comm(job.profile.comm, 1.0 / rate_hz);
    if (!comm) {
      return Status::error(job.param + " " + number(job.level) + ": " + comm.reason());
    }
  }
  return Status::ok();
}

// Runs every job in order, one Summary CSV row each.
Status run_jobs(const std::filesystem::path& config_yaml,
                const std::filesystem::path& label,
                const std::vector<Job>& jobs,
                const std::filesystem::path& out_dir,
                bool keep_runs,
                std::optional<double> duration_s) {
  const Status checked = check_jobs(config_yaml, jobs);
  if (!checked) {
    return Status::error(label.string() + ": " + checked.reason());
  }
  std::error_code ec;
  std::filesystem::create_directories(out_dir, ec);
  const std::filesystem::path path = out_dir / "summary.csv";
  std::ofstream csv(path, std::ios::binary);
  if (!csv) {
    return Status::error("cannot open " + path.string() + " for writing");
  }
  write_summary_header(csv);
  for (std::size_t k = 0; k < jobs.size(); ++k) {
    std::array<char, 24> id{};
    std::snprintf(id.data(), id.size(), "%04zu", k);
    const std::filesystem::path run_csv =
        keep_runs ? out_dir / ("run_" + std::string(id.data()) + ".csv") : "";
    const common::Result<RunSummary> r = run_profile(
        config_yaml, jobs[k].profile, label, run_csv, duration_s, nullptr, jobs[k].config_override);
    if (!r) {
      return Status::error(r.reason());
    }
    write_summary_row(csv,
                      SummaryRow{k, jobs[k].param, jobs[k].level, jobs[k].profile.seed, r.value()});
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
  std::vector<Job> jobs(runs, Job{"none", 0.0, profile.value(), std::nullopt});
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
      const bool config = config_param(p.name);
      common::Result<Profile> profile =
          config ? pipeline::load_imperfections<kN>(spec.value().base_profile)
                 : pipeline::load_imperfections_with<kN>(spec.value().base_profile, p.name, level);
      if (!profile) {
        return Status::error(sweep_yaml.string() + ": " + profile.reason());
      }
      std::optional<ConfigOverride> config_override;
      if (config) {
        const auto checked = pipeline::load_config_with<kN>(config_yaml, p.name, level);
        if (!checked) {
          return Status::error(sweep_yaml.string() + ": " + checked.reason());
        }
        config_override = ConfigOverride{p.name, level};
      }
      for (const std::uint64_t seed : spec.value().seeds) {
        profile.value().seed = seed;
        jobs.push_back(Job{p.name, level, profile.value(), config_override});
      }
    }
  }
  return run_jobs(config_yaml, spec.value().base_profile, jobs, out_dir, false, duration_s);
}

}  // namespace cobot::bench
