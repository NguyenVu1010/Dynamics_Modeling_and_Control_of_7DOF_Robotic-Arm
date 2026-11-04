#include "cobot/studio/runner.hpp"

#include <array>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <system_error>
#include <utility>

#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/studio/metrics.hpp"
#include "cobot/studio/param_catalog.hpp"

namespace cobot::studio {
namespace {

using common::Status;

std::string join(const std::vector<std::string>& lines) {
  std::string out;
  for (const std::string& l : lines) {
    out += (out.empty() ? "" : "\n") + l;
  }
  return out;
}

// A new, empty <work_dir>/run_NNNN.
common::Result<fs::path> new_run_dir(const fs::path& work_dir) {
  std::error_code ec;
  fs::create_directories(work_dir, ec);
  for (std::size_t i = 1;; ++i) {
    std::array<char, 24> name{};
    std::snprintf(name.data(), name.size(), "run_%04zu", i);
    const fs::path dir = work_dir / name.data();
    if (fs::create_directory(dir, ec)) {
      return common::Result<fs::path>::ok(dir);
    }
    if (ec) {
      return common::Result<fs::path>::error("cannot create " + dir.string() + ": " + ec.message());
    }
  }
}

// A materialised run, ready for bench::run_in_memory.
struct Prepared {
  StudioConfig config;
  Materialized inputs;
  bench::RunRequest request;
};

// Materialises an already validated configuration and checks its comm group against the control
// rate (as the bench does before a run).
common::Result<Prepared> prepare(const StudioConfig& config, const fs::path& work_dir) {
  using Out = common::Result<Prepared>;
  const common::Result<fs::path> dir = new_run_dir(work_dir);
  if (!dir) {
    return Out::error(dir.reason());
  }
  const common::Result<Materialized> m = materialize(config, dir.value());
  if (!m) {
    return Out::error(m.reason());
  }
  const Status saved = save_studio_file(config, dir.value() / "studio.yaml");
  if (!saved) {
    return Out::error(saved.reason());
  }
  const Materialized& in = m.value();
  common::Result<imperfect::ImperfectionConfig<bench::kN>> imp =
      pipeline::load_imperfections<bench::kN>(in.imperfections_yaml);
  const common::Result<pipeline::Config<bench::kN>> c =
      pipeline::load_config<bench::kN>(in.config_yaml);
  if (!imp || !c) {
    return Out::error(imp ? c.reason() : imp.reason());
  }
  const Status comm = imperfect::check_comm(imp.value().comm, 1.0 / c.value().control_rate_hz);
  if (!comm) {
    return Out::error(in.imperfections_yaml.string() + ": " + comm.reason());
  }
  bench::RunRequest request{in.config_yaml,
                            std::move(imp.value()),
                            in.imperfections_yaml,
                            config.duration_s,
                            config.waypoints_yaml,
                            std::nullopt};
  return Out::ok(Prepared{config, in, std::move(request)});
}

// Runs a prepared configuration; the records are kept when `keep`.
common::Result<RunEntry> execute(const Prepared& p, RunKind kind, bool keep) {
  using Out = common::Result<RunEntry>;
  common::Result<bench::RunData> run = bench::run_in_memory(p.request);
  if (!run) {
    return Out::error(run.reason());
  }
  RunEntry e;
  e.kind = kind;
  e.config = p.config;
  e.inputs = p.inputs;
  e.diff = describe_diff(p.config);
  e.label = p.config.base.name + (e.diff.empty() ? "" : " " + e.diff);
  e.errors = std::make_shared<const ErrorTrace>(make_error_trace(run.value()));
  e.meta = run.value().meta;
  e.summary = run.value().summary;
  if (keep) {
    e.data = std::make_shared<const bench::RunData>(std::move(run.value()));
  }
  return Out::ok(std::move(e));
}

std::string number(double v) {
  std::array<char, 32> text{};
  std::snprintf(text.data(), text.size(), "%g", v);
  return text.data();
}

}  // namespace

common::Result<RunEntry> run_once(const StudioConfig& config, const fs::path& work_dir) {
  const std::vector<std::string> issues = validate(config);
  if (!issues.empty()) {
    return common::Result<RunEntry>::error(join(issues));
  }
  const common::Result<Prepared> p = prepare(config, work_dir);
  if (!p) {
    return common::Result<RunEntry>::error(p.reason());
  }
  return execute(p.value(), RunKind::kBench, true);
}

SweepRunner::SweepRunner(fs::path work_dir, int max_parallel) : work_dir_(std::move(work_dir)) {
  if (max_parallel != 1) {
    throw std::invalid_argument("SweepRunner runs one at a time: max_parallel must be 1, got " +
                                std::to_string(max_parallel));
  }
}

Status SweepRunner::run(const SweepRequest& request,
                        const std::atomic<bool>& cancel,
                        const ResultFn& on_result,
                        const ProgressFn& on_progress) {
  const ParamInfo* param = find_param(request.param);
  if (param == nullptr || !param->sweepable) {
    return Status::error((param == nullptr ? "unknown sweep param " : "param is not sweepable: ") +
                         request.param);
  }
  if (request.levels.empty() || request.seeds.empty()) {
    return Status::error("a sweep needs at least one level and one seed");
  }
  // Every level is checked before the first file is written.
  std::vector<StudioConfig> levels;
  for (const double level : request.levels) {
    StudioConfig c = request.base;
    bool replaced = false;
    for (pipeline::KeyOverride& o : c.overrides) {
      if (o.key == request.param) {
        o.value = level;
        replaced = true;
      }
    }
    if (!replaced) {
      c.overrides.push_back(pipeline::KeyOverride{request.param, level});
    }
    const std::vector<std::string> issues = validate(c);
    if (!issues.empty()) {
      return Status::error(request.param + " level " + number(level) + ": " + join(issues));
    }
    levels.push_back(std::move(c));
  }
  std::vector<Prepared> jobs;
  std::vector<SweepPoint> points;
  for (std::size_t l = 0; l < levels.size(); ++l) {
    for (const std::uint64_t seed : request.seeds) {
      StudioConfig c = levels[l];
      c.seed = seed;
      common::Result<Prepared> p = prepare(c, work_dir_);
      if (!p) {
        return Status::error(request.param + " level " + number(request.levels[l]) + ": " +
                             p.reason());
      }
      jobs.push_back(std::move(p.value()));
      points.push_back(SweepPoint{request.param, request.levels[l], seed});
    }
  }
  for (std::size_t k = 0; k < jobs.size() && !cancel.load(); ++k) {
    common::Result<RunEntry> e =
        execute(jobs[k], RunKind::kSweep, points[k].seed == request.seeds.front());
    if (!e) {
      return Status::error(e.reason());
    }
    e.value().sweep = points[k];
    e.value().label += " seed " + std::to_string(points[k].seed);
    if (on_result) {
      on_result(std::move(e.value()));
    }
    if (on_progress) {
      on_progress(k + 1, jobs.size());
    }
  }
  return Status::ok();
}

}  // namespace cobot::studio
