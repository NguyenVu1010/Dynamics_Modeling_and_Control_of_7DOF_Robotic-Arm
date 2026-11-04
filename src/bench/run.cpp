#include "run.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iomanip>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "cobot/common/lie.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/imperfect/perturb.hpp"
#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/path_validation.hpp"
#include "cobot/pipeline/recorder.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"
#include "cobot/pipeline/waypoint_file.hpp"

#include "engine.hpp"
#include "metrics.hpp"
#include "run_data.hpp"

#ifndef COBOT_GIT_DESCRIBE
#define COBOT_GIT_DESCRIBE ""
#endif

namespace cobot::bench {
namespace {

using Loop = pipeline::ControlLoop<kN>;
using common::Status;

constexpr double kUsPerS = 1e6;
// A run keeps every record in memory until the loop ends: at most this many bytes of records.
constexpr double kMaxLogBytes = 2147483648.0;  // 2 GiB
constexpr double kBytesPerMb = 1048576.0;

// `urdf` is the plant's model (tau_max = URDF effort M); `nominal` is the controller's model
// (pipeline::controller_model, tau_max = peak torque P), used by the loop and validate_path.
struct Setup {
  pipeline::Config<kN> config;
  model::Arm7 urdf;
  model::Arm7 nominal;
};

common::Result<Setup> load_setup(const std::filesystem::path& config_yaml,
                                 const std::optional<ConfigOverride>& config_override) {
  using Out = common::Result<Setup>;
  const common::Result<pipeline::Config<kN>> config =
      config_override ? pipeline::load_config_with<kN>(
                            config_yaml, config_override->first, config_override->second)
                      : pipeline::load_config<kN>(config_yaml);
  if (!config) {
    return Out::error(config.reason());
  }
  const pipeline::Config<kN>& c = config.value();
  const common::Result<model::UrdfChain> chain =
      model::parse_urdf_chain(c.urdf_path, c.base_link, c.tip_link);
  const common::Result<model::Arm7> arm = chain
                                              ? model::make_model<kN>(chain.value(), c.gravity_b)
                                              : common::Result<model::Arm7>::error(chain.reason());
  if (!arm) {
    return Out::error(c.urdf_path.string() + ": " + arm.reason());
  }
  return Out::ok(Setup{c, arm.value(), pipeline::controller_model<kN>(arm.value(), c)});
}

// "waypoint 2: torque joint 2 0.95 > 0.9"; a joint_limit value is a distance below its margin.
std::string issue_line(const pipeline::WaypointIssue& w) {
  std::array<char, 64> numbers{};
  const char* op = w.issue == pipeline::PathIssue::kJointLimit ? "<" : ">";
  std::snprintf(numbers.data(), numbers.size(), " %.4g %s %.4g", w.value, op, w.limit);
  const std::string joint = w.joint >= 0 ? " joint " + std::to_string(w.joint) : "";
  return "waypoint " + std::to_string(w.waypoint) + ": " + pipeline::to_string(w.issue) + joint +
         numbers.data();
}

}  // namespace

Status run_checked(const std::filesystem::path& config_yaml,
                   const std::filesystem::path& imperfections_yaml,
                   std::optional<std::uint64_t> seed,
                   const std::filesystem::path& out_csv,
                   std::optional<double> duration_s,
                   const std::optional<std::filesystem::path>& waypoints_yaml,
                   bool& path_invalid) {
  path_invalid = false;
  if (out_csv.empty()) {  // run_profile treats an empty path as "no Run CSV"; run must write one
    return Status::error("--out must name a file");
  }
  common::Result<imperfect::ImperfectionConfig<kN>> imp =
      pipeline::load_imperfections<kN>(imperfections_yaml);
  if (!imp) {
    return Status::error(imp.reason());
  }
  imp.value().seed = seed.value_or(imp.value().seed);
  std::optional<WaypointInput> waypoints;
  if (waypoints_yaml) {
    waypoints = WaypointInput{*waypoints_yaml};
  }
  const common::Result<RunSummary> result = run_profile(config_yaml,
                                                        imp.value(),
                                                        imperfections_yaml,
                                                        out_csv,
                                                        duration_s,
                                                        waypoints ? &*waypoints : nullptr);
  path_invalid = waypoints && waypoints->path_invalid;
  return result ? Status::ok() : Status::error(result.reason());
}

Status run(const std::filesystem::path& config_yaml,
           const std::filesystem::path& imperfections_yaml,
           std::optional<std::uint64_t> seed,
           const std::filesystem::path& out_csv,
           std::optional<double> duration_s,
           const std::optional<std::filesystem::path>& waypoints_yaml) {
  bool path_invalid = false;
  return run_checked(
      config_yaml, imperfections_yaml, seed, out_csv, duration_s, waypoints_yaml, path_invalid);
}

Status run(const std::filesystem::path& config_yaml,
           const std::filesystem::path& imperfections_yaml,
           std::optional<std::uint64_t> seed,
           const std::filesystem::path& out_csv,
           std::optional<double> duration_s) {
  return run(config_yaml, imperfections_yaml, seed, out_csv, duration_s, std::nullopt);
}

namespace {

// One run of `request`; `path_invalid` is set when validate_path rejects the waypoint list.
common::Result<RunData> run_engine(const RunRequest& request, bool& path_invalid) {
  using Out = common::Result<RunData>;
  const imperfect::ImperfectionConfig<kN>& imperfections = request.imperfections;
  const std::filesystem::path& label = request.label;
  const common::Result<Setup> setup = load_setup(request.config_yaml, request.config_override);
  if (!setup) {
    return Out::error(setup.reason());
  }
  pipeline::Config<kN> config = setup.value().config;
  std::vector<planning::Waypoint> list;
  if (request.waypoints_yaml) {
    config.command_source = pipeline::CommandSource::kExternal;
    common::Result<std::vector<planning::Waypoint>> loaded =
        pipeline::load_waypoints(*request.waypoints_yaml, config.base_link, config.tcp_offset);
    if (!loaded) {
      return Out::error(loaded.reason());
    }
    list = std::move(loaded.value());
  }
  const std::optional<double>& duration_s = request.duration_s;
  if (duration_s && (!std::isfinite(*duration_s) || *duration_s <= 0.0)) {
    return Out::error("duration_s must be > 0, got " + std::to_string(*duration_s));
  }
  const double period_s = 1.0 / config.control_rate_hz;
  const Status comm = imperfect::check_comm(imperfections.comm, period_s);
  if (!comm) {
    return Out::error(label.string() + ": " + comm.reason());
  }
  RunData run;
  RunMeta& meta = run.meta;
  meta.config_yaml = request.config_yaml;
  meta.imperfections_label = label;
  meta.seed = imperfections.seed;
  meta.period_s = period_s;
  meta.git = COBOT_GIT_DESCRIBE;
  for (int i = 0; i < kN; ++i) {
    meta.tau_max(i) = setup.value().nominal.joints[static_cast<std::size_t>(i)].tau_max;
  }
  meta.rated = config.limits.rated_torque_nm;
  try {
    // The controller keeps the nominal model; the plant gets the perturbed URDF one (torque cap
    // M), with friction and motor lag, so the I/O decorator does not apply them again.
    const model::Arm7 plant_model =
        imperfect::perturb_model(setup.value().urdf, imperfections.model, imperfections.seed);
    dynamics::SimPlant<kN> plant(
        plant_model, config.home_q_rad, imperfect::plant_effects(imperfections));
    pipeline::SimJointInterface<kN> sim(plant,
                                        period_s,
                                        imperfections.timing.period_jitter_std_us / kUsPerS,
                                        imperfections.seed,
                                        imperfections.timing.overrun_prob,
                                        imperfections.timing.overrun_cycles);
    pipeline::SimTruthProbe<kN> probe(plant);
    imperfect::ImperfectJointInterface<kN> io(
        sim, imperfections, period_s, false, config.limits.peak_torque_nm);
    // The loop records into `recorder`, which gets run.records (one row per cycle) once the
    // cycle count is known (after the waypoint check) and before the first step; start()
    // records nothing.
    pipeline::HeapStorage<kN> no_rows(1, 1);
    pipeline::Recorder<kN> recorder(no_rows.memory(), pipeline::FullPolicy::kStopWhenFull);
    pipeline::RecorderControl control;
    const std::unique_ptr<Loop> loop =
        std::make_unique<Loop>(setup.value().nominal, config, &recorder);

    hal::JointState<kN> state;
    hal::JointCommand<kN> cmd;
    static_cast<void>(io.read(state));
    const Status started = loop->start(state);
    if (!started) {
      return Out::error(started.reason());
    }
    // Waypoint run: the list is checked from the standby pose and the measured q, and the
    // trajectory validate_path returns is exactly the one run.
    std::optional<planning::WaypointTrajectory> trajectory;
    if (request.waypoints_yaml) {
      const std::string yaml = request.waypoints_yaml->string();
      const common::Result<pipeline::PathValidation<kN>> checked =
          pipeline::validate_path<kN>(setup.value().nominal,
                                      config.clik,
                                      config.friction_compensation,
                                      state.q,
                                      loop->reference_pose(),
                                      list,
                                      config.cartesian_limits,
                                      config.path_validation);
      if (!checked) {
        return Out::error(yaml + ": " + checked.reason());
      }
      const pipeline::PathValidation<kN>& v = checked.value();
      if (!v.ok || !v.trajectory) {
        std::string text = yaml + ": path validation failed";
        for (const pipeline::WaypointIssue& issue : v.issues) {
          text += "\n" + issue_line(issue);
        }
        path_invalid = true;
        return Out::error(text);
      }
      trajectory = v.trajectory;
      const Status loaded = loop->load(*trajectory);
      if (!loaded) {
        return Out::error(yaml + ": " + loaded.reason());
      }
      meta.waypoints_yaml = request.waypoints_yaml;
      for (std::size_t i = 0; i < trajectory->size(); ++i) {
        meta.waypoint_node_times_s.push_back(trajectory->node_time_s(i));
        meta.waypoint_stop.push_back(trajectory->stops_at(i));
        meta.waypoint_positions_m.push_back(trajectory->waypoint(i).T_b_goal.p);
      }
    }
    meta.goal_time_s = trajectory ? trajectory->duration_s() : config.trajectory.duration_s;
    meta.window_end_s = meta.goal_time_s + config.trajectory.hold_s;
    const double run_s = duration_s.value_or(meta.window_end_s);
    const std::int64_t cycles = std::llround(run_s * config.control_rate_hz);
    const double log_bytes =
        static_cast<double>(cycles) * static_cast<double>(sizeof(pipeline::LogRecord<kN>));
    if (log_bytes > kMaxLogBytes) {
      std::array<char, 128> text{};
      std::snprintf(text.data(),
                    text.size(),
                    "duration_s %g: the in-memory log would need %.0f MB (limit %.0f MB)",
                    run_s,
                    std::ceil(log_bytes / kBytesPerMb),
                    kMaxLogBytes / kBytesPerMb);
      return Out::error(text.data());
    }
    run.records.resize(static_cast<std::size_t>(std::max<std::int64_t>(cycles, 1)));
    recorder = pipeline::Recorder<kN>(
        pipeline::RecorderMemory<kN>{
            &control, {run.records.data(), nullptr}, nullptr, run.records.size(), 1},
        pipeline::FullPolicy::kStopWhenFull);
    for (int i = 0; i < kN; ++i) {
      meta.plant_link_mass_kg(i) = plant_model.links[static_cast<std::size_t>(i)].mass_kg;
    }
    meta.q_offset_rad = io.q_offset_rad();
    meta.torque_scale = io.torque_scale();

    for (std::int64_t k = 0; k < cycles; ++k) {
      // A failed read leaves a stale stamp and a fault zero torque: the loop's safety logic
      // handles both and the CSV records the mode.
      static_cast<void>(io.read(state));
      static_cast<void>(loop->step(state, cmd, probe.sample()));
      static_cast<void>(io.write(cmd));
      sim.advance();
    }
    if (recorder.dropped() > 0) {
      return Out::error(label.string() + ": log rows dropped");
    }
    run.records.resize(recorder.size());
  } catch (const std::exception& e) {
    return Out::error(label.string() + ": " + e.what());
  }
  // After the loop: the summary from the recorded rows.
  run.summary = summarize(
      run.records,
      meta.tau_max,
      meta.rated,
      std::llround(meta.window_end_s * static_cast<double>(common::kNanosecondsPerSecond)));
  return Out::ok(std::move(run));
}

}  // namespace

common::Result<RunData> run_in_memory(const RunRequest& request) {
  bool path_invalid = false;
  return run_engine(request, path_invalid);
}

common::Result<RunSummary> run_profile(const std::filesystem::path& config_yaml,
                                       const imperfect::ImperfectionConfig<kN>& imperfections,
                                       const std::filesystem::path& label,
                                       const std::filesystem::path& out_csv,
                                       std::optional<double> duration_s,
                                       WaypointInput* waypoints,
                                       const std::optional<ConfigOverride>& config_override) {
  using Out = common::Result<RunSummary>;
  RunRequest request{config_yaml, imperfections, label, duration_s, std::nullopt, config_override};
  if (waypoints != nullptr) {
    request.waypoints_yaml = waypoints->yaml;
  }
  bool path_invalid = false;
  const common::Result<RunData> run = run_engine(request, path_invalid);
  if (waypoints != nullptr) {
    waypoints->path_invalid = path_invalid;
  }
  if (!run) {
    return Out::error(run.reason());
  }
  if (!out_csv.empty()) {
    const Status written = write_run_files(run.value(), out_csv);
    if (!written) {
      return Out::error(written.reason());
    }
  }
  return Out::ok(run.value().summary);
}

common::Result<TimingStats> timing(const std::filesystem::path& config_yaml, std::int64_t steps) {
  using Out = common::Result<TimingStats>;
  const common::Result<Setup> setup = load_setup(config_yaml, std::nullopt);
  if (!setup || steps <= 0) {
    return Out::error(setup ? "steps must be > 0" : setup.reason());
  }
  pipeline::Config<kN> config = setup.value().config;
  config.deterministic = false;
  const double period_s = 1.0 / config.control_rate_hz;
  try {
    dynamics::SimPlant<kN> plant(setup.value().urdf, config.home_q_rad);
    pipeline::SimJointInterface<kN> sim(plant, period_s);
    pipeline::SimTruthProbe<kN> probe(plant);
    // log.mode off: no recorder; otherwise one row per step (the copy is part of the step time).
    std::optional<pipeline::HeapStorage<kN>> storage;
    std::optional<pipeline::Recorder<kN>> recorder;
    if (config.log_mode != pipeline::LogMode::kOff) {
      storage.emplace(static_cast<std::size_t>(steps), 1);
      recorder.emplace(storage->memory(), pipeline::FullPolicy::kStopWhenFull);
    }
    const std::unique_ptr<Loop> loop =
        std::make_unique<Loop>(setup.value().nominal, config, recorder ? &*recorder : nullptr);
    hal::JointState<kN> state;
    hal::JointCommand<kN> cmd;
    static_cast<void>(sim.read(state));
    const Status started = loop->start(state);
    if (!started) {
      return Out::error(started.reason());
    }
    std::vector<double> us(static_cast<std::size_t>(steps));
    for (double& sample : us) {
      static_cast<void>(sim.read(state));
      const pipeline::PlantTruth<kN>& truth = probe.sample();  // outside the timed window
      const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
      static_cast<void>(loop->step(state, cmd, truth));
      const std::chrono::duration<double, std::micro> dt = std::chrono::steady_clock::now() - t0;
      sample = dt.count();
      static_cast<void>(sim.write(cmd));
      sim.advance();
    }
    std::sort(us.begin(), us.end());
    const auto rank = [&us](double p) {  // nearest-rank percentile
      const auto k = static_cast<std::size_t>(std::ceil(p * static_cast<double>(us.size())));
      return us[std::max<std::size_t>(k, 1) - 1];
    };
    return Out::ok(TimingStats{rank(0.50), rank(0.95), us.back()});
  } catch (const std::exception& e) {
    return Out::error(config_yaml.string() + ": " + e.what());
  }
}

}  // namespace cobot::bench
