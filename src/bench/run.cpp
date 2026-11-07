#include "run.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <exception>
#include <fstream>
#include <iomanip>
#include <memory>
#include <optional>
#include <pthread.h>
#include <sched.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include "cobot/bridge/endpoint.hpp"
#include "cobot/bridge/wire.hpp"
#include "cobot/common/lie.hpp"
#include "cobot/control/algorithm.hpp"
#include "cobot/dynamics/dynamics.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/imperfect/perturb.hpp"
#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/path_validation.hpp"
#include "cobot/pipeline/recorder.hpp"
#include "cobot/pipeline/sim_drive.hpp"
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
constexpr std::int64_t kNsPerS = 1000000000;

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

// The simulated drive of drive_position (feature drive-loop), none for the torque laws: J =
// diag(M(home_q)) of the controller model, the controller's peak torque, the profile's actuator
// chain with the decorator's torque scale, set points at the servo rate (every period when 0).
std::optional<pipeline::SimDrive<kN>> make_drive(const Setup& setup,
                                                 const pipeline::Config<kN>& config,
                                                 const imperfect::ImperfectionConfig<kN>& profile,
                                                 const common::VecN<kN>& torque_scale) {
  if (config.algorithm != control::Algorithm::kDrivePosition) {
    return std::nullopt;
  }
  const double period_s = 1.0 / config.control_rate_hz;
  const double servo_hz = profile.comm.servo_rate_hz;
  const common::MatN<kN> m = dynamics::mass_matrix(setup.nominal, config.home_q_rad);
  return pipeline::SimDrive<kN>(config.drive,
                                m.diagonal(),
                                config.limits.peak_torque_nm,
                                profile,
                                torque_scale,
                                period_s,
                                servo_hz > 0.0 ? 1.0 / servo_hz : period_s);
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
    // drive_position: the drive sits after the decorator, between the bus and the plant.
    std::optional<pipeline::SimDrive<kN>> drive =
        make_drive(setup.value(), config, imperfections, io.torque_scale());
    if (drive) {
      sim.attach_drive(&*drive);
    }
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
  // After the loop: the summary from the recorded rows; a drive run rates the motor torque.
  run.summary = summarize(
      run.records,
      meta.tau_max,
      meta.rated,
      std::llround(meta.window_end_s * static_cast<double>(common::kNanosecondsPerSecond)),
      config.algorithm == control::Algorithm::kDrivePosition ? TorqueSource::kMotor
                                                             : TorqueSource::kCommand);
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

namespace {

using Server = bridge::ServerEndpoint<kN>;
using Client = bridge::ClientEndpoint<kN>;

// Nearest-rank percentile of sorted values.
double percentile(const std::vector<double>& sorted, double p) {
  const auto k = static_cast<std::size_t>(std::ceil(p * static_cast<double>(sorted.size())));
  return sorted[std::max<std::size_t>(k, 1) - 1];
}

void sleep_until(std::int64_t t_ns) {
  timespec ts{};
  ts.tv_sec = static_cast<time_t>(t_ns / kNsPerS);
  ts.tv_nsec = static_cast<long>(t_ns % kNsPerS);
  while (::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {
  }
}

bool pin_to(int cpu) {
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  return ::pthread_setaffinity_np(::pthread_self(), sizeof(set), &set) == 0;
}

// Pins the calling (loop) thread to the first allowed CPU when at least two are allowed and
// restores the caller's affinity when destroyed. timing() pins in every paced run, whatever the
// log mode, and whenever a reader runs, so the PF-02 modes are measured with the same placement.
class LoopPin {
 public:
  LoopPin() = default;
  LoopPin(const LoopPin&) = delete;
  LoopPin& operator=(const LoopPin&) = delete;
  LoopPin(LoopPin&&) = delete;
  LoopPin& operator=(LoopPin&&) = delete;
  ~LoopPin() {
    if (pinned_) {
      ::pthread_setaffinity_np(::pthread_self(), sizeof(saved_), &saved_);
    }
  }
  // The CPU for the reader (the last allowed one), -1 without pinning.
  int pin() {
    CPU_ZERO(&saved_);
    if (::sched_getaffinity(0, sizeof(saved_), &saved_) != 0 || CPU_COUNT(&saved_) < 2) {
      return -1;
    }
    int first = -1;
    int last = -1;
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
      if (CPU_ISSET(cpu, &saved_)) {
        first = first < 0 ? cpu : first;
        last = cpu;
      }
    }
    pinned_ = pin_to(first);
    return pinned_ ? last : -1;
  }

 private:
  cpu_set_t saved_{};
  bool pinned_ = false;
};

// Drains the bridge state ring in a loop with yield; after stop() it drains once more, so
// samples + client.lost() accounts for every publish. Joined when destroyed.
class StateReader {
 public:
  StateReader() = default;
  StateReader(const StateReader&) = delete;
  StateReader& operator=(const StateReader&) = delete;
  StateReader(StateReader&&) = delete;
  StateReader& operator=(StateReader&&) = delete;
  ~StateReader() { stop(); }

  void start(Client& client, int cpu) {
    thread_ = std::thread([this, &client, cpu] {
      pinned_ = cpu >= 0 && pin_to(cpu);
      std::vector<bridge::StateSample<kN>> got;
      got.reserve(bridge::kStateSlots);
      for (;;) {
        const bool last = stop_.load(std::memory_order_acquire);
        got.clear();
        samples_ += client.read_states(got);
        if (last) {
          return;
        }
        std::this_thread::yield();
      }
    });
  }
  void stop() {
    stop_.store(true, std::memory_order_release);
    if (thread_.joinable()) {
      thread_.join();
    }
  }
  // After stop().
  std::uint64_t samples() const { return samples_; }
  bool pinned() const { return pinned_; }

 private:
  std::atomic<bool> stop_{false};
  std::thread thread_;
  std::uint64_t samples_ = 0;
  bool pinned_ = false;
};

// Removes the bridge region `name` (when not empty) on every exit path.
class RegionRemover {
 public:
  explicit RegionRemover(std::string name) : name_(std::move(name)) {}
  RegionRemover(const RegionRemover&) = delete;
  RegionRemover& operator=(const RegionRemover&) = delete;
  RegionRemover(RegionRemover&&) = delete;
  RegionRemover& operator=(RegionRemover&&) = delete;
  ~RegionRemover() {
    if (!name_.empty()) {
      static_cast<void>(Server::remove(name_));
    }
  }
  const std::string& name() const { return name_; }

 private:
  std::string name_;
};

}  // namespace

common::Result<TimingStats> timing(const std::filesystem::path& config_yaml, std::int64_t steps) {
  return timing(config_yaml, steps, TimingOptions{});
}

common::Result<TimingStats> timing(const std::filesystem::path& config_yaml,
                                   std::int64_t steps,
                                   const TimingOptions& options) {
  using Out = common::Result<TimingStats>;
  const common::Result<Setup> setup = load_setup(config_yaml, std::nullopt);
  if (!setup || steps <= 0) {
    return Out::error(setup ? "steps must be > 0" : setup.reason());
  }
  pipeline::Config<kN> config = setup.value().config;
  config.deterministic = false;
  config.log_mode = options.log_mode.value_or(config.log_mode);
  const bool live = config.log_mode == pipeline::LogMode::kRecorderLive;
  const double period_s = 1.0 / config.control_rate_hz;
  const std::int64_t period_ns = std::llround(period_s * static_cast<double>(kNsPerS));
  try {
    dynamics::SimPlant<kN> plant(setup.value().urdf, config.home_q_rad);
    pipeline::SimJointInterface<kN> sim(plant, period_s);
    // drive_position: a drive on the ideal profile (it runs outside the timed window).
    std::optional<pipeline::SimDrive<kN>> drive = make_drive(
        setup.value(), config, imperfect::ImperfectionConfig<kN>{}, common::VecN<kN>::Ones());
    if (drive) {
      sim.attach_drive(&*drive);
    }
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
    // recorder_live: the bridge region and its reader.
    const RegionRemover region(live ? "timing_" + std::to_string(::getpid()) : std::string());
    std::optional<Server> server;
    std::optional<Client> client;
    if (live) {
      bridge::SessionInfo<kN> session{};
      session.control_rate_hz = config.control_rate_hz;
      common::Result<Server> created = Server::create(region.name(), session);
      if (!created) {
        return Out::error(created.reason());
      }
      server.emplace(std::move(created.value()));
      if (options.reader) {
        common::Result<Client> opened = Client::open(region.name());
        if (!opened) {
          return Out::error(opened.reason());
        }
        client.emplace(std::move(opened.value()));
      }
    }
    hal::JointState<kN> state;
    hal::JointCommand<kN> cmd;
    static_cast<void>(sim.read(state));
    const Status started = loop->start(state);
    if (!started) {
      return Out::error(started.reason());
    }
    std::vector<double> us(static_cast<std::size_t>(steps));
    std::vector<double> late_us(static_cast<std::size_t>(steps), 0.0);
    TimingStats stats;
    LoopPin loop_pin;
    const int reader_cpu = options.paced || client ? loop_pin.pin() : -1;
    StateReader reader;
    if (client) {
      reader.start(*client, reader_cpu);
    }
    std::int64_t deadline = bridge::monotonic_ns();
    for (std::size_t k = 0; k < us.size(); ++k) {
      if (options.paced) {
        deadline += period_ns;
        sleep_until(deadline);
        const std::int64_t late_ns = bridge::monotonic_ns() - deadline;
        stats.late_cycles += late_ns > period_ns ? 1 : 0;
        late_us[k] = static_cast<double>(late_ns) / 1e3;
      }
      static_cast<void>(sim.read(state));
      const pipeline::PlantTruth<kN>& truth = probe.sample();  // outside the timed window
      const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
      static_cast<void>(loop->step(state, cmd, truth));
      if (server) {
        server->publish(bridge::to_sample(loop->last_record(), k + 1, 0));
      }
      const std::chrono::duration<double, std::micro> dt = std::chrono::steady_clock::now() - t0;
      us[k] = dt.count();
      static_cast<void>(sim.write(cmd));
      sim.advance();
    }
    reader.stop();
    if (client) {
      stats.reader_samples = reader.samples();
      stats.reader_lost = client->lost();
      stats.reader_on_other_cpu = reader.pinned();
    }
    std::sort(us.begin(), us.end());
    std::sort(late_us.begin(), late_us.end());
    stats.p50_us = percentile(us, 0.50);
    stats.p95_us = percentile(us, 0.95);
    stats.p99_us = percentile(us, 0.99);
    stats.max_us = us.back();
    stats.late_p99_us = percentile(late_us, 0.99);
    return Out::ok(stats);
  } catch (const std::exception& e) {
    return Out::error(config_yaml.string() + ": " + e.what());
  }
}

}  // namespace cobot::bench
