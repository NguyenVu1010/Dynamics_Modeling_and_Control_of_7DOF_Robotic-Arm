#include "serve.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <ctime>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <system_error>
#include <thread>
#include <utility>

#include "cobot/bridge/command_service.hpp"
#include "cobot/bridge/endpoint.hpp"
#include "cobot/bridge/loop_port.hpp"
#include "cobot/bridge/wire.hpp"
#include "cobot/control/algorithm.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/imperfect/perturb.hpp"
#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/recorder.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"

#include "run_data.hpp"

namespace cobot::bench {
namespace {

namespace fs = std::filesystem;
using common::Status;
using Loop = pipeline::ControlLoop<kN>;
using Service = bridge::CommandService<kN>;
using Server = bridge::ServerEndpoint<kN>;
using Applied = std::pair<model::Arm7, pipeline::Config<kN>>;

constexpr std::int64_t kNsPerS = 1000000000;
constexpr std::int64_t kPollNs = 5000000;         // CommandService::poll period
constexpr std::int64_t kExportNs = 200000000;     // Recorder export period
constexpr std::int64_t kWaitNs = 1000000000;      // swap of the log buffers / of a runtime
constexpr std::int64_t kLateResetNs = 100000000;  // pacing restarts after falling this far behind
constexpr std::int64_t kWaitStepNs = 1000000;     // wait granularity of the bridge thread
constexpr double kUsPerS = 1e6;
enum SwapResult : int { kSwapPending, kSwapped, kSwapRefused };

// Config forced to the studio bridge's modes, URDF model and controller model.
struct Arm {
  pipeline::Config<kN> config;
  model::Arm7 urdf;
  model::Arm7 controller;
};

common::Result<Arm> load_arm(const fs::path& yaml) {
  using Out = common::Result<Arm>;
  const common::Result<pipeline::Config<kN>> loaded = pipeline::load_config<kN>(yaml);
  if (!loaded) {
    return Out::error(loaded.reason());
  }
  pipeline::Config<kN> config = loaded.value();
  if (config.algorithm == control::Algorithm::kDrivePosition) {  // owner question C4
    return Out::error(
        "cobot.control.algorithm drive_position is bench only (no drive model on this host)");
  }
  config.command_source = pipeline::CommandSource::kExternal;
  config.log_mode = pipeline::LogMode::kRecorderLive;
  const common::Result<model::UrdfChain> chain =
      model::parse_urdf_chain(config.urdf_path, config.base_link, config.tip_link);
  const common::Result<model::Arm7> arm =
      chain ? model::make_model<kN>(chain.value(), config.gravity_b)
            : common::Result<model::Arm7>::error(chain.reason());
  if (!arm) {
    return Out::error(config.urdf_path.string() + ": " + arm.reason());
  }
  return Out::ok(Arm{config, arm.value(), pipeline::controller_model<kN>(arm.value(), config)});
}

bridge::SessionInfo<kN> session_of(const Arm& arm,
                                   const fs::path& config,
                                   const fs::path& imperfections,
                                   std::uint64_t seed) {
  bridge::SessionInfo<kN> s{};
  bridge::copy_text(s.config_path, bridge::kPathChars, config.string());
  bridge::copy_text(s.imperfections_path, bridge::kPathChars, imperfections.string());
  s.seed = seed;
  s.command_source_external = 1;
  s.control_rate_hz = arm.config.control_rate_hz;
  s.soft_margin_rad = arm.config.safety.soft_margin_rad;
  s.jog_margin_rad = arm.config.manual.jog_margin_rad;
  for (std::size_t i = 0; i < static_cast<std::size_t>(kN); ++i) {
    const auto& joint = arm.controller.joints[i];
    s.q_min[i] = joint.q_min;
    s.q_max[i] = joint.q_max;
    s.qd_max[i] = joint.qd_max;
    s.tau_max[i] = joint.tau_max;
    s.rated[i] = arm.config.limits.rated_torque_nm(static_cast<Eigen::Index>(i));
  }
  s.T_tip_tcp = bridge::to_wire(arm.config.tcp_offset.T_tip_tcp);
  return s;
}

void sleep_until(std::int64_t t_ns) {
  timespec ts{};
  ts.tv_sec = static_cast<time_t>(t_ns / kNsPerS);
  ts.tv_nsec = static_cast<long>(t_ns % kNsPerS);
  while (::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {
  }
}

void nap() {
  std::this_thread::sleep_for(std::chrono::nanoseconds(kWaitStepNs));
}

// What apply config replaces: config, controller model, I/O decorator and loop.
struct Runtime {
  pipeline::Config<kN> config;
  model::Arm7 controller;
  std::unique_ptr<imperfect::ImperfectJointInterface<kN>> io;
  std::unique_ptr<Loop> loop;
};

class Host {
 public:
  Host(ServeOptions options, imperfect::ImperfectionConfig<kN> imperfections, Arm arm)
      : options_(std::move(options)), imp_(std::move(imperfections)), arm_(std::move(arm)) {}
  Host(const Host&) = delete;
  Host& operator=(const Host&) = delete;
  Host(Host&&) = delete;
  Host& operator=(Host&&) = delete;
  // The region is removed only by the host that created it.
  ~Host() {
    service_.reset();
    port_.reset();
    if (server_) {
      server_.reset();
      static_cast<void>(Server::remove(options_.name));
    }
  }

  Status run(const std::atomic<bool>& stop, const std::function<void(const std::string&)>& ready);

 private:
  std::unique_ptr<Runtime> make_runtime(const Arm& arm,
                                        const imperfect::ImperfectionConfig<kN>& imp) {
    auto rt = std::make_unique<Runtime>();
    rt->config = arm.config;
    rt->controller = arm.controller;
    rt->io = std::make_unique<imperfect::ImperfectJointInterface<kN>>(
        *sim_, imp, period_s_, false, arm.config.limits.peak_torque_nm);
    rt->loop = std::make_unique<Loop>(rt->controller, rt->config, recorder_.get());
    return rt;
  }
  common::Result<Service::ExportInfo> export_log();
  common::Result<Applied> apply(const fs::path& config, const fs::path& imperfections);
  void bridge_loop();
  void control_step(hal::JointState<kN>& state, hal::JointCommand<kN>& cmd);

  ServeOptions options_;
  imperfect::ImperfectionConfig<kN> imp_;
  Arm arm_;
  std::uint64_t seed_ = 0;  // the decorator's seed in use (bridge thread after start)
  double period_s_ = 0.0;
  fs::path run_csv_;
  std::unique_ptr<dynamics::SimPlant<kN>> plant_;
  std::unique_ptr<pipeline::SimJointInterface<kN>> sim_;
  std::unique_ptr<pipeline::SimTruthProbe<kN>> probe_;
  std::unique_ptr<pipeline::HeapStorage<kN>> storage_;
  std::unique_ptr<pipeline::Recorder<kN>> recorder_;
  std::unique_ptr<Runtime> current_;  // control thread
  std::atomic<Runtime*> offered_{nullptr};
  std::atomic<Runtime*> retired_{nullptr};
  std::atomic<int> swap_result_{kSwapPending};
  std::atomic<bool> control_running_{true};
  std::atomic<bool> bridge_stop_{false};
  std::mutex export_mutex_;  // one exporter at a time
  std::optional<Server> server_;
  std::unique_ptr<bridge::LoopPort<kN>> port_;
  std::unique_ptr<Service> service_;
};

common::Result<Service::ExportInfo> Host::export_log() {
  using Out = common::Result<Service::ExportInfo>;
  const std::lock_guard<std::mutex> lock(export_mutex_);
  pipeline::Recorder<kN>& rec = *recorder_;
  Status written = Status::ok();
  if (control_running_.load()) {
    // False while an earlier request is pending or its retired buffer is held (that export timed
    // out): wait for that swap and export its buffer, as the controller's export_log does.
    static_cast<void>(rec.request_swap());
    const std::int64_t deadline = bridge::monotonic_ns() + kWaitNs;
    while (!rec.swap_done()) {
      if (!control_running_.load() || bridge::monotonic_ns() > deadline) {
        return Out::error("export: the control loop did not swap the log buffers");
      }
      nap();
    }
    written = pipeline::export_csv<kN>(run_csv_, rec.retired(), true);
    rec.release_retired();
  } else {  // the producer has stopped: the retired buffer (if swapped), then the active one
    if (rec.swap_done()) {
      written = pipeline::export_csv<kN>(run_csv_, rec.retired(), true);
      rec.release_retired();
    }
    const Status active = pipeline::export_csv<kN>(run_csv_, rec.active(), true);
    written = written ? active : written;
    rec.clear();
  }
  if (!written) {
    return Out::error(written.reason());
  }
  std::error_code ec;
  const std::uintmax_t size = fs::file_size(run_csv_, ec);
  if (ec) {
    return Out::error(run_csv_.string() + ": " + ec.message());
  }
  return Out::ok(Service::ExportInfo{run_csv_, static_cast<std::uint64_t>(size)});
}

common::Result<Applied> Host::apply(const fs::path& config, const fs::path& imperfections) {
  using Out = common::Result<Applied>;
  common::Result<imperfect::ImperfectionConfig<kN>> imp =
      pipeline::load_imperfections<kN>(imperfections);
  if (!imp) {
    return Out::error(imp.reason());
  }
  imp.value().seed = options_.seed.value_or(imp.value().seed);  // --seed wins, as at start
  const common::Result<Arm> arm = load_arm(config);
  if (!arm) {
    return Out::error(arm.reason());
  }
  const double rate = arm.value().config.control_rate_hz;
  if (rate != arm_.config.control_rate_hz) {
    return Out::error("apply config: cobot.control_rate_hz " + std::to_string(rate) +
                      " differs from the host's rate " +
                      std::to_string(arm_.config.control_rate_hz));
  }
  const Status comm = imperfect::check_comm(imp.value().comm, period_s_);
  if (!comm) {
    return Out::error(imperfections.string() + ": " + comm.reason());
  }
  // Owned by the control thread once it swapped it in, else taken back here.
  Runtime* const offered = make_runtime(arm.value(), imp.value()).release();
  swap_result_.store(kSwapPending);
  offered_.store(offered, std::memory_order_release);
  const std::int64_t deadline = bridge::monotonic_ns() + kWaitNs;
  while (swap_result_.load(std::memory_order_acquire) == kSwapPending) {
    if (!control_running_.load() || bridge::monotonic_ns() > deadline) {
      Runtime* mine = offered;
      if (offered_.compare_exchange_strong(mine, nullptr)) {
        const std::unique_ptr<Runtime> back(offered);
        return Out::error("apply config: the control loop did not take the new runtime");
      }
    }
    nap();
  }
  if (swap_result_.load() == kSwapRefused) {
    const std::unique_ptr<Runtime> back(offered);
    return Out::error("apply config: not in standby");
  }
  const std::unique_ptr<Runtime> old(retired_.exchange(nullptr));  // destroyed here
  seed_ = imp.value().seed;
  return Out::ok(Applied{arm.value().controller, arm.value().config});
}

void Host::control_step(hal::JointState<kN>& state, hal::JointCommand<kN>& cmd) {
  Runtime* next = offered_.load(std::memory_order_acquire);
  bool swapped = false;
  if (next != nullptr && offered_.compare_exchange_strong(next, nullptr)) {
    swapped = current_->loop->mode() == pipeline::LoopMode::kStandby;
    if (swapped) {
      retired_.store(current_.release(), std::memory_order_release);
      current_.reset(next);
    }
    swap_result_.store(swapped ? kSwapped : kSwapRefused, std::memory_order_release);
  }
  Runtime& rt = *current_;
  // A failed read leaves a stale stamp: the loop's safety logic handles it.
  static_cast<void>(rt.io->read(state));
  if (swapped) {
    static_cast<void>(rt.loop->start(state));
  }
  const pipeline::PlantTruth<kN>& truth = probe_->sample();
  port_->before_step(*rt.loop, state);
  static_cast<void>(rt.loop->step(state, cmd, truth));
  static_cast<void>(rt.io->write(cmd));
  port_->after_step(*rt.loop);
  sim_->advance();
}

void Host::bridge_loop() {
  std::int64_t next_poll = bridge::monotonic_ns();
  std::int64_t next_export = next_poll + kExportNs;
  while (!bridge_stop_.load()) {
    const std::int64_t now = bridge::monotonic_ns();
    service_->poll(now);
    if (now >= next_export) {
      static_cast<void>(export_log());
      next_export = now + kExportNs;
    }
    next_poll += kPollNs;
    if (bridge::monotonic_ns() - next_poll > kLateResetNs) {
      next_poll = bridge::monotonic_ns();
    }
    sleep_until(next_poll);
  }
  server_->clear_owner();
}

Status Host::run(const std::atomic<bool>& stop,
                 const std::function<void(const std::string&)>& ready) {
  const pipeline::Config<kN>& config = arm_.config;
  period_s_ = 1.0 / config.control_rate_hz;
  const auto period_ns = std::llround(period_s_ * static_cast<double>(kNsPerS));
  const Status comm = imperfect::check_comm(imp_.comm, period_s_);
  if (!comm) {
    return Status::error(options_.imperfections_yaml.string() + ": " + comm.reason());
  }
  std::error_code ec;
  fs::create_directories(options_.out_dir / "segments", ec);
  if (ec) {
    return Status::error(options_.out_dir.string() + ": " + ec.message());
  }
  run_csv_ = options_.out_dir / "run.csv";
  fs::remove(run_csv_, ec);
  // The plant as run_profile builds it: perturbed URDF model, plant effects, timing jitter.
  const model::Arm7 plant_model = imperfect::perturb_model(arm_.urdf, imp_.model, imp_.seed);
  plant_ = std::make_unique<dynamics::SimPlant<kN>>(
      plant_model, config.home_q_rad, imperfect::plant_effects(imp_));
  sim_ =
      std::make_unique<pipeline::SimJointInterface<kN>>(*plant_,
                                                        period_s_,
                                                        imp_.timing.period_jitter_std_us / kUsPerS,
                                                        imp_.seed,
                                                        imp_.timing.overrun_prob,
                                                        imp_.timing.overrun_cycles);
  probe_ = std::make_unique<pipeline::SimTruthProbe<kN>>(*plant_);
  const auto rows = static_cast<std::size_t>(
      std::max<long long>(std::llround(config.log_capacity_s * config.control_rate_hz), 1));
  storage_ = std::make_unique<pipeline::HeapStorage<kN>>(rows, 2);
  recorder_ = std::make_unique<pipeline::Recorder<kN>>(storage_->memory(),
                                                       pipeline::FullPolicy::kOverwriteOldest);
  current_ = make_runtime(arm_, imp_);

  const fs::path config_path = fs::absolute(options_.config_yaml, ec);
  const fs::path imp_path = fs::absolute(options_.imperfections_yaml, ec);
  seed_ = imp_.seed;
  const bridge::SessionInfo<kN> session = session_of(arm_, config_path, imp_path, seed_);
  common::Result<Server> created = Server::create(options_.name, session);
  if (!created) {
    return Status::error(created.reason());
  }
  server_.emplace(std::move(created.value()));
  port_ = std::make_unique<bridge::LoopPort<kN>>(&*server_);
  Service::Hooks hooks;
  hooks.export_log = [this] { return export_log(); };
  hooks.apply_config = [this](const fs::path& c, const fs::path& i) { return apply(c, i); };
  hooks.seed = [this] { return seed_; };
  service_ = std::make_unique<Service>(*server_,
                                       *port_,
                                       arm_.controller,
                                       config,
                                       session,
                                       options_.out_dir / "segments",
                                       std::move(hooks));
  hal::JointState<kN> state;
  hal::JointCommand<kN> cmd;
  static_cast<void>(current_->io->read(state));
  Status started = current_->loop->start(state);
  if (!started) {
    return started;
  }
  if (ready) {
    ready("/cobot_" + options_.name);
  }
  std::thread bridge_thread([this] { bridge_loop(); });
  const std::int64_t steps =
      options_.duration_s ? std::llround(*options_.duration_s * config.control_rate_hz) : -1;
  std::int64_t deadline = bridge::monotonic_ns();
  for (std::int64_t k = 0; (steps < 0 || k < steps) && !stop.load(); ++k) {
    deadline += period_ns;
    if (bridge::monotonic_ns() - deadline > kLateResetNs) {
      deadline = bridge::monotonic_ns();
    }
    sleep_until(deadline);
    control_step(state, cmd);
  }
  control_running_.store(false);
  bridge_stop_.store(true);
  bridge_thread.join();
  const common::Result<Service::ExportInfo> last = export_log();
  const std::unique_ptr<Runtime> old(retired_.exchange(nullptr));
  return last ? Status::ok() : Status::error(last.reason());
}

}  // namespace

Status serve(const ServeOptions& options,
             const std::atomic<bool>& stop,
             const std::function<void(const std::string&)>& ready) {
  if (options.duration_s && !(std::isfinite(*options.duration_s) && *options.duration_s > 0.0)) {
    return Status::error("duration_s must be > 0, got " + std::to_string(*options.duration_s));
  }
  common::Result<imperfect::ImperfectionConfig<kN>> imp =
      pipeline::load_imperfections<kN>(options.imperfections_yaml);
  if (!imp) {
    return Status::error(imp.reason());
  }
  imp.value().seed = options.seed.value_or(imp.value().seed);
  const common::Result<Arm> arm = load_arm(options.config_yaml);
  if (!arm) {
    return Status::error(arm.reason());
  }
  try {
    Host host(options, imp.value(), arm.value());
    return host.run(stop, ready);
  } catch (const std::exception& e) {
    return Status::error("serve: " + std::string(e.what()));
  }
}

}  // namespace cobot::bench
