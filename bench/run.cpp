#include "run.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/imperfect/perturb.hpp"
#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"

#ifndef COBOT_GIT_DESCRIBE
#define COBOT_GIT_DESCRIBE ""
#endif

namespace cobot::bench {
namespace {

constexpr int kN = model::kArmDof;
using Vec = common::VecN<kN>;
using Loop = pipeline::ControlLoop<kN>;
using common::Status;

constexpr double kUsPerS = 1e6;
// The control thread waits for the writer at this many records in flight, so a slow disk
// never drops a row (the ring holds kLogCapacity - 1 records).
constexpr std::size_t kMaxInFlight = pipeline::kLogCapacity - 2;
constexpr std::chrono::microseconds kWriterIdle{200};

struct Setup {
  pipeline::Config<kN> config;
  model::Arm7 nominal;
};

common::Result<Setup> load_setup(const std::filesystem::path& config_yaml) {
  using Out = common::Result<Setup>;
  const common::Result<pipeline::Config<kN>> config = pipeline::load_config<kN>(config_yaml);
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
  return Out::ok(Setup{c, arm.value()});
}

void write_list(std::ostream& os, const char* key, const Vec& v) {
  os << key << ": [";
  for (int i = 0; i < kN; ++i) {
    os << (i > 0 ? ", " : "") << v(i);
  }
  os << "]\n";
}

}  // namespace

Status run(const std::filesystem::path& config_yaml,
           const std::filesystem::path& imperfections_yaml,
           std::optional<std::uint64_t> seed,
           const std::filesystem::path& out_csv,
           std::optional<double> duration_s) {
  const common::Result<Setup> setup = load_setup(config_yaml);
  if (!setup) {
    return Status::error(setup.reason());
  }
  common::Result<imperfect::ImperfectionConfig<kN>> imp =
      pipeline::load_imperfections<kN>(imperfections_yaml);
  if (!imp) {
    return Status::error(imp.reason());
  }
  const pipeline::Config<kN>& config = setup.value().config;
  imperfect::ImperfectionConfig<kN>& imperfections = imp.value();
  imperfections.seed = seed.value_or(imperfections.seed);
  const double duration =
      duration_s.value_or(config.trajectory.duration_s + config.trajectory.hold_s);
  if (!std::isfinite(duration) || duration <= 0.0) {
    return Status::error("duration_s must be > 0, got " + std::to_string(duration));
  }
  const std::int64_t cycles = std::llround(duration * config.control_rate_hz);
  const double period_s = 1.0 / config.control_rate_hz;
  const std::string meta_path = out_csv.string() + ".meta.yaml";
  std::ofstream csv(out_csv, std::ios::binary);
  std::ofstream meta(meta_path, std::ios::binary);
  if (!csv || !meta) {
    return Status::error("cannot open " + out_csv.string() + " (or its .meta.yaml) for writing");
  }
  try {
    // The controller keeps the nominal model; the plant gets the perturbed one, with friction
    // and motor lag, so the I/O decorator does not apply them again.
    const model::Arm7 plant_model =
        imperfect::perturb_model(setup.value().nominal, imperfections.model, imperfections.seed);
    dynamics::SimPlant<kN> plant(
        plant_model, config.home_q_rad, imperfect::plant_effects(imperfections));
    pipeline::SimJointInterface<kN> sim(
        plant, period_s, imperfections.timing.period_jitter_std_us / kUsPerS, imperfections.seed);
    imperfect::ImperfectJointInterface<kN> io(sim, imperfections, period_s, false);
    const std::unique_ptr<Loop> loop = std::make_unique<Loop>(setup.value().nominal, config);

    Vec mass;
    for (int i = 0; i < kN; ++i) {
      mass(i) = plant_model.links[static_cast<std::size_t>(i)].mass_kg;
    }
    meta << std::setprecision(17) << "config: '" << config_yaml.string() << "'\n"
         << "imperfections: '" << imperfections_yaml.string() << "'\n"
         << "seed: " << imperfections.seed << "\n";
    write_list(meta, "q_offset_rad", io.q_offset_rad());
    write_list(meta, "torque_scale", io.torque_scale());
    write_list(meta, "plant_link_mass_kg", mass);
    if (!std::string(COBOT_GIT_DESCRIBE).empty()) {
      meta << "git: '" << COBOT_GIT_DESCRIBE << "'\n";
    }

    hal::JointState<kN> state;
    hal::JointCommand<kN> cmd;
    static_cast<void>(io.read(state));
    const Status started = loop->start(state);
    if (!started) {
      return Status::error(started.reason());
    }
    // Writer thread: drains the ring into the CSV; reads `done` before draining, so every
    // record pushed before done is set gets written.
    pipeline::write_csv_header<kN>(csv);
    std::atomic<bool> done{false};
    std::atomic<std::size_t> written{0};
    std::thread writer([&] {
      pipeline::LogRecord<kN> r;
      bool last = false;
      while (!last) {
        last = done.load(std::memory_order_acquire);
        bool any = false;
        while (loop->log().pop(r)) {
          pipeline::write_csv_row<kN>(csv, r);
          written.fetch_add(1, std::memory_order_release);
          any = true;
        }
        if (!any && !last) {
          std::this_thread::sleep_for(kWriterIdle);
        }
      }
    });
    for (std::int64_t k = 0; k < cycles; ++k) {
      while (static_cast<std::size_t>(k) - written.load(std::memory_order_acquire) >=
             kMaxInFlight) {
        std::this_thread::yield();
      }
      // A failed read leaves a stale stamp and a fault zero torque: the loop's safety logic
      // handles both and the CSV records the mode.
      static_cast<void>(io.read(state));
      static_cast<void>(loop->step(state, cmd));
      static_cast<void>(io.write(cmd));
      sim.advance();
    }
    done.store(true, std::memory_order_release);
    writer.join();
    if (loop->log().dropped() > 0) {
      return Status::error(out_csv.string() + ": log rows dropped");
    }
  } catch (const std::exception& e) {
    return Status::error(imperfections_yaml.string() + ": " + e.what());
  }
  csv.flush();
  meta.flush();
  return csv && meta ? Status::ok() : Status::error("cannot write " + out_csv.string());
}

common::Result<TimingStats> timing(const std::filesystem::path& config_yaml, std::int64_t steps) {
  using Out = common::Result<TimingStats>;
  const common::Result<Setup> setup = load_setup(config_yaml);
  if (!setup || steps <= 0) {
    return Out::error(setup ? "steps must be > 0" : setup.reason());
  }
  pipeline::Config<kN> config = setup.value().config;
  config.deterministic = false;
  const double period_s = 1.0 / config.control_rate_hz;
  try {
    dynamics::SimPlant<kN> plant(setup.value().nominal, config.home_q_rad);
    pipeline::SimJointInterface<kN> sim(plant, period_s);
    const std::unique_ptr<Loop> loop = std::make_unique<Loop>(setup.value().nominal, config);
    hal::JointState<kN> state;
    hal::JointCommand<kN> cmd;
    pipeline::LogRecord<kN> record;
    static_cast<void>(sim.read(state));
    const Status started = loop->start(state);
    if (!started) {
      return Out::error(started.reason());
    }
    std::vector<double> us(static_cast<std::size_t>(steps));
    for (double& sample : us) {
      static_cast<void>(sim.read(state));
      const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
      static_cast<void>(loop->step(state, cmd));
      const std::chrono::duration<double, std::micro> dt = std::chrono::steady_clock::now() - t0;
      sample = dt.count();
      static_cast<void>(sim.write(cmd));
      sim.advance();
      while (loop->log().pop(record)) {
      }
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
