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

#include "cobot/common/lie.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/imperfect/perturb.hpp"
#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"

#include "engine.hpp"

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
constexpr double kSatFraction = 0.999;  // cobot_eval.metrics.SAT_FRACTION

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

// RunSummary of the logged rows, fed one record at a time by the writer thread.
class Metrics {
 public:
  Metrics(const Vec& tau_max, std::int64_t t_eval_ns) : t_eval_ns_(t_eval_ns) {
    tau_max_ = tau_max;
  }

  void add(const pipeline::LogRecord<kN>& r) {
    if (rows_ > 0) {
      last_dt_ns_ = r.t.ns - prev_t_ns_;
      sat_ns_ += prev_sat_ ? last_dt_ns_ : 0;
    }
    if (rows_ == 0 || r.t.ns <= t_eval_ns_) {
      final_e_ = r.T_b_e;
      final_d_ = r.T_b_d;
    }
    // PropagateNaN and the explicit isnan keep a NaN torque visible in the summary
    // (std::max and the default maxCoeff may drop it).
    const double ratio =
        (r.tau.cwiseAbs().array() / tau_max_.array()).maxCoeff<Eigen::PropagateNaN>();
    if (std::isnan(ratio) || ratio > s_.peak_tau_ratio) {
      s_.peak_tau_ratio = std::isnan(s_.peak_tau_ratio) ? s_.peak_tau_ratio : ratio;
    }
    prev_sat_ = ratio >= kSatFraction;
    prev_t_ns_ = r.t.ns;
    ++rows_;
    s_.held = s_.held || r.mode == pipeline::LoopMode::kHolding;
    if (r.mode == pipeline::LoopMode::kTracking || r.mode == pipeline::LoopMode::kFinished) {
      const Vec e = r.q_ref - r.q;
      sum_sq_ += e.cwiseProduct(e);
      ++error_rows_;
    }
  }

  RunSummary finish() const {
    RunSummary s = s_;
    s.final_pos_err_m = (final_d_.p - final_e_.p).norm();
    s.final_rot_err_rad = common::so3_log(final_d_.R * final_e_.R.transpose()).norm();
    if (error_rows_ > 0) {
      const Vec mean_sq = sum_sq_ / static_cast<double>(error_rows_);
      s.rms_joint_err_rad = mean_sq.cwiseSqrt().maxCoeff<Eigen::PropagateNaN>();
    }
    const std::int64_t sat_ns = sat_ns_ + (prev_sat_ && rows_ > 1 ? last_dt_ns_ : 0);
    s.sat_time_s = static_cast<double>(sat_ns) / static_cast<double>(common::kNanosecondsPerSecond);
    return s;
  }

 private:
  Vec tau_max_ = Vec::Zero();
  std::int64_t t_eval_ns_;
  RunSummary s_;
  common::Pose final_e_;
  common::Pose final_d_;
  Vec sum_sq_ = Vec::Zero();
  std::size_t rows_ = 0;
  std::size_t error_rows_ = 0;
  std::int64_t prev_t_ns_ = 0;
  std::int64_t last_dt_ns_ = 0;
  std::int64_t sat_ns_ = 0;
  bool prev_sat_ = false;
};

}  // namespace

Status run(const std::filesystem::path& config_yaml,
           const std::filesystem::path& imperfections_yaml,
           std::optional<std::uint64_t> seed,
           const std::filesystem::path& out_csv,
           std::optional<double> duration_s) {
  if (out_csv.empty()) {  // run_profile treats an empty path as "no Run CSV"; run must write one
    return Status::error("--out must name a file");
  }
  common::Result<imperfect::ImperfectionConfig<kN>> imp =
      pipeline::load_imperfections<kN>(imperfections_yaml);
  if (!imp) {
    return Status::error(imp.reason());
  }
  imp.value().seed = seed.value_or(imp.value().seed);
  const common::Result<RunSummary> result =
      run_profile(config_yaml, imp.value(), imperfections_yaml, out_csv, duration_s);
  return result ? Status::ok() : Status::error(result.reason());
}

common::Result<RunSummary> run_profile(const std::filesystem::path& config_yaml,
                                       const imperfect::ImperfectionConfig<kN>& imperfections,
                                       const std::filesystem::path& label,
                                       const std::filesystem::path& out_csv,
                                       std::optional<double> duration_s) {
  using Out = common::Result<RunSummary>;
  const common::Result<Setup> setup = load_setup(config_yaml);
  if (!setup) {
    return Out::error(setup.reason());
  }
  const pipeline::Config<kN>& config = setup.value().config;
  const double duration =
      duration_s.value_or(config.trajectory.duration_s + config.trajectory.hold_s);
  if (!std::isfinite(duration) || duration <= 0.0) {
    return Out::error("duration_s must be > 0, got " + std::to_string(duration));
  }
  const std::int64_t cycles = std::llround(duration * config.control_rate_hz);
  const double period_s = 1.0 / config.control_rate_hz;
  const bool keep = !out_csv.empty();
  std::ofstream csv;
  std::ofstream meta;
  if (keep) {
    csv.open(out_csv, std::ios::binary);
    meta.open(out_csv.string() + ".meta.yaml", std::ios::binary);
    if (!csv || !meta) {
      return Out::error("cannot open " + out_csv.string() + " (or its .meta.yaml) for writing");
    }
  }
  Vec tau_max;
  for (int i = 0; i < kN; ++i) {
    tau_max(i) = setup.value().nominal.joints[static_cast<std::size_t>(i)].tau_max;
  }
  const double t_eval_s = config.trajectory.duration_s + config.trajectory.hold_s;
  Metrics metrics(tau_max,
                  std::llround(t_eval_s * static_cast<double>(common::kNanosecondsPerSecond)));
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
         << "imperfections: '" << label.string() << "'\n"
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
      return Out::error(started.reason());
    }
    // Writer thread: drains the ring into the CSV; reads `done` before draining, so every
    // record pushed before done is set gets written.
    if (keep) {
      pipeline::write_csv_header<kN>(csv);
    }
    std::atomic<bool> done{false};
    std::atomic<std::size_t> written{0};
    std::thread writer([&] {
      pipeline::LogRecord<kN> r;
      bool last = false;
      while (!last) {
        last = done.load(std::memory_order_acquire);
        bool any = false;
        while (loop->log().pop(r)) {
          if (keep) {
            pipeline::write_csv_row<kN>(csv, r);
          }
          metrics.add(r);
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
      return Out::error(label.string() + ": log rows dropped");
    }
  } catch (const std::exception& e) {
    return Out::error(label.string() + ": " + e.what());
  }
  csv.flush();
  meta.flush();
  if (keep && (!csv || !meta)) {
    return Out::error("cannot write " + out_csv.string());
  }
  return Out::ok(metrics.finish());
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
