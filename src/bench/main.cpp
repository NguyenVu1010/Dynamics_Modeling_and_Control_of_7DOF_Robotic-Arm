// cobot_bench: the control loop against SimPlant without ROS (design 11.2).
// Exit 0 on a completed run (tools/run_e2e.py judges it), 2 on bad arguments or configuration
// (a bad waypoint file included) with the reason on stderr, 1 when `run --waypoints` rejects the
// list (one line per issue on stderr, no Run CSV written). `serve` (feature controller-bridge)
// prints "serving /cobot_<name>" and runs until SIGINT / SIGTERM or --duration-s. `tune` (feature
// auto-tune) exits 0 with a result (also when no level is admissible) and 1 on bad input.
#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <exception>
#include <filesystem>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/task.hpp"

#include "engine.hpp"
#include "run.hpp"
#include "run_data.hpp"
#include "serve.hpp"
#include "tune.hpp"

namespace {

constexpr int kExitPathInvalid = 1;
constexpr int kExitBadInput = 2;

const char* const kUsage =
    "usage: cobot_bench run --config <yaml> --imperfections <yaml> [--seed <u64>] --out <csv> "
    "[--duration-s <s>] [--waypoints <yaml>]\n"
    "       cobot_bench montecarlo --config <yaml> --imperfections <yaml> --runs <n> "
    "--seed0 <u64> --out <dir> [--keep-runs] [--duration-s <s>]\n"
    "       cobot_bench sweep --config <yaml> --sweep <yaml> --out <dir> [--duration-s <s>]\n"
    "       cobot_bench timing --config <yaml> --steps <n> [--paced] "
    "[--log-mode off|recorder|recorder_live]\n"
    "       cobot_bench serve --config <yaml> --imperfections <yaml> [--seed <u64>] "
    "[--name <name>] [--out <dir>] [--duration-s <s>]\n"
    "       cobot_bench tune --hardware <model.yaml> --algorithm <name> --task <task.yaml> "
    "[--levels a,b,c] [--seeds <n>] [--jobs <n>] --out <dir>\n";

const char* const kKeepRuns = "--keep-runs";
const char* const kPaced = "--paced";
const char* const kLogMode = "--log-mode";

// "--name value" pairs after the mode (--keep-runs and --paced take no value and map to ""); throws
// std::invalid_argument on a bad command line.
std::map<std::string, std::string> parse_flags(int argc,
                                               char** argv,
                                               const std::set<std::string>& known,
                                               const std::set<std::string>& required) {
  std::map<std::string, std::string> flags;
  for (int i = 2; i < argc; i += 2) {
    const std::string name = argv[i];
    const bool is_switch = name == kKeepRuns || name == kPaced;
    if (known.count(name) == 0 || (!is_switch && i + 1 >= argc) ||
        !flags.emplace(name, is_switch ? "" : argv[i + 1]).second) {
      throw std::invalid_argument("unknown, repeated or valueless argument " + name);
    }
    i -= is_switch ? 1 : 0;
  }
  for (const std::string& name : required) {
    if (flags.count(name) == 0) {
      throw std::invalid_argument("missing " + name);
    }
  }
  return flags;
}

// Whole-string unsigned integer (std::stoull alone accepts "12x" and "-1").
std::uint64_t parse_u64(const std::string& text) {
  std::size_t used = 0;
  const std::uint64_t v = std::stoull(text, &used);
  if (used != text.size() || text.find('-') != std::string::npos) {
    throw std::invalid_argument("not an unsigned integer: " + text);
  }
  return v;
}

std::optional<double> parse_duration(const std::map<std::string, std::string>& f) {
  if (f.count("--duration-s") == 0) {
    return std::nullopt;
  }
  std::size_t used = 0;
  const double v = std::stod(f.at("--duration-s"), &used);
  if (used != f.at("--duration-s").size()) {
    throw std::invalid_argument("not a number: " + f.at("--duration-s"));
  }
  return v;
}

cobot::pipeline::LogMode parse_log_mode(const std::string& text) {
  if (text == "off") {
    return cobot::pipeline::LogMode::kOff;
  }
  if (text == "recorder") {
    return cobot::pipeline::LogMode::kRecorder;
  }
  if (text == "recorder_live") {
    return cobot::pipeline::LogMode::kRecorderLive;
  }
  throw std::invalid_argument("--log-mode: expected off, recorder or recorder_live, got " + text);
}

const char* log_mode_name(cobot::pipeline::LogMode mode) {
  switch (mode) {
    case cobot::pipeline::LogMode::kOff:
      return "off";
    case cobot::pipeline::LogMode::kRecorder:
      return "recorder";
    case cobot::pipeline::LogMode::kRecorderLive:
      return "recorder_live";
  }
  return "recorder";
}

// cobot.log.mode of a config that bench::timing already loaded.
cobot::pipeline::LogMode configured_log_mode(const std::string& config_yaml) {
  const auto config = cobot::pipeline::load_config<cobot::bench::kN>(config_yaml);
  return config ? config.value().log_mode : cobot::pipeline::LogMode::kRecorder;
}

int report(const char* mode, const cobot::common::Status& status) {
  if (!status) {
    std::fprintf(stderr, "cobot_bench %s: %s\n", mode, status.reason().c_str());
    return kExitBadInput;
  }
  return 0;
}

// cobot_bench serve: SIGINT / SIGTERM end the run (a final export follows).
std::atomic<bool> g_stop{false};

extern "C" void on_stop_signal(int /*signal*/) {
  g_stop.store(true);
}

// ./results/serve-<YYYYmmdd-HHMMSS>
std::filesystem::path default_serve_dir() {
  const std::time_t now = std::time(nullptr);
  std::tm local{};
  localtime_r(&now, &local);
  char stamp[32] = {};
  std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &local);
  return std::filesystem::path("results") / (std::string("serve-") + stamp);
}

int serve_main(int argc, char** argv) {
  const auto f =
      parse_flags(argc,
                  argv,
                  {"--config", "--imperfections", "--seed", "--name", "--out", "--duration-s"},
                  {"--config", "--imperfections"});
  cobot::bench::ServeOptions options;
  options.config_yaml = f.at("--config");
  options.imperfections_yaml = f.at("--imperfections");
  if (f.count("--seed") > 0) {
    options.seed = parse_u64(f.at("--seed"));
  }
  if (f.count("--name") > 0) {
    options.name = f.at("--name");
  }
  options.out_dir =
      f.count("--out") > 0 ? std::filesystem::path(f.at("--out")) : default_serve_dir();
  options.duration_s = parse_duration(f);
  std::signal(SIGINT, on_stop_signal);
  std::signal(SIGTERM, on_stop_signal);
  const cobot::common::Status st =
      cobot::bench::serve(options, g_stop, [](const std::string& region) {
        std::printf("serving %s\n", region.c_str());
        std::fflush(stdout);
      });
  return report("serve", st);
}

// `text` as a whole-string number of `flag` (std::invalid_argument naming the flag otherwise).
std::uint64_t parse_count(const std::string& flag, const std::string& text) {
  try {
    return parse_u64(text);
  } catch (const std::exception&) {
    throw std::invalid_argument(flag + ": not an unsigned integer: " + text);
  }
}

// "a,b,c": every element a whole-string number.
std::vector<double> parse_levels(const std::string& text) {
  std::vector<double> levels;
  std::size_t begin = 0;
  for (;;) {
    const std::size_t comma = text.find(',', begin);
    const std::string item = text.substr(begin, comma - begin);
    std::size_t used = 0;
    double v = 0.0;
    try {
      v = std::stod(item, &used);
    } catch (const std::exception&) {
      used = 0;
    }
    if (item.empty() || used != item.size()) {
      throw std::invalid_argument("--levels: expected numbers separated by commas, got " + text);
    }
    levels.push_back(v);
    if (comma == std::string::npos) {
      return levels;
    }
    begin = comma + 1;
  }
}

void print_tune(const cobot::bench::TuneResult& t) {
  std::printf("%s (%s)\n%8s %5s %4s %12s %6s %12s %12s %6s\n",
              t.key.c_str(),
              cobot::control::to_string(t.algorithm),
              "level",
              "seeds",
              "held",
              "sat_frac_max",
              "decays",
              "rms_median",
              "final_max",
              "chosen");
  for (const cobot::bench::TuneRow& r : t.rows) {
    const bool chosen = t.chosen && *t.chosen == r.level;
    std::printf("%8g %5zu %4zu %12.4g %6d %12.4g %12.4g %6d\n",
                r.level,
                r.seeds,
                r.held,
                r.sat_frac_max,
                r.decays_all ? 1 : 0,
                r.rms_joint_err_median,
                r.final_pos_err_max,
                chosen ? 1 : 0);
  }
  if (t.chosen) {
    std::printf("chosen: %g\n", *t.chosen);
  } else {
    std::printf("chosen: none\n");
  }
  std::printf("reason: %s\n", t.reason.c_str());
}

int tune_main(int argc, char** argv) {
  constexpr int kExitTuneBadInput = 1;
  try {
    const auto f = parse_flags(
        argc,
        argv,
        {"--hardware", "--algorithm", "--task", "--levels", "--seeds", "--jobs", "--out"},
        {"--hardware", "--algorithm", "--task", "--out"});
    const auto algorithm = cobot::control::algorithm_from_string(f.at("--algorithm"));
    if (!algorithm) {
      throw std::invalid_argument("--algorithm: unknown " + f.at("--algorithm") + " (registered: " +
                                  cobot::control::registered_algorithm_names() + ")");
    }
    cobot::bench::TuneOptions options;
    if (f.count("--levels") > 0) {
      options.levels = parse_levels(f.at("--levels"));
    }
    if (f.count("--seeds") > 0) {
      const std::uint64_t n = parse_count("--seeds", f.at("--seeds"));
      if (n == 0) {
        throw std::invalid_argument("--seeds must be >= 1");
      }
      for (std::uint64_t s = 1; s <= n; ++s) {
        options.seeds.push_back(s);
      }
    }
    if (f.count("--jobs") > 0) {
      const std::uint64_t n = parse_count("--jobs", f.at("--jobs"));
      if (n > std::numeric_limits<unsigned>::max()) {
        throw std::invalid_argument("--jobs: out of range: " + f.at("--jobs"));
      }
      options.jobs = static_cast<unsigned>(n);
    }
    const auto model = cobot::pipeline::load_hardware_model(f.at("--hardware"));
    if (!model) {
      throw std::invalid_argument(model.reason());
    }
    const auto task = cobot::pipeline::load_task(f.at("--task"));
    if (!task) {
      throw std::invalid_argument(task.reason());
    }
    const auto result = cobot::bench::tune(model.value(), *algorithm, task.value(), options);
    if (!result) {
      throw std::invalid_argument(result.reason());
    }
    const std::filesystem::path out_dir = f.at("--out");
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    const cobot::common::Status written =
        cobot::bench::write_tune_csv(out_dir / "tune.csv", result.value());
    if (!written) {
      throw std::invalid_argument(written.reason());
    }
    print_tune(result.value());
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "cobot_bench tune: %s\n%s", e.what(), kUsage);
    return kExitTuneBadInput;
  }
}

int main_impl(int argc, char** argv) {
  const std::string mode = argc > 1 ? argv[1] : "";
  if (mode == "run") {
    const auto f = parse_flags(
        argc,
        argv,
        {"--config", "--imperfections", "--seed", "--out", "--duration-s", "--waypoints"},
        {"--config", "--imperfections", "--out"});
    std::optional<std::uint64_t> seed;
    if (f.count("--seed") > 0) {
      seed = parse_u64(f.at("--seed"));
    }
    std::optional<std::filesystem::path> waypoints;
    if (f.count("--waypoints") > 0) {
      waypoints = f.at("--waypoints");
    }
    bool path_invalid = false;
    const int code = report("run",
                            cobot::bench::run_checked(f.at("--config"),
                                                      f.at("--imperfections"),
                                                      seed,
                                                      f.at("--out"),
                                                      parse_duration(f),
                                                      waypoints,
                                                      path_invalid));
    return path_invalid ? kExitPathInvalid : code;
  }
  if (mode == "serve") {
    return serve_main(argc, argv);
  }
  if (mode == "tune") {
    return tune_main(argc, argv);
  }
  if (mode == "montecarlo") {
    const auto f = parse_flags(
        argc,
        argv,
        {"--config", "--imperfections", "--runs", "--seed0", "--out", kKeepRuns, "--duration-s"},
        {"--config", "--imperfections", "--runs", "--seed0", "--out"});
    return report("montecarlo",
                  cobot::bench::montecarlo(f.at("--config"),
                                           f.at("--imperfections"),
                                           parse_u64(f.at("--runs")),
                                           parse_u64(f.at("--seed0")),
                                           f.at("--out"),
                                           f.count(kKeepRuns) > 0,
                                           parse_duration(f)));
  }
  if (mode == "sweep") {
    const auto f = parse_flags(argc,
                               argv,
                               {"--config", "--sweep", "--out", "--duration-s"},
                               {"--config", "--sweep", "--out"});
    return report(
        "sweep",
        cobot::bench::sweep(f.at("--config"), f.at("--sweep"), f.at("--out"), parse_duration(f)));
  }
  if (mode == "timing") {
    const auto f =
        parse_flags(argc, argv, {"--config", "--steps", kPaced, kLogMode}, {"--config", "--steps"});
    const std::uint64_t steps = parse_u64(f.at("--steps"));
    cobot::bench::TimingOptions options;
    options.paced = f.count(kPaced) > 0;
    if (f.count(kLogMode) > 0) {
      options.log_mode = parse_log_mode(f.at(kLogMode));
    }
    const bool pf01 = !options.paced && !options.log_mode;
    const cobot::common::Result<cobot::bench::TimingStats> stats =
        cobot::bench::timing(f.at("--config"), static_cast<std::int64_t>(steps), options);
    if (!stats) {
      std::fprintf(stderr, "cobot_bench timing: %s\n", stats.reason().c_str());
      return kExitBadInput;
    }
    const cobot::bench::TimingStats& s = stats.value();
    if (pf01) {  // PF-01 line, unchanged
      std::printf("p50_us=%.3f p95_us=%.3f max_us=%.3f\n", s.p50_us, s.p95_us, s.max_us);
      return 0;
    }
    std::printf(
        "p50_us=%.3f p95_us=%.3f p99_us=%.3f max_us=%.3f log_mode=%s paced=%d late_cycles=%llu "
        "late_p99_us=%.3f reader_samples=%llu reader_lost=%llu reader_cpu=%s\n",
        s.p50_us,
        s.p95_us,
        s.p99_us,
        s.max_us,
        log_mode_name(options.log_mode ? *options.log_mode : configured_log_mode(f.at("--config"))),
        options.paced ? 1 : 0,
        static_cast<unsigned long long>(s.late_cycles),
        s.late_p99_us,
        static_cast<unsigned long long>(s.reader_samples),
        static_cast<unsigned long long>(s.reader_lost),
        s.reader_on_other_cpu ? "other" : "shared");
    return 0;
  }
  throw std::invalid_argument("unknown mode '" + mode + "'");
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return main_impl(argc, argv);
  } catch (const std::exception& e) {  // std::stoull / std::stod throw on bad numbers too
    std::fprintf(stderr, "cobot_bench: %s\n%s", e.what(), kUsage);
    return kExitBadInput;
  }
}
