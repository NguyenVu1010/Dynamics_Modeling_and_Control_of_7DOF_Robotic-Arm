// cobot_bench: the control loop against SimPlant without ROS (design 11.2).
// Exit 0 on a completed run (tools/run_e2e.py judges it), 2 on bad arguments or configuration
// (a bad waypoint file included) with the reason on stderr, 1 when `run --waypoints` rejects the
// list (one line per issue on stderr, no Run CSV written). `serve` (feature controller-bridge)
// prints "serving /cobot_<name>" and runs until SIGINT / SIGTERM or --duration-s. `tune` (feature
// auto-tune) exits 0 with a result (also when no level is admissible) and 1 on bad input. `verify`
// (feature verify) exits 0 for every verdict (a FAIL is a result) and 1 on bad input, no file
// written; with lists of models and algorithms (feature verify-analysis) it compares the pairs.
#include <algorithm>
#include <atomic>
#include <chrono>
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

#include "compare.hpp"
#include "engine.hpp"
#include "report.hpp"
#include "run.hpp"
#include "run_data.hpp"
#include "serve.hpp"
#include "tune.hpp"
#include "verify.hpp"
#include "verify_outputs.hpp"

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
    "[--levels a,b,c] [--seeds <n>] [--jobs <n>] --out <dir>\n"
    "       cobot_bench verify --hardware <model.yaml>[,<model.yaml>...] --algorithm "
    "<name>[,<name>...] --task <task.yaml> [--runs <n>] [--seed0 <u64>] [--quick] [--no-tune] "
    "[--levels a,b,c] [--tune-seeds <n>] [--no-sensitivity] [--sensitivity-seeds <n>] "
    "[--ablation-seeds <n>] [--jobs <n>] --out <dir>\n";

const char* const kKeepRuns = "--keep-runs";
const char* const kPaced = "--paced";
const char* const kQuick = "--quick";
const char* const kNoTune = "--no-tune";
const char* const kNoSensitivity = "--no-sensitivity";
const char* const kLogMode = "--log-mode";

// "--name value" pairs after the mode (the switches --keep-runs, --paced, --quick, --no-tune and
// --no-sensitivity take no value and map to ""); throws std::invalid_argument on a bad command
// line.
std::map<std::string, std::string> parse_flags(int argc,
                                               char** argv,
                                               const std::set<std::string>& known,
                                               const std::set<std::string>& required) {
  std::map<std::string, std::string> flags;
  for (int i = 2; i < argc; i += 2) {
    const std::string name = argv[i];
    const bool is_switch = name == kKeepRuns || name == kPaced || name == kQuick ||
                           name == kNoTune || name == kNoSensitivity;
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

// --jobs of tune and verify.
unsigned parse_jobs(const std::string& text) {
  const std::uint64_t n = parse_count("--jobs", text);
  if (n > std::numeric_limits<unsigned>::max()) {
    throw std::invalid_argument("--jobs: out of range: " + text);
  }
  return static_cast<unsigned>(n);
}

// A count of `flag` that must be >= 1.
std::uint64_t parse_positive(const std::string& flag, const std::string& text) {
  const std::uint64_t n = parse_count(flag, text);
  if (n == 0) {
    throw std::invalid_argument(flag + " must be >= 1");
  }
  return n;
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
      const std::uint64_t n = parse_positive("--seeds", f.at("--seeds"));
      for (std::uint64_t s = 1; s <= n; ++s) {
        options.seeds.push_back(s);
      }
    }
    if (f.count("--jobs") > 0) {
      options.jobs = parse_jobs(f.at("--jobs"));
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

// "a,b,c" of `flag`: every item non-empty (std::invalid_argument naming the flag otherwise).
std::vector<std::string> parse_list(const std::string& flag, const std::string& text) {
  std::vector<std::string> items;
  std::size_t begin = 0;
  for (;;) {
    const std::size_t comma = text.find(',', begin);
    items.push_back(text.substr(begin, comma - begin));
    if (items.back().empty()) {
      std::string reason = flag;
      reason += ": empty item in ";
      reason += text;
      throw std::invalid_argument(reason);
    }
    if (comma == std::string::npos) {
      return items;
    }
    begin = comma + 1;
  }
}

int verify_main(int argc, char** argv) {
  constexpr int kExitVerifyBadInput = 1;
  const auto start = std::chrono::steady_clock::now();
  try {
    const auto f = parse_flags(argc,
                               argv,
                               {"--hardware",
                                "--algorithm",
                                "--task",
                                "--runs",
                                "--seed0",
                                kQuick,
                                kNoTune,
                                kNoSensitivity,
                                "--levels",
                                "--tune-seeds",
                                "--sensitivity-seeds",
                                "--ablation-seeds",
                                "--jobs",
                                "--out"},
                               {"--hardware", "--algorithm", "--task", "--out"});
    std::vector<cobot::control::Algorithm> algorithms;
    for (const std::string& name : parse_list("--algorithm", f.at("--algorithm"))) {
      const auto algorithm = cobot::control::algorithm_from_string(name);
      if (!algorithm) {
        throw std::invalid_argument("--algorithm: unknown " + name + " (registered: " +
                                    cobot::control::registered_algorithm_names() + ")");
      }
      if (std::find(algorithms.begin(), algorithms.end(), *algorithm) != algorithms.end()) {
        throw std::invalid_argument("--algorithm: " + name + " given twice");
      }
      algorithms.push_back(*algorithm);
    }
    const std::vector<std::string> model_yamls = parse_list("--hardware", f.at("--hardware"));
    cobot::bench::VerifyOptions options;
    if (f.count("--runs") > 0) {
      options.runs = parse_positive("--runs", f.at("--runs"));
    }
    if (f.count("--seed0") > 0) {
      options.seed0 = parse_count("--seed0", f.at("--seed0"));
    }
    options.quick = f.count(kQuick) > 0;
    options.tune = f.count(kNoTune) == 0;
    options.sensitivity = f.count(kNoSensitivity) == 0;
    if (f.count("--sensitivity-seeds") > 0) {
      options.sensitivity_seeds =
          parse_positive("--sensitivity-seeds", f.at("--sensitivity-seeds"));
    }
    if (f.count("--ablation-seeds") > 0) {
      options.ablation_seeds = parse_positive("--ablation-seeds", f.at("--ablation-seeds"));
    }
    if (f.count("--levels") > 0) {
      options.tuning.levels = parse_levels(f.at("--levels"));
    }
    if (f.count("--tune-seeds") > 0) {
      const std::uint64_t n = parse_positive("--tune-seeds", f.at("--tune-seeds"));
      for (std::uint64_t s = 1; s <= n; ++s) {
        options.tuning.seeds.push_back(s);
      }
    }
    if (f.count("--jobs") > 0) {
      options.jobs = parse_jobs(f.at("--jobs"));
    }
    const std::vector<std::filesystem::path> model_paths(model_yamls.begin(), model_yamls.end());
    // Every model and every input file is checked before the first run.
    const auto in = cobot::bench::load_verify_inputs(model_paths, algorithms, f.at("--task"));
    if (!in) {
      throw std::invalid_argument(in.reason());
    }
    const cobot::bench::VerifyInputs& v = in.value();
    const auto rows = cobot::bench::compare(v.models, algorithms, v.task, options);
    if (!rows) {
      throw std::invalid_argument(rows.reason());
    }
    // One pair in the cross product (one model, one algorithm): report.{json,md} (+ tune.csv) in
    // --out, as a single call always wrote. More: one directory per listed pair and
    // comparison.{json,md}, where a pair the model does not list is named "not listed".
    const auto written =
        cobot::bench::write_verify_outputs(f.at("--out"), v.pairs, rows.value(), v.inputs);
    if (!written) {
      throw std::invalid_argument(written.reason());
    }
    const std::string printed =
        v.pairs.size() == 1
            ? cobot::bench::report_md(*rows.value().front().result, v.inputs.front())
            : cobot::bench::comparison_md(rows.value());
    const std::chrono::duration<double> wall = std::chrono::steady_clock::now() - start;
    std::printf("%s\nwall time: %.1f s\n", printed.c_str(), wall.count());
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "cobot_bench verify: %s\n%s", e.what(), kUsage);
    return kExitVerifyBadInput;
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
  if (mode == "verify") {
    return verify_main(argc, argv);
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
