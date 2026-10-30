// cobot_bench: the control loop against SimPlant without ROS (design 11.2).
// Exit 0 on a completed run (tools/run_e2e.py judges it), 2 on bad arguments or configuration
// (a bad waypoint file included) with the reason on stderr, 1 when `run --waypoints` rejects the
// list (one line per issue on stderr, no Run CSV written).
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>

#include "engine.hpp"
#include "run.hpp"

namespace {

constexpr int kExitPathInvalid = 1;
constexpr int kExitBadInput = 2;

const char* const kUsage =
    "usage: cobot_bench run --config <yaml> --imperfections <yaml> [--seed <u64>] --out <csv> "
    "[--duration-s <s>] [--waypoints <yaml>]\n"
    "       cobot_bench montecarlo --config <yaml> --imperfections <yaml> --runs <n> "
    "--seed0 <u64> --out <dir> [--keep-runs] [--duration-s <s>]\n"
    "       cobot_bench sweep --config <yaml> --sweep <yaml> --out <dir> [--duration-s <s>]\n"
    "       cobot_bench timing --config <yaml> --steps <n>\n";

const char* const kKeepRuns = "--keep-runs";

// "--name value" pairs after the mode (--keep-runs takes no value and maps to ""); throws
// std::invalid_argument on a bad command line.
std::map<std::string, std::string> parse_flags(int argc,
                                               char** argv,
                                               const std::set<std::string>& known,
                                               const std::set<std::string>& required) {
  std::map<std::string, std::string> flags;
  for (int i = 2; i < argc; i += 2) {
    const std::string name = argv[i];
    const bool is_switch = name == kKeepRuns;
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

int report(const char* mode, const cobot::common::Status& status) {
  if (!status) {
    std::fprintf(stderr, "cobot_bench %s: %s\n", mode, status.reason().c_str());
    return kExitBadInput;
  }
  return 0;
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
    const auto f = parse_flags(argc, argv, {"--config", "--steps"}, {"--config", "--steps"});
    const std::uint64_t steps = parse_u64(f.at("--steps"));
    const cobot::common::Result<cobot::bench::TimingStats> stats =
        cobot::bench::timing(f.at("--config"), static_cast<std::int64_t>(steps));
    if (!stats) {
      std::fprintf(stderr, "cobot_bench timing: %s\n", stats.reason().c_str());
      return kExitBadInput;
    }
    std::printf("p50_us=%.3f p95_us=%.3f max_us=%.3f\n",
                stats.value().p50_us,
                stats.value().p95_us,
                stats.value().max_us);
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
