#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <sys/wait.h>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/task.hpp"

// Helpers of the bench_verify tests (brief verify, PR 6a): the reference files, one output
// directory per test, a child cobot_bench, and a strict JSON reader that keeps the key order of
// every object, so the report's keys, nulls and number texts can be pinned.
//
// API under test (src/bench, static library cobot_bench_run), as the brief names it:
//   verify.hpp   Interval, wilson, Verdict, Counts, WorstMetric, verdict, run_passes,
//                GainSource {kTuned, kDefaultNoAdmissible, kDefaultTuningOff},
//                Assumption {key, kind, range, scale, sensitivity},
//                SeedRun {seed, summary, passed}, VerifyOptions, VerifyResult {algorithm, model,
//                task, gain_key, gain_used, gain_source, tuning, runs, counts, pass_rate, wilson,
//                hold_rate, worst, verdict, quick, assumptions, causes, suggestions}, verify
//   report.hpp   InputFile {path, sha256}, report_inputs, report_json, report_md
//   sha256.hpp   sha256_hex
//   engine.hpp / metrics.hpp   SettleThresholds {pos_m, rot_rad}, RunSummary::settle_time_s, the
//                summarize overload with the thresholds
namespace cobot::bench::verify_test {

namespace fs = std::filesystem;

inline const fs::path kRepo = COBOT_REPO_DIR;

#if defined(__OPTIMIZE__)
inline constexpr bool kOptimised = true;
#else
inline constexpr bool kOptimised = false;
#endif

// ADR-023, in English in report.json and as the first line of report.md.
inline const char* const kDisclaimer =
    "Screening and comparison tool, not a certification: it does not replace a safety "
    "assessment to ISO 10218 or ISO/TS 15066, nor measurements on the real robot.";

inline bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

inline std::string read_text(const fs::path& p) {
  std::ifstream in(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

inline std::string first_line(const std::string& text) {
  return text.substr(0, text.find('\n'));
}

inline fs::path model_path(const std::string& name) {
  return kRepo / "config/hardware" / (name + ".yaml");
}

inline fs::path task_path(const std::string& name) {
  return kRepo / "config/tasks" / (name + ".yaml");
}

inline pipeline::HardwareModel load_model(const std::string& name) {
  const auto m = pipeline::load_hardware_model(model_path(name));
  EXPECT_TRUE(m.is_ok()) << m.reason();
  return m ? m.value() : pipeline::HardwareModel{};
}

inline pipeline::Task load_task(const std::string& name) {
  const auto t = pipeline::load_task(task_path(name));
  EXPECT_TRUE(t.is_ok()) << t.reason();
  return t ? t.value() : pipeline::Task{};
}

// An empty directory of its own for the running test (ctest runs tests in parallel processes).
inline fs::path test_dir() {
  const ::testing::TestInfo* info = ::testing::UnitTest::GetInstance()->current_test_info();
  std::string name = std::string(info->test_suite_name()) + "." + info->name();
  for (char& c : name) {
    c = c == '/' ? '_' : c;
  }
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "bench_verify" / name;
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

// %.17g, the report's number format.
inline std::string g17(double v) {
  std::array<char, 40> text{};
  std::snprintf(text.data(), text.size(), "%.17g", v);
  return text.data();
}

// ---- strict JSON reader -------------------------------------------------------------------

struct Json {
  enum class Type { kNull, kBool, kNumber, kString, kArray, kObject };
  Type type = Type::kNull;
  bool boolean = false;
  double number = 0.0;
  std::string text;               // a string's value, or a number's source text
  std::vector<Json> items;        // array elements
  std::vector<std::string> keys;  // object keys, in file order
  std::vector<Json> values;       // object values, keys[i] -> values[i]

  bool is(Type t) const { return type == t; }

  // The value of `key` in an object, nullptr when absent (or not an object).
  const Json* find(const std::string& key) const {
    for (std::size_t i = 0; i < keys.size(); ++i) {
      if (keys[i] == key) {
        return &values[i];
      }
    }
    return nullptr;
  }
};

class JsonReader {
 public:
  explicit JsonReader(const std::string& text) : s_(text) {}

  // The whole text as one value; nullopt on any syntax error or trailing text. NaN, Infinity,
  // trailing commas and single quotes are errors, as in RFC 8259.
  std::optional<Json> document() {
    Json v;
    if (!value(v, 0)) {
      return std::nullopt;
    }
    skip_space();
    if (pos_ != s_.size()) {
      return std::nullopt;
    }
    return v;
  }

 private:
  static constexpr int kMaxDepth = 64;

  void skip_space() {
    while (pos_ < s_.size() &&
           (s_[pos_] == ' ' || s_[pos_] == '\n' || s_[pos_] == '\r' || s_[pos_] == '\t')) {
      ++pos_;
    }
  }

  bool literal(const char* word) {
    const std::string w(word);
    if (s_.compare(pos_, w.size(), w) != 0) {
      return false;
    }
    pos_ += w.size();
    return true;
  }

  bool digit() const { return pos_ < s_.size() && s_[pos_] >= '0' && s_[pos_] <= '9'; }

  bool digits() {
    if (!digit()) {
      return false;
    }
    while (digit()) {
      ++pos_;
    }
    return true;
  }

  bool number(Json& v) {
    const std::size_t begin = pos_;
    if (s_[pos_] == '-') {
      ++pos_;
    }
    if (pos_ < s_.size() && s_[pos_] == '0') {
      ++pos_;
    } else if (!digits()) {
      return false;
    }
    if (pos_ < s_.size() && s_[pos_] == '.') {
      ++pos_;
      if (!digits()) {
        return false;
      }
    }
    if (pos_ < s_.size() && (s_[pos_] == 'e' || s_[pos_] == 'E')) {
      ++pos_;
      if (pos_ < s_.size() && (s_[pos_] == '+' || s_[pos_] == '-')) {
        ++pos_;
      }
      if (!digits()) {
        return false;
      }
    }
    v.type = Json::Type::kNumber;
    v.text = s_.substr(begin, pos_ - begin);
    v.number = std::strtod(v.text.c_str(), nullptr);
    return true;
  }

  bool string(std::string& out) {
    if (pos_ >= s_.size() || s_[pos_] != '"') {
      return false;
    }
    ++pos_;
    while (pos_ < s_.size()) {
      const char c = s_[pos_++];
      if (c == '"') {
        return true;
      }
      if (static_cast<unsigned char>(c) < 0x20) {
        return false;
      }
      if (c != '\\') {
        out += c;
        continue;
      }
      if (pos_ >= s_.size()) {
        return false;
      }
      const char e = s_[pos_++];
      switch (e) {
        case '"':
        case '\\':
        case '/':
          out += e;
          break;
        case 'b':
          out += '\b';
          break;
        case 'f':
          out += '\f';
          break;
        case 'n':
          out += '\n';
          break;
        case 'r':
          out += '\r';
          break;
        case 't':
          out += '\t';
          break;
        case 'u': {
          if (pos_ + 4 > s_.size()) {
            return false;
          }
          const std::string hex = s_.substr(pos_, 4);
          char* end = nullptr;
          const long code = std::strtol(hex.c_str(), &end, 16);
          if (end != hex.c_str() + 4) {
            return false;
          }
          pos_ += 4;
          out += code < 0x80 ? static_cast<char>(code) : '?';
          break;
        }
        default:
          return false;
      }
    }
    return false;
  }

  bool value(Json& v, int depth) {
    if (depth > kMaxDepth) {
      return false;
    }
    skip_space();
    if (pos_ >= s_.size()) {
      return false;
    }
    const char c = s_[pos_];
    if (c == '{') {
      return object(v, depth);
    }
    if (c == '[') {
      return array(v, depth);
    }
    if (c == '"') {
      v.type = Json::Type::kString;
      return string(v.text);
    }
    if (c == '-' || (c >= '0' && c <= '9')) {
      return number(v);
    }
    if (literal("true")) {
      v.type = Json::Type::kBool;
      v.boolean = true;
      return true;
    }
    if (literal("false")) {
      v.type = Json::Type::kBool;
      return true;
    }
    if (literal("null")) {
      v.type = Json::Type::kNull;
      return true;
    }
    return false;
  }

  bool array(Json& v, int depth) {
    v.type = Json::Type::kArray;
    ++pos_;  // [
    skip_space();
    if (pos_ < s_.size() && s_[pos_] == ']') {
      ++pos_;
      return true;
    }
    while (true) {
      Json item;
      if (!value(item, depth + 1)) {
        return false;
      }
      v.items.push_back(std::move(item));
      skip_space();
      if (pos_ >= s_.size()) {
        return false;
      }
      const char c = s_[pos_++];
      if (c == ']') {
        return true;
      }
      if (c != ',') {
        return false;
      }
    }
  }

  bool object(Json& v, int depth) {
    v.type = Json::Type::kObject;
    ++pos_;  // {
    skip_space();
    if (pos_ < s_.size() && s_[pos_] == '}') {
      ++pos_;
      return true;
    }
    while (true) {
      skip_space();
      std::string key;
      if (!string(key)) {
        return false;
      }
      skip_space();
      if (pos_ >= s_.size() || s_[pos_] != ':') {
        return false;
      }
      ++pos_;
      Json item;
      if (!value(item, depth + 1)) {
        return false;
      }
      v.keys.push_back(std::move(key));
      v.values.push_back(std::move(item));
      skip_space();
      if (pos_ >= s_.size()) {
        return false;
      }
      const char c = s_[pos_++];
      if (c == '}') {
        return true;
      }
      if (c != ',') {
        return false;
      }
    }
  }

  const std::string& s_;
  std::size_t pos_ = 0;
};

inline std::optional<Json> parse_json(const std::string& text) {
  return JsonReader(text).document();
}

// ---- child cobot_bench (test_bench_verify_runs only) ------------------------------------------

#if defined(COBOT_BENCH_EXE)

struct Cli {
  int exit_code = -1;
  std::string output;  // stdout and stderr
};

inline Cli run_bench(const std::string& args) {
  const std::string cmd = std::string("'") + COBOT_BENCH_EXE + "' " + args + " 2>&1";
  Cli cli;
  FILE* pipe = popen(cmd.c_str(), "r");
  if (pipe == nullptr) {
    return cli;
  }
  std::array<char, 4096> buf{};
  while (fgets(buf.data(), static_cast<int>(buf.size()), pipe) != nullptr) {
    cli.output += buf.data();
  }
  const int status = pclose(pipe);
  cli.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  return cli;
}

inline std::string quoted(const fs::path& p) {
  return "'" + p.string() + "'";
}

// `cobot_bench verify` on a model, an algorithm and a task, without --out.
inline std::string verify_args(const std::string& model,
                               const std::string& algorithm,
                               const std::string& task = "reference_move") {
  return "verify --hardware " + quoted(model_path(model)) + " --algorithm " + algorithm +
         " --task " + quoted(task_path(task));
}

// None of the files `cobot_bench verify` writes is in `out` (it may not exist at all).
inline bool no_report_file(const fs::path& out) {
  return !fs::exists(out / "report.json") && !fs::exists(out / "report.md") &&
         !fs::exists(out / "tune.csv");
}

#endif  // COBOT_BENCH_EXE

}  // namespace cobot::bench::verify_test
