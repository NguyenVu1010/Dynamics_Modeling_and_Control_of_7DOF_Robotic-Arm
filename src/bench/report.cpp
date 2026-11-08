#include "report.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>

#include "sha256.hpp"

namespace cobot::bench {
namespace {

const char* kind_name(pipeline::ProvenanceKind kind) {
  switch (kind) {
    case pipeline::ProvenanceKind::kPublished:
      return "published";
    case pipeline::ProvenanceKind::kDerived:
      return "derived";
    case pipeline::ProvenanceKind::kAssumed:
      return "assumed";
  }
  return "assumed";
}

// %.17g, or `nan_text` for a value that is not finite.
std::string num(double v, const char* nan_text = "null") {
  if (!std::isfinite(v)) {
    return nan_text;
  }
  std::array<char, 40> text{};
  std::snprintf(text.data(), text.size(), "%.17g", v);
  return text.data();
}

// %.4g for the MD tables, "-" for NaN.
std::string short_num(double v) {
  std::array<char, 40> text{};
  std::snprintf(text.data(), text.size(), "%.4g", v);
  return std::isfinite(v) ? std::string(text.data()) : std::string("-");
}

std::string str(const std::string& s) {
  std::string out = "\"";
  for (const char c : s) {
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (static_cast<unsigned char>(c) < 0x20) {
      std::array<char, 8> esc{};
      std::snprintf(esc.data(), esc.size(), "\\u%04x", static_cast<unsigned>(c));
      out += esc.data();
    } else {
      out += c;
    }
  }
  return out + "\"";
}

std::string pair(double a, double b) {
  return "[" + num(a) + ", " + num(b) + "]";
}

std::string numbers(const std::vector<double>& v) {
  std::string out = "[";
  for (std::size_t i = 0; i < v.size(); ++i) {
    out += (i > 0 ? ", " : "") + num(v[i]);
  }
  return out + "]";
}

const char* verdict_line(Verdict v) {
  switch (v) {
    case Verdict::kPass:
      return "PASS";
    case Verdict::kMarginal:
      return "MARGINAL, measure on hardware";
    case Verdict::kFail:
      return "FAIL";
  }
  return "FAIL";
}

const char* gain_note(GainSource source) {
  switch (source) {
    case GainSource::kTuned:
      return "tuned";
    case GainSource::kDefaultNoAdmissible:
      return "no admissible gain: verdict on the model's default gain";
    case GainSource::kDefaultTuningOff:
      return "tuning off: verdict on the model's gain";
  }
  return "tuned";
}

bool is_chosen(const TuneResult& t, const TuneRow& r) {
  return t.chosen && *t.chosen == r.level;
}

std::string bounds(const Assumption& a) {
  const std::array<double, 2> b = a.range ? *a.range : a.scale ? *a.scale : std::array<double, 2>{};
  return std::string(a.range ? "range " : "scale ") + short_num(b[0]) + " .. " + short_num(b[1]);
}

}  // namespace

common::Result<std::vector<InputFile>> report_inputs(const std::filesystem::path& model_yaml,
                                                     const pipeline::HardwareModel& model,
                                                     const std::filesystem::path& task_yaml,
                                                     const pipeline::Task& task) {
  using Out = common::Result<std::vector<InputFile>>;
  std::vector<std::filesystem::path> files = {
      model_yaml, model.imperfections, model.controller, task_yaml};
  if (task.waypoints) {
    files.push_back(*task.waypoints);
  }
  std::vector<InputFile> out;
  for (const std::filesystem::path& f : files) {
    std::ifstream in(f, std::ios::binary);
    const std::string bytes{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    if (!in.is_open() || in.bad()) {
      return Out::error("cannot read " + f.string());
    }
    out.push_back(InputFile{f.lexically_normal().string(), sha256_hex(bytes)});
  }
  return Out::ok(std::move(out));
}

std::string report_json(const VerifyResult& r, const std::vector<InputFile>& inputs) {
  std::string j = "{\n  \"schema\": 1,\n  \"disclaimer\": " + str(kVerifyDisclaimer) + ",\n";
  j += "  \"model\": " + str(r.model) +
       ",\n  \"algorithm\": " + str(control::to_string(r.algorithm)) +
       ",\n  \"task\": " + str(r.task) + ",\n";
  j += "  \"verdict\": " + str(to_string(r.verdict)) +
       ",\n  \"quick\": " + (r.quick ? "true" : "false") +
       ",\n  \"runs\": " + std::to_string(r.counts.runs) + ",\n";
  j += "  \"seeds\": " + (r.runs.empty() ? std::string("[]")
                                         : "[" + std::to_string(r.runs.front().seed) + ", " +
                                               std::to_string(r.runs.back().seed) + "]");
  j += ",\n  \"pass_count\": " + std::to_string(r.counts.passed) +
       ",\n  \"pass_rate\": " + num(r.pass_rate) +
       ",\n  \"wilson\": " + pair(r.wilson.lo, r.wilson.hi) +
       ",\n  \"hold_rate\": " + num(r.hold_rate) + ",\n  \"worst\": {";
  for (std::size_t i = 0; i < r.worst.size(); ++i) {
    const WorstMetric& w = r.worst[i];
    j += std::string(i > 0 ? "," : "") + "\n    " + str(w.name) + ": {\"value\": " + num(w.value) +
         ", \"threshold\": " + num(w.threshold) + ", \"margin_limit\": " + num(w.margin_limit) +
         "}";
  }
  j += "\n  },\n  \"tuning\": {\n    \"key\": " + str(r.gain_key) +
       ",\n    \"gain_used\": " + num(r.gain_used) +
       ",\n    \"gain_source\": " + str(to_string(r.gain_source)) +
       ",\n    \"levels\": " + numbers(r.tuning ? r.tuning->levels : std::vector<double>{}) +
       ",\n    \"rows\": [";
  if (r.tuning) {
    for (std::size_t i = 0; i < r.tuning->rows.size(); ++i) {
      const TuneRow& t = r.tuning->rows[i];
      j += std::string(i > 0 ? "," : "") + "\n      {\"level\": " + num(t.level) +
           ", \"seeds\": " + std::to_string(t.seeds) + ", \"held\": " + std::to_string(t.held) +
           ", \"sat_frac_max\": " + num(t.sat_frac_max) +
           ", \"decays_all\": " + (t.decays_all ? "1" : "0") +
           ", \"rms_joint_err_median\": " + num(t.rms_joint_err_median) +
           ", \"final_pos_err_max\": " + num(t.final_pos_err_max) +
           ", \"chosen\": " + (is_chosen(*r.tuning, t) ? "1" : "0") + "}";
    }
  }
  const bool rows = r.tuning && !r.tuning->rows.empty();
  j += std::string(rows ? "\n    " : "") +
       "],\n    \"chosen\": " + (r.tuning && r.tuning->chosen ? num(*r.tuning->chosen) : "null") +
       ",\n    \"reason\": " + str(r.tuning ? r.tuning->reason : "tuning off") +
       "\n  },\n  \"assumptions\": [";
  for (std::size_t i = 0; i < r.assumptions.size(); ++i) {
    const Assumption& a = r.assumptions[i];
    j += std::string(i > 0 ? "," : "") + "\n    {\"key\": " + str(a.key) +
         ", \"kind\": " + str(kind_name(a.kind));
    if (a.range) {
      j += ", \"range\": " + pair((*a.range)[0], (*a.range)[1]);
    }
    if (a.scale) {
      j += ", \"scale\": " + pair((*a.scale)[0], (*a.scale)[1]);
    }
    j += ", \"sensitivity\": " + str(a.sensitivity) + "}";
  }
  j += std::string(r.assumptions.empty() ? "" : "\n  ") + "],\n  \"causes\": [";
  for (std::size_t i = 0; i < r.causes.size(); ++i) {
    j += (i > 0 ? ", " : "") + str(r.causes[i]);
  }
  j += "],\n  \"suggestions\": [";
  for (std::size_t i = 0; i < r.suggestions.size(); ++i) {
    j += (i > 0 ? ", " : "") + str(r.suggestions[i]);
  }
  j += "],\n  \"versions\": {\n    \"git\": " + str(COBOT_GIT_DESCRIBE) + ",\n    \"inputs\": [";
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    j += std::string(i > 0 ? "," : "") + "\n      {\"path\": " + str(inputs[i].path) +
         ", \"sha256\": " + str(inputs[i].sha256) + "}";
  }
  j += std::string(inputs.empty() ? "" : "\n    ") + "]\n  }\n}\n";
  return j;
}

std::string report_md(const VerifyResult& r, const std::vector<InputFile>& inputs) {
  // The disclaimer, then the verdict line, then the heading.
  std::string m = std::string(kVerifyDisclaimer) + "\n\n**" + verdict_line(r.verdict) + "**" +
                  (r.quick ? " (quick mode: never PASS, fewer than 100 runs or --quick)" : "") +
                  "\n\n# Verify: " + r.model + ", " + control::to_string(r.algorithm) + ", " +
                  r.task + "\n\nGain: " + r.gain_key + " = " + short_num(r.gain_used) + " (" +
                  gain_note(r.gain_source) + ")\n\n## Runs\n\n";
  const std::string seeds = r.runs.empty() ? "-"
                                           : std::to_string(r.runs.front().seed) + ".." +
                                                 std::to_string(r.runs.back().seed);
  m += "| runs | seeds | passed | pass rate | Wilson 95 % | held | hold rate |\n"
       "|---|---|---|---|---|---|---|\n| " +
       std::to_string(r.counts.runs) + " | " + seeds + " | " + std::to_string(r.counts.passed) +
       " | " + short_num(r.pass_rate) + " | " + short_num(r.wilson.lo) + " .. " +
       short_num(r.wilson.hi) + " | " + std::to_string(r.counts.held) + " | " +
       short_num(r.hold_rate) +
       " |\n\n## Worst metrics (runs that did not hold)\n\n"
       "| metric | worst | threshold | margin limit |\n|---|---|---|---|\n";
  for (const WorstMetric& w : r.worst) {
    m += "| " + w.name + " | " + short_num(w.value) + " | " + short_num(w.threshold) + " | " +
         short_num(w.margin_limit) + " |\n";
  }
  m += "\n## Tuning\n\n";
  if (r.tuning) {
    m += r.tuning->key + ": " + r.tuning->reason +
         "\n\n| level | seeds | held | sat_frac_max | decays_all | rms_joint_err_median | "
         "final_pos_err_max | chosen |\n|---|---|---|---|---|---|---|---|\n";
    for (const TuneRow& t : r.tuning->rows) {
      m += "| " + short_num(t.level) + " | " + std::to_string(t.seeds) + " | " +
           std::to_string(t.held) + " | " + short_num(t.sat_frac_max) + " | " +
           (t.decays_all ? "1" : "0") + " | " + short_num(t.rms_joint_err_median) + " | " +
           short_num(t.final_pos_err_max) + " | " + (is_chosen(*r.tuning, t) ? "1" : "0") + " |\n";
    }
  } else {
    m += "tuning off\n";
  }
  m += "\n## Assumptions\n\n| key | kind | bounds | sensitivity |\n|---|---|---|---|\n";
  for (const Assumption& a : r.assumptions) {
    m += "| " + a.key + " | " + kind_name(a.kind) + " | " + bounds(a) + " | " + a.sensitivity +
         " |\n";
  }
  m += "\n## Versions\n\ngit: " + std::string(COBOT_GIT_DESCRIBE) +
       "\n\n| input | sha256 |\n|---|---|\n";
  for (const InputFile& f : inputs) {
    m += "| " + f.path + " | " + f.sha256 + " |\n";
  }
  return m;
}

}  // namespace cobot::bench
