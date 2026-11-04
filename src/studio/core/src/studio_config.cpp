#include "cobot/studio/studio_config.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <set>
#include <system_error>
#include <utility>
#include <variant>

#include <yaml-cpp/yaml.h>

#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/studio/param_catalog.hpp"

namespace cobot::studio {
namespace {

constexpr int kN = model::kArmDof;
constexpr double kMmPerM = 1e3;
constexpr double kSpeedMinPct = 1.0;
constexpr double kSpeedMaxPct = 100.0;
constexpr int kVersion = 1;

std::string text(const char* format, double v) {
  std::array<char, 32> out{};
  std::snprintf(out.data(), out.size(), format, v);
  return out.data();
}

std::string exact(double v) {
  return text("%.17g", v);
}

// `p` relative to `dir` when they share more than the root, else absolute.
std::string relative_to(const fs::path& p, const fs::path& dir) {
  std::error_code ec;
  const fs::path abs = fs::absolute(p, ec).lexically_normal();
  const fs::path rel = abs.lexically_relative(dir);
  const auto up = std::count(rel.begin(), rel.end(), fs::path(".."));
  const fs::path below_root = dir.relative_path();
  const auto depth = std::distance(below_root.begin(), below_root.end());
  return (rel.empty() || up >= depth ? abs : rel).generic_string();
}

YAML::Node value_node(const pipeline::KeyOverride& o) {
  if (const double* d = std::get_if<double>(&o.value)) {
    return YAML::Node(exact(*d));
  }
  if (const auto* v = std::get_if<std::vector<double>>(&o.value)) {
    YAML::Node seq(YAML::NodeType::Sequence);
    seq.SetStyle(YAML::EmitterStyle::Flow);
    for (const double x : *v) {
      seq.push_back(exact(x));
    }
    return seq;
  }
  if (const bool* b = std::get_if<bool>(&o.value)) {
    return YAML::Node(*b ? "true" : "false");
  }
  return YAML::Node(std::get<std::string>(o.value));
}

// Checks that every key of the map `n` (labelled `path`) is in `allowed`.
std::string unknown_keys(const YAML::Node& n,
                         const std::string& path,
                         const std::set<std::string>& allowed) {
  std::string bad;
  for (const auto& kv : n) {
    if (allowed.count(kv.first.Scalar()) == 0U) {
      bad += (bad.empty() ? "" : "\n") + path + "." + kv.first.Scalar() + ": unknown key";
    }
  }
  return bad;
}

common::Result<pipeline::KeyOverride> parse_override(const std::string& key, const YAML::Node& n) {
  using Out = common::Result<pipeline::KeyOverride>;
  if (n.IsSequence()) {
    std::vector<double> v;
    for (const YAML::Node& x : n) {
      double d = 0.0;
      if (!x.IsScalar() || !YAML::convert<double>::decode(x, d)) {
        return Out::error("studio.overrides." + key + ": expected numbers");
      }
      v.push_back(d);
    }
    return Out::ok(pipeline::KeyOverride{key, v});
  }
  if (!n.IsScalar()) {
    return Out::error("studio.overrides." + key + ": expected a scalar or a list");
  }
  const std::string& s = n.Scalar();
  double d = 0.0;
  if (s == "true" || s == "false") {
    return Out::ok(pipeline::KeyOverride{key, s == "true"});
  }
  if (YAML::convert<double>::decode(n, d)) {
    return Out::ok(pipeline::KeyOverride{key, d});
  }
  return Out::ok(pipeline::KeyOverride{key, s});
}

common::Result<StudioConfig> read_studio(const YAML::Node& root, const fs::path& dir) {
  using Out = common::Result<StudioConfig>;
  const YAML::Node s = root["studio"];
  if (!s.IsMap()) {
    return Out::error("expected a mapping under studio");
  }
  std::string bad = unknown_keys(s,
                                 "studio",
                                 {"version",
                                  "base",
                                  "overrides",
                                  "seed",
                                  "tolerance",
                                  "speed_pct",
                                  "waypoints",
                                  "duration_s"});
  if (!bad.empty()) {
    return Out::error(bad);
  }
  if (!s["version"] || s["version"].as<int>() != kVersion) {
    return Out::error("studio.version: expected 1");
  }
  StudioConfig c;
  const YAML::Node base = s["base"];
  if (!base.IsMap() || !base["name"] || !base["config"] || !base["imperfections"]) {
    return Out::error("studio.base: expected {name, config, imperfections}");
  }
  bad = unknown_keys(base, "studio.base", {"name", "config", "imperfections"});
  if (!bad.empty()) {
    return Out::error(bad);
  }
  c.base.name = base["name"].as<std::string>();
  c.base.config_yaml = (dir / base["config"].as<std::string>()).lexically_normal();
  c.base.imperfections_yaml = (dir / base["imperfections"].as<std::string>()).lexically_normal();
  if (s["overrides"] && !s["overrides"].IsNull()) {
    for (const auto& kv : s["overrides"]) {
      common::Result<pipeline::KeyOverride> o = parse_override(kv.first.Scalar(), kv.second);
      if (!o) {
        return Out::error(o.reason());
      }
      c.overrides.push_back(std::move(o.value()));
    }
  }
  if (s["seed"] && !s["seed"].IsNull()) {
    c.seed = s["seed"].as<std::uint64_t>();
  }
  if (const YAML::Node t = s["tolerance"]; t && !t.IsNull()) {
    bad = unknown_keys(
        t, "studio.tolerance", {"goal_pos_mm", "goal_rot_deg", "path_pos_mm", "path_rot_deg"});
    if (!bad.empty()) {
      return Out::error(bad);
    }
    const auto read = [&t](const char* key, double scale, double& out) {
      if (t[key]) {
        out = t[key].as<double>() * scale;
      }
    };
    read("goal_pos_mm", 1.0 / kMmPerM, c.tolerance.goal_pos_m);
    read("goal_rot_deg", kDegToRad, c.tolerance.goal_rot_rad);
    read("path_pos_mm", 1.0 / kMmPerM, c.tolerance.path_pos_m);
    read("path_rot_deg", kDegToRad, c.tolerance.path_rot_rad);
  }
  if (s["speed_pct"]) {
    c.speed_pct = s["speed_pct"].as<double>();
  }
  if (s["waypoints"] && !s["waypoints"].IsNull()) {
    c.waypoints_yaml = (dir / s["waypoints"].as<std::string>()).lexically_normal();
  }
  if (s["duration_s"] && !s["duration_s"].IsNull()) {
    c.duration_s = s["duration_s"].as<double>();
  }
  return Out::ok(std::move(c));
}

std::string value_text(const pipeline::KeyOverride& o) {
  if (const double* d = std::get_if<double>(&o.value)) {
    return text("%g", *d);
  }
  if (const auto* v = std::get_if<std::vector<double>>(&o.value)) {
    std::string out = "[";
    for (std::size_t i = 0; i < v->size(); ++i) {
      out += (i > 0 ? ", " : "") + text("%g", (*v)[i]);
    }
    return out + "]";
  }
  if (const bool* b = std::get_if<bool>(&o.value)) {
    return *b ? "true" : "false";
  }
  return std::get<std::string>(o.value);
}

bool is_config_key(const std::string& key) {
  return key.rfind("config.", 0) == 0;
}

// Issues of one override against its catalog entry (kind and input limits).
void check_override(const pipeline::KeyOverride& o, std::vector<std::string>& issues) {
  const ParamInfo* p = find_param(o.key);
  if (p == nullptr) {
    issues.push_back(o.key + ": unknown key (not in the parameter catalog)");
    return;
  }
  std::vector<double> numbers;
  if (const double* d = std::get_if<double>(&o.value)) {
    numbers.push_back(*d);
  } else if (const auto* v = std::get_if<std::vector<double>>(&o.value)) {
    numbers = *v;
  }
  const bool numeric = p->kind == ParamKind::kNumber || p->kind == ParamKind::kPerJoint;
  const bool kind_ok =
      numeric ? (std::holds_alternative<double>(o.value) ||
                 (p->kind == ParamKind::kPerJoint &&
                  std::holds_alternative<std::vector<double>>(o.value)))
              : (p->kind == ParamKind::kBool ? std::holds_alternative<bool>(o.value)
                                             : std::holds_alternative<std::string>(o.value));
  if (!kind_ok) {
    issues.push_back(o.key + ": wrong value type " + value_text(o));
    return;
  }
  if (p->kind == ParamKind::kChoice) {
    const auto& s = std::get<std::string>(o.value);
    bool listed = false;
    for (const std::string& c : p->choices) {
      listed = listed || c == s;
    }
    if (!listed) {
      issues.push_back(o.key + ": '" + s + "' is not one of the choices");
    }
  }
  for (const double v : numbers) {
    if (!std::isfinite(v) || v < p->min || v > p->max) {
      issues.push_back(o.key + ": " + text("%g", v) + " is outside [" + text("%g", p->min) + ", " +
                       text("%g", p->max) + "] " + p->unit);
      return;
    }
  }
}

}  // namespace

common::Result<StudioConfig> load_studio_file(const fs::path& yaml) {
  using Out = common::Result<StudioConfig>;
  std::error_code ec;
  if (!fs::is_regular_file(yaml, ec)) {
    return Out::error("cannot open " + yaml.string() + ": no such file");
  }
  try {
    const YAML::Node root = YAML::LoadFile(yaml.string());
    common::Result<StudioConfig> c =
        read_studio(root, fs::absolute(yaml, ec).parent_path().lexically_normal());
    return c ? std::move(c) : Out::error(yaml.string() + ": " + c.reason());
  } catch (const YAML::Exception& e) {
    return Out::error(yaml.string() + ": " + e.what());
  }
}

common::Status save_studio_file(const StudioConfig& config, const fs::path& yaml) {
  std::error_code ec;
  const fs::path dir = fs::absolute(yaml, ec).parent_path().lexically_normal();
  fs::create_directories(dir, ec);
  YAML::Node s(YAML::NodeType::Map);
  s["version"] = kVersion;
  YAML::Node base(YAML::NodeType::Map);
  base.SetStyle(YAML::EmitterStyle::Flow);
  base["name"] = config.base.name;
  base["config"] = relative_to(config.base.config_yaml, dir);
  base["imperfections"] = relative_to(config.base.imperfections_yaml, dir);
  s["base"] = base;
  YAML::Node overrides(YAML::NodeType::Map);
  for (const pipeline::KeyOverride& o : config.overrides) {
    overrides[o.key] = value_node(o);
  }
  s["overrides"] = overrides;
  s["seed"] = config.seed ? YAML::Node(std::to_string(*config.seed)) : YAML::Node(YAML::Null);
  YAML::Node t(YAML::NodeType::Map);
  t.SetStyle(YAML::EmitterStyle::Flow);
  t["goal_pos_mm"] = exact(config.tolerance.goal_pos_m * kMmPerM);
  t["goal_rot_deg"] = exact(config.tolerance.goal_rot_rad / kDegToRad);
  t["path_pos_mm"] = exact(config.tolerance.path_pos_m * kMmPerM);
  t["path_rot_deg"] = exact(config.tolerance.path_rot_rad / kDegToRad);
  s["tolerance"] = t;
  s["speed_pct"] = exact(config.speed_pct);
  s["waypoints"] = config.waypoints_yaml ? YAML::Node(relative_to(*config.waypoints_yaml, dir))
                                         : YAML::Node(YAML::Null);
  s["duration_s"] =
      config.duration_s ? YAML::Node(exact(*config.duration_s)) : YAML::Node(YAML::Null);
  YAML::Node root(YAML::NodeType::Map);
  root["studio"] = s;
  YAML::Emitter out;
  out << root;
  std::ofstream file(yaml, std::ios::binary);
  file << out.c_str() << '\n';
  file.flush();
  return file ? common::Status::ok() : common::Status::error("cannot write " + yaml.string());
}

std::vector<std::string> validate(const StudioConfig& config) {
  std::vector<std::string> issues;
  for (const pipeline::KeyOverride& o : config.overrides) {
    check_override(o, issues);
  }
  if (!(config.speed_pct >= kSpeedMinPct && config.speed_pct <= kSpeedMaxPct)) {
    issues.push_back("speed_pct: " + text("%g", config.speed_pct) + " is outside [1, 100] %");
  }
  if (config.duration_s && !(*config.duration_s > 0.0)) {
    issues.push_back("duration_s: must be > 0, got " + text("%g", *config.duration_s));
  }
  if (!issues.empty()) {
    return issues;
  }
  std::vector<pipeline::KeyOverride> controller;
  std::vector<pipeline::KeyOverride> imperfections;
  for (const pipeline::KeyOverride& o : config.overrides) {
    (is_config_key(o.key) ? controller : imperfections).push_back(o);
  }
  const auto c = pipeline::load_config_with<kN>(config.base.config_yaml, controller);
  if (!c) {
    issues.push_back(c.reason());
  }
  const auto i =
      pipeline::load_imperfections_with<kN>(config.base.imperfections_yaml, imperfections);
  if (!i) {
    issues.push_back(i.reason());
  }
  return issues;
}

common::Result<Materialized> materialize(const StudioConfig& config, const fs::path& dir) {
  using Out = common::Result<Materialized>;
  std::error_code ec;
  fs::create_directories(dir, ec);
  std::vector<pipeline::KeyOverride> controller;
  std::vector<pipeline::KeyOverride> imperfections;
  for (const pipeline::KeyOverride& o : config.overrides) {
    (is_config_key(o.key) ? controller : imperfections).push_back(o);
  }
  if (config.seed) {
    imperfections.push_back(pipeline::KeyOverride{"seed", std::to_string(*config.seed)});
  }
  const Materialized m{dir / "config.yaml", dir / "imperfections.yaml"};
  const common::Status c =
      pipeline::write_config_with(config.base.config_yaml, controller, m.config_yaml);
  if (!c) {
    return Out::error(c.reason());
  }
  const common::Status i = pipeline::write_imperfections_with(
      config.base.imperfections_yaml, imperfections, m.imperfections_yaml);
  if (!i) {
    return Out::error(i.reason());
  }
  return Out::ok(m);
}

std::string describe_diff(const StudioConfig& config) {
  std::string out;
  for (const pipeline::KeyOverride& o : config.overrides) {
    const std::string name = is_config_key(o.key) ? o.key.substr(o.key.rfind('.') + 1) : o.key;
    out += (out.empty() ? "" : ", ") + name + "=" + value_text(o);
  }
  return out;
}

}  // namespace cobot::studio
