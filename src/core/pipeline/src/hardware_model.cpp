#include "cobot/pipeline/hardware_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <set>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "cobot/common/dof.hpp"
#include "cobot/model/model.hpp"

#include "yaml_reader.hpp"

namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;
using yaml_io::Reader;

constexpr int kDof = model::kArmDof;

// --- active leaves of a profile ---

enum class LeafShape { kNumber, kList, kBool, kChoice };

struct Leaf {
  std::string key;
  LeafShape shape = LeafShape::kNumber;
  double value = 0.0;  // kNumber only
  bool on = false;     // not at its off value
};

Leaf leaf_of(const std::string& key, const YAML::Node& n) {
  Leaf leaf;
  leaf.key = key;
  if (key == "sensor.position_source") {
    leaf.shape = LeafShape::kChoice;
    leaf.on = n.Scalar() != "link";
  } else if (n.IsSequence()) {
    leaf.shape = LeafShape::kList;
    leaf.on =
        std::any_of(n.begin(), n.end(), [](const YAML::Node& e) { return e.as<double>() != 0.0; });
  } else if (YAML::convert<double>::decode(n, leaf.value)) {
    leaf.on = leaf.value != 0.0;
  } else {
    leaf.shape = LeafShape::kBool;
    leaf.on = n.as<bool>();
  }
  return leaf;
}

void collect_leaves(const YAML::Node& map, const std::string& prefix, std::vector<Leaf>& out) {
  for (const auto& kv : map) {
    const std::string name = kv.first.Scalar();
    const std::string key = prefix + name;
    if (name == "source" || key == "seed" || key.rfind("model.payload_", 0) == 0 ||
        kv.second.IsNull()) {
      continue;
    }
    if (kv.second.IsMap()) {
      collect_leaves(kv.second, key + ".", out);
    } else {
      out.push_back(leaf_of(key, kv.second));
    }
  }
}

Result<std::vector<Leaf>> active_leaves(const fs::path& profile) {
  using Out = Result<std::vector<Leaf>>;
  const auto valid = load_imperfections<kDof>(profile);
  if (!valid) {
    return Out::error(valid.reason());
  }
  std::vector<Leaf> leaves;
  try {
    collect_leaves(YAML::LoadFile(profile.string())["imperfections"], "", leaves);
  } catch (const YAML::Exception& e) {
    return Out::error(profile.string() + ": " + e.what());
  }
  const auto is_on = [&leaves](const char* key) {
    return std::any_of(
        leaves.begin(), leaves.end(), [key](const Leaf& l) { return l.key == key && l.on; });
  };
  const bool differentiated = is_on("sensor.qd_from_diff");
  const bool ripple = is_on("actuator.torque_ripple_frac");
  const auto off = [&](const Leaf& l) {
    return !l.on || (l.key == "sensor.qd_filter_hz" && !differentiated) ||
           (l.key == "actuator.ripple_periods_per_rev" && !ripple);
  };
  leaves.erase(std::remove_if(leaves.begin(), leaves.end(), off), leaves.end());
  return Out::ok(std::move(leaves));
}

// --- syntax of the model file ---

// A plain true/false is a bool, a plain number a double, a list of numbers a list, anything else
// (quoted text included) a string for the controller loader to judge.
void read_override(Reader& overrides,
                   const std::string& key,
                   const YAML::Node& n,
                   std::vector<KeyOverride>& out) {
  double number = 0.0;
  const bool quoted = n.Tag() == "!";
  if (n.IsSequence()) {
    std::vector<double> list;
    for (const YAML::Node& e : n) {
      if (!e.IsScalar() || !YAML::convert<double>::decode(e, number)) {
        overrides.fail(key, "expected a list of numbers, got '" + YAML::Dump(n) + "'");
        return;
      }
      list.push_back(number);
    }
    out.push_back({key, list});
  } else if (!n.IsScalar()) {
    overrides.fail(key,
                   "expected a number, a list, a bool or a string, got '" + YAML::Dump(n) + "'");
  } else if (!quoted && YAML::convert<double>::decode(n, number)) {
    out.push_back({key, number});
  } else if (!quoted && (n.Scalar() == "true" || n.Scalar() == "false")) {
    out.push_back({key, n.Scalar() == "true"});
  } else {
    out.push_back({key, n.Scalar()});
  }
}

void read_algorithms(Reader& h, HardwareModel& m) {
  const YAML::Node all = h.node("algorithms");
  if (all.IsNull()) {
    return;
  }
  Reader algorithms = h.child(all, "algorithms");
  if (!all.IsMap()) {
    return;
  }
  for (const auto& kv : all) {
    const std::string name = kv.first.Scalar();
    const std::optional<control::Algorithm> id = control::algorithm_from_string(name);
    if (!id) {
      algorithms.fail(
          name, "unknown algorithm (registered: " + control::registered_algorithm_names() + ")");
      continue;
    }
    if (kv.second.IsNull()) {
      algorithms.fail(name + ".overrides", "missing");
      continue;
    }
    Reader algorithm = algorithms.child(kv.second, name);
    const YAML::Node overrides = algorithm.node("overrides");
    std::vector<KeyOverride>& out = m.algorithms[*id];
    if (!overrides.IsNull()) {
      Reader list = algorithm.child(overrides, "overrides");
      for (const auto& o : overrides.IsMap() ? overrides : YAML::Node()) {
        read_override(list, o.first.Scalar(), o.second, out);
      }
    }
    algorithm.finish();
  }
}

std::optional<std::array<double, 2>> read_bound(Reader& e, const std::string& key) {
  if (!e.has(key)) {
    return std::nullopt;
  }
  VecN<2> v = VecN<2>::Constant(std::nan(""));
  e.vector<2>(key, yaml_io::kAny, false, v);
  return std::array<double, 2>{v[0], v[1]};
}

void read_provenance(Reader& h, HardwareModel& m) {
  const YAML::Node list = h.node("provenance");
  if (!list.IsNull() && !list.IsSequence()) {
    h.fail("provenance", "expected a list");
  } else if (list.IsSequence()) {
    for (std::size_t i = 0; i < list.size(); ++i) {
      Reader e = h.child(list[i], "provenance[" + std::to_string(i) + "]");
      ProvenanceEntry entry;
      e.scalar("key", entry.key);
      std::string kind;
      if (e.scalar("kind", kind)) {
        if (kind == "published") {
          entry.kind = ProvenanceKind::kPublished;
        } else if (kind == "derived") {
          entry.kind = ProvenanceKind::kDerived;
        } else if (kind == "assumed") {
          entry.kind = ProvenanceKind::kAssumed;
        } else {
          e.fail("kind", "expected published, derived or assumed, got '" + kind + "'");
        }
      }
      entry.range = read_bound(e, "range");
      entry.scale = read_bound(e, "scale");
      e.finish();
      m.provenance.push_back(std::move(entry));
    }
  }
}

void read_model(Reader& root, const fs::path& dir, HardwareModel& m) {
  Reader h = root.map("hardware");
  h.scalar("name", m.name);
  h.scalar("description", m.description);
  std::string file;
  if (h.scalar("imperfections", file)) {
    m.imperfections = (dir / file).lexically_normal();
  }
  if (h.scalar("controller", file)) {
    m.controller = (dir / file).lexically_normal();
  }
  Reader l = h.map("limits");
  VecN<kDof> rated = VecN<kDof>::Zero();
  VecN<kDof> peak = VecN<kDof>::Zero();
  l.vector<kDof>("rated_torque_nm", yaml_io::kPositive, false, rated);
  l.vector<kDof>("peak_torque_nm", yaml_io::kPositive, false, peak);
  l.finish();
  m.rated_torque_nm.assign(rated.data(), rated.data() + kDof);
  m.peak_torque_nm.assign(peak.data(), peak.data() + kDof);
  read_algorithms(h, m);
  read_provenance(h, m);
  h.finish();
}

// --- rules across the files, after the syntax is clean ---

std::string entry_label(std::size_t i) {
  return "hardware.provenance[" + std::to_string(i) + "]";
}

// One entry against its leaf; a line per problem.
void check_bounds(const ProvenanceEntry& e,
                  const Leaf& leaf,
                  const std::string& label,
                  std::vector<std::string>& errors) {
  if (leaf.shape == LeafShape::kBool || leaf.shape == LeafShape::kChoice) {
    if (e.range || e.scale) {
      errors.push_back(label + ": a bool or choice key takes neither range nor scale");
    }
    return;
  }
  if (e.range && e.scale) {
    errors.push_back(label + ": give one of range and scale, not both");
    return;
  }
  if (!e.range && !e.scale) {
    if (e.kind != ProvenanceKind::kPublished) {
      errors.push_back(label + ": derived and assumed need range or scale");
    }
    return;
  }
  if (e.range && leaf.shape != LeafShape::kNumber) {
    errors.push_back(label + ".range: the profile writes a list here, use scale");
  } else if (e.scale && leaf.shape != LeafShape::kList) {
    errors.push_back(label + ".scale: the profile writes a scalar here, use range");
  } else if (e.range && !((*e.range)[0] <= leaf.value && leaf.value <= (*e.range)[1])) {
    errors.push_back(label + ".range: must hold the file's value " + std::to_string(leaf.value));
  } else if (e.scale && !((*e.scale)[0] >= 0.0 && (*e.scale)[0] <= 1.0 && (*e.scale)[1] >= 1.0)) {
    errors.push_back(label + ".scale: must hold 1 with a lower bound >= 0");
  }
}

void check_provenance(const HardwareModel& m,
                      const std::vector<Leaf>& active,
                      std::vector<std::string>& errors) {
  std::set<std::string> seen;
  for (std::size_t i = 0; i < m.provenance.size(); ++i) {
    const ProvenanceEntry& e = m.provenance[i];
    const std::string label = entry_label(i);
    const auto leaf =
        std::find_if(active.begin(), active.end(), [&e](const Leaf& l) { return l.key == e.key; });
    if (!seen.insert(e.key).second) {
      errors.push_back(label + ".key: " + e.key + " is listed twice");
    } else if (leaf == active.end()) {
      errors.push_back(label + ".key: " + e.key + " is not an active key of " +
                       m.imperfections.string());
    } else {
      check_bounds(e, *leaf, label, errors);
    }
  }
  for (const Leaf& l : active) {
    if (seen.count(l.key) == 0U) {
      errors.push_back("hardware.provenance: no entry for the active key " + l.key);
    }
  }
}

// The task owns the trajectory and the payload (run_overrides): a model that sets them would leave
// a payload the task did not declare, or fail only later, at the run, on a key given twice.
void check_task_owned_keys(const std::string& algorithm,
                           const std::vector<KeyOverride>& overrides,
                           std::vector<std::string>& errors) {
  for (const KeyOverride& o : overrides) {
    if (o.key.rfind("config.payload.", 0) == 0 || o.key.rfind("config.trajectory.", 0) == 0) {
      errors.push_back("hardware.algorithms." + algorithm + ".overrides." + o.key +
                       ": owned by the task");
    }
  }
}

std::vector<std::string> check_model(const HardwareModel& m) {
  std::vector<std::string> errors;
  const Result<std::vector<Leaf>> active = active_leaves(m.imperfections);
  if (active) {
    check_provenance(m, active.value(), errors);
  } else {
    errors.push_back("hardware.imperfections: " + active.reason());
  }
  const auto base = load_config<kDof>(m.controller);
  if (!base) {
    errors.push_back("hardware.controller: " + base.reason());
    return errors;
  }
  if (base.value().payload.kg != 0.0) {
    errors.emplace_back("hardware.controller: cobot.payload is owned by the task");
  }
  for (const auto& kv : m.algorithms) {
    const std::string name(control::to_string(kv.first));
    check_task_owned_keys(name, kv.second, errors);
    const auto config = config_for<kDof>(m, kv.first);
    if (!config) {
      errors.push_back("hardware.algorithms." + name + ": " + config.reason());
    }
  }
  return errors;
}

}  // namespace

Result<HardwareModel> load_hardware_model(const fs::path& yaml) {
  std::error_code ec;
  const fs::path dir = fs::absolute(yaml, ec).parent_path();
  Result<HardwareModel> loaded = yaml_io::load<HardwareModel>(
      yaml, [&dir](Reader& r, HardwareModel& m) { read_model(r, dir, m); });
  if (!loaded) {
    return loaded;
  }
  const std::vector<std::string> errors = check_model(loaded.value());
  if (errors.empty()) {
    return loaded;
  }
  std::string msg = yaml.string() + ":";
  for (const std::string& e : errors) {
    msg += "\n" + e;
  }
  return Result<HardwareModel>::error(msg);
}

std::vector<KeyOverride> model_overrides(const HardwareModel& model, control::Algorithm algorithm) {
  std::vector<KeyOverride> out{
      {"config.control.algorithm", std::string(control::to_string(algorithm))},
      {"config.limits.rated_torque_nm", model.rated_torque_nm},
      {"config.limits.peak_torque_nm", model.peak_torque_nm},
  };
  const auto listed = model.algorithms.find(algorithm);
  if (listed != model.algorithms.end()) {
    out.insert(out.end(), listed->second.begin(), listed->second.end());
  }
  return out;
}

template <int N>
Result<Config<N>> config_for(const HardwareModel& model, control::Algorithm algorithm) {
  if (model.algorithms.count(algorithm) == 0U) {
    std::string names;
    for (const auto& kv : model.algorithms) {
      names += (names.empty() ? "" : ", ") + std::string(control::to_string(kv.first));
    }
    return Result<Config<N>>::error("hardware model " + model.name + " does not list algorithm " +
                                    control::to_string(algorithm) + " (listed: " + names + ")");
  }
  return load_config_with<N>(model.controller, model_overrides(model, algorithm));
}

Result<std::vector<std::string>> active_imperfection_keys(const fs::path& profile) {
  using Out = Result<std::vector<std::string>>;
  const Result<std::vector<Leaf>> leaves = active_leaves(profile);
  if (!leaves) {
    return Out::error(leaves.reason());
  }
  std::vector<std::string> keys;
  keys.reserve(leaves.value().size());
  for (const Leaf& l : leaves.value()) {
    keys.push_back(l.key);
  }
  return Out::ok(std::move(keys));
}

Result<ProfileValue> profile_value(const fs::path& profile, const std::string& key) {
  using Out = Result<ProfileValue>;
  try {
    const YAML::Node root = YAML::LoadFile(profile.string());
    YAML::Node node = root["imperfections"];
    std::size_t begin = 0;
    for (;;) {
      const std::size_t dot = key.find('.', begin);
      const YAML::Node& map = node;  // const: a lookup never adds the key
      const YAML::Node child = node.IsMap() ? map[key.substr(begin, dot - begin)] : YAML::Node();
      if (!child.IsDefined() || child.IsNull()) {
        return Out::error(profile.string() + ": no key " + key);
      }
      node.reset(child);
      if (dot == std::string::npos) {
        break;
      }
      begin = dot + 1;
    }
    if (node.IsSequence()) {
      return Out::ok(ProfileValue{node.as<std::vector<double>>()});
    }
    return Out::ok(ProfileValue{node.as<double>()});
  } catch (const YAML::Exception& e) {
    return Out::error(profile.string() + ": " + key + ": " + e.what());
  }
}

#define COBOT_HARDWARE_INSTANTIATE(N) \
  template Result<Config<(N)>> config_for<(N)>(const HardwareModel&, control::Algorithm);
COBOT_INSTANTIATE_DOF(COBOT_HARDWARE_INSTANTIATE)
#undef COBOT_HARDWARE_INSTANTIATE

}  // namespace cobot::pipeline
