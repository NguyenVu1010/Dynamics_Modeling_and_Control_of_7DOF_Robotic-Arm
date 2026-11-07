#pragma once

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <set>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"

// Strict YAML reading shared by the loaders of this module (config.cpp, hardware_model.cpp,
// task.cpp): every problem becomes one line "<full.key.path>: <reason>" and all lines are returned
// together, so a file is fixed in one pass. Internal to pipeline, the only module that parses YAML.
namespace cobot::pipeline::yaml_io {

namespace fs = std::filesystem;
using common::Result;
using common::VecN;

// Allowed values of a number and the message when a value is outside them.
struct Rule {
  bool (*ok)(double);
  const char* text;
};
inline const Rule kAny{[](double) { return true; }, ""};
inline const Rule kPositive{[](double v) { return v > 0.0; }, "must be > 0"};
inline const Rule kNonNegative{[](double v) { return v >= 0.0; }, "must be >= 0"};
inline const Rule kUnitInterval{[](double v) { return v > 0.0 && v <= 1.0; }, "must be in (0, 1]"};

// One YAML mapping. Each key is looked up once; finish() reports keys never asked for. A
// missing or non-map node gives a reader whose lookups do nothing, so errors do not cascade.
class Reader {
 public:
  Reader(const YAML::Node& node, std::string path, std::vector<std::string>& errors)
      : node_(node), path_(std::move(path)), errors_(&errors) {}

  void fail(const std::string& key, const std::string& reason) {
    errors_->push_back(full(key) + ": " + reason);
  }

  void finish() {
    if (!node_.IsMap()) {
      return;
    }
    for (const auto& kv : node_) {
      if (used_.count(kv.first.Scalar()) == 0U) {
        fail(kv.first.Scalar(), "unknown key");
      }
    }
  }

  // The raw child node (empty after reporting "missing"), for lists the helpers do not cover.
  YAML::Node node(const std::string& key) { return get(key); }

  // Reader over `n`, an element of a list of this mapping labelled `label`.
  Reader child(const YAML::Node& n, const std::string& label) {
    if (!n.IsMap()) {
      fail(label, "expected a mapping");
    }
    return {n.IsMap() ? n : YAML::Node(), full(label), *errors_};
  }

  // True when `key` is present and not null, for optional sections; reports nothing. A key
  // present with a null value (`key:` or `key: ~`) counts as absent but is marked as read, so
  // finish() does not call it unknown.
  bool has(const std::string& key) {
    const YAML::Node& map = node_;  // const lookup never inserts the key
    if (!node_.IsMap() || !map[key].IsDefined()) {
      return false;
    }
    used_.insert(key);
    return !map[key].IsNull();
  }

  Reader map(const std::string& key) {
    const YAML::Node n = get(key);
    if (n && !n.IsMap()) {
      fail(key, "expected a mapping");
    }
    return {n && n.IsMap() ? n : YAML::Node(), full(key), *errors_};
  }

  // bool, int, std::uint64_t or non-empty std::string; true when `out` was set.
  template <typename V>
  bool scalar(const std::string& key, V& out) {
    const YAML::Node n = get(key);
    V v{};
    if (!n) {
      return false;
    }
    if (!n.IsScalar() || n.Scalar().empty() || !YAML::convert<V>::decode(n, v)) {
      fail(key, "invalid value '" + YAML::Dump(n) + "'");
      return false;
    }
    out = v;
    return true;
  }

  void number(const std::string& key, const Rule& rule, double& out) {
    const YAML::Node n = get(key);
    double v = 0.0;
    if (n && check(key, n, rule, v)) {
      out = v;
    }
  }

  // Exactly M numbers; with `broadcast`, one scalar sets all M entries.
  template <int M>
  void vector(const std::string& key, const Rule& rule, bool broadcast, VecN<M>& out) {
    const YAML::Node n = get(key);
    double s = 0.0;
    if (!n) {
      return;
    }
    if (broadcast && n.IsScalar()) {
      if (check(key, n, rule, s)) {
        out.setConstant(s);
      }
      return;
    }
    if (!n.IsSequence() || n.size() != static_cast<std::size_t>(M)) {
      fail(key,
           std::string(broadcast ? "expected a scalar or " : "expected ") + std::to_string(M) +
               " values, got " +
               (n.IsSequence() ? std::to_string(n.size()) : "'" + YAML::Dump(n) + "'"));
      return;
    }
    VecN<M> v = VecN<M>::Zero();
    bool ok = true;
    for (int i = 0; i < M; ++i) {
      const std::string label = key + "[" + std::to_string(i) + "]";
      ok = check(label, n[static_cast<std::size_t>(i)], rule, v[i]) && ok;
    }
    if (ok) {
      out = v;
    }
  }

 private:
  std::string full(const std::string& key) const { return path_.empty() ? key : path_ + "." + key; }

  // A finite number from a scalar node that satisfies `rule`; reports under `label`.
  bool check(const std::string& label, const YAML::Node& n, const Rule& rule, double& out) {
    if (!n.IsScalar() || !YAML::convert<double>::decode(n, out) || !std::isfinite(out)) {
      fail(label, "expected a number, got '" + YAML::Dump(n) + "'");
      return false;
    }
    if (!rule.ok(out)) {
      fail(label, std::string(rule.text) + ", got " + n.Scalar());
      return false;
    }
    return true;
  }

  // The child node, or an empty node (after reporting "missing") when the key is absent.
  YAML::Node get(const std::string& key) {
    used_.insert(key);
    if (!node_.IsMap()) {
      return {};
    }
    const YAML::Node& map = node_;  // const lookup never inserts the key
    const YAML::Node n = map[key];
    if (!n.IsDefined() || n.IsNull()) {
      fail(key, "missing");
      return {};
    }
    return n;
  }

  YAML::Node node_;
  std::string path_;
  std::vector<std::string>* errors_;
  std::set<std::string> used_;
};

// No change to the loaded document.
inline std::string keep(YAML::Node& /*root*/) {
  return {};
}

// Loads the file, applies `edit(root)` (a non-empty return is the error), runs
// `read(root, value)` and turns all collected problems into one error.
template <typename T, typename ReadFn, typename EditFn = std::string (*)(YAML::Node&)>
Result<T> load(const fs::path& yaml, ReadFn read, EditFn edit = keep) {
  std::error_code ec;
  if (!fs::is_regular_file(yaml, ec)) {
    return Result<T>::error("cannot open " + yaml.string() + ": no such file");
  }
  std::vector<std::string> errors;
  T value{};
  try {
    // NOLINTNEXTLINE(misc-const-correctness): edit() may change the document
    YAML::Node node = YAML::LoadFile(yaml.string());
    if (!node.IsMap()) {
      return Result<T>::error(yaml.string() + ": expected a mapping at the top level");
    }
    const std::string bad = edit(node);
    if (!bad.empty()) {
      return Result<T>::error(yaml.string() + ": " + bad);
    }
    Reader root(node, "", errors);
    read(root, value);
    root.finish();
  } catch (const YAML::Exception& e) {
    return Result<T>::error(yaml.string() + ": " + e.what());
  }
  if (errors.empty()) {
    return Result<T>::ok(std::move(value));
  }
  std::string msg = yaml.string() + ":";
  for (const std::string& e : errors) {
    msg += "\n" + e;
  }
  return Result<T>::error(msg);
}

}  // namespace cobot::pipeline::yaml_io
