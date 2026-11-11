#include "cobot/studio/noise_level.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace cobot::studio {
namespace {

using Value = decltype(pipeline::KeyOverride::value);
using List = std::vector<double>;

// Profile comes first so it wins when several levels match (every base 0).
constexpr std::array<NoiseLevel, 4> kLevels = {
    NoiseLevel::kProfile, NoiseLevel::kOff, NoiseLevel::kX2, NoiseLevel::kX5};

// The base value with every element multiplied by k; empty for a bool or string base.
std::optional<Value> scaled(const Value& base, double k) {
  if (const auto* d = std::get_if<double>(&base)) {
    return Value{*d * k};
  }
  if (const auto* list = std::get_if<List>(&base)) {
    List out = *list;
    for (double& x : out) {
      x *= k;
    }
    return Value{std::move(out)};
  }
  return std::nullopt;
}

// value == base * k element-wise. A double value against a list base stands for every entry
// (pipeline::KeyOverride).
bool matches(const Value& value, const Value& base, double k) {
  const std::optional<Value> want = scaled(base, k);
  if (!want) {
    return false;
  }
  if (const auto* d = std::get_if<double>(&value)) {
    if (const auto* list = std::get_if<List>(&*want)) {
      return std::all_of(list->begin(), list->end(), [&](double x) { return x == *d; });
    }
  }
  return value == *want;
}

const Value& effective(const StudioConfig& config, const std::string& key, const Value& base) {
  for (const pipeline::KeyOverride& o : config.overrides) {
    if (o.key == key) {
      return o.value;
    }
  }
  return base;
}

bool level_matches(const StudioConfig& config,
                   const std::map<std::string, pipeline::KeyOverride>& base,
                   double k) {
  for (const char* key : kNoiseKeys) {
    const auto it = base.find(key);
    if (it == base.end()) {
      continue;
    }
    if (!matches(effective(config, key, it->second.value), it->second.value, k)) {
      return false;
    }
  }
  return true;
}

bool is_noise_key(const std::string& key) {
  return std::any_of(kNoiseKeys.begin(), kNoiseKeys.end(), [&](const char* k) { return key == k; });
}

}  // namespace

double noise_factor(NoiseLevel level) {
  switch (level) {
    case NoiseLevel::kOff:
      return 0.0;
    case NoiseLevel::kProfile:
      return 1.0;
    case NoiseLevel::kX2:
      return 2.0;
    case NoiseLevel::kX5:
      return 5.0;
    case NoiseLevel::kCustom:
      break;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

NoiseLevel noise_level(const StudioConfig& config,
                       const std::map<std::string, pipeline::KeyOverride>& base) {
  for (const NoiseLevel level : kLevels) {
    if (level_matches(config, base, noise_factor(level))) {
      return level;
    }
  }
  return NoiseLevel::kCustom;
}

void apply_noise_level(StudioConfig& config,
                       const std::map<std::string, pipeline::KeyOverride>& base,
                       NoiseLevel level) {
  if (level == NoiseLevel::kCustom) {
    return;
  }
  auto& o = config.overrides;
  o.erase(
      std::remove_if(
          o.begin(), o.end(), [](const pipeline::KeyOverride& x) { return is_noise_key(x.key); }),
      o.end());
  const double k = noise_factor(level);
  if (k == 1.0) {
    return;
  }
  for (const char* key : kNoiseKeys) {
    const auto it = base.find(key);
    if (it == base.end()) {
      continue;
    }
    std::optional<Value> v = scaled(it->second.value, k);
    if (v && !(*v == it->second.value)) {
      o.push_back(pipeline::KeyOverride{key, std::move(*v)});
    }
  }
}

}  // namespace cobot::studio
