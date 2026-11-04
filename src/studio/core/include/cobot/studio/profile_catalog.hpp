#pragma once

#include <filesystem>
#include <string>
#include <vector>

// Base pairs of the studio (feature studio-core, cobot_studio design 6.7, ADR-021).
namespace cobot::studio {

namespace fs = std::filesystem;

// A controller config and an imperfection profile run together.
struct BasePair {
  std::string name;  // the profile's file stem
  fs::path config_yaml;
  fs::path imperfections_yaml;
};

// One pair per config_dir/imperfections/*.yaml, ordered ideal, realistic, pessimistic,
// premium_fr3, mid_kr1205, budget_xarm7, then the others alphabetically. The controller config
// is config_dir/<name>.yaml when it exists, else config_dir/default.yaml.
std::vector<BasePair> discover_base_pairs(const fs::path& config_dir);

}  // namespace cobot::studio
