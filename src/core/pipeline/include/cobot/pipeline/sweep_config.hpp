#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/imperfect/config.hpp"

// Sensitivity sweep files (config/sweeps/*.yaml, feature `sensitivity`, design 11.2), parsed
// here because pipeline is the only module that reads YAML.
namespace cobot::pipeline {

struct SweepParam {
  std::string name;  // dotted key of the imperfection profile, `<group>.<key>`
  std::vector<double> levels;
};

struct SweepSpec {
  std::filesystem::path base_profile;
  std::vector<std::uint64_t> seeds;
  std::vector<SweepParam> params;
};

// Keys `base_profile`, `seeds` (non-empty list of u64), `params` (non-empty list of
// {name, levels}). A relative base_profile is resolved against the sweep file's directory or the
// nearest ancestor where it exists (hardware.yaml names it from the repository root).
common::Result<SweepSpec> load_sweep(const std::filesystem::path& yaml);

// load_imperfections with `<group>.<key>` = name replaced by the scalar `level` (broadcast to
// every joint for per-joint keys) before validation. A name that is not a key of the profile
// is an error naming it.
template <int N>
common::Result<imperfect::ImperfectionConfig<N>> load_imperfections_with(
    const std::filesystem::path& yaml, const std::string& name, double level);

}  // namespace cobot::pipeline
