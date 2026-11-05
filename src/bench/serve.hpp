#pragma once

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>

#include "cobot/common/result.hpp"

// cobot_bench serve (feature controller-bridge, plan owner question B3): a paced bench host of
// the studio bridge, so the studio, cobot_bridge_cli and the tests drive a control loop without
// Gazebo.
namespace cobot::bench {

struct ServeOptions {
  std::filesystem::path config_yaml, imperfections_yaml;
  std::optional<std::uint64_t> seed;
  std::string name = "cobot_bench";  // region /cobot_<name>
  std::filesystem::path out_dir;     // Run CSV <out_dir>/run.csv; segments <out_dir>/segments
  std::optional<double> duration_s;  // empty: until `stop`
};

// Paced (clock_nanosleep, CLOCK_MONOTONIC) control loop against SimPlant built as run_profile
// builds it, command_source forced external, log.mode forced recorder_live; Recorder =
// HeapStorage of two buffers of log.capacity_s (kOverwriteOldest); bridge thread:
// CommandService::poll every 5 ms and an export every 200 ms (swap + append, one exporter at a
// time). apply_config rebuilds config, controller model, decorator and ControlLoop over the same
// SimPlant and Recorder (plant effects keep the start profile, like Gazebo physics) and swaps them
// at a step boundary in standby; the old runtime is destroyed on the bridge thread. `ready(region
// name)` is called once the region exists. Returns after stop/duration with a final export; the
// region is removed.
common::Status serve(const ServeOptions& options,
                     const std::atomic<bool>& stop,
                     const std::function<void(const std::string&)>& ready = {});

}  // namespace cobot::bench
