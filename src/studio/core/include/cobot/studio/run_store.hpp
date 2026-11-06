#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "cobot/studio/studio_config.hpp"

#include "engine.hpp"
#include "run_data.hpp"

// The runs of a studio session (feature studio-core, ADR-021). Not thread-safe: owned by the GUI
// thread; workers hand finished entries over (runner.hpp ResultFn).
namespace cobot::studio {

struct ErrorTrace;  // metrics.hpp

enum class RunKind { kBench, kSweep, kGazebo /* phase B */ };

struct SweepPoint {
  std::string param;
  double level = 0.0;
  std::uint64_t seed = 0;
};

struct RunEntry {
  std::uint64_t id = 0;  // assigned by RunStore::add
  std::string label;
  RunKind kind = RunKind::kBench;
  StudioConfig config;
  Materialized inputs;  // reproducible with cobot_bench
  // Shared read-only with the chart and export code; null for summary-only sweep seeds.
  std::shared_ptr<const bench::RunData> data;
  std::shared_ptr<const ErrorTrace> errors;  // always set by the runner (~40 B per row)
  bench::RunMeta meta;                       // copy, also when data is null
  bench::RunSummary summary;
  std::optional<SweepPoint> sweep;
  std::string diff;  // describe_diff(config)
  std::string note;  // live runs: why the segment is incomplete (truncated); empty otherwise
};

class RunStore {
 public:
  std::uint64_t add(RunEntry entry);  // assigns the next id (never reused)
  bool remove(std::uint64_t id);      // false when absent
  void clear();
  const RunEntry* find(std::uint64_t id) const;  // nullptr when absent
  const std::vector<RunEntry>& runs() const;     // insertion order
  std::size_t memory_bytes() const;              // records only

 private:
  std::vector<RunEntry> runs_;
  std::uint64_t next_id_ = 1;
};

}  // namespace cobot::studio
