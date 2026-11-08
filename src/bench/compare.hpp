#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/task.hpp"

#include "verify.hpp"

// The comparison of several hardware models and algorithms (feature verify-analysis, design 10.2
// G16, docs/conventions.md "Verify comparison"): the CLI with lists and the studio use this code.
namespace cobot::bench {

// "<model>__<algorithm>": the directory of a pair's report files.
std::string pair_name(const std::string& model, control::Algorithm algorithm);

struct ComparisonPair {
  std::size_t model = 0;  // index into the models
  control::Algorithm algorithm = control::Algorithm::kComputedTorque;
  bool listed = false;  // the model lists the algorithm
};

// The cross product, models outer and algorithms inner, in the given orders. No listed pair (empty
// lists included) is an error naming the algorithms.
common::Result<std::vector<ComparisonPair>> comparison_pairs(
    const std::vector<pipeline::HardwareModel>& models,
    const std::vector<control::Algorithm>& algorithms);

struct ComparisonRow {
  std::string model;
  control::Algorithm algorithm = control::Algorithm::kComputedTorque;
  std::optional<VerifyResult> result;  // absent: not listed
};

// verify of every listed pair, one after the other (each with options.jobs), rows in the order of
// comparison_pairs; the first error stops and comes back.
common::Result<std::vector<ComparisonRow>> compare(
    const std::vector<pipeline::HardwareModel>& models,
    const std::vector<control::Algorithm>& algorithms,
    const pipeline::Task& task,
    const VerifyOptions& options);

// comparison.json: {"disclaimer", "rows": [{model, algorithm, verdict ("pass" | "fail" |
// "marginal" | "not listed"), pass_count, runs, wilson, worst: {<metric>: value}, gain_used,
// gain_source}]}, numbers %.17g (NaN null); comparison.md: the disclaimer on its first line, then
// one table row per pair. Same rows, same bytes (report.cpp).
std::string comparison_json(const std::vector<ComparisonRow>& rows);
std::string comparison_md(const std::vector<ComparisonRow>& rows);

}  // namespace cobot::bench
