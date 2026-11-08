#pragma once

#include <filesystem>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/control/algorithm.hpp"
#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/task.hpp"

#include "compare.hpp"
#include "report.hpp"

// The inputs and the output files of `cobot_bench verify` (feature studio-verify, ADR-023): the
// CLI and the studio's verify job load and write through this code, so both give the same
// messages and the same bytes.
namespace cobot::bench {

struct VerifyInputs {
  pipeline::Task task;
  std::vector<pipeline::HardwareModel> models;  // in the given order
  std::vector<std::vector<InputFile>> inputs;   // report_inputs of each model
  std::vector<ComparisonPair> pairs;            // comparison_pairs(models, algorithms)
};

// The task, then every model and its input files, then the pairs, all before any run. Errors are
// the CLI's (the text after "cobot_bench verify: "): the loader's reason, "--algorithm: <name>
// given twice", "--hardware: model <name> given twice", comparison_pairs' reason.
common::Result<VerifyInputs> load_verify_inputs(const std::vector<std::filesystem::path>& models,
                                                const std::vector<control::Algorithm>& algorithms,
                                                const std::filesystem::path& task);

// One pair in the cross product: report.{json,md} (+ tune.csv) in out_dir itself. More: one
// <pair_name> directory per listed pair and comparison.{json,md} in out_dir. Every file goes to a
// temporary name first and is renamed only once all are written: all files or none. The written
// paths, in writing order.
common::Result<std::vector<std::filesystem::path>> write_verify_outputs(
    const std::filesystem::path& out_dir,
    const std::vector<ComparisonPair>& pairs,
    const std::vector<ComparisonRow>& rows,
    const std::vector<std::vector<InputFile>>& inputs);

}  // namespace cobot::bench
