#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/pipeline/hardware_model.hpp"
#include "cobot/pipeline/task.hpp"

#include "verify.hpp"

// report.json and report.md of a verify result (feature verify, docs/conventions.md "Verify
// report"): the same content, keys in a fixed order, numbers %.17g in the JSON (NaN is null), no
// wall time, so the same inputs give the same bytes whatever the number of jobs.
namespace cobot::bench {

// ADR-023: the first line of report.md and the `disclaimer` of report.json.
inline constexpr const char* kVerifyDisclaimer =
    "Screening and comparison tool, not a certification: it does not replace a safety "
    "assessment to ISO 10218 or ISO/TS 15066, nor measurements on the real robot.";

struct InputFile {
  std::string path;
  std::string sha256;  // sha256_hex of the file's bytes
};

// The hardware model file, its imperfection and controller files, the task file and its waypoint
// file (when it has one), each with the sha256 of its bytes. A file that cannot be read is an
// error naming it.
common::Result<std::vector<InputFile>> report_inputs(const std::filesystem::path& model_yaml,
                                                     const pipeline::HardwareModel& model,
                                                     const std::filesystem::path& task_yaml,
                                                     const pipeline::Task& task);

std::string report_json(const VerifyResult& result, const std::vector<InputFile>& inputs);
std::string report_md(const VerifyResult& result, const std::vector<InputFile>& inputs);

}  // namespace cobot::bench
