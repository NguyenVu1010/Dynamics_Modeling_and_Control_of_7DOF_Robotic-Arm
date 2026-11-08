#include "compare.hpp"

#include <utility>

namespace cobot::bench {

std::string pair_name(const std::string& model, control::Algorithm algorithm) {
  return model + "__" + control::to_string(algorithm);
}

common::Result<std::vector<ComparisonPair>> comparison_pairs(
    const std::vector<pipeline::HardwareModel>& models,
    const std::vector<control::Algorithm>& algorithms) {
  using Out = common::Result<std::vector<ComparisonPair>>;
  std::vector<ComparisonPair> pairs;
  bool any = false;
  std::string names;
  for (const control::Algorithm a : algorithms) {
    names += (names.empty() ? "" : ", ") + std::string(control::to_string(a));
  }
  std::string model_names;
  for (const pipeline::HardwareModel& m : models) {
    model_names += (model_names.empty() ? "" : ", ") + m.name;
  }
  for (std::size_t m = 0; m < models.size(); ++m) {
    for (const control::Algorithm a : algorithms) {
      const bool listed = models[m].algorithms.count(a) > 0;
      pairs.push_back(ComparisonPair{m, a, listed});
      any = any || listed;
    }
  }
  if (!any) {
    return Out::error("no hardware model of (" + model_names + ") lists an algorithm of (" + names +
                      ")");
  }
  return Out::ok(std::move(pairs));
}

common::Result<std::vector<ComparisonRow>> compare(
    const std::vector<pipeline::HardwareModel>& models,
    const std::vector<control::Algorithm>& algorithms,
    const pipeline::Task& task,
    const VerifyOptions& options) {
  using Out = common::Result<std::vector<ComparisonRow>>;
  const common::Result<std::vector<ComparisonPair>> pairs = comparison_pairs(models, algorithms);
  if (!pairs) {
    return Out::error(pairs.reason());
  }
  std::vector<ComparisonRow> rows;
  for (const ComparisonPair& p : pairs.value()) {
    ComparisonRow row{models[p.model].name, p.algorithm, std::nullopt};
    if (p.listed) {
      common::Result<VerifyResult> r = verify(models[p.model], p.algorithm, task, options);
      if (!r) {
        // A cancel stays the bare "cancelled" of verify (verify.hpp), so a caller can tell it.
        if (r.reason() == "cancelled") {
          return Out::error("cancelled");
        }
        return Out::error(pair_name(row.model, p.algorithm) + ": " + r.reason());
      }
      row.result = std::move(r.value());
    }
    rows.push_back(std::move(row));
  }
  return Out::ok(std::move(rows));
}

}  // namespace cobot::bench
