#include "cobot/studio/verify_job.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <string>
#include <system_error>
#include <utility>

#include "verify_outputs.hpp"  // bench

namespace cobot::studio {

namespace fs = std::filesystem;

common::Result<VerifyOutcome> run_verify_job(const VerifyRequest& request,
                                             const std::atomic<bool>& cancel,
                                             const VerifyProgressFn& progress) {
  using Out = common::Result<VerifyOutcome>;
  if (request.out_dir.empty()) {
    return Out::error("verify: no output directory");
  }
  const common::Result<bench::VerifyInputs> in =
      bench::load_verify_inputs(request.hardware, request.algorithms, request.task);
  if (!in) {
    return Out::error(in.reason());
  }
  const bench::VerifyInputs& v = in.value();
  const auto listed = static_cast<std::size_t>(std::count_if(
      v.pairs.begin(), v.pairs.end(), [](const bench::ComparisonPair& p) { return p.listed; }));
  bench::VerifyOptions options = request.options;
  options.cancel = &cancel;
  // bench::compare runs the listed pairs one after the other; each pair opens with ("tune", 0)
  // when it tunes, else with ("runs", 0), so a new pair starts at either of them unless the
  // ("runs", 0) follows its own tune step.
  std::size_t pair = 0;
  bool started = false;
  std::string last;
  options.progress = [&](std::string_view phase, std::size_t done, std::size_t total) {
    const bool opens = done == 0 && (phase == "tune" || (phase == "runs" && last != "tune"));
    if (opens && started) {
      ++pair;
    }
    started = true;
    last = std::string(phase);
    if (progress) {
      progress(phase, std::min(pair, listed - 1), listed, done, total);
    }
  };
  if (cancel.load()) {
    return Out::error("cancelled");
  }
  common::Result<std::vector<bench::ComparisonRow>> rows =
      bench::compare(v.models, request.algorithms, v.task, options);
  if (!rows) {
    return Out::error(rows.reason());
  }
  // The writer runs only after every run: a late cancel still writes nothing.
  if (cancel.load()) {
    return Out::error("cancelled");
  }
  common::Result<std::vector<fs::path>> files =
      bench::write_verify_outputs(request.out_dir, v.pairs, rows.value(), v.inputs);
  if (!files) {
    return Out::error(files.reason());
  }
  return Out::ok(VerifyOutcome{std::move(rows.value()), std::move(files.value())});
}

common::Result<fs::path> new_verify_dir(const fs::path& work_dir) {
  std::error_code ec;
  fs::create_directories(work_dir, ec);
  for (std::size_t i = 1;; ++i) {
    std::array<char, 24> name{};
    std::snprintf(name.data(), name.size(), "verify_%04zu", i);
    const fs::path dir = work_dir / name.data();
    if (fs::create_directory(dir, ec)) {
      return common::Result<fs::path>::ok(dir);
    }
    if (ec) {
      return common::Result<fs::path>::error("cannot create " + dir.string() + ": " + ec.message());
    }
  }
}

}  // namespace cobot::studio
