#include "verify_outputs.hpp"

#include <algorithm>
#include <fstream>
#include <functional>
#include <string>
#include <system_error>
#include <utility>

#include "tune.hpp"

namespace cobot::bench {
namespace {

namespace fs = std::filesystem;
using common::Status;

// One file to write: `write` fills the given (temporary) path.
struct OutFile {
  fs::path path;
  std::function<Status(const fs::path&)> write;
};

Status write_text(const fs::path& path, const std::string& text) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << text;
  out.flush();
  return out ? Status::ok() : Status::error("cannot write " + path.string());
}

fs::path temp_of(const fs::path& p) {
  return p.parent_path() / ("." + p.filename().string() + ".tmp");
}

Status write_all(const std::vector<OutFile>& files) {
  const auto discard = [&](bool finals) {
    for (const OutFile& f : files) {
      std::error_code ignored;
      fs::remove(temp_of(f.path), ignored);
      if (finals) {
        fs::remove(f.path, ignored);
      }
    }
  };
  for (const OutFile& f : files) {
    std::error_code ec;
    fs::create_directories(f.path.parent_path(), ec);
    const Status st = f.write(temp_of(f.path));
    if (!st) {
      discard(false);
      return st;
    }
  }
  for (const OutFile& f : files) {
    std::error_code ec;
    fs::rename(temp_of(f.path), f.path, ec);
    if (ec) {
      discard(true);
      return Status::error("cannot write " + f.path.string() + ": " + ec.message());
    }
  }
  return Status::ok();
}

}  // namespace

common::Result<VerifyInputs> load_verify_inputs(const std::vector<fs::path>& models,
                                                const std::vector<control::Algorithm>& algorithms,
                                                const fs::path& task) {
  using Out = common::Result<VerifyInputs>;
  for (auto a = algorithms.begin(); a != algorithms.end(); ++a) {
    if (std::find(algorithms.begin(), a, *a) != a) {
      return Out::error(std::string("--algorithm: ") + control::to_string(*a) + " given twice");
    }
  }
  common::Result<pipeline::Task> t = pipeline::load_task(task);
  if (!t) {
    return Out::error(t.reason());
  }
  VerifyInputs in;
  in.task = std::move(t.value());
  for (const fs::path& yaml : models) {
    common::Result<pipeline::HardwareModel> model = pipeline::load_hardware_model(yaml);
    if (!model) {
      return Out::error(model.reason());
    }
    const common::Result<std::vector<InputFile>> files =
        report_inputs(yaml, model.value(), task, in.task);
    if (!files) {
      return Out::error(files.reason());
    }
    for (const pipeline::HardwareModel& earlier : in.models) {
      if (earlier.name == model.value().name) {
        return Out::error("--hardware: model " + earlier.name + " given twice");
      }
    }
    in.models.push_back(std::move(model.value()));
    in.inputs.push_back(files.value());
  }
  common::Result<std::vector<ComparisonPair>> pairs = comparison_pairs(in.models, algorithms);
  if (!pairs) {
    return Out::error(pairs.reason());
  }
  in.pairs = std::move(pairs.value());
  return Out::ok(std::move(in));
}

common::Result<std::vector<fs::path>> write_verify_outputs(
    const fs::path& out_dir,
    const std::vector<ComparisonPair>& pairs,
    const std::vector<ComparisonRow>& rows,
    const std::vector<std::vector<InputFile>>& inputs) {
  using Out = common::Result<std::vector<fs::path>>;
  const bool single = pairs.size() == 1;
  std::vector<OutFile> files;
  const auto text = [](std::string s) {
    return [s = std::move(s)](const fs::path& p) { return write_text(p, s); };
  };
  for (std::size_t i = 0; i < rows.size() && i < pairs.size(); ++i) {
    const ComparisonRow& row = rows[i];
    if (!row.result) {
      continue;
    }
    const VerifyResult& r = *row.result;
    const std::vector<InputFile>& in = inputs.at(pairs[i].model);
    const fs::path dir = single ? out_dir : out_dir / pair_name(row.model, row.algorithm);
    files.push_back(OutFile{dir / "report.json", text(report_json(r, in))});
    files.push_back(OutFile{dir / "report.md", text(report_md(r, in))});
    if (r.tuning) {
      files.push_back(OutFile{dir / "tune.csv",
                              [&r](const fs::path& p) { return write_tune_csv(p, *r.tuning); }});
    }
  }
  if (!single) {
    files.push_back(OutFile{out_dir / "comparison.json", text(comparison_json(rows))});
    files.push_back(OutFile{out_dir / "comparison.md", text(comparison_md(rows))});
  }
  const Status st = write_all(files);
  if (!st) {
    return Out::error(st.reason());
  }
  std::vector<fs::path> written;
  written.reserve(files.size());
  for (const OutFile& f : files) {
    written.push_back(f.path);
  }
  return Out::ok(std::move(written));
}

}  // namespace cobot::bench
