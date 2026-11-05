#include "studio_worker.hpp"

#include <utility>

namespace cobot::studio {

StudioWorker::StudioWorker(std::filesystem::path work_dir) : work_dir_(std::move(work_dir)) {
  qRegisterMetaType<EntryHandle>();
}

void StudioWorker::run(const StudioConfig& config) {
  common::Result<RunEntry> e = run_once(config, work_dir_);
  if (!e) {
    emit finished(QString::fromStdString(e.reason()));
    return;
  }
  emit result(EntryHandle{std::make_shared<RunEntry>(std::move(e.value()))});
  emit finished(QString());
}

void StudioWorker::sweep(const SweepRequest& request) {
  SweepRunner runner(work_dir_);
  const common::Status s = runner.run(
      request,
      cancel_,
      [this](RunEntry&& e) { emit result(EntryHandle{std::make_shared<RunEntry>(std::move(e))}); },
      [this](std::size_t done, std::size_t total) {
        emit progress(static_cast<int>(done), static_cast<int>(total));
      });
  emit finished(s ? QString() : QString::fromStdString(s.reason()));
}

void StudioWorker::load_live(const SegmentReady& s, const bridge::SessionInfo<bench::kN>& session) {
  common::Result<RunEntry> e = make_live_entry(s.csv, s.info, session, live_runs_ + 1);
  if (!e) {
    emit live_failed(QString::fromStdString(e.reason()));
    return;
  }
  ++live_runs_;
  emit result(EntryHandle{std::make_shared<RunEntry>(std::move(e.value()))});
}

}  // namespace cobot::studio
