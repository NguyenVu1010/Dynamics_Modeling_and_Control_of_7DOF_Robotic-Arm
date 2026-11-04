#pragma once

#include <QMetaType>
#include <QObject>
#include <QString>
#include <atomic>
#include <filesystem>
#include <memory>

#include "cobot/studio/run_store.hpp"
#include "cobot/studio/runner.hpp"
#include "cobot/studio/studio_config.hpp"

// The worker side of cobot_studio (feature studio-app, ADR-021): run_once and SweepRunner::run
// execute on the QThread this object lives in; each finished RunEntry crosses to the GUI thread
// by a queued signal. Only the GUI thread touches the RunStore.
namespace cobot::studio {

// A finished entry handed to the GUI thread (the worker drops its reference after emitting).
struct EntryHandle {
  std::shared_ptr<RunEntry> entry;
};

class StudioWorker : public QObject {
  Q_OBJECT
 public:
  explicit StudioWorker(std::filesystem::path work_dir);

  // Called on the worker thread (QMetaObject::invokeMethod with a functor).
  void run(const StudioConfig& config);
  void sweep(const SweepRequest& request);
  // Any thread: the sweep stops before its next run; arm() before queueing a new job.
  void cancel() { cancel_.store(true); }
  void arm() { cancel_.store(false); }

 signals:
  void result(cobot::studio::EntryHandle entry);
  void progress(int done, int total);
  void finished(QString error);  // empty on success

 private:
  std::filesystem::path work_dir_;
  std::atomic<bool> cancel_{false};
};

}  // namespace cobot::studio

Q_DECLARE_METATYPE(cobot::studio::EntryHandle)
