#pragma once

#include <QMetaType>
#include <QObject>
#include <QString>
#include <atomic>
#include <filesystem>
#include <memory>

#include "cobot/bridge/wire.hpp"
#include "cobot/studio/bridge_client.hpp"
#include "cobot/studio/run_store.hpp"
#include "cobot/studio/runner.hpp"
#include "cobot/studio/studio_config.hpp"
#include "cobot/studio/verify_job.hpp"

// The worker side of cobot_studio (feature studio-app, ADR-021): run_once and SweepRunner::run
// execute on the QThread this object lives in; each finished RunEntry crosses to the GUI thread
// by a queued signal. Only the GUI thread touches the RunStore.
namespace cobot::studio {

// A finished entry handed to the GUI thread (the worker drops its reference after emitting).
struct EntryHandle {
  std::shared_ptr<RunEntry> entry;
};

// A finished verify job handed to the GUI thread (feature studio-verify).
struct VerifyHandle {
  std::shared_ptr<VerifyOutcome> outcome;
};

class StudioWorker : public QObject {
  Q_OBJECT
 public:
  explicit StudioWorker(std::filesystem::path work_dir);

  // Called on the worker thread (QMetaObject::invokeMethod with a functor).
  void run(const StudioConfig& config);
  void sweep(const SweepRequest& request);
  // A finished live run (feature studio-control): make_live_entry, then `result`; an error goes
  // to `live_failed`. Live runs are numbered in arrival order.
  void load_live(const SegmentReady& s, const bridge::SessionInfo<bench::kN>& session);
  // Any thread: the sweep stops before its next run; arm() before queueing a new job.
  void cancel() { cancel_.store(true); }
  void arm() { cancel_.store(false); }
  // A verify job (feature studio-verify): run_verify_job with its own cancel flag, progress and
  // the outcome as queued signals. cancel_verify() from any thread stops it before its next run.
  void verify(const VerifyRequest& request);
  void cancel_verify() { verify_cancel_.store(true); }
  void arm_verify() { verify_cancel_.store(false); }

 signals:
  void result(cobot::studio::EntryHandle entry);
  void progress(int done, int total);
  void finished(QString error);  // empty on success
  void live_failed(QString error);
  void verify_progress(QString phase, int pair, int pairs, int done, int total);
  void verify_finished(cobot::studio::VerifyHandle outcome, QString error);  // error empty: ok

 private:
  std::filesystem::path work_dir_;
  std::atomic<bool> cancel_{false};
  std::atomic<bool> verify_cancel_{false};
  int live_runs_ = 0;  // worker thread
};

}  // namespace cobot::studio

Q_DECLARE_METATYPE(cobot::studio::EntryHandle)
Q_DECLARE_METATYPE(cobot::studio::VerifyHandle)
