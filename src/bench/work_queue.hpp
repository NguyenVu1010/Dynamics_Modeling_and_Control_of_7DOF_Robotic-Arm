#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cobot/common/result.hpp"

// A small work queue of the bench (feature auto-tune; verify reuses it): `count` items on `jobs`
// worker threads. Each item writes its own result slot, so what comes out does not depend on the
// scheduling. Off the control path: threads and heap allocation are fine here.
namespace cobot::bench {

// Runs work(i) for every i in [0, count) on min(jobs, count) threads (at least one), items taken
// in order. After the first failed item no new item starts; the error of the lowest failed index
// among those that ran comes back. An exception thrown by `work` is an error with its what().
inline common::Status run_queue(std::size_t count,
                                unsigned jobs,
                                const std::function<common::Status(std::size_t)>& work) {
  std::atomic<std::size_t> next{0};
  std::atomic<bool> failed{false};
  std::mutex mutex;
  std::optional<std::pair<std::size_t, std::string>> first_error;
  const auto worker = [&]() {
    for (;;) {
      if (failed.load(std::memory_order_acquire)) {
        return;
      }
      const std::size_t i = next.fetch_add(1, std::memory_order_relaxed);
      if (i >= count) {
        return;
      }
      common::Status st = common::Status::ok();
      try {
        st = work(i);
      } catch (const std::exception& e) {
        st = common::Status::error(std::string(e.what()));
      }
      if (!st) {
        const std::lock_guard<std::mutex> lock(mutex);
        if (!first_error || i < first_error->first) {
          first_error = std::make_pair(i, st.reason());
        }
        failed.store(true, std::memory_order_release);
      }
    }
  };
  const std::size_t threads = std::max<std::size_t>(1, std::min<std::size_t>(jobs, count));
  std::vector<std::thread> pool;
  pool.reserve(threads - 1);
  try {
    for (std::size_t t = 1; t < threads; ++t) {
      pool.emplace_back(worker);
    }
  } catch (const std::exception& e) {
    // A thread could not start: stop the ones already running and report it, never terminate.
    failed.store(true, std::memory_order_release);
    for (std::thread& t : pool) {
      t.join();
    }
    return common::Status::error(std::string("cannot start a worker thread: ") + e.what());
  }
  worker();
  for (std::thread& t : pool) {
    t.join();
  }
  return first_error ? common::Status::error(first_error->second) : common::Status::ok();
}

}  // namespace cobot::bench
