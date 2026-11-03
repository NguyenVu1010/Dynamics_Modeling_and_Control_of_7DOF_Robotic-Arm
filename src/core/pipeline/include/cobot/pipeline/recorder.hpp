#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <ostream>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/pipeline/log_record.hpp"

// In-memory log of the control loop (feature pipeline-recorder, design
// docs/design/2026-11-02-cobot-studio-design.md 6.1, ADR-020). The host owns a storage
// (preallocated record buffers) and a Recorder over it; ControlLoop::step() copies each record
// into it with record(), which never allocates, locks, blocks or does I/O. The CSV is written off
// the control thread: after the run (bench) or from a retired buffer (two buffers: the exporter
// thread asks for a swap, the next record() switches buffers in O(1), the exporter writes the
// retired one and releases it).
//
// Threads: record() runs on the control thread only; request_swap(), swap_done(), retired() and
// release_retired() on one exporter thread. active() and clear() only while no record() can run.
// size(), dropped(), overwritten() and recorded() are atomic loads, readable from any thread at
// any time.
namespace cobot::pipeline {

enum class FullPolicy {
  kStopWhenFull,     // a full buffer rejects the record and counts it in dropped()
  kOverwriteOldest,  // a full buffer overwrites its oldest row and counts it in overwritten()
};

// Atomics shared by the producer (control thread) and one exporter thread. Self-contained (no
// pointers, only lock-free atomics) so that a storage can place it in shared memory.
struct RecorderControl {
  std::atomic<std::uint32_t> active{0};      // index of the buffer record() writes
  std::atomic<std::uint32_t> swap_state{0};  // Recorder's kSwapIdle / kSwapRequested / kSwapDone
  std::array<std::atomic<std::uint64_t>, 2> written{};  // records written into each buffer
  std::atomic<std::uint64_t> dropped{0};
  std::atomic<std::uint64_t> overwritten{0};
  std::atomic<std::uint64_t> recorded{0};
};
static_assert(std::atomic<std::uint32_t>::is_always_lock_free, "RecorderControl in shared memory");
static_assert(std::atomic<std::uint64_t>::is_always_lock_free, "RecorderControl in shared memory");

template <int N>
struct RecorderMemory {  // non-owning view handed to Recorder by a storage
  RecorderControl* control = nullptr;
  std::array<LogRecord<N>*, 2> buffers{};  // buffers[1] null with one buffer
  std::uint64_t* sequence = nullptr;       // per slot, 2 * capacity (optional for heap)
  std::size_t capacity = 0;                // records per buffer, > 0
  int buffer_count = 1;                    // 1 or 2
};

// Heap storage (bench, tests): everything allocated and value-initialised in the constructor, so
// the pages are touched before the run.
template <int N>
class HeapStorage {
 public:
  // Throws std::invalid_argument when capacity == 0 or buffers is not 1 or 2.
  HeapStorage(std::size_t capacity, int buffers);
  RecorderMemory<N> memory();

 private:
  std::unique_ptr<RecorderControl> control_;
  std::vector<LogRecord<N>> records_;  // buffers * capacity
  std::vector<std::uint64_t> sequence_;
  std::size_t capacity_;
  int buffers_;
};

// Ordered view, oldest first; two parts when a kOverwriteOldest buffer has wrapped.
template <int N>
struct RecordSpan {
  const LogRecord<N>* first = nullptr;
  std::size_t first_size = 0;
  const LogRecord<N>* second = nullptr;
  std::size_t second_size = 0;
  std::size_t size() const { return first_size + second_size; }
  const LogRecord<N>& operator[](std::size_t i) const {
    return i < first_size ? first[i] : second[i - first_size];
  }
};

template <int N>
class Recorder {
 public:
  // The storage must outlive the Recorder. Resets the counters of memory.control. Throws
  // std::invalid_argument when `memory` does not describe 1 or 2 buffers of capacity > 0.
  Recorder(RecorderMemory<N> memory, FullPolicy policy);

  // Control thread only. O(1), no allocation, lock or I/O. Performs a pending swap first.
  void record(const LogRecord<N>& r) noexcept;

  // Exporter thread. false with one buffer, while a swap is pending, or while the retired
  // buffer has not been released.
  bool request_swap() noexcept;
  bool swap_done() const noexcept;         // acquire; true once record() switched buffers
  RecordSpan<N> retired() const noexcept;  // valid between swap_done() and release_retired()
  void release_retired() noexcept;

  // Only while no record() can run (producer stopped or never started).
  RecordSpan<N> active() const noexcept;
  void clear() noexcept;

  std::size_t size() const noexcept { return active().size(); }  // rows in the active buffer
  std::uint64_t dropped() const noexcept { return load(memory_.control->dropped); }
  std::uint64_t overwritten() const noexcept { return load(memory_.control->overwritten); }
  // Every accepted record since construction.
  std::uint64_t recorded() const noexcept { return load(memory_.control->recorded); }
  std::size_t capacity() const noexcept { return memory_.capacity; }
  FullPolicy policy() const noexcept { return policy_; }

 private:
  static std::uint64_t load(const std::atomic<std::uint64_t>& a) noexcept {
    return a.load(std::memory_order_acquire);
  }
  RecordSpan<N> span_of(std::uint32_t buffer) const noexcept;

  RecorderMemory<N> memory_;
  FullPolicy policy_;
};

// The Run CSV of `rows` (write_csv_header when `header`, then write_csv_row per row, oldest
// first).
template <int N>
void write_csv(std::ostream& os, const RecordSpan<N>& rows, bool header);

// write_csv into `csv`, appended (append) or replacing the file. The header is written only when
// the file is new or empty. Errors name the path.
template <int N>
common::Status export_csv(const std::filesystem::path& csv, const RecordSpan<N>& rows, bool append);

}  // namespace cobot::pipeline
