#include "cobot/pipeline/recorder.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <stdexcept>
#include <string>
#include <system_error>

#include "cobot/common/dof.hpp"

// Single producer: record() is the only writer of the active buffer, its `written` count and the
// counters, so it updates them with plain load/store (no read-modify-write). A row is copied
// before its count is published (release), and the exporter reads a buffer only after acquiring
// swap_state == kSwapDone, which record() stores after the last write into that buffer.
namespace cobot::pipeline {
namespace {

constexpr std::uint32_t kSwapIdle = 0;       // no swap pending, no retired buffer held
constexpr std::uint32_t kSwapRequested = 1;  // the next record() swaps
constexpr std::uint32_t kSwapDone = 2;       // the other buffer is retired until released

void bump(std::atomic<std::uint64_t>& counter) {
  counter.store(counter.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
}

}  // namespace

template <int N>
HeapStorage<N>::HeapStorage(std::size_t capacity, int buffers)
    : capacity_(capacity), buffers_(buffers) {
  if (capacity == 0) {
    throw std::invalid_argument("recorder: capacity must be > 0");
  }
  if (buffers != 1 && buffers != 2) {
    throw std::invalid_argument("recorder: buffers must be 1 or 2, got " + std::to_string(buffers));
  }
  const std::size_t slots = capacity * static_cast<std::size_t>(buffers);
  control_ = std::make_unique<RecorderControl>();
  records_.resize(slots);
  sequence_.resize(slots);
}

template <int N>
RecorderMemory<N> HeapStorage<N>::memory() {
  RecorderMemory<N> m;
  m.control = control_.get();
  m.buffers[0] = records_.data();
  m.buffers[1] = buffers_ == 2 ? records_.data() + capacity_ : nullptr;
  m.sequence = sequence_.data();
  m.capacity = capacity_;
  m.buffer_count = buffers_;
  return m;
}

template <int N>
Recorder<N>::Recorder(RecorderMemory<N> memory, FullPolicy policy)
    : memory_(memory), policy_(policy) {
  const bool two = memory.buffer_count == 2;
  if (memory.control == nullptr || memory.buffers[0] == nullptr || memory.capacity == 0 ||
      (memory.buffer_count != 1 && !two) || (two && memory.buffers[1] == nullptr)) {
    throw std::invalid_argument("recorder: memory must hold 1 or 2 buffers of capacity > 0");
  }
  RecorderControl& c = *memory.control;
  c.active.store(0, std::memory_order_relaxed);
  c.written[0].store(0, std::memory_order_relaxed);
  c.written[1].store(0, std::memory_order_relaxed);
  c.dropped.store(0, std::memory_order_relaxed);
  c.overwritten.store(0, std::memory_order_relaxed);
  c.recorded.store(0, std::memory_order_relaxed);
  c.swap_state.store(kSwapIdle, std::memory_order_release);
}

template <int N>
void Recorder<N>::record(const LogRecord<N>& r) noexcept {
  RecorderControl& c = *memory_.control;
  if (c.swap_state.load(std::memory_order_acquire) == kSwapRequested) {
    const std::uint32_t next = 1U - c.active.load(std::memory_order_relaxed);
    c.written[next].store(0, std::memory_order_relaxed);
    c.active.store(next, std::memory_order_relaxed);
    c.swap_state.store(kSwapDone, std::memory_order_release);
  }
  const std::uint32_t a = c.active.load(std::memory_order_relaxed);
  const std::uint64_t n = c.written[a].load(std::memory_order_relaxed);
  const std::uint64_t cap = memory_.capacity;
  if (n >= cap) {
    if (policy_ == FullPolicy::kStopWhenFull) {
      bump(c.dropped);
      return;
    }
    bump(c.overwritten);
  }
  const std::size_t slot =
      n < cap ? static_cast<std::size_t>(n) : static_cast<std::size_t>(n % cap);
  memory_.buffers[a][slot] = r;
  const std::uint64_t count = c.recorded.load(std::memory_order_relaxed) + 1;
  if (memory_.sequence != nullptr) {
    memory_.sequence[a * memory_.capacity + slot] = count;  // 1-based record number
  }
  c.written[a].store(n + 1, std::memory_order_release);
  c.recorded.store(count, std::memory_order_release);
}

template <int N>
bool Recorder<N>::request_swap() noexcept {
  if (memory_.buffer_count != 2) {
    return false;
  }
  std::uint32_t expected = kSwapIdle;
  return memory_.control->swap_state.compare_exchange_strong(
      expected, kSwapRequested, std::memory_order_acq_rel, std::memory_order_acquire);
}

template <int N>
bool Recorder<N>::swap_done() const noexcept {
  return memory_.control->swap_state.load(std::memory_order_acquire) == kSwapDone;
}

template <int N>
RecordSpan<N> Recorder<N>::retired() const noexcept {
  if (!swap_done()) {
    return {};
  }
  return span_of(1U - memory_.control->active.load(std::memory_order_relaxed));
}

template <int N>
void Recorder<N>::release_retired() noexcept {
  std::uint32_t expected = kSwapDone;
  memory_.control->swap_state.compare_exchange_strong(
      expected, kSwapIdle, std::memory_order_acq_rel, std::memory_order_acquire);
}

template <int N>
RecordSpan<N> Recorder<N>::active() const noexcept {
  return span_of(memory_.control->active.load(std::memory_order_acquire));
}

template <int N>
void Recorder<N>::clear() noexcept {
  RecorderControl& c = *memory_.control;
  c.written[c.active.load(std::memory_order_relaxed)].store(0, std::memory_order_release);
}

template <int N>
RecordSpan<N> Recorder<N>::span_of(std::uint32_t buffer) const noexcept {
  const std::uint64_t n = memory_.control->written[buffer].load(std::memory_order_acquire);
  const std::size_t cap = memory_.capacity;
  const LogRecord<N>* rows = memory_.buffers[buffer];
  RecordSpan<N> s;
  s.first = rows;
  if (n <= cap) {
    s.first_size = static_cast<std::size_t>(n);
    return s;
  }
  // Wrapped: the oldest row is the next one to be overwritten.
  const auto next = static_cast<std::size_t>(n % cap);
  s.first = rows + next;
  s.first_size = cap - next;
  if (next > 0) {
    s.second = rows;
    s.second_size = next;
  }
  return s;
}

template <int N>
void write_csv(std::ostream& os, const RecordSpan<N>& rows, bool header) {
  if (header) {
    write_csv_header<N>(os);
  }
  for (std::size_t i = 0; i < rows.size(); ++i) {
    write_csv_row<N>(os, rows[i]);
  }
}

template <int N>
common::Status export_csv(const std::filesystem::path& csv,
                          const RecordSpan<N>& rows,
                          bool append) {
  std::error_code ec;
  const std::uintmax_t bytes = std::filesystem::file_size(csv, ec);
  const bool header = !append || ec || bytes == 0;
  std::ofstream os(csv, std::ios::binary | (append ? std::ios::app : std::ios::trunc));
  if (!os) {
    return common::Status::error("cannot open " + csv.string() + " for writing");
  }
  write_csv<N>(os, rows, header);
  os.flush();
  if (!os) {
    return common::Status::error("cannot write " + csv.string());
  }
  return common::Status::ok();
}

#define COBOT_INSTANTIATE(N)                                                             \
  template class HeapStorage<N>;                                                         \
  template class Recorder<N>;                                                            \
  template void write_csv<N>(std::ostream & os, const RecordSpan<N>& rows, bool header); \
  template common::Status export_csv<N>(                                                 \
      const std::filesystem::path& csv, const RecordSpan<N>& rows, bool append);
COBOT_INSTANTIATE_DOF(COBOT_INSTANTIATE)
#undef COBOT_INSTANTIATE

}  // namespace cobot::pipeline
