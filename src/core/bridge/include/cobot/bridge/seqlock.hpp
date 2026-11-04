#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <type_traits>

// One-writer seqlock slot (feature shm-bridge, ADR-022 "Các vòng"), placed in the bridge region
// and used in-process by the controller. The writer never waits; a reader that sees an odd
// sequence or a change during its copy discards the copy and never spins.
//
// The payload copy is a formal data race under the C++ memory model (the standard seqlock pattern
// with fences and std::memcpy of a trivially copyable type); a torn copy is always discarded.
namespace cobot::bridge {

template <typename T>
struct SeqSlot {
  static_assert(std::is_trivially_copyable_v<T>, "seqlock payload is copied with memcpy");

  std::atomic<std::uint64_t> seq{0};  // even: stable; odd: being written
  T value;

  // Single writer: seq odd, release fence, memcpy, seq even (release). The write starts from the
  // next even base, so a slot left odd by a writer that died mid-write recovers on the next write.
  void write(const T& v) noexcept {
    const std::uint64_t s = next_base();
    seq.store(s + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    std::memcpy(&value, &v, sizeof(T));
    seq.store(s + 2, std::memory_order_release);
  }

  // Even sequence the next write starts from; that write ends at next_base() + 2.
  std::uint64_t next_base() const noexcept {
    return (seq.load(std::memory_order_relaxed) + 1U) & ~std::uint64_t{1};
  }

  // False if odd or changed during the copy; never spins.
  bool try_read(T& out) const noexcept {
    const std::uint64_t s = seq.load(std::memory_order_acquire);
    return (s & 1U) == 0 && try_read_at(out, s);
  }

  // As try_read, but true only when the slot holds the write whose sequence is `expected` (even):
  // the state ring uses it to tell sample k from a later sample in the same slot.
  bool try_read_at(T& out, std::uint64_t expected) const noexcept {
    if (seq.load(std::memory_order_acquire) != expected) {
      return false;
    }
    std::memcpy(&out, &value, sizeof(T));
    std::atomic_thread_fence(std::memory_order_acquire);
    return seq.load(std::memory_order_relaxed) == expected;
  }
};

}  // namespace cobot::bridge
