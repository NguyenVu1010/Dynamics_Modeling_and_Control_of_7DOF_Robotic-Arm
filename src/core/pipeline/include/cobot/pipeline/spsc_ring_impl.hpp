#pragma once

#include <atomic>
#include <cstddef>
#include <memory>

// Member definitions of SpscRing (generic in T, so they live in a header). Included at the
// end of spsc_ring.hpp. head_ is written only by the producer and tail_ only by the
// consumer; each publishes its index with a release store that the other side reads with
// an acquire load, so a slot's contents are visible before its index.
namespace cobot::pipeline {

template <typename T, std::size_t Capacity>
SpscRing<T, Capacity>::SpscRing() : buffer_(std::make_unique<T[]>(Capacity)) {}

template <typename T, std::size_t Capacity>
bool SpscRing<T, Capacity>::push(const T& item) noexcept {
  constexpr std::size_t kMask = Capacity - 1;
  const std::size_t head = head_.load(std::memory_order_relaxed);
  const std::size_t next = (head + 1) & kMask;
  if (next == tail_.load(std::memory_order_acquire)) {
    dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  buffer_[head] = item;
  head_.store(next, std::memory_order_release);
  return true;
}

template <typename T, std::size_t Capacity>
bool SpscRing<T, Capacity>::pop(T& out) noexcept {
  constexpr std::size_t kMask = Capacity - 1;
  const std::size_t tail = tail_.load(std::memory_order_relaxed);
  if (tail == head_.load(std::memory_order_acquire)) {
    return false;
  }
  out = buffer_[tail];
  tail_.store((tail + 1) & kMask, std::memory_order_release);
  return true;
}

}  // namespace cobot::pipeline
