#pragma once

#include <atomic>
#include <cstddef>
#include <memory>

// Single-producer single-consumer ring buffer (feature `pipeline-config`). The control
// thread pushes, the host's writer thread pops. Lock-free, no allocation, never blocks.
// Capacity must be a power of two; one slot is kept empty, so it holds Capacity - 1 items.
// The storage is allocated once in the constructor (records are large; keep it off the stack).
namespace cobot::pipeline {

template <typename T, std::size_t Capacity>
class SpscRing {
  static_assert(Capacity >= 2 && (Capacity & (Capacity - 1)) == 0, "Capacity: power of two");

 public:
  SpscRing();

  // false when full (the item is dropped and counted in dropped()).
  bool push(const T& item) noexcept;
  // false when empty.
  bool pop(T& out) noexcept;
  std::size_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }

 private:
  std::unique_ptr<T[]> buffer_;
  std::atomic<std::size_t> head_{0};  // next write, owned by the producer
  std::atomic<std::size_t> tail_{0};  // next read, owned by the consumer
  std::atomic<std::size_t> dropped_{0};
};

}  // namespace cobot::pipeline

#include "cobot/pipeline/spsc_ring_impl.hpp"
