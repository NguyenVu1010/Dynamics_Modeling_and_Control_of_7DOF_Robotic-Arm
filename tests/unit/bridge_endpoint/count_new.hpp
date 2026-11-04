#pragma once

#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <new>

// Global operator new counter (feature shm-bridge): include this header in exactly ONE source file
// of a test binary. While counting() is on, every global operator new in the binary is counted, so
// an allocation inside the bridge or bench libraries (compiled without EIGEN_RUNTIME_NO_MALLOC) is
// caught too. Every form is replaced (plain, array, sized deletes and the std::nothrow_t forms
// GoogleTest uses) so allocations and deallocations stay paired under ASan. Atomic, because the
// bench runs a reader thread. Pattern of tests/unit/pipeline_recorder/test_recorder_no_alloc.cpp.
namespace cobot::count_new {

inline std::atomic<bool>& counting() {
  static std::atomic<bool> on{false};
  return on;
}

inline std::atomic<std::size_t>& calls() {
  static std::atomic<std::size_t> n{0};
  return n;
}

// Counts the allocations made while it lives.
class Scope {
 public:
  Scope() {
    calls().store(0);
    counting().store(true);
  }
  Scope(const Scope&) = delete;
  Scope& operator=(const Scope&) = delete;
  Scope(Scope&&) = delete;
  Scope& operator=(Scope&&) = delete;
  ~Scope() { counting().store(false); }

  static std::size_t count() { return calls().load(); }
};

}  // namespace cobot::count_new

void* operator new(std::size_t size) {
  if (cobot::count_new::counting().load(std::memory_order_relaxed)) {
    cobot::count_new::calls().fetch_add(1, std::memory_order_relaxed);
  }
  if (void* p = std::malloc(size == 0 ? 1 : size)) {
    return p;
  }
  throw std::bad_alloc();
}
void* operator new[](std::size_t size) {
  return ::operator new(size);
}
void operator delete(void* p) noexcept {
  std::free(p);
}
void operator delete[](void* p) noexcept {
  std::free(p);
}
void operator delete(void* p, std::size_t /*size*/) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::size_t /*size*/) noexcept {
  std::free(p);
}
void* operator new(std::size_t size, const std::nothrow_t& /*tag*/) noexcept {
  try {
    return ::operator new(size);
  } catch (...) {
    return nullptr;
  }
}
void* operator new[](std::size_t size, const std::nothrow_t& /*tag*/) noexcept {
  try {
    return ::operator new[](size);
  } catch (...) {
    return nullptr;
  }
}
void operator delete(void* p, const std::nothrow_t& /*tag*/) noexcept {
  ::operator delete(p);
}
void operator delete[](void* p, const std::nothrow_t& /*tag*/) noexcept {
  ::operator delete[](p);
}
