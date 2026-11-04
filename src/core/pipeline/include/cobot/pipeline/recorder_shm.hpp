#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <initializer_list>
#include <optional>
#include <string>
#include <type_traits>

#include "cobot/common/result.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/pipeline/recorder.hpp"

// Black-box log (feature blackbox-log, design docs/design/2026-11-02-cobot-studio-design.md 6.1,
// ADR-020), Linux only: the two Recorder buffers live in the POSIX shared-memory region
// "/cobot_log_<name>", which outlives a crash of its writer. The region holds a header (magic,
// layout version, N, record size, fingerprint, capacity, region size, the number of the last
// exported record and the pid of the exporter at work), the RecorderControl, two buffers and the
// per-slot record numbers. A record is fully copied before record() publishes it (release) and
// readers acquire, so a row copied halfway is never exported.
//
// Exactly once: every record carries its record number (1, 2, ...). The header keeps the number
// of the last exported record; the rows of a buffer that the controller exported and released
// (Recorder::release_retired) count as exported too. One exporter at a time: the export marks
// the region with its pid, a dead holder's mark is taken over. One Recorder per region (the
// Recorder restarts the record numbers).
namespace cobot::pipeline {

inline constexpr std::uint32_t kShmLogLayoutVersion = 2;  // bump on any LogRecord or header change

static_assert(std::is_standard_layout_v<LogRecord<1>>, "ADR-020: LogRecord in shared memory");

namespace shm_detail {
constexpr std::uint64_t kFnvBasis = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;
constexpr int kBitsPerByte = 8;
constexpr std::uint64_t kByteMask = 0xFFU;

constexpr std::uint64_t fnv1a(std::initializer_list<std::uint64_t> values) {
  std::uint64_t h = kFnvBasis;
  for (const std::uint64_t v : values) {
    for (int i = 0; i < kBitsPerByte; ++i) {
      h = (h ^ ((v >> (kBitsPerByte * i)) & kByteMask)) * kFnvPrime;
    }
  }
  return h;
}
}  // namespace shm_detail

// Constexpr fingerprint of LogRecord<N>: hash of sizeof/offsetof of every field, N, sizeof(Pose).
template <int N>
constexpr std::uint64_t log_record_fingerprint() {
  using R = LogRecord<N>;
  static_assert(std::is_standard_layout_v<R>, "ADR-020: LogRecord in shared memory");
  return shm_detail::fnv1a({static_cast<std::uint64_t>(N),
                            sizeof(R),
                            alignof(R),
                            sizeof(Pose),
                            offsetof(R, t),
                            offsetof(R, mode),
                            offsetof(R, q),
                            offsetof(R, qd),
                            offsetof(R, q_ref),
                            offsetof(R, qd_ref),
                            offsetof(R, tau),
                            offsetof(R, T_b_e),
                            offsetof(R, T_b_d),
                            offsetof(R, task_error),
                            offsetof(R, step_time_us),
                            offsetof(R, hold_reason),
                            offsetof(R, qdd_ref),
                            offsetof(R, q_raw),
                            offsetof(R, q_true),
                            offsetof(R, qd_true),
                            offsetof(R, qdd_true),
                            offsetof(R, q_motor_true),
                            offsetof(R, tau_motor),
                            offsetof(R, tau_friction),
                            offsetof(R, cmd_mode)});
}

struct DumpResult {
  std::filesystem::path csv;
  std::size_t rows = 0;
  std::uint64_t lost = 0;  // recorded but overwritten before any export
};

// Which rows ShmStorage::export_unexported looks at.
enum class ShmExport {
  kAll,      // the retired buffer (if a swap is done) then the active one
  kRetired,  // only the retired buffer (exporter thread of a running controller, after a swap)
};

template <int N>
class ShmStorage {  // move-only; the destructor unmaps and never unlinks
 public:
  // Region "/cobot_log_<name>", name matching [A-Za-z0-9_]{1,64}. Header + RecorderControl + two
  // buffers + per-slot sequence. Size reserved with posix_fallocate: ENOSPC -> error
  // "/dev/shm too small: need <n> MB" (never a SIGBUS later). Fails if the region exists.
  // lock_memory: mlock the mapping; failure -> warning text in `lock_error()`, not an error.
  static common::Result<ShmStorage> create(const std::string& name,
                                           std::size_t capacity,
                                           bool lock_memory);
  // Read-write mapping of an existing region after checking, in this order, magic, layout
  // version, N, record size, fingerprint, capacity/size consistency. A mismatch is an error naming
  // the field and both values ("dof: region 6, reader 7").
  static common::Result<ShmStorage> open_existing(const std::string& name);
  static common::Status remove(const std::string& name);  // shm_unlink; missing is ok
  static bool exists(const std::string& name);

  ShmStorage(ShmStorage&& other) noexcept;
  ShmStorage& operator=(ShmStorage&& other) = delete;  // hold it in a std::unique_ptr
  ShmStorage(const ShmStorage&) = delete;
  ShmStorage& operator=(const ShmStorage&) = delete;
  ~ShmStorage();

  RecorderMemory<N> memory();
  const std::string& lock_error() const;

  // Appends every row of `scope` recorded and not yet exported, oldest first, to `csv` (header if
  // new; the file is not touched when there is no row) and marks them exported unless `keep`.
  // Not on the control thread: it allocates and does I/O.
  common::Result<DumpResult> export_unexported(const std::filesystem::path& csv,
                                               ShmExport scope,
                                               bool keep);

 private:
  ShmStorage(unsigned char* base, std::size_t size, std::size_t capacity);

  unsigned char* base_ = nullptr;
  std::size_t size_ = 0;
  std::size_t capacity_ = 0;
  std::string lock_error_;
};

// Every record recorded and not yet exported, oldest first, appended to `csv` (header if new).
// A slot that may be mid-write is never exported. Marks them exported unless `keep`.
template <int N>
common::Result<DumpResult> dump_shm_log(const std::string& name,
                                        const std::filesystem::path& csv,
                                        bool keep);

// At controller start-up, before creating the new region: when `name` exists and holds unexported
// rows, dump them to "<log_path>.recovered-<YYYYmmdd-HHMMSS>.csv" and return that path; nullopt
// when there was nothing to recover. Removes the old region in both cases. Layout mismatch -> error
// (the caller logs it, removes the region and continues).
template <int N>
common::Result<std::optional<std::filesystem::path>> recover_shm_log(
    const std::string& name, const std::filesystem::path& log_path);

}  // namespace cobot::pipeline
