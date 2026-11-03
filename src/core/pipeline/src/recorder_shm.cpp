#include "cobot/pipeline/recorder_shm.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <filesystem>
#include <limits>
#include <new>
#include <optional>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <system_error>
#include <unistd.h>
#include <utility>
#include <vector>

#include "cobot/common/dof.hpp"

#include "recorder_state.hpp"

// Region layout (all offsets fixed by the header's capacity):
//   [ShmHeader | RecorderControl | buffer 0 | buffer 1 | sequence[2 * capacity]]
// Exporting reads the live buffer of n written rows as the records n - capacity + 2 .. n once it
// is full (the slot the next record() overwrites, possibly mid-write by a writer killed there, is
// left out) and 1 .. n before. A retired buffer is never written again, so it is read whole:
// n - capacity + 1 .. n. The records of one buffer are consecutive record numbers, so the first
// one is sequence(last) - n + 1; every record before it was in the other buffer, exported and
// released by the controller, or is counted by ShmHeader::exported. A live buffer is re-read after
// the copy (seqlock): rows the writer may have reached meanwhile are dropped, and a swap meanwhile
// restarts the export.
namespace cobot::pipeline {
namespace {

using recorder_state::kSwapDone;

constexpr char kMagic[8] = {'C', 'O', 'B', 'O', 'T', 'L', 'O', 'G'};
constexpr std::size_t kMaxName = 64;
constexpr std::size_t kAlign = 64;
constexpr std::size_t kMiB = std::size_t{1024} * 1024;
constexpr int kBuffers = 2;
constexpr int kExportAttempts = 16;  // a swap during the copy restarts the export
constexpr int kLockAttempts = 4;     // another exporter racing for a dead holder's mark
constexpr mode_t kMode = 0600;

struct ShmHeader {
  char magic[8];
  std::uint32_t layout_version;
  std::uint32_t dof;
  std::uint32_t record_size;
  std::uint32_t buffer_count;
  std::uint64_t fingerprint;
  std::uint64_t capacity;
  std::uint64_t region_size;
  std::atomic<std::uint64_t> exported;  // record number of the last exported row, 0 = none
  std::atomic<std::int32_t> exporter;   // pid of the export at work, 0 = none
};
static_assert(std::is_standard_layout_v<ShmHeader>, "header in shared memory");
static_assert(std::atomic<std::int32_t>::is_always_lock_free, "header in shared memory");

constexpr std::size_t align_up(std::size_t v) {
  return (v + kAlign - 1) / kAlign * kAlign;
}

template <int N>
struct Layout {
  static constexpr std::size_t kControl = align_up(sizeof(ShmHeader));
  static constexpr std::size_t kRecords = align_up(kControl + sizeof(RecorderControl));
  static constexpr std::size_t kSlotBytes =
      kBuffers * (sizeof(LogRecord<N>) + sizeof(std::uint64_t));
  static constexpr std::size_t kMaxCapacity =
      (std::numeric_limits<std::size_t>::max() / 2 - kRecords) / kSlotBytes;
  static std::size_t sequence(std::size_t capacity) {
    return kRecords + kBuffers * capacity * sizeof(LogRecord<N>);
  }
  static std::size_t size(std::size_t capacity) {
    return sequence(capacity) + kBuffers * capacity * sizeof(std::uint64_t);
  }
};

bool valid_name(const std::string& name) {
  if (name.empty() || name.size() > kMaxName) {
    return false;
  }
  for (const char c : name) {
    const bool ok =
        (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_';
    if (!ok) {
      return false;
    }
  }
  return true;
}

std::string region_of(const std::string& name) {
  return "/cobot_log_" + name;
}

std::string errno_text(int err) {
  return std::error_code(err, std::generic_category()).message();
}

std::string mismatch(const char* field, std::uint64_t region, std::uint64_t reader) {
  return std::string(field) + ": region " + std::to_string(region) + ", reader " +
         std::to_string(reader);
}

ShmHeader& header_of(unsigned char* base) {
  return *std::launder(reinterpret_cast<ShmHeader*>(base));
}

// Marks the region with this process's pid while an export runs. The mark of a dead process (or
// a stale one of this process: its exports are serialised by the host) is taken over.
class ExportLock {
 public:
  explicit ExportLock(ShmHeader& h) : h_(h) {
    const std::int32_t me = ::getpid();
    for (int attempt = 0; attempt < kLockAttempts && !owned_; ++attempt) {
      std::int32_t holder = 0;
      if (h_.exporter.compare_exchange_strong(holder, me, std::memory_order_acq_rel)) {
        owned_ = true;
      } else if (holder == me || (::kill(holder, 0) != 0 && errno == ESRCH)) {
        owned_ = h_.exporter.compare_exchange_strong(holder, me, std::memory_order_acq_rel);
      } else {
        holder_ = holder;
        return;
      }
    }
  }
  ExportLock(const ExportLock&) = delete;
  ExportLock& operator=(const ExportLock&) = delete;
  ExportLock(ExportLock&&) = delete;
  ExportLock& operator=(ExportLock&&) = delete;
  ~ExportLock() {
    if (owned_) {
      h_.exporter.store(0, std::memory_order_release);
    }
  }
  bool owned() const { return owned_; }
  std::int32_t holder() const { return holder_; }

 private:
  ShmHeader& h_;
  bool owned_ = false;
  std::int32_t holder_ = 0;
};

// Rows of one export, oldest first, with what was found missing.
template <int N>
struct Collected {
  std::vector<LogRecord<N>> rows;
  std::uint64_t last = 0;  // record number of the newest row
  std::uint64_t lost = 0;
};

// Appends the unexported rows of `buffer` to `out`; false when the buffer was reset meanwhile.
// `live`: the active buffer of a writer that may still be running (or died inside record()).
template <int N>
bool collect(const RecorderMemory<N>& m,
             std::uint32_t buffer,
             std::uint64_t exported,
             bool live,
             Collected<N>& out) {
  const std::uint64_t cap = m.capacity;
  const std::uint64_t n = m.control->written[buffer].load(std::memory_order_acquire);
  if (n == 0) {
    return true;
  }
  const std::uint64_t newest = m.sequence[buffer * cap + (n - 1) % cap];
  const std::uint64_t before = newest - n;  // record number before this buffer's first row
  const std::uint64_t floor = std::max(exported, before);
  // Buffer-local index, 1-based. A full live buffer's slot (n % cap) is the next one written.
  const std::uint64_t first = live ? (n >= cap ? n - cap + 2 : 1) : (n > cap ? n - cap + 1 : 1);
  const std::uint64_t from = std::max(first, floor - before + 1);
  const std::size_t start = out.rows.size();
  for (std::uint64_t j = from; j <= n; ++j) {
    out.rows.push_back(m.buffers[buffer][static_cast<std::size_t>((j - 1) % cap)]);
  }
  // Seqlock check: rows the writer may have reached during the copy are dropped.
  std::atomic_thread_fence(std::memory_order_acquire);
  const std::uint64_t again = m.control->written[buffer].load(std::memory_order_relaxed);
  if (again < n) {
    return false;
  }
  if (live && again >= cap && again - cap + 2 > from) {
    const auto drop = static_cast<std::size_t>(
        std::min<std::uint64_t>(again - cap + 2 - from, out.rows.size() - start));
    out.rows.erase(out.rows.begin() + static_cast<std::ptrdiff_t>(start),
                   out.rows.begin() + static_cast<std::ptrdiff_t>(start + drop));
  }
  if (newest > floor) {
    out.lost += (newest - floor) - (out.rows.size() - start);
    out.last = newest;
  }
  return true;
}

std::string stamp_now() {
  const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm tm{};
  localtime_r(&now, &tm);
  char buf[32] = {};
  std::strftime(buf, sizeof(buf), "%Y%m%d-%H%M%S", &tm);
  return buf;
}

}  // namespace

template <int N>
ShmStorage<N>::ShmStorage(unsigned char* base, std::size_t size, std::size_t capacity)
    : base_(base), size_(size), capacity_(capacity) {}

template <int N>
ShmStorage<N>::ShmStorage(ShmStorage&& other) noexcept
    : base_(std::exchange(other.base_, nullptr)),
      size_(std::exchange(other.size_, 0)),
      capacity_(std::exchange(other.capacity_, 0)),
      lock_error_(std::move(other.lock_error_)) {}

template <int N>
ShmStorage<N>::~ShmStorage() {
  if (base_ != nullptr) {
    ::munmap(base_, size_);
  }
}

template <int N>
common::Result<ShmStorage<N>> ShmStorage<N>::create(const std::string& name,
                                                    std::size_t capacity,
                                                    bool lock_memory) {
  using R = common::Result<ShmStorage<N>>;
  if (!valid_name(name)) {
    return R::error("black-box log: bad region name '" + name + "' (want [A-Za-z0-9_]{1,64})");
  }
  if (capacity == 0 || capacity > Layout<N>::kMaxCapacity) {
    return R::error("black-box log: capacity " + std::to_string(capacity) + " out of range");
  }
  const std::string region = region_of(name);
  const std::size_t size = Layout<N>::size(capacity);
  const int fd = ::shm_open(region.c_str(), O_CREAT | O_EXCL | O_RDWR, kMode);
  if (fd < 0) {
    return R::error("black-box log: cannot create " + region + ": " + errno_text(errno));
  }
  const int err = ::posix_fallocate(fd, 0, static_cast<off_t>(size));
  void* p =
      err == 0 ? ::mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0) : MAP_FAILED;
  const int map_err = errno;
  ::close(fd);
  if (p == MAP_FAILED) {
    ::shm_unlink(region.c_str());
    if (err == ENOSPC) {
      return R::error("black-box log " + region + ": /dev/shm too small: need " +
                      std::to_string((size + kMiB - 1) / kMiB) + " MB");
    }
    return R::error("black-box log: cannot reserve " + region + ": " +
                    errno_text(err != 0 ? err : map_err));
  }
  auto* base = static_cast<unsigned char*>(p);
  auto* h = new (base) ShmHeader{};
  h->layout_version = kShmLogLayoutVersion;
  h->dof = static_cast<std::uint32_t>(N);
  h->record_size = static_cast<std::uint32_t>(sizeof(LogRecord<N>));
  h->buffer_count = kBuffers;
  h->fingerprint = log_record_fingerprint<N>();
  h->capacity = capacity;
  h->region_size = size;
  new (base + Layout<N>::kControl) RecorderControl{};
  std::atomic_thread_fence(std::memory_order_release);
  std::memcpy(h->magic, kMagic, sizeof(kMagic));  // last: a complete header
  ShmStorage storage(base, size, capacity);
  if (lock_memory && ::mlock(base, size) != 0) {
    storage.lock_error_ = "mlock of " + region + " (" + std::to_string(size / kMiB) +
                          " MB) failed: " + errno_text(errno) + "; it may be swapped out";
  }
  return R::ok(std::move(storage));
}

template <int N>
common::Result<ShmStorage<N>> ShmStorage<N>::open_existing(const std::string& name) {
  using R = common::Result<ShmStorage<N>>;
  if (!valid_name(name)) {
    return R::error("black-box log: bad region name '" + name + "'");
  }
  const std::string region = region_of(name);
  const int fd = ::shm_open(region.c_str(), O_RDWR, 0);
  if (fd < 0) {
    return R::error("black-box log: no region " + region + ": " + errno_text(errno));
  }
  struct stat st {};
  const bool stat_ok = ::fstat(fd, &st) == 0;
  const auto size = stat_ok ? static_cast<std::size_t>(st.st_size) : std::size_t{0};
  void* p = size >= sizeof(ShmHeader)
                ? ::mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)
                : MAP_FAILED;
  ::close(fd);
  if (p == MAP_FAILED) {
    return R::error("black-box log " + region + ": region too small or unreadable (" +
                    std::to_string(size) + " bytes)");
  }
  ShmStorage storage(static_cast<unsigned char*>(p), size, 0);
  const ShmHeader& h = header_of(storage.base_);
  std::string why;
  if (std::memcmp(h.magic, kMagic, sizeof(kMagic)) != 0) {
    why = "bad magic (not a black-box log region)";
  } else if (h.layout_version != kShmLogLayoutVersion) {
    why = mismatch("layout version", h.layout_version, kShmLogLayoutVersion);
  } else if (h.dof != static_cast<std::uint32_t>(N)) {
    why = mismatch("dof", h.dof, N);
  } else if (h.record_size != sizeof(LogRecord<N>)) {
    why = mismatch("record size", h.record_size, sizeof(LogRecord<N>));
  } else if (h.fingerprint != log_record_fingerprint<N>()) {
    why = mismatch("fingerprint", h.fingerprint, log_record_fingerprint<N>());
  } else if (h.buffer_count != kBuffers || h.capacity == 0 ||
             h.capacity > Layout<N>::kMaxCapacity || h.region_size != size ||
             Layout<N>::size(static_cast<std::size_t>(h.capacity)) != size) {
    why = "capacity " + std::to_string(h.capacity) + " does not match the region size: region " +
          std::to_string(size) + " bytes, header " + std::to_string(h.region_size);
  }
  if (!why.empty()) {
    return R::error("black-box log " + region + ": " + why);
  }
  storage.capacity_ = static_cast<std::size_t>(h.capacity);
  return R::ok(std::move(storage));
}

template <int N>
common::Status ShmStorage<N>::remove(const std::string& name) {
  if (!valid_name(name)) {
    return common::Status::error("black-box log: bad region name '" + name + "'");
  }
  if (::shm_unlink(region_of(name).c_str()) != 0 && errno != ENOENT) {
    return common::Status::error("black-box log: cannot remove " + region_of(name) + ": " +
                                 errno_text(errno));
  }
  return common::Status::ok();
}

template <int N>
bool ShmStorage<N>::exists(const std::string& name) {
  if (!valid_name(name)) {
    return false;
  }
  const int fd = ::shm_open(region_of(name).c_str(), O_RDONLY, 0);
  if (fd < 0) {
    return false;
  }
  ::close(fd);
  return true;
}

template <int N>
RecorderMemory<N> ShmStorage<N>::memory() {
  RecorderMemory<N> m;
  if (base_ == nullptr) {
    return m;
  }
  auto* records = std::launder(reinterpret_cast<LogRecord<N>*>(base_ + Layout<N>::kRecords));
  m.control = std::launder(reinterpret_cast<RecorderControl*>(base_ + Layout<N>::kControl));
  m.buffers[0] = records;
  m.buffers[1] = records + capacity_;
  m.sequence =
      std::launder(reinterpret_cast<std::uint64_t*>(base_ + Layout<N>::sequence(capacity_)));
  m.capacity = capacity_;
  m.buffer_count = kBuffers;
  return m;
}

template <int N>
const std::string& ShmStorage<N>::lock_error() const {
  return lock_error_;
}

template <int N>
common::Result<DumpResult> ShmStorage<N>::export_unexported(const std::filesystem::path& csv,
                                                            ShmExport scope,
                                                            bool keep) {
  using R = common::Result<DumpResult>;
  if (base_ == nullptr) {
    return R::error("black-box log: no region mapped");
  }
  ShmHeader& h = header_of(base_);
  const ExportLock lock(h);
  if (!lock.owned()) {
    return R::error("black-box log: another export is running (pid " +
                    std::to_string(lock.holder()) + ")");
  }
  const RecorderMemory<N> m = memory();
  const RecorderControl& c = *m.control;
  Collected<N> got;
  bool consistent = false;
  for (int attempt = 0; attempt < kExportAttempts && !consistent; ++attempt) {
    got = Collected<N>{};
    const std::uint64_t exported = h.exported.load(std::memory_order_acquire);
    const bool done = c.swap_state.load(std::memory_order_acquire) == kSwapDone;
    const std::uint32_t active = c.active.load(std::memory_order_acquire);
    // An empty active buffer next to a full one: the writer died inside the swap, before it
    // acknowledged it (every swap is followed by a record into the new buffer).
    const bool retired = done || c.written[active].load(std::memory_order_acquire) == 0;
    bool ok = !retired || collect<N>(m, 1U - active, exported, false, got);
    if (scope == ShmExport::kAll) {
      ok = ok && collect<N>(m, active, std::max(exported, got.last), true, got);
    }
    // A swap meanwhile changes `active`; a second one needs a release, which waits for this
    // export (the controller releases only after exporting).
    consistent = ok && c.active.load(std::memory_order_acquire) == active;
  }
  if (!consistent) {
    return R::error("black-box log: the buffers kept swapping during the export, try again");
  }
  DumpResult result;
  result.csv = csv;
  if (got.rows.empty()) {
    return R::ok(result);
  }
  RecordSpan<N> span;
  span.first = got.rows.data();
  span.first_size = got.rows.size();
  const common::Status written = export_csv<N>(csv, span, true);
  if (!written) {
    return R::error(written.reason());
  }
  if (!keep) {
    h.exported.store(got.last, std::memory_order_release);
  }
  result.rows = got.rows.size();
  result.lost = got.lost;
  return R::ok(result);
}

template <int N>
common::Result<DumpResult> dump_shm_log(const std::string& name,
                                        const std::filesystem::path& csv,
                                        bool keep) {
  common::Result<ShmStorage<N>> storage = ShmStorage<N>::open_existing(name);
  if (!storage) {
    return common::Result<DumpResult>::error(storage.reason());
  }
  return storage.value().export_unexported(csv, ShmExport::kAll, keep);
}

template <int N>
common::Result<std::optional<std::filesystem::path>> recover_shm_log(
    const std::string& name, const std::filesystem::path& log_path) {
  using R = common::Result<std::optional<std::filesystem::path>>;
  if (!ShmStorage<N>::exists(name)) {
    return R::ok(std::nullopt);
  }
  std::filesystem::path out = log_path;
  out += ".recovered-" + stamp_now() + ".csv";
  std::size_t rows = 0;
  {
    common::Result<DumpResult> dumped = dump_shm_log<N>(name, out, false);
    if (!dumped) {
      return R::error(dumped.reason());
    }
    rows = dumped.value().rows;
  }
  const common::Status removed = ShmStorage<N>::remove(name);
  if (!removed) {
    return R::error(removed.reason());
  }
  return R::ok(rows > 0 ? std::optional<std::filesystem::path>(out) : std::nullopt);
}

#define COBOT_INSTANTIATE(N)                                                        \
  template class ShmStorage<N>;                                                     \
  template common::Result<DumpResult> dump_shm_log<N>(                              \
      const std::string& name, const std::filesystem::path& csv, bool keep);        \
  template common::Result<std::optional<std::filesystem::path>> recover_shm_log<N>( \
      const std::string& name, const std::filesystem::path& log_path);
COBOT_INSTANTIATE_DOF(COBOT_INSTANTIATE)
#undef COBOT_INSTANTIATE

}  // namespace cobot::pipeline
