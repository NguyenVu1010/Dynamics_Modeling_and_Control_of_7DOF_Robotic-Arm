#pragma once

#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/log_record.hpp"
#include "cobot/pipeline/recorder.hpp"
#include "cobot/pipeline/recorder_shm.hpp"

// Shared helpers of the black-box log tests (feature blackbox-log, cobot_studio design 6.1,
// ADR-020): unique region names removed on every exit path, a raw read-write mapping of a region
// to corrupt its header, records tagged by their stamp, and a small Run CSV reader.
//
// Region names: ShmStorage<N>::create(name, ...) maps "/cobot_log_<name>"; the tests pass
// name = "test_<pid>_<n>", so every region a test makes is "/cobot_log_test_<pid>_<n>".
namespace cobot::pipeline::shm_test {

namespace fs = std::filesystem;

constexpr int kDof = 7;
using Rec = LogRecord<kDof>;
using Storage = ShmStorage<kDof>;

// "/cobot_log_" + name: the POSIX shared-memory name of a region (contract of ShmStorage).
inline std::string region_of(const std::string& name) {
  return "/cobot_log_" + name;
}

// test_<pid>_<n>, n counting up within this process.
inline std::string unique_name() {
  static int counter = 0;
  return "test_" + std::to_string(static_cast<long>(::getpid())) + "_" + std::to_string(counter++);
}

// Owns one unique region name: unlinks the region (if any) when built and when destroyed, so a
// failing test never leaves a region behind. Uses shm_unlink directly, independent of
// ShmStorage::remove.
class RegionGuard {
 public:
  RegionGuard() : name_(unique_name()) { unlink(); }
  explicit RegionGuard(std::string name) : name_(std::move(name)) { unlink(); }
  RegionGuard(const RegionGuard&) = delete;
  RegionGuard& operator=(const RegionGuard&) = delete;
  RegionGuard(RegionGuard&&) = delete;
  RegionGuard& operator=(RegionGuard&&) = delete;
  ~RegionGuard() { unlink(); }

  const std::string& name() const { return name_; }
  std::string region() const { return region_of(name_); }
  // The region as a file under /dev/shm (Linux maps POSIX shared memory there).
  fs::path dev_shm_path() const { return fs::path("/dev/shm") / ("cobot_log_" + name_); }
  bool present() const { return fs::exists(dev_shm_path()); }

 private:
  void unlink() const { static_cast<void>(::shm_unlink(region().c_str())); }
  std::string name_;
};

// Raw read-write mapping of a whole region (test only), to read and corrupt header bytes.
class RawRegion {
 public:
  explicit RawRegion(const std::string& name) {
    fd_ = ::shm_open(region_of(name).c_str(), O_RDWR, 0);
    if (fd_ < 0) {
      return;
    }
    struct stat st {};
    if (::fstat(fd_, &st) != 0 || st.st_size <= 0) {
      return;
    }
    size_ = static_cast<std::size_t>(st.st_size);
    void* p = ::mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (p != MAP_FAILED) {
      bytes_ = static_cast<unsigned char*>(p);
    }
  }
  RawRegion(const RawRegion&) = delete;
  RawRegion& operator=(const RawRegion&) = delete;
  RawRegion(RawRegion&&) = delete;
  RawRegion& operator=(RawRegion&&) = delete;
  ~RawRegion() {
    if (bytes_ != nullptr) {
      ::munmap(bytes_, size_);
    }
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  bool ok() const { return bytes_ != nullptr; }
  std::size_t size() const { return size_; }
  unsigned char* bytes() { return bytes_; }

  // First offset in [from, to) that is a multiple of `align` and holds the bytes of `value`.
  template <typename T>
  std::optional<std::size_t> find(const T& value,
                                  std::size_t from,
                                  std::size_t to,
                                  std::size_t align) const {
    for (std::size_t off = from; off + sizeof(T) <= to && off + sizeof(T) <= size_; off += align) {
      if (std::memcmp(bytes_ + off, &value, sizeof(T)) == 0) {
        return off;
      }
    }
    return std::nullopt;
  }

  template <typename T>
  void write(std::size_t off, const T& value) {
    std::memcpy(bytes_ + off, &value, sizeof(T));
  }

  // Shrinks (or grows) the region, as a damaged or foreign region would be.
  bool truncate(std::size_t bytes) const {
    return ::ftruncate(fd_, static_cast<off_t>(bytes)) == 0;
  }

 private:
  int fd_ = -1;
  std::size_t size_ = 0;
  unsigned char* bytes_ = nullptr;
};

// A record whose stamp is `t` ns; q, tau and the task error carry t too, so a row copied from
// the wrong slot or only partly copied shows in more than one field. q1 = t exactly.
inline Rec make_record(std::int64_t t) {
  Rec r;
  r.t.ns = t;
  r.mode = (t % 2 == 0) ? LoopMode::kTracking : LoopMode::kHolding;
  const auto v = static_cast<double>(t);
  for (int i = 0; i < kDof; ++i) {
    r.q(i) = v + 0.125 * i;
    r.qd(i) = -v - 0.25 * i;
    r.q_ref(i) = 2.0 * v + i;
    r.qd_ref(i) = 0.5 * v - i;
    r.tau(i) = 3.0 * v + 0.0625 * i;
  }
  r.T_b_e.p = common::Vec3(v, 1.0, -v);
  r.T_b_d.p = common::Vec3(-v, 2.0, v);
  r.task_error.setConstant(v * 1e-3);
  r.step_time_us = v + 0.5;
  r.hold_reason = (t % 3 == 0) ? HoldReason::kNone : HoldReason::kOverspeed;
  return r;
}

// t = first, first + 1, ..., last.
inline std::vector<std::int64_t> range(std::int64_t first, std::int64_t last) {
  std::vector<std::int64_t> out;
  for (std::int64_t t = first; t <= last; ++t) {
    out.push_back(t);
  }
  return out;
}

inline std::string read_file(const fs::path& p) {
  const std::ifstream in(p, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

inline std::vector<std::string> split(const std::string& text, char sep) {
  std::vector<std::string> out;
  std::string item;
  std::istringstream in(text);
  while (std::getline(in, item, sep)) {
    out.push_back(item);
  }
  return out;
}

// Number of columns of the Run CSV header for kDof (from write_csv_header, so a contract
// change of the columns needs no edit here).
inline std::size_t header_columns() {
  std::ostringstream os;
  write_csv_header<kDof>(os);
  std::string line = os.str();
  while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
    line.pop_back();
  }
  return split(line, ',').size();
}

// A Run CSV read back: the header line count and each data row split into fields.
struct CsvFile {
  int header_lines = 0;  // lines starting with "t_ns"
  std::vector<std::vector<std::string>> rows;
};

inline CsvFile read_csv(const fs::path& p) {
  CsvFile f;
  for (const std::string& line : split(read_file(p), '\n')) {
    if (line.empty()) {
      continue;
    }
    if (line.rfind("t_ns", 0) == 0) {
      ++f.header_lines;
      continue;
    }
    f.rows.push_back(split(line, ','));
  }
  return f;
}

// t_ns (first column) of every row.
inline std::vector<std::int64_t> csv_stamps(const CsvFile& f) {
  std::vector<std::int64_t> out;
  out.reserve(f.rows.size());
  for (const auto& row : f.rows) {
    out.push_back(row.empty() ? -1 : std::strtoll(row[0].c_str(), nullptr, 10));
  }
  return out;
}

inline double field_double(const std::vector<std::string>& row, std::size_t col) {
  return col < row.size() ? std::strtod(row[col].c_str(), nullptr) : std::nan("");
}

// Every row has the header's column count, finite q1..qN (columns 2 .. 2 + N - 1) and, for rows
// made by make_record, q1 equal to its stamp.
inline void expect_rows_well_formed(const CsvFile& f, bool q1_is_stamp) {
  const std::size_t cols = header_columns();
  for (std::size_t k = 0; k < f.rows.size(); ++k) {
    const auto& row = f.rows[k];
    ASSERT_EQ(row.size(), cols) << "row " << k;
    for (int j = 0; j < kDof; ++j) {
      ASSERT_TRUE(std::isfinite(field_double(row, 2 + static_cast<std::size_t>(j))))
          << "row " << k << " q" << (j + 1) << " = " << row[2 + static_cast<std::size_t>(j)];
    }
    if (q1_is_stamp) {
      ASSERT_EQ(field_double(row, 2), std::strtod(row[0].c_str(), nullptr)) << "row " << k;
    }
  }
}

// A fresh directory for one test's output files.
inline fs::path out_dir(const std::string& test) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "pipeline_recorder_shm" / test;
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

// `file` is prefix + "YYYYmmdd-HHMMSS" (digits) + suffix. Written out instead of std::regex,
// which trips -Wmaybe-uninitialized in GCC 13 at -O3 with the sanitizers.
inline bool is_stamped_name(const std::string& file,
                            const std::string& prefix,
                            const std::string& suffix) {
  const std::string pattern = "dddddddd-dddddd";
  if (file.size() != prefix.size() + pattern.size() + suffix.size() ||
      file.compare(0, prefix.size(), prefix) != 0 ||
      file.compare(file.size() - suffix.size(), suffix.size(), suffix) != 0) {
    return false;
  }
  for (std::size_t i = 0; i < pattern.size(); ++i) {
    const char c = file[prefix.size() + i];
    const bool ok = pattern[i] == 'd' ? (c >= '0' && c <= '9') : c == pattern[i];
    if (!ok) {
      return false;
    }
  }
  return true;
}

}  // namespace cobot::pipeline::shm_test
