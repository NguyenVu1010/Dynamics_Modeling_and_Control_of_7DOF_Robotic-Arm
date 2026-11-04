#pragma once

#include <algorithm>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <optional>
#include <poll.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <utility>

#include "cobot/bridge/endpoint.hpp"
#include "cobot/bridge/wire.hpp"

// Shared helpers of the studio bridge tests (feature shm-bridge, cobot_studio design 6.5,
// ADR-022): unique region names removed on every exit path, a raw read-write mapping of a region
// to corrupt its header, and state samples whose every field is a function of the step, so a
// sample mixed from two writes shows in more than one field.
//
// Region names: ServerEndpoint<N>::create(name, ...) maps "/cobot_<name>"; the tests pass
// name = "test_<pid>_<n>", so every region a test makes is "/cobot_test_<pid>_<n>".
namespace cobot::bridge::test {

constexpr int kDof = 7;
using Sample = StateSample<kDof>;
using Server = ServerEndpoint<kDof>;
using Client = ClientEndpoint<kDof>;
using Session = SessionInfo<kDof>;
using Cmd = Command<kDof>;

constexpr std::int64_t kMs = 1000000;  // ns per ms

// "/cobot_" + name: the POSIX shared-memory name of a region (contract of the endpoints).
inline std::string region_of(const std::string& name) {
  return "/cobot_" + name;
}

// test_<pid>_<n>, n counting up within this process.
inline std::string unique_name() {
  static int counter = 0;
  return "test_" + std::to_string(static_cast<long>(::getpid())) + "_" + std::to_string(counter++);
}

inline std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return s;
}

inline bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

// Owns one unique region name: unlinks the region (if any) when built and when destroyed, so a
// failing test never leaves a region behind. Uses shm_unlink directly, independent of
// ServerEndpoint::remove.
class RegionGuard {
 public:
  RegionGuard() : name_(unique_name()) { unlink(); }
  RegionGuard(const RegionGuard&) = delete;
  RegionGuard& operator=(const RegionGuard&) = delete;
  RegionGuard(RegionGuard&&) = delete;
  RegionGuard& operator=(RegionGuard&&) = delete;
  ~RegionGuard() { unlink(); }

  const std::string& name() const { return name_; }
  std::string region() const { return region_of(name_); }
  // Linux maps POSIX shared memory under /dev/shm.
  bool present() const {
    struct stat st {};
    return ::stat(("/dev/shm/cobot_" + name_).c_str(), &st) == 0;
  }

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
  T read(std::size_t off) const {
    T value{};
    std::memcpy(&value, bytes_ + off, sizeof(T));
    return value;
  }

  template <typename T>
  void write(std::size_t off, const T& value) {
    std::memcpy(bytes_ + off, &value, sizeof(T));
  }

 private:
  int fd_ = -1;
  std::size_t size_ = 0;
  unsigned char* bytes_ = nullptr;
};

// Offsets of the identity fields of the header, found by value in a fresh region. The byte layout
// is the builder's (design 9); the tests rely only on: the 64-bit magic is at offset 0 (first
// header field, design 6.5), then come the layout version, N, the state sample size, the command
// size, the response size and the state, command and response ring capacities, in that order (the
// order ClientEndpoint::open checks them), each a 4- or 8-byte little-endian integer within the
// first kIdentityBytes bytes. Each value is searched after the previous one, so the version (1)
// cannot be taken from a later field.
inline constexpr std::size_t kIdentityBytes = 512;

struct Identity {
  std::optional<std::size_t> version, dof, state_size, command_size, response_size;
  std::optional<std::size_t> state_slots, command_slots, response_slots;
};

inline Identity find_identity(const RawRegion& raw) {
  Identity id;
  std::size_t from = sizeof(std::uint64_t);
  const auto next = [&raw, &from](std::size_t value) {
    const std::optional<std::size_t> off =
        raw.find(static_cast<std::uint32_t>(value), from, kIdentityBytes, 4);
    if (off) {
      from = *off + sizeof(std::uint32_t);
    }
    return off;
  };
  id.version = next(kBridgeLayoutVersion);
  id.dof = next(static_cast<std::size_t>(kDof));
  id.state_size = next(sizeof(Sample));
  id.command_size = next(sizeof(Cmd));
  id.response_size = next(sizeof(Response));
  id.state_slots = next(kStateSlots);
  id.command_slots = next(kCommandSlots);
  id.response_slots = next(kResponseSlots);
  return id;
}

// ---------------------------------------------------------------- forked children

// Forks; the child runs `body` and leaves with _exit(body()) (never returns into GoogleTest, never
// runs destructors or atexit handlers of the parent's objects).
template <typename F>
pid_t fork_child(F&& body) {
  const pid_t pid = ::fork();
  if (pid == 0) {
    ::_exit(body());
  }
  return pid;
}

// A forked child that is always reaped: killed with SIGKILL and waited for when destroyed unless
// already reaped, also when the test fails early.
class Child {
 public:
  explicit Child(pid_t pid) : pid_(pid) {}
  Child(const Child&) = delete;
  Child& operator=(const Child&) = delete;
  Child(Child&&) = delete;
  Child& operator=(Child&&) = delete;
  ~Child() { kill_and_reap(); }

  pid_t pid() const { return pid_; }

  // Waits up to `timeout` for the child to exit and returns its exit code; -1 when it was killed
  // by a signal or did not exit in time (then it is killed and reaped).
  int wait_exit(std::chrono::milliseconds timeout) {
    if (pid_ <= 0) {
      return -1;
    }
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    for (;;) {
      int status = 0;
      const pid_t r = ::waitpid(pid_, &status, WNOHANG);
      if (r == pid_) {
        pid_ = -1;
        return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
      }
      if (r < 0 || std::chrono::steady_clock::now() > deadline) {
        kill_and_reap();
        return -1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  void kill_and_reap() {
    if (pid_ <= 0) {
      return;
    }
    ::kill(pid_, SIGKILL);
    int status = 0;
    ::waitpid(pid_, &status, 0);
    pid_ = -1;
  }

 private:
  pid_t pid_;
};

// A pipe whose two ends are closed when destroyed.
class Pipe {
 public:
  Pipe() {
    if (::pipe(fd_) != 0) {
      fd_[0] = fd_[1] = -1;
    }
  }
  Pipe(const Pipe&) = delete;
  Pipe& operator=(const Pipe&) = delete;
  Pipe(Pipe&&) = delete;
  Pipe& operator=(Pipe&&) = delete;
  ~Pipe() {
    for (const int fd : fd_) {
      if (fd >= 0) {
        ::close(fd);
      }
    }
  }

  bool ok() const { return fd_[0] >= 0 && fd_[1] >= 0; }
  int read_end() const { return fd_[0]; }
  int write_end() const { return fd_[1]; }

  // Whole-object write and blocking read of a trivially copyable value.
  template <typename T>
  bool put(const T& value) const {
    return ::write(fd_[1], &value, sizeof(T)) == static_cast<ssize_t>(sizeof(T));
  }
  template <typename T>
  bool get(T& value) const {
    return ::read(fd_[0], &value, sizeof(T)) == static_cast<ssize_t>(sizeof(T));
  }
  // get() once a value is there, false after `timeout_ms` without one (a child that died).
  template <typename T>
  bool get_within(T& value, int timeout_ms) const {
    pollfd p{fd_[0], POLLIN, 0};
    return ::poll(&p, 1, timeout_ms) == 1 && (p.revents & POLLIN) != 0 && get(value);
  }
  // True when a value can be read without blocking (waits up to `timeout_ms`).
  bool readable(int timeout_ms) const {
    pollfd p{fd_[0], POLLIN, 0};
    return ::poll(&p, 1, timeout_ms) == 1 && (p.revents & POLLIN) != 0;
  }

 private:
  int fd_[2] = {-1, -1};
};

// ---------------------------------------------------------------- patterned samples

// Every field of sample `step` is a function of the step (b = step * 8, all values exact in
// double for step < 2^40): q[i] = b + i, qd[i] = -(b + i), q_ref[i] = b + i + 0.25,
// qd_ref[i] = b + i + 0.5, tau[i] = b + i + 0.75, q_true[i] = b - i, T_b_e.p[k] = b + 10 + k,
// T_b_e.R[k] = b + 20 + k, T_b_d.p[k] = b + 30 + k, T_b_d.R[k] = b + 40 + k,
// task_error[k] = b + 50 + k, step_time_us = b + 60, t_ns = step * 1000, the four bytes
// step % 6, step % 7, step % 5, step % 3.
inline Sample make_sample(std::uint64_t step) {
  Sample s{};
  const double b = static_cast<double>(step) * 8.0;
  s.step = step;
  s.t_ns = static_cast<std::int64_t>(step) * 1000;
  s.mode = static_cast<std::uint8_t>(step % 6);
  s.hold_reason = static_cast<std::uint8_t>(step % 7);
  s.cmd_mode = static_cast<std::uint8_t>(step % 5);
  s.manual_stop = static_cast<std::uint8_t>(step % 3);
  for (int i = 0; i < kDof; ++i) {
    const double v = b + i;
    s.q[i] = v;
    s.qd[i] = -v;
    s.q_ref[i] = v + 0.25;
    s.qd_ref[i] = v + 0.5;
    s.tau[i] = v + 0.75;
    s.q_true[i] = b - i;
  }
  for (int k = 0; k < 3; ++k) {
    s.T_b_e.p[k] = b + 10 + k;
    s.T_b_d.p[k] = b + 30 + k;
  }
  for (int k = 0; k < 9; ++k) {
    s.T_b_e.R[k] = b + 20 + k;
    s.T_b_d.R[k] = b + 40 + k;
  }
  for (int k = 0; k < 6; ++k) {
    s.task_error[k] = b + 50 + k;
  }
  s.step_time_us = b + 60;
  return s;
}

// True when every field of `s` is the one make_sample(s.step) gives (no field from another step).
inline bool sample_intact(const Sample& s) {
  const Sample w = make_sample(s.step);
  bool ok = s.t_ns == w.t_ns && s.mode == w.mode && s.hold_reason == w.hold_reason &&
            s.cmd_mode == w.cmd_mode && s.manual_stop == w.manual_stop &&
            s.step_time_us == w.step_time_us;
  for (int i = 0; i < kDof; ++i) {
    ok = ok && s.q[i] == w.q[i] && s.qd[i] == w.qd[i] && s.q_ref[i] == w.q_ref[i] &&
         s.qd_ref[i] == w.qd_ref[i] && s.tau[i] == w.tau[i] && s.q_true[i] == w.q_true[i];
  }
  for (int k = 0; k < 3; ++k) {
    ok = ok && s.T_b_e.p[k] == w.T_b_e.p[k] && s.T_b_d.p[k] == w.T_b_d.p[k];
  }
  for (int k = 0; k < 9; ++k) {
    ok = ok && s.T_b_e.R[k] == w.T_b_e.R[k] && s.T_b_d.R[k] == w.T_b_d.R[k];
  }
  for (int k = 0; k < 6; ++k) {
    ok = ok && s.task_error[k] == w.task_error[k];
  }
  return ok;
}

// A session whose content is tagged by `tag` (paths, seed and the per-joint limits).
inline Session make_session(int tag) {
  Session s{};
  const std::string t = std::to_string(tag);
  copy_text(s.config_path, kPathChars, "/tmp/config_" + t + ".yaml");
  copy_text(s.imperfections_path, kPathChars, "/tmp/imperfections_" + t + ".yaml");
  s.seed = 1000U + static_cast<std::uint64_t>(tag);
  s.command_source_external = 1;
  s.control_rate_hz = 1000.0;
  s.soft_margin_rad = 0.05 * tag;
  s.jog_margin_rad = 0.03 * tag;
  for (int i = 0; i < kDof; ++i) {
    s.q_min[i] = -1.0 - i - tag;
    s.q_max[i] = 1.0 + i + tag;
    s.qd_max[i] = 2.0 + i + tag;
    s.tau_max[i] = 100.0 + i + tag;
    s.rated[i] = 50.0 + i + tag;
  }
  s.T_tip_tcp.p[2] = 0.1 * tag;
  for (int k = 0; k < 9; ++k) {
    s.T_tip_tcp.R[k] = (k % 4 == 0) ? 1.0 : 0.0;
  }
  return s;
}

}  // namespace cobot::bridge::test
