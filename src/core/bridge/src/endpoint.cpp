#include "cobot/bridge/endpoint.hpp"

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <new>
#include <random>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <system_error>
#include <unistd.h>
#include <utility>
#include <vector>

#include "cobot/bridge/seqlock.hpp"
#include "cobot/common/dof.hpp"

// Region layout: Region<N> below, from offset 0. The identity fields follow the magic in the
// order ClientEndpoint::open checks them. The magic is stored last at creation (release), so a
// region that is still being written is never accepted. State sample k (1, 2, ...) lives in slot
// k % kStateSlots; that slot's sequence after sample k is 2 * ceil(k / kStateSlots), which tells
// sample k from a later one in the same slot. state_head is the number of the newest stable
// sample. Ring entries are copied before the counter that publishes them (release/acquire).
namespace cobot::bridge {
namespace detail {

using Counter = std::atomic<std::uint64_t>;
constexpr std::size_t kLine = 64;

struct Header {
  Counter magic{0};  // offset 0
  std::uint32_t layout_version = 0, dof = 0, state_size = 0, command_size = 0, response_size = 0;
  std::uint32_t state_slots = 0, command_slots = 0, response_slots = 0;
  Counter server_pid{0}, server_beat_ns{0};
  Counter owner_token{0}, owner_pid{0}, owner_beat_ns{0};
  Counter dropped_responses{0};
};

template <int N>
struct Region {
  Header h{};
  alignas(kLine) SeqSlot<SessionInfo<N>> session;
  alignas(kLine) SeqSlot<JogRequest> jog;
  alignas(kLine) Counter state_head{0};
  alignas(kLine) Counter command_head{0};
  alignas(kLine) Counter command_tail{0};
  alignas(kLine) Counter response_head{0};
  alignas(kLine) Counter response_tail{0};
  alignas(kLine) SeqSlot<StateSample<N>> states[kStateSlots];
  Command<N> commands[kCommandSlots];
  Response responses[kResponseSlots];
};

}  // namespace detail

namespace {

using detail::Header;
using detail::Region;
constexpr std::size_t kMaxName = 64;
constexpr std::size_t kMiB = std::size_t{1024} * 1024;
constexpr mode_t kMode = 0600;
constexpr std::int64_t kNsPerS = 1000000000;

bool valid_name(const std::string& name) {
  if (name.empty() || name.size() > kMaxName) {
    return false;
  }
  for (const char c : name) {
    if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')) {
      return false;
    }
  }
  return true;
}

std::string bad_name(const std::string& name) {
  return "bridge: bad name '" + name + "' (want [A-Za-z0-9_]{1,64})";
}

std::string region_of(const std::string& name) {
  return "/cobot_" + name;
}

std::string errno_text(int err) {
  return std::error_code(err, std::generic_category()).message();
}

// False only when the process is known to be gone (kill(pid, 0) == ESRCH).
bool pid_alive(std::uint64_t pid) noexcept {
  return pid == 0 || ::kill(static_cast<pid_t>(pid), 0) == 0 || errno != ESRCH;
}

std::uint64_t load(const detail::Counter& c) noexcept {
  return c.load(std::memory_order_acquire);
}

// Maps a whole region of `size` bytes read-write; nullptr on failure. MAP_POPULATE prefaults the
// page tables so the control thread's first publish into a ring page takes no page fault.
void* map(int fd, std::size_t size) {
  void* p = ::mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_POPULATE, fd, 0);
  return p == MAP_FAILED ? nullptr : p;
}

// Server pid of an existing bridge region that is still served, 0 when stale (server gone, not a
// bridge region, or unreadable). A region whose magic is not stored yet is still being created:
// its server pid counts as well once stored. Not covered: a region found before its creator's
// posix_fallocate or server_pid store reads as stale and is replaced, so concurrent create() calls
// for one name are not supported (one controller per name).
std::uint64_t live_server(const std::string& region) {
  const int fd = ::shm_open(region.c_str(), O_RDWR, 0);
  if (fd < 0) {
    return 0;
  }
  struct stat st {};
  const bool big = ::fstat(fd, &st) == 0 && static_cast<std::size_t>(st.st_size) >= sizeof(Header);
  void* p = big ? map(fd, sizeof(Header)) : nullptr;
  ::close(fd);
  if (p == nullptr) {
    return 0;
  }
  const auto* h = std::launder(static_cast<Header*>(p));
  const std::uint64_t magic = load(h->magic);
  const std::uint64_t pid = magic == kBridgeMagic || magic == 0 ? load(h->server_pid) : 0;
  ::munmap(p, sizeof(Header));
  return pid != 0 && pid_alive(pid) ? pid : 0;
}

std::string hex(std::uint64_t v) {
  char buf[19];
  std::snprintf(buf, sizeof(buf), "0x%016llx", static_cast<unsigned long long>(v));
  return buf;
}

std::string mismatch(const char* field, std::uint64_t region, std::uint64_t client) {
  return std::string(field) + ": region " + std::to_string(region) + ", client " +
         std::to_string(client);
}

// Why the header does not describe a Region<N> of `size` bytes; empty when it does.
template <int N>
std::string check_header(const Header& h, std::size_t size) {
  struct Field {
    const char* name;
    std::uint32_t region;
    std::size_t client;
  };
  if (const std::uint64_t magic = load(h.magic); magic != kBridgeMagic) {
    return "magic: region " + hex(magic) + ", client " + hex(kBridgeMagic) +
           " (not a bridge region)";
  }
  const Field fields[] = {
      {"layout version", h.layout_version, kBridgeLayoutVersion},
      {"dof", h.dof, static_cast<std::size_t>(N)},
      {"state sample size", h.state_size, sizeof(StateSample<N>)},
      {"command size", h.command_size, sizeof(Command<N>)},
      {"response size", h.response_size, sizeof(Response)},
      {"state ring slots", h.state_slots, kStateSlots},
      {"command ring slots", h.command_slots, kCommandSlots},
      {"response ring slots", h.response_slots, kResponseSlots},
  };
  for (const Field& f : fields) {
    if (f.region != f.client) {
      return mismatch(f.name, f.region, f.client);
    }
  }
  if (size < sizeof(Region<N>)) {
    return mismatch("region bytes", size, sizeof(Region<N>));
  }
  return {};
}

std::uint64_t random_token() {
  std::random_device rd;
  std::uint64_t t = 0;
  while (t == 0) {
    t = (static_cast<std::uint64_t>(rd()) << 32U) ^ static_cast<std::uint64_t>(rd());
  }
  return t;
}

}  // namespace

std::int64_t monotonic_ns() noexcept {
  timespec ts{};
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * kNsPerS + ts.tv_nsec;
}

// ---------------------------------------------------------------- server

template <int N>
ServerEndpoint<N>::ServerEndpoint(Region<N>* region, std::size_t size) noexcept
    : region_(region), size_(size) {}

template <int N>
ServerEndpoint<N>::ServerEndpoint(ServerEndpoint&& other) noexcept
    : region_(std::exchange(other.region_, nullptr)),
      size_(std::exchange(other.size_, 0)),
      generation_(other.generation_),
      pending_token_(other.pending_token_),
      pending_since_ns_(other.pending_since_ns_) {}

template <int N>
ServerEndpoint<N>& ServerEndpoint<N>::operator=(ServerEndpoint&& other) noexcept {
  if (this != &other) {
    if (region_ != nullptr) {
      ::munmap(region_, size_);
    }
    region_ = std::exchange(other.region_, nullptr);
    size_ = std::exchange(other.size_, 0);
    generation_ = other.generation_;
    pending_token_ = other.pending_token_;
    pending_since_ns_ = other.pending_since_ns_;
  }
  return *this;
}

template <int N>
ServerEndpoint<N>::~ServerEndpoint() {
  if (region_ != nullptr) {
    ::munmap(region_, size_);
  }
}

template <int N>
common::Result<ServerEndpoint<N>> ServerEndpoint<N>::create(const std::string& name,
                                                            const SessionInfo<N>& session) {
  using R = common::Result<ServerEndpoint<N>>;
  if (!valid_name(name)) {
    return R::error(bad_name(name));
  }
  const std::string region = region_of(name);
  const std::size_t size = sizeof(Region<N>);
  int fd = ::shm_open(region.c_str(), O_CREAT | O_EXCL | O_RDWR, kMode);
  if (fd < 0 && errno == EEXIST) {
    const std::uint64_t pid = live_server(region);
    if (pid != 0) {
      return R::error("bridge " + region + " is served by pid " + std::to_string(pid));
    }
    ::shm_unlink(region.c_str());  // stale: the server is gone
    fd = ::shm_open(region.c_str(), O_CREAT | O_EXCL | O_RDWR, kMode);
  }
  if (fd < 0) {
    const int open_err = errno;
    return R::error("bridge: cannot create " + region + ": " + errno_text(open_err));
  }
  const int err = ::posix_fallocate(fd, 0, static_cast<off_t>(size));
  void* p = err == 0 ? map(fd, size) : nullptr;
  const int map_err = errno;
  ::close(fd);
  if (p == nullptr) {
    ::shm_unlink(region.c_str());
    if (err == ENOSPC) {
      return R::error("bridge " + region + ": /dev/shm too small: need " +
                      std::to_string((size + kMiB - 1) / kMiB) + " MB");
    }
    return R::error("bridge: cannot reserve " + region + ": " +
                    errno_text(err != 0 ? err : map_err));
  }
  auto* r = new (p) Region<N>;  // zero pages from posix_fallocate; atomics start at 0
  Header& h = r->h;
  h.layout_version = kBridgeLayoutVersion;
  h.dof = static_cast<std::uint32_t>(N);
  h.state_size = static_cast<std::uint32_t>(sizeof(StateSample<N>));
  h.command_size = static_cast<std::uint32_t>(sizeof(Command<N>));
  h.response_size = static_cast<std::uint32_t>(sizeof(Response));
  h.state_slots = static_cast<std::uint32_t>(kStateSlots);
  h.command_slots = static_cast<std::uint32_t>(kCommandSlots);
  h.response_slots = static_cast<std::uint32_t>(kResponseSlots);
  h.server_pid.store(static_cast<std::uint64_t>(::getpid()), std::memory_order_relaxed);
  h.server_beat_ns.store(static_cast<std::uint64_t>(monotonic_ns()), std::memory_order_relaxed);
  SessionInfo<N> s = session;
  s.generation = 0;
  r->session.write(s);
  h.magic.store(kBridgeMagic, std::memory_order_release);  // last: a complete header
  return R::ok(ServerEndpoint(r, size));
}

template <int N>
common::Status ServerEndpoint<N>::remove(const std::string& name) {
  if (!valid_name(name)) {
    return common::Status::error(bad_name(name));
  }
  if (::shm_unlink(region_of(name).c_str()) != 0 && errno != ENOENT) {
    const int err = errno;
    return common::Status::error("bridge: cannot remove " + region_of(name) + ": " +
                                 errno_text(err));
  }
  return common::Status::ok();
}

template <int N>
void ServerEndpoint<N>::publish(const StateSample<N>& s) noexcept {
  const std::uint64_t k = region_->state_head.load(std::memory_order_relaxed) + 1;
  region_->states[k % kStateSlots].write(s);
  region_->state_head.store(k, std::memory_order_release);
}

template <int N>
bool ServerEndpoint<N>::read_jog(JogRequest& out) const noexcept {
  return region_->jog.try_read(out) && out.refresh != 0;
}

template <int N>
std::uint64_t ServerEndpoint<N>::owner_token() const noexcept {
  return load(region_->h.owner_token);
}

template <int N>
bool ServerEndpoint<N>::pop_command(Command<N>& out) noexcept {
  const std::uint64_t t = region_->command_tail.load(std::memory_order_relaxed);
  if (load(region_->command_head) == t) {
    return false;
  }
  std::memcpy(&out, &region_->commands[t % kCommandSlots], sizeof(Command<N>));
  region_->command_tail.store(t + 1, std::memory_order_release);
  return true;
}

template <int N>
bool ServerEndpoint<N>::push_response(const Response& r) noexcept {
  const std::uint64_t h = region_->response_head.load(std::memory_order_relaxed);
  if (h - load(region_->response_tail) >= kResponseSlots) {
    region_->h.dropped_responses.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  std::memcpy(&region_->responses[h % kResponseSlots], &r, sizeof(Response));
  region_->response_head.store(h + 1, std::memory_order_release);
  return true;
}

template <int N>
void ServerEndpoint<N>::beat(std::int64_t now_ns) noexcept {
  region_->h.server_beat_ns.store(static_cast<std::uint64_t>(now_ns), std::memory_order_release);
}

template <int N>
OwnerState ServerEndpoint<N>::owner_state(std::int64_t now_ns) const noexcept {
  const Header& h = region_->h;
  const std::uint64_t token = load(h.owner_token);
  if (token == 0) {
    return OwnerState::kNone;
  }
  const std::uint64_t pid = load(h.owner_pid);
  if (pid == 0) {
    // A claim in progress: its pid and heartbeat follow the token. Bounded by the heartbeat
    // timeout from the first time this server saw it, so a claimer killed between the token CAS
    // and its stamps does not hold the region forever.
    if (pending_token_ != token) {
      pending_token_ = token;
      pending_since_ns_ = now_ns;
    }
    return now_ns - pending_since_ns_ > kHeartbeatTimeoutNs ? OwnerState::kHeartbeatLost
                                                            : OwnerState::kAlive;
  }
  if (!pid_alive(pid)) {
    return OwnerState::kProcessGone;
  }
  const auto beat = static_cast<std::int64_t>(load(h.owner_beat_ns));
  return now_ns - beat > kHeartbeatTimeoutNs ? OwnerState::kHeartbeatLost : OwnerState::kAlive;
}

template <int N>
void ServerEndpoint<N>::clear_owner() noexcept {
  Header& h = region_->h;
  std::uint64_t token = load(h.owner_token);
  if (token == 0) {
    return;
  }
  // Stamps first, so the next owner's token is never paired with this owner's pid or heartbeat.
  h.owner_pid.store(0, std::memory_order_release);
  h.owner_beat_ns.store(0, std::memory_order_release);
  h.owner_token.compare_exchange_strong(token, 0, std::memory_order_acq_rel);
}

template <int N>
void ServerEndpoint<N>::set_session(const SessionInfo<N>& s) noexcept {
  SessionInfo<N> next = s;
  next.generation = ++generation_;
  region_->session.write(next);
}

template <int N>
std::uint64_t ServerEndpoint<N>::dropped_responses() const noexcept {
  return region_->h.dropped_responses.load(std::memory_order_relaxed);
}

// ---------------------------------------------------------------- client

template <int N>
ClientEndpoint<N>::ClientEndpoint(Region<N>* region,
                                  std::size_t size,
                                  std::string region_name) noexcept
    : region_(region), size_(size), region_name_(std::move(region_name)) {}

template <int N>
ClientEndpoint<N>::ClientEndpoint(ClientEndpoint&& other) noexcept
    : region_(std::exchange(other.region_, nullptr)),
      size_(std::exchange(other.size_, 0)),
      region_name_(std::move(other.region_name_)),
      token_(other.token_),
      next_id_(other.next_id_),
      next_sample_(other.next_sample_),
      lost_(other.lost_) {}

template <int N>
ClientEndpoint<N>& ClientEndpoint<N>::operator=(ClientEndpoint&& other) noexcept {
  if (this != &other) {
    if (region_ != nullptr) {
      ::munmap(region_, size_);
    }
    region_ = std::exchange(other.region_, nullptr);
    size_ = std::exchange(other.size_, 0);
    region_name_ = std::move(other.region_name_);
    token_ = other.token_;
    next_id_ = other.next_id_;
    next_sample_ = other.next_sample_;
    lost_ = other.lost_;
  }
  return *this;
}

template <int N>
ClientEndpoint<N>::~ClientEndpoint() {
  if (region_ != nullptr) {
    ::munmap(region_, size_);
  }
}

template <int N>
common::Result<ClientEndpoint<N>> ClientEndpoint<N>::open(const std::string& name) {
  using R = common::Result<ClientEndpoint<N>>;
  if (!valid_name(name)) {
    return R::error(bad_name(name));
  }
  const std::string region = region_of(name);
  const int fd = ::shm_open(region.c_str(), O_RDWR, 0);
  if (fd < 0) {
    const int err = errno;
    return R::error(err == ENOENT ? "no bridge " + region
                                  : "bridge: cannot open " + region + ": " + errno_text(err));
  }
  struct stat st {};
  const std::size_t size =
      ::fstat(fd, &st) == 0 ? static_cast<std::size_t>(st.st_size) : std::size_t{0};
  void* p = size >= sizeof(Header) ? map(fd, size) : nullptr;
  ::close(fd);
  if (p == nullptr) {
    return R::error("bridge " + region + ": region too small or unreadable (" +
                    std::to_string(size) + " bytes)");
  }
  ClientEndpoint c(std::launder(static_cast<Region<N>*>(p)), size, region);
  const std::string why = check_header<N>(c.region_->h, size);
  if (!why.empty()) {
    return R::error("bridge " + region + ": " + why);
  }
  c.token_ = random_token();
  const std::uint64_t head = load(c.region_->state_head);
  c.next_sample_ = (head > kStateSlots ? head - kStateSlots : 0) + 1;
  return R::ok(std::move(c));
}

template <int N>
common::Status ClientEndpoint<N>::claim(std::int64_t now_ns) {
  Header& h = region_->h;
  const auto me = static_cast<std::uint64_t>(::getpid());
  std::uint64_t held = load(h.owner_token);
  // The pid and heartbeat are written only once the token is ours, so a losing concurrent claim
  // never stamps its pid next to the winner's token. Until then they are 0 (release() and
  // clear_owner() zero them before freeing the token), which the server reads as a claim in
  // progress.
  if (held == 0 && h.owner_token.compare_exchange_strong(held, token_, std::memory_order_acq_rel)) {
    held = token_;
  }
  if (held != token_) {
    return common::Status::error("bridge " + region_name_ + ": held by pid " +
                                 std::to_string(load(h.owner_pid)));
  }
  h.owner_beat_ns.store(static_cast<std::uint64_t>(now_ns), std::memory_order_release);
  h.owner_pid.store(me, std::memory_order_release);
  return common::Status::ok();
}

template <int N>
void ClientEndpoint<N>::release() noexcept {
  if (!is_owner()) {
    return;
  }
  Header& h = region_->h;
  h.owner_pid.store(0, std::memory_order_release);
  h.owner_beat_ns.store(0, std::memory_order_release);
  std::uint64_t mine = token_;
  h.owner_token.compare_exchange_strong(mine, 0, std::memory_order_acq_rel);
}

template <int N>
bool ClientEndpoint<N>::is_owner() const noexcept {
  return load(region_->h.owner_token) == token_;
}

template <int N>
void ClientEndpoint<N>::beat(std::int64_t now_ns) noexcept {
  if (is_owner()) {
    // The pid again too: a clear_owner() racing this owner's claim may have zeroed it.
    region_->h.owner_beat_ns.store(static_cast<std::uint64_t>(now_ns), std::memory_order_release);
    region_->h.owner_pid.store(static_cast<std::uint64_t>(::getpid()), std::memory_order_release);
  }
}

template <int N>
std::int64_t ClientEndpoint<N>::owner_pid() const noexcept {
  return static_cast<std::int64_t>(load(region_->h.owner_pid));
}

template <int N>
std::int64_t ClientEndpoint<N>::server_pid() const noexcept {
  return static_cast<std::int64_t>(load(region_->h.server_pid));
}

template <int N>
bool ClientEndpoint<N>::server_alive(std::int64_t now_ns) const noexcept {
  const auto beat = static_cast<std::int64_t>(load(region_->h.server_beat_ns));
  const std::uint64_t pid = load(region_->h.server_pid);
  return now_ns - beat < kHeartbeatTimeoutNs && pid != 0 && pid_alive(pid);
}

template <int N>
common::Result<std::uint64_t> ClientEndpoint<N>::send(Command<N> c) {
  using R = common::Result<std::uint64_t>;
  if (!is_owner()) {
    return R::error("bridge " + region_name_ + ": not the owner");
  }
  const std::uint64_t h = load(region_->command_head);
  if (h - load(region_->command_tail) >= kCommandSlots) {
    return R::error("bridge " + region_name_ + ": command ring full");
  }
  c.id = ++next_id_;
  c.owner_token = token_;
  std::memcpy(&region_->commands[h % kCommandSlots], &c, sizeof(Command<N>));
  region_->command_head.store(h + 1, std::memory_order_release);
  return R::ok(c.id);
}

template <int N>
bool ClientEndpoint<N>::poll_response(Response& out) noexcept {
  const std::uint64_t owner = load(region_->h.owner_token);
  if (owner != 0 && owner != token_) {
    return false;  // one consumer: the responses belong to the owner
  }
  // Without an owner several clients may poll: the tail moves by CAS, so each response goes to
  // exactly one of them, and a copy whose slot was freed (and maybe refilled) under it is dropped.
  std::uint64_t t = load(region_->response_tail);
  if (load(region_->response_head) == t) {
    return false;
  }
  Response copy;
  std::memcpy(&copy, &region_->responses[t % kResponseSlots], sizeof(Response));
  if (!region_->response_tail.compare_exchange_strong(t, t + 1, std::memory_order_acq_rel)) {
    return false;  // another client took it; poll again
  }
  out = copy;
  return true;
}

template <int N>
void ClientEndpoint<N>::write_jog(JogRequest j) noexcept {
  if (!is_owner()) {
    return;  // one writer: only the owner writes the slot
  }
  // Refresh counts the slot's writes, not this client's: a new owner's first write always differs
  // from the last one the server saw. One writer (the owner), so the base cannot move meanwhile.
  j.owner_token = token_;
  j.refresh = region_->jog.next_base() / 2 + 1;
  region_->jog.write(j);
}

template <int N>
std::size_t ClientEndpoint<N>::read_states(std::vector<StateSample<N>>& out) {
  const std::uint64_t head = load(region_->state_head);
  std::uint64_t k = next_sample_;
  if (head < k) {
    return 0;
  }
  if (head - k + 1 > kStateSlots) {  // overwritten before this call
    lost_ += head - kStateSlots + 1 - k;
    k = head - kStateSlots + 1;
  }
  std::size_t got = 0;
  for (; k <= head; ++k) {
    const std::uint64_t expected = 2 * ((k + kStateSlots - 1) / kStateSlots);
    out.emplace_back();
    if (region_->states[k % kStateSlots].try_read_at(out.back(), expected)) {
      ++got;
    } else {
      out.pop_back();
      ++lost_;
    }
  }
  next_sample_ = head + 1;
  return got;
}

template <int N>
std::uint64_t ClientEndpoint<N>::lost() const noexcept {
  return lost_;
}

template <int N>
bool ClientEndpoint<N>::session(SessionInfo<N>& out) const noexcept {
  return region_->session.try_read(out);
}

#define COBOT_INSTANTIATE(N)        \
  template class ServerEndpoint<N>; \
  template class ClientEndpoint<N>;
COBOT_INSTANTIATE_DOF(COBOT_INSTANTIATE)
#undef COBOT_INSTANTIATE

}  // namespace cobot::bridge
