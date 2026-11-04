#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "cobot/bridge/wire.hpp"
#include "cobot/common/result.hpp"

// Endpoints of the studio bridge (feature shm-bridge, cobot_studio design 6.5, ADR-022), Linux
// only: the POSIX shared-memory region "/cobot_<name>", name [A-Za-z0-9_]{1,64}. The region holds
// a header (magic, layout version, N, the three record sizes and ring capacities, server pid and
// heartbeat, owner token, pid and heartbeat), the session info and the jog request (seqlocks), the
// state ring (control thread writes, never waits: overwrite + per-slot seqlock), the command ring
// (studio -> bridge thread) and the response ring (bridge thread -> studio), both SPSC with
// head/tail counters. No command is executed here.
namespace cobot::bridge {

std::int64_t monotonic_ns() noexcept;  // CLOCK_MONOTONIC, same clock in every process

enum class OwnerState { kNone, kAlive, kHeartbeatLost, kProcessGone };

namespace detail {
template <int N>
struct Region;
}

template <int N>
class ServerEndpoint {  // controller side; move-only; destructor unmaps, never unlinks
 public:
  // Creates the region (posix_fallocate, "/dev/shm too small: need <n> MB" on ENOSPC). An
  // existing region whose server pid is alive -> error "bridge /cobot_<name> is served by pid
  // <p>"; a stale one (server gone) is unlinked and recreated. Writes the header and `session`.
  static common::Result<ServerEndpoint> create(const std::string& name,
                                               const SessionInfo<N>& session);
  static common::Status remove(const std::string& name);  // shm_unlink; missing is ok

  ServerEndpoint(ServerEndpoint&& other) noexcept;
  ServerEndpoint& operator=(ServerEndpoint&& other) noexcept;
  ServerEndpoint(const ServerEndpoint&) = delete;
  ServerEndpoint& operator=(const ServerEndpoint&) = delete;
  ~ServerEndpoint();

  // Control thread (no allocation, lock, wait or I/O):
  void publish(const StateSample<N>& s) noexcept;  // overwrite oldest, never waits
  bool read_jog(JogRequest& out) const noexcept;   // false when torn (retry next step) or none
  std::uint64_t owner_token() const noexcept;      // 0 = nobody
  // Bridge thread:
  bool pop_command(Command<N>& out) noexcept;
  bool push_response(const Response& r) noexcept;  // false when the ring is full (counted)
  void beat(std::int64_t now_ns) noexcept;         // server heartbeat
  // kNone without owner; kProcessGone when kill(pid, 0) == ESRCH; kHeartbeatLost when the owner's
  // heartbeat is older than kHeartbeatTimeoutNs; kAlive while a claim is in progress (pid 0),
  // kHeartbeatLost once that has lasted kHeartbeatTimeoutNs since this server first saw it.
  OwnerState owner_state(std::int64_t now_ns) const noexcept;
  void clear_owner() noexcept;                         // CAS the current token -> 0
  void set_session(const SessionInfo<N>& s) noexcept;  // generation + 1
  std::uint64_t dropped_responses() const noexcept;

 private:
  ServerEndpoint(detail::Region<N>* region, std::size_t size) noexcept;
  detail::Region<N>* region_ = nullptr;
  std::size_t size_ = 0;
  std::uint64_t generation_ = 0;
  // owner_state(): the claim-in-progress token (pid 0) and when it was first seen.
  mutable std::uint64_t pending_token_ = 0;
  mutable std::int64_t pending_since_ns_ = 0;
};

template <int N>
class ClientEndpoint {  // studio side; move-only
 public:
  // Maps an existing region read-write after checking, in this order: magic, layout version, N
  // ("dof"), state sample size, command size, response size, the three ring capacities. A
  // mismatch is an error naming the field and both values ("dof: region 6, client 7"); "no bridge
  // /cobot_<name>" when absent.
  static common::Result<ClientEndpoint> open(const std::string& name);

  ClientEndpoint(ClientEndpoint&& other) noexcept;
  ClientEndpoint& operator=(ClientEndpoint&& other) noexcept;
  ClientEndpoint(const ClientEndpoint&) = delete;
  ClientEndpoint& operator=(const ClientEndpoint&) = delete;
  ~ClientEndpoint();

  // Authority (ADR-022): this client's random non-zero token CAS 0 -> token, owner pid =
  // getpid(), heartbeat = now. Held by someone else -> error "held by pid <p>". Claiming twice
  // from the same client is ok.
  common::Status claim(std::int64_t now_ns);
  void release() noexcept;  // CAS own token -> 0; no-op if not the owner
  bool is_owner() const noexcept;
  void beat(std::int64_t now_ns) noexcept;  // only while owner
  std::int64_t owner_pid() const noexcept;
  bool server_alive(std::int64_t now_ns) const noexcept;  // heartbeat < 200 ms old, pid alive
  // Commands: id and token filled in; checks ownership right before the push. Errors "not the
  // owner", "command ring full".
  common::Result<std::uint64_t> send(Command<N> c);
  // False while another client owns the region (the responses are the owner's). Without an owner
  // any client may poll (each response reaches one of them), including responses left by a
  // previous owner: claim before polling to get only your own.
  bool poll_response(Response& out) noexcept;
  // Owner only: token filled in; refresh = the slot's write count + 1, so it changes on every
  // write, also across owners.
  void write_jog(JogRequest j) noexcept;
  // Appends every sample published since the last call (since open: the samples still in the
  // ring), oldest first; samples overwritten before they were read, or torn while copying, are
  // skipped and counted in lost().
  std::size_t read_states(std::vector<StateSample<N>>& out);
  std::uint64_t lost() const noexcept;
  bool session(SessionInfo<N>& out) const noexcept;  // seqlock read; false if torn

 private:
  ClientEndpoint(detail::Region<N>* region, std::size_t size, std::string region_name) noexcept;
  detail::Region<N>* region_ = nullptr;
  std::size_t size_ = 0;
  std::string region_name_;
  std::uint64_t token_ = 0;
  std::uint64_t next_id_ = 0;
  std::uint64_t next_sample_ = 1;  // number of the next state sample to read
  std::uint64_t lost_ = 0;
};

}  // namespace cobot::bridge
